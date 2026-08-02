"""Module for the LaunchInspector class."""

import asyncio
import collections.abc
import contextlib
import dataclasses
import logging
import platform
import signal
import threading
from collections.abc import Coroutine, Iterable

import launch.logging
import osrf_pycommon.process_utils
from launch.event import Event
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import IncludeLaunchDescription, Shutdown
from launch.launch_context import LaunchContext
from launch.launch_description import LaunchDescription
from launch.launch_description_entity import LaunchDescriptionEntity

try:
    # ROS Humble
    from launch.some_actions_type import SomeActionsType
except ImportError:
    # ROS Jazzy
    from launch.some_entities_type import SomeEntitiesType as SomeActionsType
from launch.utilities import AsyncSafeSignalManager, is_a_subclass

from .event_handlers import OnIncludeLaunchDescription
from .launch_dump import LaunchDump
from .ros_cmdline import parse_ros_cmdline
from .visitor import visit_entity


class LaunchInspector:
    """Service that manages the event loop and runtime for launched system."""

    def __init__(
        self,
        *,
        argv: Iterable[str] | None = None,
        noninteractive: bool = False,
        debug: bool = False,
    ) -> None:
        """
        Create a LaunchInspector.

        :param: argv stored in the context for access by the entities, None results in []
        :param: noninteractive if True (not default), this service will assume it has
            no terminal associated e.g. it is being executed from a non interactive script
        :param: debug if True (not default), asyncio the logger are seutp for debug
        """
        # Setup logging and debugging.
        launch.logging.launch_config.level = logging.DEBUG if debug else logging.INFO
        self.__debug = debug
        self.__argv = argv if argv is not None else []

        # Setup logging
        self.__logger = launch.logging.get_logger("dump_launch")

        # Setup context and register a built-in event handler for bootstrapping.
        self.__context = LaunchContext(argv=self.__argv, noninteractive=noninteractive)
        self.__context.register_event_handler(OnIncludeLaunchDescription())
        # self.__context.register_event_handler(
        #     OnProcessStart(on_start=self.__on_process_start)
        # )
        self.__context.register_event_handler(OnProcessExit(on_exit=self.__on_process_exit))
        self.__context.register_event_handler(OnShutdown(on_shutdown=self.__on_shutdown))

        # Setup storage for state.
        self._entity_future_pairs = []  # type: List[Tuple[LaunchDescriptionEntity, asyncio.Future]]

        # Used to allow asynchronous use of self.__loop_from_run_thread without
        # it being set to None by run() as it exits.
        self.__loop_from_run_thread_lock = threading.RLock()
        self.__loop_from_run_thread = None
        self.__this_task = None

        # Used to indicate when shutdown() has been called.
        self.__shutting_down = False
        self.__shutdown_when_idle = False

        # Used to keep track of whether or not there were unexpected exceptions.
        self.__return_code = 0

        # Used to collect executed nodes in this launch
        self.__launch_dump: LaunchDump = LaunchDump(
            load_node=[],
            file_data={},
            node=[],
            container=[],
            lifecycle_node=[],
        )

    def emit_event(self, event: Event) -> None:
        """
        Emit an event synchronously and thread-safely.

        If the LaunchInspector is not running, the event is queued until it is.
        """
        future = None
        with self.__loop_from_run_thread_lock:
            if self.__loop_from_run_thread is not None:
                # loop is in use, asynchronously emit the event
                future = asyncio.run_coroutine_threadsafe(
                    self.__context.emit_event(event), self.__loop_from_run_thread
                )
            else:
                # loop is not in use, synchronously emit the event, and it will be processed later
                self.__context.emit_event_sync(event)

        if future is not None:
            # Block until asynchronously emitted event is emitted by loop
            future.result()

    def include_launch_description(self, launch_description: LaunchDescription) -> None:
        """
        Evaluate a given LaunchDescription and visits all of its entities.

        This method is thread-safe.
        """
        self.emit_event(IncludeLaunchDescription(launch_description))

    def _prune_and_count_entity_future_pairs(self):
        needs_prune = False
        for pair in self._entity_future_pairs:
            if pair[1].done():
                needs_prune = True
        if needs_prune:
            self._entity_future_pairs = [
                pair for pair in self._entity_future_pairs if not pair[1].done()
            ]
        return len(self._entity_future_pairs)

    def _prune_and_count_context_completion_futures(self):
        needs_prune = False
        for future in self.__context._completion_futures:
            if future.done():
                needs_prune = True
        if needs_prune:
            self.__context._completion_futures = [
                f for f in self.__context._completion_futures if not f.done()
            ]
        return len(self.__context._completion_futures)

    def _is_idle(self):
        number_of_entity_future_pairs = self._prune_and_count_entity_future_pairs()
        number_of_entity_future_pairs += self._prune_and_count_context_completion_futures()
        return number_of_entity_future_pairs == 0 and self.__context._event_queue.empty()

    @contextlib.contextmanager
    def _prepare_run_loop(self):
        try:
            # Acquire the lock and initialize the loop.
            with self.__loop_from_run_thread_lock:
                if self.__loop_from_run_thread is not None:
                    raise RuntimeError("LaunchInspector cannot be run multiple times concurrently.")
                this_loop = asyncio.get_event_loop()

                if self.__debug:
                    this_loop.set_debug(True)

                # Set the asyncio loop for the context.
                self.__context._set_asyncio_loop(this_loop)
                # Recreate the event queue to ensure the same event loop is being used.
                new_queue = asyncio.Queue()
                while True:
                    try:
                        new_queue.put_nowait(self.__context._event_queue.get_nowait())
                    except asyncio.QueueEmpty:
                        break
                self.__context._event_queue = new_queue
                self.__loop_from_run_thread = this_loop

            # Get current task.
            try:
                # Python 3.7+
                this_task = asyncio.current_task(this_loop)
            except AttributeError:
                this_task = asyncio.Task.current_task(this_loop)

            self.__this_task = this_task
            # Setup custom signal handlers for SIGINT, SIGTERM and maybe SIGQUIT.
            sigint_received = False

            def _on_sigint(signum):
                nonlocal sigint_received
                base_msg = "user interrupted with ctrl-c (SIGINT)"
                if not sigint_received:
                    self.__logger.warning(base_msg)
                    ret = self._shutdown(
                        reason="ctrl-c (SIGINT)", due_to_sigint=True, force_sync=True
                    )
                    assert ret is None, ret
                    sigint_received = True
                else:
                    self.__logger.warning(f"{base_msg} again, ignoring...")

            def _on_sigterm(signum):
                signame = signal.Signals(signum).name
                self.__logger.error(f"user interrupted with ctrl-\\ ({signame}), terminating...")
                # TODO(wjwwood): try to terminate running subprocesses before exiting.
                self.__logger.error(f"using {signame} can result in orphaned processes")
                self.__logger.error("make sure no processes launched are still running")
                this_loop.call_soon(this_task.cancel)

            with AsyncSafeSignalManager(this_loop) as manager:
                # Setup signal handlers
                manager.handle(signal.SIGINT, _on_sigint)
                manager.handle(signal.SIGTERM, _on_sigterm)
                if platform.system() != "Windows":
                    manager.handle(signal.SIGQUIT, _on_sigterm)
                # Yield asyncio loop and current task.
                yield this_loop, this_task
        finally:
            # No matter what happens, unset the loop.
            with self.__loop_from_run_thread_lock:
                self.__context._set_asyncio_loop(None)
                self.__loop_from_run_thread = None
                self.__shutting_down = False

    async def _process_one_event(self) -> None:
        next_event = await self.__context._event_queue.get()
        await self.__process_event(next_event)

    async def __process_event(self, event: Event) -> None:
        self.__logger.debug(f"processing event: '{event}'")

        for event_handler in tuple(self.__context._event_handlers):
            if event_handler.matches(event):
                self.__logger.debug(f"processing event: '{event}' ✓ '{event_handler}'")
                self.__context._push_locals()
                entities = event_handler.handle(event, self.__context)
                entities = (
                    entities if isinstance(entities, collections.abc.Iterable) else (entities,)
                )

                for entity in [e for e in entities if e is not None]:
                    if not is_a_subclass(entity, LaunchDescriptionEntity):
                        raise RuntimeError(
                            f"expected a LaunchDescriptionEntity from event_handler, got '{entity}'"
                        )

                    pairs = visit_entity(entity, self.__context, self.__launch_dump)

                    for entity, future in pairs:
                        self._entity_future_pairs.append((entity, future))

                self.__context._pop_locals()
            else:
                pass
                # Keep this commented for now, since it's very chatty.
                # self.__logger.debug(
                #     'launch.LaunchInspector',
                #     "processing event: '{}' x '{}'".format(event, event_handler))

    async def run_async(self, *, shutdown_when_idle=True) -> int:
        """
        Visit all entities of all included LaunchDescription instances asynchronously.

        This should only ever be run from the main thread and not concurrently with other
        asynchronous runs.

        :param: shutdown_when_idle if True (default), the service will shutdown when idle.
        """
        # Make sure this has not been called from any thread but the main thread.
        if threading.current_thread() is not threading.main_thread():
            raise RuntimeError("LaunchInspector can only be run in the main thread.")

        return_code = 0
        with self._prepare_run_loop() as (this_loop, this_task):
            # Log logging configuration details.
            launch.logging.log_launch_config(logger=self.__logger)

            # Setup the exception handler to make sure we return non-0 when there are errors.
            def _on_exception(loop, context):
                nonlocal return_code
                return_code = 1
                return loop.default_exception_handler(context)

            this_loop.set_exception_handler(_on_exception)

            process_one_event_task = None
            while True:
                try:
                    # Check if we're idle, i.e. no on-going entities (actions) or events in
                    # the queue
                    is_idle = self._is_idle()  # self._entity_future_pairs is pruned here
                    if not self.__shutting_down and shutdown_when_idle and is_idle:
                        ret = self._shutdown(reason="idle", due_to_sigint=False)
                        if ret is not None:
                            ret = await ret
                        assert ret is None, ret
                        continue

                    # Stop running if we're shutting down and there's no more work
                    if self.__shutting_down and is_idle:
                        if process_one_event_task is not None and not process_one_event_task.done():
                            process_one_event_task.cancel()
                        break

                    # Collect futures to wait on
                    # We only need to wait on futures if there are no events to wait on
                    entity_futures = []

                    if self.__context._event_queue.empty():
                        for _entity, future in self._entity_future_pairs:
                            # NOTE: Do NOT filter futures here. It's not working.
                            entity_futures.append(future)
                        entity_futures.extend(self.__context._completion_futures)

                    # If the current task is done, create a new task to process any events
                    # in the queue
                    if process_one_event_task is None or process_one_event_task.done():
                        process_one_event_task = this_loop.create_task(self._process_one_event())

                    # Add the process event task to the list of awaitables
                    entity_futures.append(process_one_event_task)

                    # Wait on events and futures
                    self.__logger.debug(f"await on futures: '{entity_futures}'")

                    completed_tasks, _ = await asyncio.wait(
                        entity_futures, return_when=asyncio.FIRST_COMPLETED
                    )
                    # Propagate exception from completed tasks
                    completed_tasks_exceptions = [task.exception() for task in completed_tasks]
                    completed_tasks_exceptions = list(filter(None, completed_tasks_exceptions))
                    if completed_tasks_exceptions:
                        self.__logger.debug("An exception was raised in an async action/event")
                        # in case there is more than one completed_task, log other exceptions
                        for completed_tasks_exception in completed_tasks_exceptions[1:]:
                            self.__logger.error(completed_tasks_exception)
                        raise completed_tasks_exceptions[0]

                except KeyboardInterrupt:
                    continue
                except asyncio.CancelledError:
                    self.__logger.error("run task was canceled")
                    return_code = 1
                    break
            return return_code

    def run(self, *, shutdown_when_idle=True) -> int:
        """
        Run an event loop and visit all entities of all included LaunchDescription instances.

        This should only ever be run from the main thread and not concurrently with
        asynchronous runs (see `run_async()` documentation).

        Note that KeyboardInterrupt is caught and ignored, as signals are handled separately.
        After the run ends, this behavior is undone.

        :param: shutdown_when_idle if True (default), the service will shutdown when idle
        """
        loop = osrf_pycommon.process_utils.get_loop()
        run_async_task = loop.create_task(self.run_async(shutdown_when_idle=shutdown_when_idle))
        while True:
            try:
                return loop.run_until_complete(run_async_task)
            except KeyboardInterrupt:
                continue

    # def __on_process_start(
    #     self, event: Event, context: LaunchContext
    # ) -> Optional[SomeActionsType]:
    #     action = event.action

    #     self.__logger.debug("perceived a process started event: '{}'".format(event))
    #     self.__logger.debug("which is triggeted by action: '{}'".format(action))

    #     if is_a(action, ComposableNodeContainer):
    #         kind = ProcessKind.COMPOSABLE_NODE_CONTAINER
    #     elif is_a(action, LifecycleNode):
    #         kind = ProcessKind.LIFECYCLE_NODE
    #     elif is_a(action, Node):
    #         kind = ProcessKind.NODE
    #     else:
    #         kind = ProcessKind.UNKNOWN

    #     cmdline = action.cmd
    #     info = ProcessRecord(
    #         kind=kind.value,
    #         cmdline=cmdline,
    #     )
    #     self.__launch_dump.process.append(info)

    #     return None

    def __on_process_exit(self, event: Event, context: LaunchContext) -> SomeActionsType | None:
        action = event.action
        cmdline = action.cmd
        parse_cmdline = parse_ros_cmdline(cmdline)

        file_data = self.__launch_dump.file_data

        for path in parse_cmdline.params_files:
            try:
                with open(path) as fp:
                    file_data[path] = fp.read()
            except (OSError, FileNotFoundError) as e:
                self.__logger.warning(f"Unable to read params file {path}: {e}")

        if parse_cmdline.log_config_file is not None:
            try:
                with open(parse_cmdline.log_config_file) as fp:
                    file_data[parse_cmdline.log_config_file] = fp.read()
            except (OSError, FileNotFoundError) as e:
                self.__logger.warning(
                    f"Unable to read log config file {parse_cmdline.log_config_file}: {e}"
                )

    def __on_shutdown(self, event: Event, context: LaunchContext) -> SomeActionsType | None:
        self.__shutting_down = True
        self.__context._set_is_shutdown(True)
        return None

    def _shutdown(self, *, reason, due_to_sigint, force_sync=False) -> Coroutine | None:
        # Assumption is that this method is only called when running.
        retval = None
        if not self.__shutting_down:
            shutdown_event = Shutdown(reason=reason, due_to_sigint=due_to_sigint)
            asyncio_event_loop = None
            try:
                asyncio_event_loop = asyncio.get_event_loop()
            except (RuntimeError, AssertionError):
                # If no event loop is set for this thread, asyncio will raise an exception.
                # The exception type depends on the version of Python, so just catch both.
                pass
            if force_sync:
                self.__context.emit_event_sync(shutdown_event)
            elif self.__loop_from_run_thread == asyncio_event_loop:
                # If in the thread of the loop.
                retval = self.__context.emit_event(shutdown_event)
            else:
                # Otherwise in a different thread, so use the thread-safe method.
                self.emit_event(shutdown_event)
        self.__shutting_down = True
        self.__context._set_is_shutdown(True)
        return retval

    def shutdown(self, force_sync=False) -> Coroutine | None:
        """
        Shutdown all on-going activities and then stop the asyncio run loop.

        This will cause the running LaunchInspector to eventually exit.

        Does nothing if the LaunchInspector is not running.

        This will return an awaitable coroutine if called from within the loop.

        This method is thread-safe.
        """
        with self.__loop_from_run_thread_lock:
            if self.__loop_from_run_thread is not None:
                return self._shutdown(
                    reason="LaunchInspector.shutdown() called",
                    due_to_sigint=False,
                    force_sync=force_sync,
                )

    def dump(self):
        # composable_node_containers = list(
        #     action.cmd for action in self.__composable_node_containers
        # )
        # lifecycle_nodes = list(action.cmd for action in self.__lifecycle_nodes)
        # nodes = list(action.cmd for action in self.__nodes)
        # actions = list(action.cmd for action in self.__actions)
        # output = {
        #     "composable_node_containers": composable_node_containers,
        #     "lifecycle_nodes": lifecycle_nodes,
        #     "nodes": nodes,
        #     "actions": actions,
        # }
        dump = dataclasses.asdict(self.__launch_dump)
        return dump

    @property
    def context(self):
        """Getter for context."""
        return self.__context

    @property
    def event_loop(self):
        """Getter for the event loop being used in the thread running the launch service."""
        return self.__loop_from_run_thread

    @property
    def task(self):
        """Return asyncio task associated with this launch service."""
        return self.__this_task
