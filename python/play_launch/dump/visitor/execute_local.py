from launch.actions.execute_local import ExecuteLocal
from launch.actions.opaque_function import OpaqueFunction
from launch.event_handler import EventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO, OnShutdown
from launch.events.process import ShutdownProcess, SignalProcess
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.utilities import is_a_subclass

from ..launch_dump import LaunchDump


def visit_execute_local(
    process: ExecuteLocal, context: LaunchContext, dump: LaunchDump
) -> list[LaunchDescriptionEntity] | None:
    """
    Execute the action.

    This does the following:
    - register an event handler for the shutdown process event
    - register an event handler for the signal process event
    - register an event handler for the stdin event
    - configures logging for the IO process event
    - create a task for the coroutine that monitors the process
    """
    process.prepare(context)
    name = process._ExecuteLocal__process_description.final_name

    if process._ExecuteLocal__executed:
        raise RuntimeError(
            f"ExecuteLocal action '{name}': executed more than once: {process.describe()}"
        )
    process._ExecuteLocal__executed = True

    if context.is_shutdown:
        # If shutdown starts before execution can start, don't start execution.
        return None

    if process._ExecuteLocal__cached_output:
        on_output_method = process._ExecuteLocal__on_process_output_cached
        flush_buffers_method = process._ExecuteLocal__flush_cached_buffers
    else:
        on_output_method = process._ExecuteLocal__on_process_output
        flush_buffers_method = process._ExecuteLocal__flush_buffers

    event_handlers = [
        EventHandler(
            matcher=lambda event: is_a_subclass(event, ShutdownProcess),
            entities=OpaqueFunction(function=process._ExecuteLocal__on_shutdown_process_event),
        ),
        EventHandler(
            matcher=lambda event: is_a_subclass(event, SignalProcess),
            entities=OpaqueFunction(function=process._ExecuteLocal__on_signal_process_event),
        ),
        OnProcessIO(
            target_action=process,
            on_stdin=process._ExecuteLocal__on_process_stdin,
            on_stdout=lambda event: on_output_method(
                event,
                process._ExecuteLocal__stdout_buffer,
                process._ExecuteLocal__stdout_logger,
            ),
            on_stderr=lambda event: on_output_method(
                event,
                process._ExecuteLocal__stderr_buffer,
                process._ExecuteLocal__stderr_logger,
            ),
        ),
        OnShutdown(
            on_shutdown=process._ExecuteLocal__on_shutdown,
        ),
        OnProcessExit(
            target_action=process,
            on_exit=process._ExecuteLocal__on_exit,
        ),
        OnProcessExit(
            target_action=process,
            on_exit=flush_buffers_method,
        ),
    ]
    for event_handler in event_handlers:
        context.register_event_handler(event_handler)

    # try:
    #     process._ExecuteLocal__completed_future = create_future(context.asyncio_loop)
    #     process._ExecuteLocal__shutdown_future = create_future(context.asyncio_loop)
    #     process._ExecuteLocal__logger = launch.logging.get_logger(name)
    #     if not isinstance(process._ExecuteLocal__output, dict):
    #         process._ExecuteLocal__output = perform_substitutions(
    #             context, process._ExecuteLocal__output
    #         )
    #     (
    #         process._ExecuteLocal__stdout_logger,
    #         process._ExecuteLocal__stderr_logger,
    #     ) = launch.logging.get_output_loggers(name, process._ExecuteLocal__output)
    #     context.asyncio_loop.create_task(
    #         process._ExecuteLocal__execute_process(context)
    #     )
    # except Exception:
    #     for event_handler in event_handlers:
    #         context.unregister_event_handler(event_handler)
    #     raise
    return None
