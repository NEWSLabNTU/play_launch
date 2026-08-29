import launch
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.utilities import (
    is_a,
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
from launch_ros.actions.composable_node_container import ComposableNodeContainer
from launch_ros.actions.node import Node
from launch_ros.descriptions import Parameter
from launch_ros.utilities import add_node_name, get_node_name_count

from ..launch_dump import LaunchDump, NodeRecord
from ..utils import param_to_kv, param_value_to_str
from .execute_process import visit_execute_process

# Global flag to track if the "unsupported on_exit" warning has been shown
_on_exit_warning_shown = False


def _classify_on_exit(handlers):
    """Split a node's on_exit actions into "shut the launch down" and everything else.

    `on_exit=Shutdown()` is the launch idiom for "this process is required; when it
    finishes, take the whole launch with it". It is how a scenario runner ends a run:
    the orchestrator exits 0 and every other node is torn down. Replaying without it
    leaves those nodes running forever -- an SSv2 interpreter was found still spinning
    40 hours after its scenario passed, holding a ROS domain and a stack of 90 nodes.

    Both `launch.actions.Shutdown` and SSv2's `ShutdownOnce` are `EmitEvent` actions
    carrying a `launch.events.Shutdown`, so match on the event rather than the class.

    Returns (shutdown, others): whether a shutdown handler is present, and the actions
    that are something else and therefore still unsupported.
    """
    from launch.actions import EmitEvent
    from launch.events import Shutdown as ShutdownEvent

    # `on_exit` accepts a single action, a list of them, or a callable. launch does not
    # always normalize it before it reaches here, and assuming a list made the dump die
    # with "'Shutdown' object is not iterable" on exactly the launch files this is for.
    if handlers is None:
        handlers = []
    elif not isinstance(handlers, (list, tuple)):
        handlers = [handlers]

    shutdown = False
    others = []
    for handler in handlers:
        event = getattr(handler, "_EmitEvent__event", None)
        if isinstance(handler, EmitEvent) and isinstance(event, ShutdownEvent):
            shutdown = True
        else:
            others.append(handler)
    return shutdown, others


def visit_node(
    node: Node, context: LaunchContext, dump: LaunchDump
) -> list[LaunchDescriptionEntity] | None:
    # Skip ComposableNodeContainers - they are handled by visit_composable_node_container
    # and should only appear in container[] array, not node[]
    if isinstance(node, ComposableNodeContainer):
        # Container is already processed by visit_composable_node_container
        # Don't add it to dump.node
        return None

    node._perform_substitutions(context)

    def substitute(subst):
        nonlocal context
        return perform_substitutions(context, normalize_to_list_of_substitutions(subst))

    executable = substitute(node.node_executable)
    # `package` is optional: launch_ros accepts an absolute `executable` with no package,
    # which is how a plain program is run under the launch system. Substituting None
    # threw `TypeError: 'NoneType' object is not iterable` from deep inside launch,
    # naming neither the node nor the field (issue 0026). The record models this
    # (`package: str | None`) and so does the Rust side, which routes a package-less
    # record to `from_raw_executable`.
    package = substitute(node.node_package) if node.node_package is not None else None

    if node._Node__ros_arguments is not None:
        ros_args = [substitute(subst) for subst in node._Node__ros_arguments]
    else:
        ros_args = None

    if node._Node__arguments is not None:
        args = [substitute(subst) for subst in node._Node__arguments]
    else:
        args = None

    if node.expanded_node_namespace == node.UNSPECIFIED_NODE_NAMESPACE:
        namespace = None
    else:
        namespace = node.expanded_node_namespace

    # Prepare the ros_specific_arguments list and add it to the context so that the
    # LocalSubstitution placeholders added to the the cmd can be expanded using the contents.
    ros_specific_arguments: dict[str, str | list[str]] = {}
    if node._Node__node_name is not None:
        ros_specific_arguments["name"] = f"__node:={node._Node__expanded_node_name}"
    if node._Node__expanded_node_namespace != "":
        ros_specific_arguments["ns"] = f"__ns:={node._Node__expanded_node_namespace}"

    # Give extensions a chance to prepare for execution
    for extension in node._Node__extensions.values():
        cmd_extension, ros_specific_arguments = extension.prepare_for_execute(
            context, ros_specific_arguments, node
        )
        node.cmd.extend(cmd_extension)
    context.extend_locals({"ros_specific_arguments": ros_specific_arguments})

    # Visit ExecuteProcess
    ret = visit_execute_process(node, context, dump)

    if node.is_node_name_fully_specified():
        add_node_name(context, node.node_name)
        node_name_count = get_node_name_count(context, node.node_name)
        if node_name_count > 1:
            execute_process_logger = launch.logging.get_logger(node.name)
            execute_process_logger.warning(
                f"there are now at least {node_name_count} nodes with the name {node.node_name} created within this "
                "launch context"
            )

    # Extract parameters
    params_files = []
    params = []
    temp_param_files = []  # Track temp files to remove from cmd
    node_params = node._Node__expanded_parameter_arguments

    if node_params is not None:
        for entry, is_file in node_params:
            if is_file:
                path = entry
                # Check if this is a temporary parameter file (created by launch system)
                # If so, extract the parameters as inline params instead
                if "/tmp/launch_params_" in path:
                    temp_param_files.append(path)
                    try:
                        import yaml

                        with open(path) as fp:
                            data = yaml.safe_load(fp)
                            # Extract params from YAML structure: {namespace/node_name: {ros__parameters: {key: value}}}
                            for _node_path, node_data in data.items():
                                if isinstance(node_data, dict) and "ros__parameters" in node_data:
                                    for param_name, param_value in node_data[
                                        "ros__parameters"
                                    ].items():
                                        # YAML, not str(). yaml.safe_load above returns real
                                        # Python objects, and str() renders them as Python
                                        # repr: str(['camera6']) is "['camera6']", whose
                                        # quotes survive into the spawned node as part of the
                                        # value. traffic_light_multi_camera_fusion then built
                                        # the topic "'camera6'/detection/rois" and aborted on
                                        # InvalidTopicNameError. str(True) is likewise "True"
                                        # where ROS wants "true".
                                        params.append((param_name, param_value_to_str(param_value)))
                    except Exception as e:
                        execute_process_logger = launch.logging.get_logger(node.name)
                        execute_process_logger.warning(
                            f"Unable to parse temp parameter file {path}: {e}"
                        )
                else:
                    # Real parameter file - keep as file
                    try:
                        with open(path) as fp:
                            data = fp.read()
                            params_files.append(data)
                            dump.file_data[path] = data
                    except Exception as e:
                        execute_process_logger = launch.logging.get_logger(node.name)
                        execute_process_logger.error(f"Unable to read parameter file {path}: {e}")
            else:
                assert is_a(entry, Parameter)
                name, value = param_to_kv(entry)
                params.append((name, value))

    # Build cmd - convert cmd list elements to strings and remove temp param files
    cmd_strings = []
    if node.cmd is not None:
        skip_next = False
        for i, cmd_elem in enumerate(node.cmd):
            if skip_next:
                skip_next = False
                continue

            # Convert to string
            if isinstance(cmd_elem, list):
                resolved = perform_substitutions(context, cmd_elem)
            elif isinstance(cmd_elem, str):
                resolved = cmd_elem
            else:
                resolved = perform_substitutions(
                    context, normalize_to_list_of_substitutions(cmd_elem)
                )

            # Check if this is --params-file followed by a temp file
            if resolved == "--params-file" and i + 1 < len(node.cmd):
                next_elem = node.cmd[i + 1]
                next_str = str(next_elem) if not isinstance(next_elem, str) else next_elem
                if any(temp_file in next_str for temp_file in temp_param_files):
                    # Skip both --params-file and the file path, add inline params instead
                    skip_next = True
                    for param_name, param_value in params:
                        cmd_strings.extend(["-p", f"{param_name}:={param_value}"])
                    continue

            cmd_strings.append(resolved)

    # launch_ros appends `--ros-args` to every Node, even one with no ROS arguments to
    # put after it. An empty section is a no-op for rcl, so a real ROS node never noticed;
    # a node given an absolute executable and no package is usually NOT a ROS program, and
    # `/bin/sleep 3600 --ros-args` exits 1 on the unrecognized option (issue 0026). Drop
    # the marker when nothing follows it. Only when it is last: a section with arguments
    # in it is load-bearing and stays.
    if cmd_strings and cmd_strings[-1] == "--ros-args":
        cmd_strings.pop()

    if node.expanded_remapping_rules is None:
        remaps = []
    else:
        remaps = node.expanded_remapping_rules

    # Extract environment variables
    env_vars = []
    if hasattr(node, "additional_env") and node.additional_env is not None:
        # additional_env can be either a dict or a list of tuples
        if isinstance(node.additional_env, dict):
            for key, value in node.additional_env.items():
                # Perform substitutions on key and value
                key_str = substitute(key) if not isinstance(key, str) else key
                value_str = substitute(value) if not isinstance(value, str) else value
                env_vars.append((key_str, value_str))
        elif isinstance(node.additional_env, list):
            for item in node.additional_env:
                if isinstance(item, tuple) and len(item) == 2:
                    key, value = item
                    # Perform substitutions on key and value
                    key_str = substitute(key) if not isinstance(key, str) else key
                    value_str = substitute(value) if not isinstance(value, str) else value
                    env_vars.append((key_str, value_str))

    # Extract respawn configuration
    # Note: Only set respawn if explicitly True - treat False/unset as None for consistency with Rust parser
    respawn = None
    respawn_delay = None
    if hasattr(node, "_ExecuteLocal__respawn"):
        try:
            # Respawn can be a bool or a substitution
            respawn_value = node._ExecuteLocal__respawn
            if isinstance(respawn_value, bool):
                # Only keep True values; False is equivalent to None (no respawn)
                if respawn_value:
                    respawn = True
            else:
                # Try to resolve substitution
                respawn_str = substitute(respawn_value)
                if respawn_str.lower() in ("true", "1", "yes"):
                    respawn = True
                # Treat false/empty as None (no respawn)
        except Exception as e:
            execute_process_logger = launch.logging.get_logger(node.name)
            execute_process_logger.warning(f"Unable to extract respawn parameter: {e}")
    if hasattr(node, "_ExecuteLocal__respawn_delay"):
        respawn_delay = node._ExecuteLocal__respawn_delay

    # on_exit: shutdown handlers are replayed (see _classify_on_exit); anything else
    # still is not, and is still warned about once.
    global _on_exit_warning_shown
    on_exit_shutdown = None
    if hasattr(node, "_ExecuteLocal__on_exit"):
        shutdown_handler, unsupported = _classify_on_exit(node._ExecuteLocal__on_exit)
        if shutdown_handler:
            on_exit_shutdown = True
        if unsupported and not _on_exit_warning_shown:
            execute_process_logger = launch.logging.get_logger("dump_launch")
            execute_process_logger.warning(
                "One or more nodes have on_exit handlers other than Shutdown, which are NOT "
                "supported by play_launch. Only respawn and on_exit=Shutdown are supported; "
                "the rest will be ignored during replay."
            )
            _on_exit_warning_shown = True

    # Extract global parameters from context (set via SetParameter action)
    # These are scope-aware - context already resolves which params apply to this node
    # See: launch_ros/actions/set_parameter.py and launch_ros/actions/node.py
    global_params_raw = context.launch_configurations.get("global_params", [])
    global_params = []
    for param in global_params_raw:
        if isinstance(param, tuple):
            # Direct parameter from SetParameter action
            name, value = param
            # YAML, not str(): a SetParameter whose value is a list round-trips through
            # str() as a Python repr -- str(['camera6']) is "['camera6']" -- and the
            # single quotes survive into the node as part of the value. The traffic light
            # multi-camera fusion node then built the topic name "'camera6'/detection/rois"
            # and aborted on InvalidTopicNameError at startup. dump_yaml is what every
            # other parameter path in this dumper already uses.
            global_params.append((name, param_value_to_str(value)))
        # Note: file paths (strings from SetParametersFromFile) are already handled
        # via __expanded_parameter_arguments by Node._perform_substitutions

    # Store a node record
    node_name = node._Node__expanded_node_name
    if "<node_name_unspecified>" in node_name:
        node_name = None

    # Get exec_name - strip counter suffix (e.g., "talker-1" -> "talker")
    # to match Rust parser behavior which doesn't include counters
    exec_name = node.name
    if exec_name and "-" in exec_name:
        # Check if last part after '-' is a number (counter)
        parts = exec_name.rsplit("-", 1)
        if len(parts) == 2 and parts[1].isdigit():
            exec_name = parts[0]

    record = NodeRecord(
        executable=executable,
        package=package,
        name=node_name,
        namespace=namespace,
        exec_name=exec_name,
        cmd=cmd_strings,
        remaps=remaps,
        params=params,
        params_files=params_files,
        ros_args=ros_args,
        args=args,
        env=env_vars if env_vars else None,
        respawn=respawn,
        respawn_delay=respawn_delay,
        on_exit_shutdown=on_exit_shutdown,
        global_params=global_params if global_params else None,
        scope=dump.current_scope_id,
    )
    dump.node.append(record)

    return ret
