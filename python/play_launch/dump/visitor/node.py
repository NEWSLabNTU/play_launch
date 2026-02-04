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
from ..utils import param_to_kv
from .execute_process import visit_execute_process

# Global flag to track if on_exit warning has been shown
_on_exit_warning_shown = False


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
    package = substitute(node.node_package)

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
                                        params.append((param_name, str(param_value)))
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

    # Detect on_exit handlers and warn (once)
    global _on_exit_warning_shown
    if not _on_exit_warning_shown and hasattr(node, "_ExecuteLocal__on_exit"):
        on_exit_handlers = node._ExecuteLocal__on_exit
        if on_exit_handlers:
            execute_process_logger = launch.logging.get_logger("dump_launch")
            execute_process_logger.warning(
                "One or more nodes have on_exit handlers which are NOT supported by play_launch. "
                "Only respawn functionality is supported. on_exit handlers will be ignored during replay."
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
            global_params.append((name, str(value)))
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
        global_params=global_params if global_params else None,
    )
    dump.node.append(record)

    return ret
