from typing import Dict, List, Optional, Union

import launch
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.utilities import (
    is_a,
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
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
) -> Optional[List[LaunchDescriptionEntity]]:
    node._perform_substitutions(context)

    def substitute(subst):
        nonlocal context
        return perform_substitutions(context, normalize_to_list_of_substitutions(subst))

    executable = substitute(node.node_executable)
    package = substitute(node.node_package)

    if node._Node__ros_arguments is not None:
        ros_args = list(substitute(subst) for subst in node._Node__ros_arguments)
    else:
        ros_args = None

    if node._Node__arguments is not None:
        args = list(substitute(subst) for subst in node._Node__arguments)
    else:
        args = None

    if node.expanded_node_namespace == node.UNSPECIFIED_NODE_NAMESPACE:
        namespace = None
    else:
        namespace = node.expanded_node_namespace

    # Prepare the ros_specific_arguments list and add it to the context so that the
    # LocalSubstitution placeholders added to the the cmd can be expanded using the contents.
    ros_specific_arguments: Dict[str, Union[str, List[str]]] = {}
    if node._Node__node_name is not None:
        ros_specific_arguments["name"] = "__node:={}".format(
            node._Node__expanded_node_name
        )
    if node._Node__expanded_node_namespace != "":
        ros_specific_arguments["ns"] = "__ns:={}".format(
            node._Node__expanded_node_namespace
        )

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
                "there are now at least {} nodes with the name {} created within this "
                "launch context".format(node_name_count, node.node_name)
            )

    # Extract parameters
    params_files = list()
    params = list()
    node_params = node._Node__expanded_parameter_arguments

    if node_params is not None:
        for entry, is_file in node_params:
            if is_file:
                path = entry
                try:
                    with open(path, "r") as fp:
                        data = fp.read()
                        params_files.append(data)
                        dump.file_data[path] = data
                except (FileNotFoundError, IOError) as e:
                    execute_process_logger = launch.logging.get_logger(node.name)
                    execute_process_logger.error(
                        f"Unable to read parameter file {path}: {e}"
                    )
                    # Continue without this params file
            else:
                assert is_a(entry, Parameter)
                name, value = param_to_kv(entry)
                params.append((name, value))

    if node.expanded_remapping_rules is None:
        remaps = list()
    else:
        remaps = node.expanded_remapping_rules

    # Extract environment variables
    env_vars = list()
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
                    value_str = (
                        substitute(value) if not isinstance(value, str) else value
                    )
                    env_vars.append((key_str, value_str))

    # Extract respawn configuration
    respawn = None
    respawn_delay = None
    if hasattr(node, "_ExecuteLocal__respawn"):
        try:
            # Respawn can be a bool or a substitution
            respawn_value = node._ExecuteLocal__respawn
            if isinstance(respawn_value, bool):
                respawn = respawn_value
            else:
                # Try to resolve substitution
                respawn_str = substitute(respawn_value)
                if respawn_str.lower() in ("true", "1", "yes"):
                    respawn = True
                elif respawn_str.lower() in ("false", "0", "no", ""):
                    respawn = False
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

    # Store a node record
    node_name = node._Node__expanded_node_name
    if "<node_name_unspecified>" in node_name:
        node_name = None

    record = NodeRecord(
        executable=executable,
        package=package,
        name=node_name,
        namespace=namespace,
        exec_name=node.name,
        cmd=node.cmd,
        remaps=remaps,
        params=params,
        params_files=params_files,
        ros_args=ros_args,
        args=args,
        env=env_vars if env_vars else None,
        respawn=respawn,
        respawn_delay=respawn_delay,
    )
    dump.node.append(record)

    return ret
