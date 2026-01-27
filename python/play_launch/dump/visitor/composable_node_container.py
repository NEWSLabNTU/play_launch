from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.utilities import is_a, normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions.composable_node_container import ComposableNodeContainer
from launch_ros.descriptions import Parameter

from ..launch_dump import ComposableNodeContainerRecord, LaunchDump
from ..utils import param_to_kv
from .execute_process import visit_execute_process


def visit_composable_node_container(
    container: ComposableNodeContainer, context: LaunchContext, dump: LaunchDump
) -> list[LaunchDescriptionEntity] | None:
    """
    Execute the action.

    Most work is delegated to :meth:`launch_ros.actions.Node.execute`, except for the
    composable nodes load action if it applies.
    """
    load_actions = None  # type: Optional[List[Action]]
    valid_composable_nodes = []

    descriptions = container._ComposableNodeContainer__composable_node_descriptions

    if descriptions:
        for node_object in descriptions:
            if node_object.condition() is None or node_object.condition().evaluate(context):
                valid_composable_nodes.append(node_object)

    if valid_composable_nodes is not None and len(valid_composable_nodes) > 0:
        from .load_composable_nodes import LoadComposableNodes

        # Perform load action once the container has started.
        load_actions = [
            LoadComposableNodes(
                composable_node_descriptions=valid_composable_nodes,
                target_container=container,
            )
        ]

    # Process container as a node (ExecuteProcess)
    container._perform_substitutions(context)

    def substitute(subst):
        nonlocal context
        return perform_substitutions(context, normalize_to_list_of_substitutions(subst))

    executable = substitute(container.node_executable)
    package = substitute(container.node_package)

    if container._Node__ros_arguments is not None:
        ros_args = [substitute(subst) for subst in container._Node__ros_arguments]
    else:
        ros_args = None

    if container._Node__arguments is not None:
        args = [substitute(subst) for subst in container._Node__arguments]
    else:
        args = None

    if container.expanded_node_namespace == container.UNSPECIFIED_NODE_NAMESPACE:
        namespace = None
    else:
        namespace = container.expanded_node_namespace

    # Prepare ros_specific_arguments
    ros_specific_arguments: dict[str, str | list[str]] = {}
    if container._Node__node_name is not None:
        ros_specific_arguments["name"] = f"__node:={container._Node__expanded_node_name}"
    if container._Node__expanded_node_namespace != "":
        ros_specific_arguments["ns"] = f"__ns:={container._Node__expanded_node_namespace}"

    # Give extensions a chance to prepare for execution
    for extension in container._Node__extensions.values():
        cmd_extension, ros_specific_arguments = extension.prepare_for_execute(
            context, ros_specific_arguments, container
        )
        container.cmd.extend(cmd_extension)
    context.extend_locals({"ros_specific_arguments": ros_specific_arguments})

    # Visit ExecuteProcess for container_actions
    container_actions = visit_execute_process(container, context, dump)

    # Extract parameters
    params_files = []
    params = []
    node_params = container._Node__expanded_parameter_arguments

    if node_params is not None:
        for entry, is_file in node_params:
            if is_file:
                path = entry
                try:
                    with open(path) as fp:
                        data = fp.read()
                        params_files.append(data)
                        dump.file_data[path] = data
                except (OSError, FileNotFoundError) as e:
                    import launch.logging

                    logger = launch.logging.get_logger(container.name)
                    logger.error(f"Unable to read parameter file {path}: {e}")
            else:
                assert is_a(entry, Parameter)
                name, value = param_to_kv(entry)
                params.append((name, value))

    if container.expanded_remapping_rules is None:
        remaps = []
    else:
        remaps = container.expanded_remapping_rules

    # Extract environment variables
    env_vars = []
    if hasattr(container, "additional_env") and container.additional_env is not None:
        if isinstance(container.additional_env, dict):
            for key, value in container.additional_env.items():
                key_str = substitute(key) if not isinstance(key, str) else key
                value_str = substitute(value) if not isinstance(value, str) else value
                env_vars.append((key_str, value_str))
        elif isinstance(container.additional_env, list):
            for item in container.additional_env:
                if isinstance(item, tuple) and len(item) == 2:
                    key, value = item
                    key_str = substitute(key) if not isinstance(key, str) else key
                    value_str = substitute(value) if not isinstance(value, str) else value
                    env_vars.append((key_str, value_str))

    # Extract respawn configuration
    respawn = None
    respawn_delay = None
    if hasattr(container, "_ExecuteLocal__respawn"):
        try:
            respawn_value = container._ExecuteLocal__respawn
            if isinstance(respawn_value, bool):
                respawn = respawn_value
            else:
                respawn_str = substitute(respawn_value)
                if respawn_str.lower() in ("true", "1", "yes"):
                    respawn = True
                elif respawn_str.lower() in ("false", "0", "no", ""):
                    respawn = False
        except Exception as e:
            import launch.logging

            logger = launch.logging.get_logger(container.name)
            logger.warning(f"Unable to extract respawn parameter: {e}")
    if hasattr(container, "_ExecuteLocal__respawn_delay"):
        respawn_delay = container._ExecuteLocal__respawn_delay

    # Extract global parameters
    global_params_raw = context.launch_configurations.get("global_params", [])
    global_params = []
    for param in global_params_raw:
        if isinstance(param, tuple):
            name, value = param
            global_params.append((name, str(value)))

    # Save container record with all node information
    node_name = container._Node__expanded_node_name
    record = ComposableNodeContainerRecord(
        executable=executable,
        package=package,
        name=node_name,
        namespace=namespace,
        exec_name=container.name,
        cmd=container.cmd,
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
    dump.container.append(record)

    if container_actions is not None and load_actions is not None:
        return container_actions + load_actions
    if container_actions is not None:
        return container_actions
    if load_actions is not None:
        return load_actions
    return None
