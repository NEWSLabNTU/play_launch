from typing import List

from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from ..launch_dump import LaunchDump


def visit_include_launch_description(
    include: IncludeLaunchDescription, context: LaunchContext, dump: LaunchDump
) -> List[LaunchDescriptionEntity]:
    """Execute the action."""
    launch_description = include.launch_description_source.get_launch_description(
        context
    )
    # If the location does not exist, then it's likely set to '<script>' or something.
    context.extend_locals(
        {
            "current_launch_file_path": include._get_launch_file(),
        }
    )
    context.extend_locals(
        {
            "current_launch_file_directory": include._get_launch_file_directory(),
        }
    )

    # Do best effort checking to see if non-optional, non-default declared arguments
    # are being satisfied.
    my_argument_names = [
        perform_substitutions(context, normalize_to_list_of_substitutions(arg_name))
        for arg_name, arg_value in include.launch_arguments
    ]
    declared_launch_arguments = launch_description.get_launch_arguments_with_include_launch_description_actions()
    for argument, ild_actions in declared_launch_arguments:
        if argument._conditionally_included or argument.default_value is not None:
            continue
        argument_names = my_argument_names
        if ild_actions is not None:
            for ild_action in ild_actions:
                argument_names.extend(
                    ild_action._try_get_arguments_names_without_context()
                )
        if argument.name not in argument_names:
            raise RuntimeError(
                "Included launch description missing required argument '{}' "
                "(description: '{}'), given: [{}]".format(
                    argument.name, argument.description, ", ".join(argument_names)
                )
            )

    # Create actions to set the launch arguments into the launch configurations.
    set_launch_configuration_actions = []
    for name, value in include.launch_arguments:
        set_launch_configuration_actions.append(SetLaunchConfiguration(name, value))

    # Set launch arguments as launch configurations and then include the launch description.
    return [*set_launch_configuration_actions, launch_description]
