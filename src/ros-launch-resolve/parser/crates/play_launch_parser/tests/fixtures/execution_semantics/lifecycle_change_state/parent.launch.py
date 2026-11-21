# The shape of slam_toolbox's online_sync_launch.py (Jazzy): a LifecycleNode
# the launch file brings up itself, with a configure event right after it and
# an OnStateTransition handler that activates it once it is inactive, both
# gated on `autostart` (#0108). A second node gets only the configure.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    autostart = LaunchConfiguration('autostart')
    slam = LifecycleNode(package='demo_nodes_cpp', executable='talker',
                         name='slam_toolbox', namespace='')
    half = LifecycleNode(package='demo_nodes_cpp', executable='talker',
                         name='configured_only', namespace='')

    configure = EmitEvent(
        event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                          transition_id=Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(autostart))
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='activating'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam),
                    transition_id=Transition.TRANSITION_ACTIVATE)),
            ]),
        condition=IfCondition(autostart))
    configure_half = EmitEvent(
        event=ChangeState(lifecycle_node_matcher=matches_action(half),
                          transition_id=Transition.TRANSITION_CONFIGURE))

    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        slam,
        configure,
        activate,
        half,
        configure_half,
    ])
