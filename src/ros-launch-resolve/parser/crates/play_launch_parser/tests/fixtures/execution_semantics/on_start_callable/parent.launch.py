# Issue #0101: `OnProcessStart(on_start=<callable>)`. launch calls it with
# the start event and the context and executes the actions it returns, as
# it executes a list. `first_list` is the list form, `second_fn` the callable.
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    first = Node(package='demo_nodes_cpp', executable='talker', name='first')
    by_list = Node(package='demo_nodes_cpp', executable='listener', name='by_list')
    by_fn = Node(package='demo_nodes_cpp', executable='listener', name='by_fn')

    def on_start(event, context):
        assert event.action is first
        return [by_fn]

    return LaunchDescription([
        first,
        RegisterEventHandler(OnProcessStart(target_action=first, on_start=[by_list])),
        RegisterEventHandler(OnProcessStart(target_action=first, on_start=on_start)),
    ])
