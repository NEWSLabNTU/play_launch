"""What a launch file's own OpaqueFunction may do with launch_ros (#0100).

Written after ceilbot's nav2_composed.launch.py: wait for a container's
list_nodes through launch_ros's node, and open the copy of a parameter file
whose substitutions ParameterFile.evaluate performed. Each function returns a
node only when it got that far, so a missing node is the failure.
"""
import os
import threading
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.ros_adapters import get_ros_node


class ListNodes:
    """Stands for composition_interfaces/srv/ListNodes; nothing is sent."""

    class Request:
        pass


def wait(context):
    node = get_ros_node(context)
    client = node.create_client(ListNodes, '/c/_container/list_nodes')
    end = time.monotonic() + 5.0
    try:
        while time.monotonic() < end and not context.is_shutdown:
            if not client.wait_for_service(timeout_sec=1.0):
                continue
            answered = threading.Event()
            future = client.call_async(ListNodes.Request())
            future.add_done_callback(lambda _: answered.set())
            if answered.wait(1.0):
                return [Node(package='demo_nodes_cpp', executable='talker', name='after_wait')]
            future.cancel()
    finally:
        node.destroy_client(client)
    return []


def read_params(context):
    params = ParameterFile(os.path.join(os.path.dirname(__file__), 'params.yaml'),
                           allow_substs=True)
    path = params.evaluate(context)
    if params.evaluate(context) != path:
        raise RuntimeError('evaluate gave a second file')
    with open(path) as f:
        robot = f.read().split(':', 1)[1].strip()
    return [Node(package='demo_nodes_cpp', executable='listener', name='after_params',
                 parameters=[{'copied': robot}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='r1'),
        OpaqueFunction(function=wait),
        OpaqueFunction(function=read_params),
    ])
