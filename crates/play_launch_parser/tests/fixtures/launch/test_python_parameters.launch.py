"""Python launch file testing parameter parsing."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with various parameter types."""
    return LaunchDescription([
        # Node with simple dict parameters
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_simple_params',
            parameters=[{
                'string_param': 'hello_world',
                'int_param': 42,
                'float_param': 3.14159,
                'bool_param': True,
            }]
        ),

        # Node with nested dict parameters
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener_nested_params',
            parameters=[{
                'robot': {
                    'name': 'my_robot',
                    'id': 123,
                },
                'config': {
                    'debug': False,
                    'rate': 10.0,
                    'advanced': {
                        'timeout': 5,
                        'retries': 3,
                    }
                }
            }]
        ),

        # Node with list of parameter dicts
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_list_params',
            parameters=[
                {'param1': 'value1'},
                {'param2': 42},
                {'param3': True},
            ]
        ),

        # Node with mixed parameter types
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener_mixed_params',
            parameters=[
                {'inline_param': 'inline_value'},
                '/path/to/params.yaml',  # Parameter file
                {
                    'nested': {
                        'value': 'deep_value'
                    }
                }
            ]
        ),

        # Node with list parameters (array values)
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_array_params',
            parameters=[{
                'joints': ['joint1', 'joint2', 'joint3'],
                'limits': [0.0, 1.0, 2.0, 3.0],
                'flags': [True, False, True],
            }]
        ),

        # Node with empty parameters
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener_no_params',
        ),
    ])
