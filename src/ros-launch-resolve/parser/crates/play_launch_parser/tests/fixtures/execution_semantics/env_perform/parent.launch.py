# Issue #0086: every substitution performed from Python, through
# `context.perform_substitution` and `launch.utilities.perform_substitutions`,
# in the shape of Autoware 2.1.0's agnocast files:
#
#   component_state_monitor.launch.py:
#     context.perform_substitution(EnvironmentVariable("ENABLE_AGNOCAST", default_value="0"))
#   agnocast_env.launch.py:
#     DeclareLaunchArgument(..., default_value=EnvironmentVariable(...))
#
# Each node's name carries what was performed, so the node set IS the
# result. The variables named PL0086_* are never set by the test; the one
# the file reads as set, it sets itself with SetEnvironmentVariable.

import os

import launch.utilities
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (AndSubstitution, AnonName,
                                  EnvironmentVariable, EqualsSubstitution,
                                  IfElseSubstitution, LaunchConfiguration,
                                  NotEqualsSubstitution, NotSubstitution,
                                  OrSubstitution, PathJoinSubstitution,
                                  PythonExpression, SubstitutionFailure,
                                  TextSubstitution, ThisLaunchFile,
                                  ThisLaunchFileDir)
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def node(name):
    return Node(package='demo_nodes_cpp', executable='talker', name=name)


def _setup(context):
    p = context.perform_substitution
    out = []

    # The issue's reproduction, verbatim.
    enabled = p(EnvironmentVariable('PL0086_ENABLE_AGNOCAST', default_value='0'))
    out.append(node('agnocast_' + enabled))

    # Set by SetEnvironmentVariable before this ran.
    out.append(node('set_' + p(EnvironmentVariable('PL0086_SET'))))

    # Unset with no default: SubstitutionFailure, as in launch.
    try:
        p(EnvironmentVariable('PL0086_MISSING'))
        out.append(node('missing_returned'))
    except SubstitutionFailure:
        out.append(node('missing_raised'))

    # A default that is itself a substitution.
    out.append(node('subdefault_' + p(EnvironmentVariable(
        'PL0086_MISSING', default_value=LaunchConfiguration('use_agnocast')))))

    out.append(node('text_' + p(TextSubstitution(text='literal'))))
    out.append(node('lc_' + p(LaunchConfiguration('use_agnocast'))))
    out.append(node('preload_' + (p(LaunchConfiguration('preload')) or 'empty')))
    out.append(node('lcdefault_' + p(LaunchConfiguration('nope', default='dflt'))))
    try:
        p(LaunchConfiguration('nope'))
        out.append(node('lcmissing_returned'))
    except SubstitutionFailure:
        out.append(node('lcmissing_raised'))

    joined = p(PathJoinSubstitution(['a', TextSubstitution(text='b'), ['c', 'd']]))
    out.append(node('join_' + joined.replace('/', '_')))
    restarted = p(PathJoinSubstitution(['a', '/abs', 'b']))
    out.append(node('joinabs_' + restarted.replace('/', '_')))

    out.append(node('dir_' + ('ok' if p(ThisLaunchFileDir()) == HERE else 'bad')))
    out.append(node('file_' + (
        'ok' if p(ThisLaunchFile()) == os.path.join(HERE, 'parent.launch.py') else 'bad')))

    anon = p(AnonName('x'))
    out.append(node('anon_' + (
        'stable' if anon == p(AnonName('x')) and anon.startswith('x_') else 'unstable')))

    out.append(node('not_' + p(NotSubstitution('false'))))
    out.append(node('and_' + p(AndSubstitution('true', 'false'))))
    out.append(node('or_' + p(OrSubstitution('true', 'false'))))
    out.append(node('eq_' + p(EqualsSubstitution('1', '1.0'))))
    out.append(node('neq_' + p(NotEqualsSubstitution('a', 'b'))))
    out.append(node('ifelse_' + p(IfElseSubstitution('true', 'yes', 'no'))))
    out.append(node('pyexpr_' + p(PythonExpression(['str(2 + 3)']))))

    # launch.utilities.perform_substitutions over a list.
    out.append(node('subs_' + launch.utilities.perform_substitutions(
        context, [TextSubstitution(text='a'), EnvironmentVariable('PL0086_SET')])))
    return out


def generate_launch_description():
    return LaunchDescription([
        # agnocast_env.launch.py's pattern.
        DeclareLaunchArgument(
            'use_agnocast',
            default_value=EnvironmentVariable('PL0086_ENABLE_AGNOCAST', default_value='0')),
        DeclareLaunchArgument(
            'preload', default_value=EnvironmentVariable('PL0086_LD_PRELOAD', default_value='')),
        SetEnvironmentVariable('PL0086_SET', 'from_set_env'),
        OpaqueFunction(function=_setup),
        # The configuration, read by a later node and by an included file.
        Node(package='demo_nodes_cpp', executable='talker',
             name=['later_', LaunchConfiguration('use_agnocast')]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(HERE, 'child.launch.py'))),
    ])
