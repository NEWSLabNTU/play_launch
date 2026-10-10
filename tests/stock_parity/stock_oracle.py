#!/usr/bin/env python3
"""Resolve a launch file with STOCK `launch`/`launch_ros`, spawning nothing.

Runs a real LaunchService over the file, with exactly two patches:

* ``ExecuteLocal.execute`` performs ``prepare()`` (every substitution in the
  command, cwd and env, exactly as stock does) and records the result instead
  of starting the process;
* ``LoadComposableNodes.execute`` builds the LoadNode requests with stock's own
  ``get_composable_node_load_request`` and records them instead of calling the
  container's service.

Everything else -- conditions, scoping, includes, OpaqueFunction, event
handlers -- is stock code. Output is JSON on stdout.

usage: stock_oracle.py <launch_file> [name:=value ...]
"""

import json
import os
import sys


def main():
    path = sys.argv[1]
    launch_args = sys.argv[2:]

    import launch
    import launch.actions.execute_local as el
    import launch_ros.actions.load_composable_nodes as lcn
    from launch_ros.actions import Node, ComposableNodeContainer
    from launch_ros.actions.lifecycle_node import LifecycleNode
    from rclpy.parameter import Parameter

    base_env = dict(os.environ)
    procs = []
    loads = []

    def read_param_file(p):
        try:
            with open(p) as f:
                return f.read()
        except OSError:
            return None

    def fake_execute(self, context):
        self.prepare(context)
        details = self.process_details
        env = details.get('env') or {}
        env_diff = {k: v for k, v in env.items() if base_env.get(k) != v}
        env_unset = sorted(k for k in base_env if k not in env)
        cmd = list(details['cmd'])
        # Node writes dict parameters to temp files; inline their content so
        # the comparison is on values, not on a temp path.
        param_files = []
        for i, a in enumerate(cmd):
            if a == '--params-file' and i + 1 < len(cmd):
                param_files.append({'path': cmd[i + 1], 'content': read_param_file(cmd[i + 1])})
        rec = {
            'kind': 'process',
            'cmd': cmd,
            'cwd': details.get('cwd'),
            'env_diff': env_diff,
            'env_unset': env_unset,
            'param_files': param_files,
        }
        if isinstance(self, Node):
            rec['kind'] = 'node'
            if isinstance(self, ComposableNodeContainer):
                rec['kind'] = 'container'
            if isinstance(self, LifecycleNode):
                rec['kind'] = 'lifecycle_node'
            try:
                rec['fqn'] = self.node_name
            except Exception:
                rec['fqn'] = None
            from launch.utilities import perform_substitutions as ps0
            from launch.utilities import normalize_to_list_of_substitutions as nl

            def ps(ctx, v):
                return ps0(ctx, nl(v))
            rec['package'] = ps(context, self.node_package)
            rec['executable'] = ps(context, self.node_executable)
        procs.append(rec)
        return None

    def fake_load(self, context):
        tc = self._LoadComposableNodes__target_container
        from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
        if isinstance(tc, ComposableNodeContainer):
            target = tc.node_name
        else:
            target = perform_substitutions(context, normalize_to_list_of_substitutions(tc))
        for desc in self._LoadComposableNodes__composable_node_descriptions:
            req = lcn.get_composable_node_load_request(desc, context)
            loads.append({
                'target': target,
                'package': req.package_name,
                'plugin': req.plugin_name,
                'name': req.node_name,
                'namespace': req.node_namespace,
                'remaps': list(req.remap_rules),
                'params': [[p.name, Parameter.from_parameter_msg(p).value] for p in req.parameters],
                'extra_args': [[p.name, Parameter.from_parameter_msg(p).value]
                               for p in req.extra_arguments],
            })
        return None

    el.ExecuteLocal.execute = fake_execute
    lcn.LoadComposableNodes.execute = fake_load

    from launch.launch_description_sources import AnyLaunchDescriptionSource
    from launch.actions import IncludeLaunchDescription

    args = []
    for a in launch_args:
        k, _, v = a.partition(':=')
        args.append((k, v))

    ls = launch.LaunchService(argv=launch_args, noninteractive=True)
    ld = launch.LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.abspath(path)),
            launch_arguments=args,
        )
    ])
    ls.include_launch_description(ld)
    rc = ls.run(shutdown_when_idle=True)

    def default(o):
        if type(o).__name__ == 'array':
            return list(o)
        return repr(o)

    out = os.environ.get('STOCK_OUT')
    with (open(out, 'w') if out else sys.stdout) as f:
        json.dump({'rc': rc, 'processes': procs, 'loads': loads, 'base_env': base_env}, f, indent=1,
                  default=default)
        f.write('\n')


if __name__ == '__main__':
    main()
