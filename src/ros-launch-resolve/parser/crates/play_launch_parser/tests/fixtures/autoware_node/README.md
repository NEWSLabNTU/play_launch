# `<autoware_node>` fixtures (issue #0095)

Autoware 2.1.0's launch action, with stock `ros2 launch` as the oracle and no
Autoware install.

- `python/autoware_agnocast_wrapper/` is copied unmodified from
  autowarefoundation/autoware_core,
  `common/autoware_agnocast_wrapper/autoware_agnocast_wrapper/`, at commit
  `34c25ee5b89578f10af87aac6083718fefcf5f21`. Copyright 2026 TIER IV, Inc.,
  Apache License 2.0 (the header of each file). Only the Python launch
  extension is here; the C++ library is not.
- `python/autoware_agnocast_wrapper-2.1.0.dist-info/` stands in for what
  `ament_python_install_package(... SETUP_CFG setup.cfg)` installs: the
  `launch.frontend.launch_extension` entry point that makes the XML and YAML
  frontends of stock `launch` import the module and so know
  `<autoware_node>`.
- `prefix/` is an ament prefix registering two `autoware_node_plugins`
  resources the way `autoware_agnocast_wrapper_register_node()` does, for
  demo_nodes_cpp's `talker` (built with Agnocast: `;1`) and `listener`
  (without: `;0`).
- Each other directory is one case of `tests/autoware_node_tests.rs`. They
  put `prefix/` on `AMENT_PREFIX_PATH` with `<set_env>`; the Python case
  puts `python/` on `sys.path` itself.

Used outside this crate too: `tests/stock_parity/cases/awnode*` (their `ENV`
files) and `tests/attr_differential.rs` (the probe's `PYTHONPATH`).

To see what stock resolves for a case:

```bash
PYTHONPATH=$PWD/python:$PYTHONPATH STOCK_OUT=out.json \
  python3 <repo>/tests/stock_parity/stock_oracle.py <case>/parent.launch.xml
```
