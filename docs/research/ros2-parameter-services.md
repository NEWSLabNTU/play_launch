# ROS 2 Parameter Services — Research Notes

**Date**: 2026-02-26
**Purpose**: Evaluate integrating runtime parameter management into play_launch

---

## Summary

ROS 2 parameters are entirely runtime-hosted. There is no standard mechanism for
static parameter introspection — all built-in tools (`ros2 param list/describe/dump`)
make DDS service calls to live nodes. However, since play_launch already maintains a
running `rclrs::Node` and knows every node name from `record.json`, it can call
parameter services directly without the slow DDS discovery step that makes rqt slow.

---

## 1. Static Parameter Extraction

### What's already available (launch-time parameters)

play_launch's Rust parser already captures these in `record.json`:

- **`<param>` tags** in launch XML/Python — stored in each node's `params` array
- **`--params-file` YAML files** — resolved by `load_and_resolve_param_file()`,
  paths stored in `params_files`

These represent the parameters the launch file *passes* to nodes, not the full set
of parameters a node *declares* internally.

### What cannot be extracted statically

| Source                                     | Why not                                                   |
|--------------------------------------------|-----------------------------------------------------------|
| `declare_parameter()` in C++/Python source | Names can be dynamic (loops, concatenation, conditionals) |
| Compiled binaries (ELF/DWARF)              | Parameters are runtime data, not embedded metadata        |
| Ament resource index                       | No `parameters` resource type exists                      |
| `package.xml` (REP-149)                    | No fields for parameter declarations                      |

### Partial sources (source-tree only, never installed)

**`generate_parameter_library`** (PickNikRobotics): Converts declarative YAML specs
into C++/Python parameter code. The YAML provides complete metadata (names, types,
defaults, ranges, descriptions). However, the CMake macro only installs the generated
header — the YAML spec is not installed to `share/`.

Format example:
```yaml
my_namespace:
  speed_limit:
    type: double
    default_value: 30.0
    description: "Maximum speed in m/s"
    read_only: false
    validation:
      bounds<>: [0.0, 100.0]
```

Supported types: `string`, `double`, `int`, `bool`, `string_array`, `double_array`,
`int_array`, `bool_array`, `string_fixed_XX`, `none`.

Adoption: Autoware is gradually integrating it (`autoware_shift_decider`,
`autoware_costmap_generator`, others), but most packages still use their own
JSON schema approach.

**Autoware JSON Schema files**: 178 `schema/*.schema.json` files across
`autoware_universe` using JSON Schema Draft-07. Rich metadata (types, descriptions,
defaults, ranges). Used for VS Code YAML validation and CI checks. Also not installed
— CMakeLists.txt only installs `config/` and `launch/` to share.

---

## 2. Parameter Service Protocol

### Automatic services

Every ROS 2 node creates six parameter services at startup:

| Service suffix               | Service type                                 | Purpose                              |
|------------------------------|----------------------------------------------|--------------------------------------|
| `/list_parameters`           | `rcl_interfaces/srv/ListParameters`          | Names with prefix/depth filter       |
| `/get_parameters`            | `rcl_interfaces/srv/GetParameters`           | Read values by name                  |
| `/set_parameters`            | `rcl_interfaces/srv/SetParameters`           | Set values (non-atomic)              |
| `/set_parameters_atomically` | `rcl_interfaces/srv/SetParametersAtomically` | All-or-nothing set                   |
| `/describe_parameters`       | `rcl_interfaces/srv/DescribeParameters`      | Type, description, ranges, read_only |
| `/get_parameter_types`       | `rcl_interfaces/srv/GetParameterTypes`       | Lightweight type-only query          |

QoS: default services profile (KEEP_LAST, depth 10, RELIABLE, VOLATILE).

### Parameter types

```
PARAMETER_NOT_SET       = 0    # Sentinel / unset
PARAMETER_BOOL          = 1
PARAMETER_INTEGER       = 2    # i64
PARAMETER_DOUBLE        = 3    # f64
PARAMETER_STRING        = 4
PARAMETER_BYTE_ARRAY    = 5
PARAMETER_BOOL_ARRAY    = 6
PARAMETER_INTEGER_ARRAY = 7
PARAMETER_DOUBLE_ARRAY  = 8
PARAMETER_STRING_ARRAY  = 9
```

Only homogeneous arrays. No nested or complex types.

### Key message types

**`ParameterValue`** — variant-style union:
```
uint8    type              # Discriminant (from ParameterType)
bool     bool_value
int64    integer_value
float64  double_value
string   string_value
byte[]   byte_array_value
bool[]   bool_array_value
int64[]  integer_array_value
float64[] double_array_value
string[] string_array_value
```

**`Parameter`**:
```
string         name
ParameterValue value
```

**`ParameterDescriptor`**:
```
string                    name
uint8                     type
string                    description
string                    additional_constraints
bool                      read_only              # Cannot change after init
bool                      dynamic_typing         # Type can change at runtime
FloatingPointRange[<=1]   floating_point_range   # {from, to, step}
IntegerRange[<=1]         integer_range          # {from, to, step}
```

**`SetParametersResult`**:
```
bool   successful
string reason          # Explanation on failure
```

### Service request/response details

**ListParameters**:
```
Request:  string[] prefixes, uint64 depth   # depth=0 means recursive
Response: ListParametersResult result       # {names[], prefixes[]}
```
Parameters are hierarchical using `.` separator. `prefixes=["camera"], depth=1`
returns `camera.resolution` but not `camera.sensor.exposure`.

**GetParameters**:
```
Request:  string[] names
Response: ParameterValue[] values           # Same order; type=NOT_SET if missing
```

**SetParameters** (non-atomic):
```
Request:  Parameter[] parameters
Response: SetParametersResult[] results     # One per parameter; partial success possible
```

**SetParametersAtomically**:
```
Request:  Parameter[] parameters
Response: SetParametersResult result        # Single result (not array); all-or-nothing
```

**DescribeParameters**:
```
Request:  string[] names
Response: ParameterDescriptor[] descriptors
```

**GetParameterTypes**:
```
Request:  string[] names
Response: uint8[] types                     # Lightweight alternative to Describe
```

### Parameter events topic

All nodes publish changes to `/parameter_events` (type `rcl_interfaces/msg/ParameterEvent`):

```
builtin_interfaces/Time stamp
string                  node           # Fully qualified node path
Parameter[]             new_parameters
Parameter[]             changed_parameters
Parameter[]             deleted_parameters
```

QoS (`rmw_qos_profile_parameter_events`): RELIABLE, VOLATILE, depth **1000**.
Late-joining subscribers do not receive past events (VOLATILE, not TRANSIENT_LOCAL).

---

## 3. How rqt_reconfigure Works

Source: `ros-visualization/rqt_reconfigure` (humble branch).

### Node discovery (the slow part)

```python
# For every node in the graph:
names_and_namespaces = node.get_node_names_and_namespaces()
for name, ns in names_and_namespaces:
    # Check if it has parameter services:
    for svc_name, svc_types in node.get_service_names_and_types_by_node(name, ns):
        if 'rcl_interfaces/srv/ListParameters' in svc_types:
            # This node has parameters
```

This relies on DDS discovery to enumerate nodes and their services — the main
source of latency.

### Per-node interaction sequence

When the user selects a node in the GUI:

1. **`list_parameters()`** → get all parameter names
2. **`get_parameters(names)`** → get current values
3. **`describe_parameters(names)`** → types, constraints, read_only flags
4. Build editor widgets (BooleanEditor, IntegerEditor, DoubleEditor, StringEditor)
5. Subscribe to `/parameter_events`, filter by `event.node`

On user edit: `set_parameters([Parameter{name, value}])` → check result.

On parameter event (from another source changing a param):
- `new_parameters` → describe + create widget
- `changed_parameters` → update widget value
- `deleted_parameters` → remove widget

### Service call mechanism

All calls go through async pattern with 1s default timeout:
```python
future = client.call_async(request)
event.wait(timeout=1.0)
```

---

## 4. Implications for play_launch

### Advantages we have

1. **We know all node names** from `record.json` — no discovery needed
2. **We already have `rclrs::Node`** instances per replay session
3. **We know launch-time parameters** (from parser) — can show initial values
   alongside runtime values
4. **We have a web UI** — can build a parameter editor without rqt dependency

### Possible integration approach

play_launch could offer parameter management through its existing web UI:

- **List**: On node startup, call `list_parameters` + `describe_parameters` using
  the node names we already know. No DDS discovery needed.
- **Display**: Show parameters in the web UI with type info, current value,
  description, read_only status, and range constraints.
- **Edit**: `set_parameters` service call from the web UI.
- **Monitor**: Subscribe to `/parameter_events` and push changes via SSE to the
  web UI (we already have SSE infrastructure).

### What we'd need

- **rclrs service client support**: Create service clients for the six parameter
  service types. We already use service clients for LoadNode/UnloadNode in
  container_actor — same pattern.
- **rcl_interfaces message types**: Need Rust bindings for `ListParameters`,
  `GetParameters`, `SetParameters`, `DescribeParameters`, `ParameterValue`,
  `ParameterDescriptor`. These are in the `rcl_interfaces` package which we
  already depend on indirectly.
- **Web UI parameter editor**: A panel per node showing parameters with
  appropriate input controls based on type and constraints.

### Key difference from rqt

rqt_reconfigure is slow because it discovers nodes via DDS graph introspection.
play_launch already knows every node — it spawned them. This means:

- Zero discovery latency
- Can pre-create service clients at spawn time
- Can query parameters as soon as the node is ready (we already detect this
  via ComponentEvent for composable nodes)
- Can show launch-time parameters immediately, then enrich with runtime
  `describe_parameters` data once the node is up

---

## 5. Autoware schema.json System

Autoware maintains hand-written JSON Schema (draft-07) files that describe node
parameters. These provide the richest static parameter metadata available —
types, descriptions, defaults, bounds, and nesting — without running any nodes.

### Structure

Every schema wraps parameters inside the `/**` > `ros__parameters` envelope,
matching the ROS 2 YAML parameter file format:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Parameters for Shift Decider Node",
  "type": "object",
  "definitions": {
    "shift_decider": {
      "type": "object",
      "properties": {
        "park_on_goal": {
          "type": "boolean",
          "description": "Setting true to park on goal.",
          "default": "true"
        }
      },
      "required": ["park_on_goal"]
    }
  },
  "properties": {
    "/**": {
      "type": "object",
      "properties": {
        "ros__parameters": {
          "$ref": "#/definitions/shift_decider"
        }
      },
      "required": ["ros__parameters"]
    }
  },
  "required": ["/**"]
}
```

Per-parameter fields:
- **`type`** (required): `boolean`, `integer`, `number`, `string`, `array`, `object`
- **`description`** (required): human-readable explanation
- **`default`** (optional): a known-good value
- **Bounds** (optional): `minimum`/`maximum` for numbers, `items` for arrays

Large schemas split into sub-files via `$ref`:
```json
"system": { "$ref": "sub/system.json#/definitions/system" }
```

### Naming convention

```
<package>/schema/<name>.schema.json  →  <package>/config/<name>*.param.yaml
```

The glob `{config_dir}/**/{base_name}*.param.yaml` means one schema can validate
multiple param files (e.g. `classifier.param.yaml` and `classifier_sim.param.yaml`).

### CI validation

GitHub Actions workflow `json-schema-check.yaml`:
1. Path filter — only runs when `**/schema/*.schema.json` or `**/config/*.param.yaml` change
2. Installs `check-jsonschema` (Python CLI)
3. For each schema, globs matching param.yaml files and validates:
   ```
   check-jsonschema --base-uri <schema_dir> --schemafile <schema> <param.yaml>
   ```

### VS Code integration

The Red Hat YAML extension (`redhat.vscode-yaml`) uses `.vscode/settings.json`:
```json
{
  "yaml.schemas": {
    "./pkg/schema/node.schema.json": "**/node/config/*.param.yaml"
  }
}
```
Provides autocomplete, type checking, bound validation, and description tooltips.

### Coverage

| Metric                              | Count     |
|-------------------------------------|-----------|
| Total packages in autoware_universe | 244       |
| Packages with schema files          | 106 (43%) |
| Total schema.json files             | 178       |
| Total param.yaml files              | 316       |

Schemas are optional. CI only validates param files that have a matching schema.
Some packages have many schemas (e.g. `autoware_pointcloud_preprocessor` has 17,
one per node).

### Relationship to generate_parameter_library

12 Autoware packages use `generate_parameter_library` for C++ codegen from YAML
specs. ~5 of those also have `schema.json` files. The two systems are independent
and maintained separately — there is no automatic derivation of one from the other.

| File                           | Purpose                                       |
|--------------------------------|-----------------------------------------------|
| `param/<name>_parameters.yaml` | generate_parameter_library spec (C++ codegen) |
| `schema/<name>.schema.json`    | JSON Schema (CI validation + VS Code)         |
| `config/<name>.param.yaml`     | Runtime parameter values                      |

### Relevance to play_launch

Schema files are **source-tree only** (never installed to `share/`), but if the
user has an Autoware workspace checked out, play_launch could:

1. Discover schema files via glob `**/schema/*.schema.json`
2. Parse them to extract parameter metadata (types, descriptions, defaults, bounds)
3. Show this metadata in the web UI alongside live parameter values
4. Use bounds for client-side validation before calling `set_parameters`

This would give play_launch richer parameter metadata than rqt_reconfigure has
access to — rqt only sees `ParameterDescriptor` (type + optional range), while
schemas also provide descriptions and defaults for all parameters.

---

## References

- [ROS 2 Parameter Design](https://design.ros2.org/articles/ros_parameters.html)
- [ROS 2 Parameters Concept](https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html)
- [rcl_interfaces package](https://github.com/ros2/rcl_interfaces)
- [ros2cli param verbs](https://github.com/ros2/ros2cli/tree/master/ros2param)
- [rqt_reconfigure (humble)](https://github.com/ros-visualization/rqt_reconfigure/tree/humble)
- [generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library)
- [rmw QoS profiles](https://github.com/ros2/rmw/blob/humble/rmw/include/rmw/qos_profiles.h)
- [Monitoring Parameter Changes (C++ tutorial)](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Monitoring-For-Parameter-Changes-CPP.html)
- [Autoware Parameter Guidelines](https://autowarefoundation.github.io/autoware-documentation/main/contributing/coding-guidelines/ros-nodes/parameters/)
- [Autoware CI Checks](https://autowarefoundation.github.io/autoware-documentation/main/contributing/pull-request-guidelines/ci-checks/)
- [check-jsonschema](https://check-jsonschema.readthedocs.io/en/latest/usage.html)
- [autoware_universe](https://github.com/autowarefoundation/autoware_universe)
