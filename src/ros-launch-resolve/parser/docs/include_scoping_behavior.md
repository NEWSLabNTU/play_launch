# ROS 2 Launch File Include Scoping Behavior

## Research Findings

This document analyzes the scoping behavior of ROS 2 launch file includes based on official documentation and observed behavior.

### Official Documentation

#### From ROS 2 Design Documents

**[ROS 2 Launch XML Format](https://design.ros2.org/articles/roslaunch_xml.html)**:
> "The included launch file description has its own scope for launch configurations."

> "Arguments are limited to the scope of their definition and thus have to be explicitly passed to included files if any."

**[ROS 2 Launch System](https://design.ros2.org/articles/roslaunch.html)**:
> "Changes to the local state by included launch descriptions persist, as they should be thought of as truly included in the same file, as if you had copied the contents of the included launch description in place of the include action."

**Apparent Contradiction**:
The XML format documentation suggests isolated scoping, while the main launch system documentation indicates configuration changes persist. The truth appears to be:
- **Arguments (`<arg>`)** are scoped - must be explicitly passed
- **Launch configurations (set via actions)** persist after include

### Observed Behavior in Autoware

#### YAML Preset Files

Autoware uses YAML files to declare launch arguments that affect subsequent XML includes:

```xml
<!-- tier4_planning_component.launch.xml -->
<include file="$(find-pkg-share autoware_launch)/config/planning/preset/default_preset.yaml"/>

<include file="$(find-pkg-share tier4_planning_launch)/launch/planning.launch.xml">
  <arg name="velocity_smoother_type_param_path"
       value="...$(var velocity_smoother_type)..."/>
</include>
```

The YAML file declares `velocity_smoother_type`, and the second include references it in an arg value.

**Key Finding**: YAML includes modify the parent context's launch configurations, making variables available to subsequent includes.

### Implementation Analysis

#### XML Include Behavior

Our parser creates isolated child contexts for XML includes:

```rust
let mut include_context = self.context.child();  // Line 505
// Process included file with child context
// Context changes are discarded after processing
```

This follows the XML format spec: each XML include has its own scope.

#### YAML Include Behavior

YAML includes modify the parent context directly:

```rust
self.process_yaml_launch_file(&resolved_path)?;  // Line 496
// Modifies self.context in place
// Changes persist in parent scope
```

This allows YAML preset files to declare arguments that affect subsequent includes.

### Parallel vs Sequential Include Processing

#### XML Includes (Safe for Parallel Processing)

Each XML include:
1. Clones parent context
2. Creates child context from clone
3. Processes in isolation
4. Discards context changes
5. Merges only records/containers/load_nodes

**Result**: No inter-dependency between sibling includes → parallel processing is safe.

#### YAML Includes (Require Sequential Processing)

YAML includes:
1. Modify parent context directly
2. Changes must be visible to subsequent includes
3. Cannot be processed in parallel with dependent includes

**Example Dependency Chain**:
```
YAML declares velocity_smoother_type
  ↓
XML include references $(var velocity_smoother_type) in arg value
```

If processed in parallel:
- YAML modifies clone context (discarded)
- XML sees original context (missing velocity_smoother_type)
- **Result**: Undefined variable error ✗

If processed sequentially:
- YAML modifies self.context
- XML sees updated self.context
- **Result**: Variable resolved correctly ✓

### Implementation Decision

**Solution**: Detect YAML includes during batch collection and process them sequentially:

```rust
// Check if this is a YAML include
let is_yaml = file_path_str.ends_with(".yaml") || file_path_str.ends_with(".yml");

if is_yaml {
    // Process any collected XML includes first (in parallel)
    // Then process YAML include sequentially
    // Break batching to restart collection after YAML
}
```

This ensures:
- ✅ XML includes processed in parallel for performance
- ✅ YAML includes processed sequentially to modify parent context
- ✅ Subsequent includes see YAML-declared variables

### Undocumented Behavior

The ROS 2 documentation does **not** explicitly document:
1. YAML launch file scoping behavior
2. The distinction between YAML and XML include semantics
3. When/how configurations from includes persist vs. are isolated

This behavior was reverse-engineered from:
- Autoware's actual usage patterns
- Testing with planning_simulator.launch.xml
- Observing that YAML preset files must affect parent scope

### Conclusion

**XML Includes**: Isolated child scope → safe for parallel processing
**YAML Includes**: Modify parent scope → must be sequential before dependent includes

This hybrid approach maintains performance (parallel XML) while preserving correctness (sequential YAML).

### Sources

- [ROS 2 Launch XML Format](https://design.ros2.org/articles/roslaunch_xml.html)
- [ROS 2 Launch System](https://design.ros2.org/articles/roslaunch.html)
- [Using Python, XML, and YAML for ROS 2 Launch Files](https://docs.ros.org/en/foxy/How-To-Guides/Launch-file-different-formats.html)
- [Managing large projects (ROS 2 Humble)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)
