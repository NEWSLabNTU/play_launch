//! Which model keys are ROS graph names, and which are not (issue #0017).
//!
//! `structure.nodes` is documented as `/ns/node_name` → instance, and a
//! consumer reading that has every reason to join it against
//! `ros2 node list`. For any `<node>` the launch file did not name, the join
//! silently fails: the key is built from the EXECUTABLE while the process
//! registers whatever name it was compiled with.
//!
//! ```text
//! model:  /localization/pose_twist_fusion_filter/autoware_ekf_localizer_node-1
//! graph:  /localization/pose_twist_fusion_filter/ekf_localizer
//! ```
//!
//! 17 of 144 nodes on one Autoware stack. No error is raised anywhere,
//! because nothing in the pipeline looks at the live system — the manifest
//! checker builds its graph from the manifest.
//!
//! **This is not fixable by renaming the node.** play_launch reproduces stock
//! `launch_ros`, which emits `-r __node:=` only when the launch file declared
//! a name (`node.py:493`). Forcing it was bug `af7c524`: it renames a node
//! away from its internally-hardcoded default and breaks discovery for
//! anything addressing it by that name, LifecycleNode services included. A
//! node the launch did not name is entitled to its own name; the model key is
//! the synthetic identifier, and inverting which one is authoritative would
//! change the running system to match the artifact.
//!
//! So the rule is: **the key is a resolve-time identity, and is a ROS name
//! only when `node_name` is set.** That rule lives here, once, rather than
//! being rediscovered by each consumer that joins against the graph — which
//! is what phase 61's stage gate had to do, inline, as the first consumer to
//! need it.

use ros_launch_manifest_model::SystemModel;

/// The node's real ROS graph name, if the model can know it.
///
/// `Some(fqn)` — the launch file named this node, so play_launch emitted
/// `-r __node:=<name>` and the key IS the graph name. Safe to join against
/// `ros2 node list`.
///
/// `None` — the key was derived from the executable (and carries the `-N`
/// ordinal that marks it as such, see #0018). The running node's name is
/// compiled into it and is NOT knowable from the launch file alone. Do not
/// join; fall back to something the model does own, such as whether the
/// process is running.
pub fn ros_graph_name<'a>(
    fqn: &'a str,
    node: &ros_launch_manifest_model::NodeInstance,
) -> Option<&'a str> {
    node.node_name.as_ref().map(|_| fqn)
}

/// Whether this model key can be matched against the live ROS graph.
///
/// The negation is not "the node is missing" — it is "the model does not know
/// this node's name". Reporting an unmatched executable-derived key as absent
/// is the failure mode this exists to prevent.
pub fn is_ros_graph_name(node: &ros_launch_manifest_model::NodeInstance) -> bool {
    node.node_name.is_some()
}

/// Every model key that is NOT a ROS graph name, in model order.
///
/// Callers that report to a human should say what these are (keys whose name
/// the launch file never declared), not merely how many.
pub fn non_graph_names(model: &SystemModel) -> Vec<&str> {
    model
        .structure
        .nodes
        .iter()
        .filter(|(_, n)| !is_ros_graph_name(n))
        .map(|(fqn, _)| fqn.as_str())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_launch_manifest_model::NodeInstance;

    fn node(name: Option<&str>) -> NodeInstance {
        NodeInstance {
            node_name: name.map(|s| s.to_string()),
            ..Default::default()
        }
    }

    #[test]
    fn a_declared_name_is_the_graph_name_and_an_undeclared_one_is_not() {
        let named = node(Some("ekf_localizer"));
        let unnamed = node(None);

        assert!(is_ros_graph_name(&named));
        assert_eq!(
            ros_graph_name("/ns/ekf_localizer", &named),
            Some("/ns/ekf_localizer")
        );

        // The whole point: the key LOOKS like an FQN and is not one.
        assert!(!is_ros_graph_name(&unnamed));
        assert_eq!(
            ros_graph_name("/ns/autoware_ekf_localizer_node-1", &unnamed),
            None
        );
    }
}
