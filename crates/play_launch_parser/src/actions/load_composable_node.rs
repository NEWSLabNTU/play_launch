//! Load composable node action

use crate::{
    actions::ComposableNodeAction,
    captures::LoadNodeCapture,
    error::{ParseError, Result},
    record::LoadNodeRecord,
    substitution::{LaunchContext, Substitution, parse_substitutions, resolve_substitutions},
    xml::{Entity, XmlEntity},
};

/// Load composable node action - loads composable nodes into an existing container
#[derive(Debug, Clone)]
pub struct LoadComposableNodeAction {
    /// Target container name (can contain substitutions)
    pub target: Vec<Substitution>,
    /// Composable nodes to load
    pub composable_nodes: Vec<ComposableNodeAction>,
}

impl LoadComposableNodeAction {
    pub fn from_entity(entity: &XmlEntity, context: &LaunchContext) -> Result<Self> {
        // Get target attribute (container name/reference)
        let target_str =
            entity
                .required_attr_str("target")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "load_composable_node".to_string(),
                    attribute: "target".to_string(),
                })?;
        let target = parse_substitutions(&target_str)?;

        // Parse composable_node children
        let mut composable_nodes = Vec::new();
        for child in entity.children() {
            match child.type_name() {
                "composable_node" | "composable-node" => {
                    composable_nodes.push(ComposableNodeAction::from_entity(&child, context)?);
                }
                other => {
                    log::debug!("Skipping '{}' in load_composable_node", other);
                }
            }
        }

        if composable_nodes.is_empty() {
            log::warn!("load_composable_node has no composable_node children");
        }

        Ok(Self {
            target,
            composable_nodes,
        })
    }

    /// Convert to LoadNodeCaptures for LaunchContext storage
    pub fn to_captures(&self, context: &LaunchContext) -> Result<Vec<LoadNodeCapture>> {
        // Resolve target container name
        let target_container_name = resolve_substitutions(&self.target, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        // Store resolved target as-is. The target_container is a reference to an
        // existing container by its node name, NOT a namespace-relative path.
        // The Python launch system (perform_substitutions) does not prepend namespace.
        // The replay builder normalizes by adding "/" if needed for matching.
        let normalized_target = target_container_name;

        // Convert composable nodes to LoadNodeCaptures
        let captures: Vec<LoadNodeCapture> = self
            .composable_nodes
            .iter()
            .map(|node| {
                // Resolve node fields
                let package = resolve_substitutions(&node.package, context).unwrap_or_default();
                let plugin = resolve_substitutions(&node.plugin, context).unwrap_or_default();
                let node_name = resolve_substitutions(&node.name, context).unwrap_or_default();
                let namespace = if let Some(ref ns) = node.namespace {
                    let ns_resolved = resolve_substitutions(ns, context).unwrap_or_default();
                    if ns_resolved.starts_with('/') {
                        ns_resolved
                    } else if ns_resolved.is_empty() {
                        context.current_namespace()
                    } else {
                        let current_ns = context.current_namespace();
                        if current_ns == "/" {
                            format!("/{}", ns_resolved)
                        } else {
                            format!("{}/{}", current_ns, ns_resolved)
                        }
                    }
                } else {
                    // No explicit namespace on the composable node.
                    // Use the accumulated namespace from push-ros-namespace,
                    // matching ROS 2 launch behavior where composable nodes
                    // inherit the context namespace.
                    context.current_namespace()
                };

                LoadNodeCapture {
                    package,
                    plugin,
                    target_container_name: normalized_target.clone(),
                    node_name,
                    namespace,
                    parameters: node.parameters.clone(),
                    remappings: node.remappings.clone(),
                    scope_id: None,
                }
            })
            .collect();

        Ok(captures)
    }

    pub fn to_load_node_records(&self, context: &LaunchContext) -> Result<Vec<LoadNodeRecord>> {
        // Resolve target container name
        let target_container_name = resolve_substitutions(&self.target, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

        // Store resolved target as-is. The target_container is a reference to an
        // existing container by its node name, NOT a namespace-relative path.
        // The Python launch system (perform_substitutions) does not prepend namespace.
        // The replay builder normalizes by adding "/" if needed for matching.
        let normalized_target = target_container_name;

        // Get global parameters from context (set via SetParameter actions)
        let global_params_vec: Vec<(String, String)> =
            context.global_parameters().into_iter().collect();

        // Convert composable nodes to LoadNodeRecords
        let records: Vec<LoadNodeRecord> = self
            .composable_nodes
            .iter()
            .map(|node| {
                // Resolve node fields
                let package = resolve_substitutions(&node.package, context).unwrap_or_default();
                let plugin = resolve_substitutions(&node.plugin, context).unwrap_or_default();
                let node_name = resolve_substitutions(&node.name, context).unwrap_or_default();
                let namespace = if let Some(ref ns) = node.namespace {
                    let ns_resolved = resolve_substitutions(ns, context).unwrap_or_default();
                    if ns_resolved.starts_with('/') {
                        ns_resolved
                    } else if ns_resolved.is_empty() {
                        context.current_namespace()
                    } else {
                        let current_ns = context.current_namespace();
                        if current_ns == "/" {
                            format!("/{}", ns_resolved)
                        } else {
                            format!("{}/{}", current_ns, ns_resolved)
                        }
                    }
                } else {
                    // No explicit namespace on the composable node.
                    // Use the accumulated namespace from push-ros-namespace,
                    // matching ROS 2 launch behavior where composable nodes
                    // inherit the context namespace.
                    context.current_namespace()
                };

                // Merge global parameters with node-specific parameters
                let mut merged_params = global_params_vec.clone();
                for (key, value) in &node.parameters {
                    // Check if this key already exists
                    if let Some(existing) = merged_params.iter_mut().find(|(k, _)| k == key) {
                        // Override global parameter with node-specific value
                        existing.1 = value.clone();
                    } else {
                        // Add new parameter
                        merged_params.push((key.clone(), value.clone()));
                    }
                }

                // Build LoadNodeRecord
                LoadNodeRecord {
                    package,
                    plugin,
                    target_container_name: normalized_target.clone(),
                    node_name,
                    namespace,
                    log_level: None,
                    remaps: node.remappings.clone(),
                    params: merged_params,
                    extra_args: node.extra_args.clone(),
                    env: None,
                    scope: None,
                }
            })
            .collect();

        Ok(records)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xml::XmlEntity;
    use roxmltree::Document;

    #[test]
    fn test_parse_load_composable_node_simple() {
        let xml = r#"
            <load_composable_node target="/control/control_container">
                <composable_node pkg="my_pkg" plugin="my_plugin" name="my_node"/>
            </load_composable_node>
        "#;

        let doc = Document::parse(xml).unwrap();
        let entity = XmlEntity::new(doc.root_element());
        let context = LaunchContext::new();

        let action = LoadComposableNodeAction::from_entity(&entity, &context).unwrap();
        assert_eq!(action.composable_nodes.len(), 1);
        // Name is now Vec<Substitution> — resolve to verify
        let name = resolve_substitutions(&action.composable_nodes[0].name, &context).unwrap();
        assert_eq!(name, "my_node");
    }

    #[test]
    fn test_parse_load_composable_node_missing_target() {
        let xml = r#"
            <load_composable_node>
                <composable_node pkg="my_pkg" plugin="my_plugin" name="my_node"/>
            </load_composable_node>
        "#;

        let doc = Document::parse(xml).unwrap();
        let entity = XmlEntity::new(doc.root_element());
        let context = LaunchContext::new();

        let result = LoadComposableNodeAction::from_entity(&entity, &context);
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_load_composable_node_with_substitution() {
        let xml = r#"
            <load_composable_node target="$(var container_name)">
                <composable_node pkg="my_pkg" plugin="my_plugin" name="my_node"/>
            </load_composable_node>
        "#;

        let doc = Document::parse(xml).unwrap();
        let entity = XmlEntity::new(doc.root_element());
        let mut context = LaunchContext::new();
        context.set_configuration("container_name".to_string(), "/test/container".to_string());

        let action = LoadComposableNodeAction::from_entity(&entity, &context).unwrap();
        let records = action.to_load_node_records(&context).unwrap();

        assert_eq!(records.len(), 1);
        assert_eq!(records[0].target_container_name, "/test/container");
    }

    #[test]
    fn test_to_load_node_records_absolute_target() {
        let xml = r#"
            <load_composable_node target="/control/control_container">
                <composable_node pkg="pkg1" plugin="plugin1" name="node1"/>
                <composable_node pkg="pkg2" plugin="plugin2" name="node2"/>
            </load_composable_node>
        "#;

        let doc = Document::parse(xml).unwrap();
        let entity = XmlEntity::new(doc.root_element());
        let context = LaunchContext::new();

        let action = LoadComposableNodeAction::from_entity(&entity, &context).unwrap();
        let records = action.to_load_node_records(&context).unwrap();

        assert_eq!(records.len(), 2);
        assert_eq!(
            records[0].target_container_name,
            "/control/control_container"
        );
        assert_eq!(records[0].node_name, "node1");
        // No explicit namespace + no push-ros-namespace = root namespace
        assert_eq!(records[0].namespace, "/");
        assert_eq!(
            records[1].target_container_name,
            "/control/control_container"
        );
        assert_eq!(records[1].node_name, "node2");
        assert_eq!(records[1].namespace, "/");
    }

    #[test]
    fn test_to_load_node_records_inherits_push_ros_namespace() {
        // Composable nodes without explicit namespace should inherit
        // the accumulated namespace from push-ros-namespace
        let xml = r#"
            <load_composable_node target="/pointcloud_container">
                <composable_node pkg="pkg1" plugin="plugin1" name="node1"/>
            </load_composable_node>
        "#;

        let doc = Document::parse(xml).unwrap();
        let entity = XmlEntity::new(doc.root_element());
        let mut context = LaunchContext::new();
        // Simulate push-ros-namespace "localization" then "util"
        context.push_namespace("localization".to_string());
        context.push_namespace("util".to_string());

        let action = LoadComposableNodeAction::from_entity(&entity, &context).unwrap();
        let records = action.to_load_node_records(&context).unwrap();

        assert_eq!(records.len(), 1);
        assert_eq!(records[0].target_container_name, "/pointcloud_container");
        assert_eq!(records[0].node_name, "node1");
        assert_eq!(records[0].namespace, "/localization/util");
    }

    #[test]
    fn test_to_load_node_records_relative_target() {
        let xml = r#"
            <load_composable_node target="my_container">
                <composable_node pkg="pkg1" plugin="plugin1" name="node1"/>
            </load_composable_node>
        "#;

        let doc = Document::parse(xml).unwrap();
        let entity = XmlEntity::new(doc.root_element());
        let mut context = LaunchContext::new();
        context.push_namespace("test_ns".to_string());

        let action = LoadComposableNodeAction::from_entity(&entity, &context).unwrap();
        let records = action.to_load_node_records(&context).unwrap();

        assert_eq!(records.len(), 1);
        // Target container is stored as-is (no namespace prepending).
        // The replay builder normalizes by adding "/" for matching.
        assert_eq!(records[0].target_container_name, "my_container");
    }

    #[test]
    fn test_to_load_node_records_with_node_namespace() {
        let xml = r#"
            <load_composable_node target="/control/control_container">
                <composable_node pkg="pkg1" plugin="plugin1" name="node1" namespace="/custom_ns"/>
            </load_composable_node>
        "#;

        let doc = Document::parse(xml).unwrap();
        let entity = XmlEntity::new(doc.root_element());
        let context = LaunchContext::new();

        let action = LoadComposableNodeAction::from_entity(&entity, &context).unwrap();
        let records = action.to_load_node_records(&context).unwrap();

        assert_eq!(records.len(), 1);
        assert_eq!(records[0].namespace, "/custom_ns");
    }
}
