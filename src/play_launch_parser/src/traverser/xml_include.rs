use super::super::LaunchTraverser;
use crate::{error::Result, file_cache::read_file_cached, xml};
use std::{collections::HashMap, path::Path};

impl LaunchTraverser {
    /// Process an XML include with an optional ROS namespace to apply
    pub(crate) fn process_xml_include_with_namespace(
        &mut self,
        resolved_path: &Path,
        include_args: &HashMap<String, String>,
        ros_namespace: Option<String>,
    ) -> Result<()> {
        log::debug!("Including XML launch file: {}", resolved_path.display());
        if let Some(ref ns) = ros_namespace {
            log::debug!("Applying ROS namespace '{}' to XML include", ns);
        }

        // Create a new context for the included file (O(1) with Arc, not O(n) clone!)
        let mut include_context = self.context.child();
        include_context.set_current_file(resolved_path.to_path_buf());

        // CRITICAL FIX: When including an XML file from Python with a ROS namespace,
        // we must clear the inherited namespace stack and push ONLY the ROS namespace.
        // Otherwise, the XML nodes will inherit the parent XML context's namespace,
        // and then we'll apply the ROS namespace on top, causing double accumulation.
        log::debug!(
            "process_xml_include_with_namespace: include_context initial namespace='{}', depth={}, ros_namespace={:?}",
            include_context.current_namespace(),
            include_context.namespace_depth(),
            ros_namespace
        );

        // Only apply namespace transformation if ros_namespace is explicitly provided and not root
        if let Some(ref ns) = ros_namespace {
            if !ns.is_empty() && ns != "/" {
                log::debug!(
                    "Clearing inherited namespace stack and setting ROS namespace: '{}'",
                    ns
                );
                // Clear the namespace stack (keep only root)
                while include_context.namespace_depth() > 1 {
                    include_context.pop_namespace();
                }
                // Push the ROS namespace from Python
                include_context.push_namespace(ns.clone());
                log::debug!(
                    "After applying ROS namespace: include_context namespace='{}'",
                    include_context.current_namespace()
                );
            } else {
                // ros_namespace is "/" or empty - keep inherited namespace from parent
                log::debug!(
                    "ros_namespace is root or empty, keeping inherited namespace: '{}'",
                    include_context.current_namespace()
                );
            }
        } else {
            // No ros_namespace provided - keep inherited namespace from parent
            log::debug!(
                "No ros_namespace provided, keeping inherited namespace: '{}'",
                include_context.current_namespace()
            );
        }

        // Apply include arguments
        for (key, value) in include_args {
            log::debug!(
                "[RUST] Setting include arg from Python: {} = {}",
                key,
                value
            );
            include_context.set_configuration(key.clone(), value.clone());
        }

        // Parse and traverse the included file
        let content = read_file_cached(resolved_path)?;
        let doc = roxmltree::Document::parse(&content)?;
        let root = xml::XmlEntity::new(doc.root_element());

        // Create temporary traverser for included file
        let mut included_traverser = LaunchTraverser {
            context: include_context,
            include_chain: self.include_chain.clone(),
            records: Vec::new(),
            containers: Vec::new(),
            load_nodes: Vec::new(),
        };
        included_traverser.traverse_entity(&root)?;

        // CRITICAL: Merge global parameters from included file back to parent context
        // SetParameter actions in Python files (called from XML includes) write to the
        // child context. We must propagate them back so into_record_json() can find them.
        for (key, value) in included_traverser.context.global_parameters() {
            self.context.set_global_parameter(key, value);
        }

        // NOTE: Do NOT merge configurations from included file back to parent context.
        // In ROS 2, included files have isolated scope — their <arg> defaults and internal
        // variables should not leak to the parent.

        // Apply ROS namespace if provided (for includes from Python OpaqueFunction)
        if let Some(ref ns) = ros_namespace
            && !ns.is_empty()
            && ns != "/"
        {
            log::debug!(
                "Applying ROS namespace '{}' to {} nodes, {} containers, {} load_nodes from XML include",
                ns,
                included_traverser.records.len(),
                included_traverser.containers.len(),
                included_traverser.load_nodes.len()
            );

            // Apply namespace to regular nodes
            for node in &mut included_traverser.records {
                if let Some(ref node_ns) = node.namespace {
                    node.namespace = Some(Self::apply_namespace_prefix(ns, node_ns));
                } else {
                    node.namespace = Some(ns.clone());
                }
            }

            // Apply namespace to containers and build a mapping
            let mut container_name_map = std::collections::HashMap::new();
            for container in &mut included_traverser.containers {
                // Build old full name
                let old_full_name = if container.namespace.starts_with('/') {
                    if container.namespace == "/" {
                        format!("/{}", container.name)
                    } else {
                        format!("{}/{}", container.namespace, container.name)
                    }
                } else {
                    format!("/{}/{}", container.namespace, container.name)
                };

                // Apply namespace prefix
                container.namespace = Self::apply_namespace_prefix(ns, &container.namespace);

                // Build new full name
                let new_full_name = if container.namespace.starts_with('/') {
                    if container.namespace == "/" {
                        format!("/{}", container.name)
                    } else {
                        format!("{}/{}", container.namespace, container.name)
                    }
                } else {
                    format!("/{}/{}", container.namespace, container.name)
                };

                container_name_map.insert(old_full_name, new_full_name);
            }

            // Apply namespace to load_nodes and update target containers
            for load_node in &mut included_traverser.load_nodes {
                load_node.namespace = Self::apply_namespace_prefix(ns, &load_node.namespace);

                // Update target_container_name if it matches a renamed container
                if let Some(new_name) = container_name_map.get(&load_node.target_container_name) {
                    log::debug!(
                        "Updating target_container_name from '{}' to '{}'",
                        load_node.target_container_name,
                        new_name
                    );
                    load_node.target_container_name = new_name.clone();
                } else {
                    // If not in map, still apply namespace prefix for external containers
                    if load_node.target_container_name.starts_with('/') {
                        let prefixed =
                            Self::apply_namespace_prefix(ns, &load_node.target_container_name);
                        if prefixed != load_node.target_container_name {
                            log::debug!(
                                "Applying namespace prefix to target_container_name: '{}' -> '{}'",
                                load_node.target_container_name,
                                prefixed
                            );
                            load_node.target_container_name = prefixed;
                        }
                    }
                }
            }
        }

        // Merge records from included file into current records
        self.records.extend(included_traverser.records);
        self.containers.extend(included_traverser.containers);
        self.load_nodes.extend(included_traverser.load_nodes);

        // Merge captures from included file's context
        // Apply namespace to captures as well
        for node in included_traverser.context.captured_nodes() {
            let mut node_copy = node.clone();
            if let Some(ref ns) = ros_namespace
                && !ns.is_empty()
                && ns != "/"
            {
                if let Some(ref node_ns) = node_copy.namespace {
                    node_copy.namespace = Some(Self::apply_namespace_prefix(ns, node_ns));
                } else {
                    node_copy.namespace = Some(ns.clone());
                }
            }
            self.context.capture_node(node_copy);
        }

        for container in included_traverser.context.captured_containers() {
            let mut container_copy = container.clone();
            if let Some(ref ns) = ros_namespace
                && !ns.is_empty()
                && ns != "/"
            {
                container_copy.namespace =
                    Self::apply_namespace_prefix(ns, &container_copy.namespace);
                container_copy.name = Self::apply_namespace_prefix(ns, &container_copy.name);
            }
            self.context.capture_container(container_copy);
        }

        for load_node in included_traverser.context.captured_load_nodes() {
            let mut load_node_copy = load_node.clone();
            log::debug!(
                "Merging load_node capture: target='{}', ros_namespace={:?}",
                load_node_copy.target_container_name,
                ros_namespace
            );
            if let Some(ref ns) = ros_namespace
                && !ns.is_empty()
                && ns != "/"
            {
                load_node_copy.namespace =
                    Self::apply_namespace_prefix(ns, &load_node_copy.namespace);
                if load_node_copy.target_container_name.starts_with('/') {
                    let old_target = load_node_copy.target_container_name.clone();
                    load_node_copy.target_container_name =
                        Self::apply_namespace_prefix(ns, &load_node_copy.target_container_name);
                    log::debug!(
                        "Applied namespace to target: '{}' -> '{}'",
                        old_target,
                        load_node_copy.target_container_name
                    );
                }
            }
            self.context.capture_load_node(load_node_copy);
        }

        log::debug!(
            "Merged XML include captures: {} load_nodes from context",
            included_traverser.context.captured_load_nodes().len()
        );

        Ok(())
    }
}
