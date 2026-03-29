use super::super::LaunchTraverser;
use crate::{
    actions::IncludeAction,
    error::{ParseError, Result},
    file_cache::read_file_cached,
    record::extract_package_from_path,
    substitution::resolve_substitutions,
    xml,
};
use std::path::Path;

/// Validate that an include path is a launch file (by extension).
/// Prevents including arbitrary files like `/etc/passwd`.
pub(crate) fn validate_include_path(path: &Path, original: &str) -> Result<()> {
    let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("");
    match ext {
        "xml" | "py" | "yaml" | "yml" => Ok(()),
        _ => Err(ParseError::IoError(std::io::Error::new(
            std::io::ErrorKind::PermissionDenied,
            format!(
                "Include path '{}' has unexpected extension '.{}'. \
                 Only .xml, .py, .yaml, .yml launch files can be included.",
                original, ext
            ),
        ))),
    }
}

impl LaunchTraverser {
    pub(crate) fn process_include(&mut self, include: &IncludeAction) -> Result<()> {
        // Resolve the file path
        let file_path_str = resolve_substitutions(&include.file, &self.context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        let file_path = Path::new(&file_path_str);

        log::trace!("Processing include: {}", file_path_str);

        // Validate the include path has a launch file extension
        validate_include_path(file_path, &file_path_str)?;

        // Resolve relative paths relative to the current launch file
        let resolved_path = if file_path.is_relative() {
            if let Some(current_file) = self.context.current_file() {
                if let Some(parent_dir) = current_file.parent() {
                    parent_dir.join(file_path)
                } else {
                    file_path.to_path_buf()
                }
            } else {
                file_path.to_path_buf()
            }
        } else {
            file_path.to_path_buf()
        };

        // Canonicalize path for circular include detection
        let canonical_path = resolved_path
            .canonicalize()
            .unwrap_or_else(|_| resolved_path.clone());

        // Check for circular includes in the current include chain
        if self.include_chain.contains(&canonical_path) {
            log::warn!("Circular include detected: {}", canonical_path.display());
            return Ok(()); // Skip circular includes
        }

        // Check include depth limit
        if self.include_chain.len() >= self.max_include_depth {
            return Err(ParseError::MaxIncludeDepthExceeded {
                max: self.max_include_depth,
                file: canonical_path.display().to_string(),
            });
        }

        log::debug!("Including launch file: {}", resolved_path.display());

        // Log include arguments being passed
        if !include.args.is_empty() {
            let arg_names: Vec<&str> = include.args.iter().map(|(k, _)| k.as_str()).collect();
            log::trace!("Include args: {:?}", arg_names);
        }

        // Check if this is a Python launch file
        // Check file extension and handle non-XML files
        if let Some(ext) = resolved_path.extension().and_then(|s| s.to_str()) {
            match ext {
                "py" => {
                    log::debug!("Including Python launch file: {}", resolved_path.display());

                    // Create args for the Python file (include args override current context)
                    let mut python_args = self.context.configurations();
                    for (key, value_subs) in &include.args {
                        let resolved_value = resolve_substitutions(value_subs, &self.context)
                            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                        log::trace!("  Include arg: {} = {}", key, resolved_value);
                        python_args.insert(key.clone(), resolved_value);
                    }

                    // Push scope for this Python include
                    let py_file_name = resolved_path
                        .file_name()
                        .and_then(|s| s.to_str())
                        .unwrap_or("unknown")
                        .to_string();
                    let py_pkg = extract_package_from_path(&resolved_path);
                    let py_ns = self.context.current_namespace();
                    let child_scope_id = self.scope_table.push(
                        py_pkg,
                        py_file_name,
                        py_ns,
                        python_args.clone(),
                        Some(self.current_scope_id),
                    );
                    let prev_scope_id = self.current_scope_id;
                    self.current_scope_id = child_scope_id;

                    // Track counts before execution to stamp new entries
                    let prev_rec = self.records.len();
                    let prev_cont = self.containers.len();
                    let prev_ln = self.load_nodes.len();
                    let prev_cap_nodes = self.context.captured_nodes().len();
                    let prev_cap_containers = self.context.captured_containers().len();
                    let prev_cap_load_nodes = self.context.captured_load_nodes().len();

                    let result = self.execute_python_file(&resolved_path, &python_args);

                    // Stamp scope on records added during this Python execution.
                    // XML records created via process_xml_include_with_namespace
                    // already have scope from their child traverser, so only stamp
                    // those that are still None.
                    for rec in &mut self.records[prev_rec..] {
                        if rec.scope.is_none() {
                            rec.scope = Some(child_scope_id);
                        }
                    }
                    for rec in &mut self.containers[prev_cont..] {
                        if rec.scope.is_none() {
                            rec.scope = Some(child_scope_id);
                        }
                    }
                    for rec in &mut self.load_nodes[prev_ln..] {
                        if rec.scope.is_none() {
                            rec.scope = Some(child_scope_id);
                        }
                    }

                    // Stamp scope on captures added during this Python execution.
                    for cap in &mut self.context.captured_nodes_mut()[prev_cap_nodes..] {
                        if cap.scope_id.is_none() {
                            cap.scope_id = Some(child_scope_id);
                        }
                    }
                    for cap in &mut self.context.captured_containers_mut()[prev_cap_containers..] {
                        if cap.scope_id.is_none() {
                            cap.scope_id = Some(child_scope_id);
                        }
                    }
                    for cap in &mut self.context.captured_load_nodes_mut()[prev_cap_load_nodes..] {
                        if cap.scope_id.is_none() {
                            cap.scope_id = Some(child_scope_id);
                        }
                    }

                    // Update scope args with all resolved configurations
                    let final_args = self.context.configurations();
                    self.scope_table.update_args(child_scope_id, final_args);

                    self.current_scope_id = prev_scope_id;
                    return result;
                }
                "yaml" | "yml" => {
                    // YAML files in <include> are always launch files
                    // (parameter files are handled in <param from="..."> context)
                    log::debug!("Including YAML launch file: {}", resolved_path.display());

                    // Push scope for this YAML include
                    let yaml_file_name = resolved_path
                        .file_name()
                        .and_then(|s| s.to_str())
                        .unwrap_or("unknown")
                        .to_string();
                    let yaml_pkg = extract_package_from_path(&resolved_path);
                    let yaml_ns = self.context.current_namespace();
                    let child_scope_id = self.scope_table.push(
                        yaml_pkg,
                        yaml_file_name,
                        yaml_ns,
                        self.context.configurations(),
                        Some(self.current_scope_id),
                    );
                    let prev_scope_id = self.current_scope_id;
                    self.current_scope_id = child_scope_id;

                    let result = self.process_yaml_launch_file(&resolved_path);

                    // Update scope args with all resolved configurations
                    let final_args = self.context.configurations();
                    self.scope_table.update_args(child_scope_id, final_args);

                    self.current_scope_id = prev_scope_id;
                    return result;
                }
                _ => {}
            }
        }

        // Create a new context for the included file (O(1) with Arc, not O(n) clone!)
        // Start with current context and apply include args
        let mut include_context = self.context.child();
        include_context.set_current_file(resolved_path.clone());
        for (key, value_subs) in &include.args {
            // IMPORTANT: Resolve substitutions in the argument value using the include_context
            // (not the parent context) so that later args can reference earlier args
            // Example: <arg name="A" value="x"/>
            //          <arg name="B" value="$(var A)/y"/>  <-- B can reference A
            let resolved_value = resolve_substitutions(value_subs, &include_context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
            log::debug!("[RUST] Setting include arg: {} = {}", key, resolved_value);
            include_context.set_configuration(key.clone(), resolved_value);
        }

        log::debug!(
            "[RUST] Include context has {} configs after setting include args",
            include_context.configurations().len()
        );

        // Parse and traverse the included file
        let content = read_file_cached(&resolved_path)?;
        let doc = roxmltree::Document::parse(&content)?;
        let root = xml::XmlEntity::new(doc.root_element());

        // Create temporary traverser for included file with extended include chain
        let mut child_chain = self.include_chain.clone();
        child_chain.push(canonical_path);

        // Push a new scope for this include
        let include_file_name = resolved_path
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("unknown")
            .to_string();
        let include_pkg = extract_package_from_path(&resolved_path);
        let include_ns = include_context.current_namespace();
        let include_args = include_context.configurations();
        let child_scope_id = self.scope_table.push(
            include_pkg,
            include_file_name,
            include_ns,
            include_args,
            Some(self.current_scope_id),
        );

        let mut included_traverser = LaunchTraverser {
            context: include_context,
            include_chain: child_chain,
            max_include_depth: self.max_include_depth,
            records: Vec::new(),
            containers: Vec::new(),
            load_nodes: Vec::new(),
            scope_table: std::mem::take(&mut self.scope_table),
            current_scope_id: child_scope_id,
        };
        included_traverser.traverse_entity(&root)?;

        // Take back the scope table (child may have added entries from nested includes)
        self.scope_table = std::mem::take(&mut included_traverser.scope_table);

        // Update scope args with all resolved configurations (includes defaults
        // from <arg default="..."/> that were processed during traversal)
        let final_args = included_traverser.context.configurations();
        self.scope_table.update_args(child_scope_id, final_args);

        // Merge records from included file into current records
        self.records.extend(included_traverser.records);
        self.containers.extend(included_traverser.containers);
        self.load_nodes.extend(included_traverser.load_nodes);

        // Merge captures from included file's context, stamping scope_id
        let child_scope = included_traverser.current_scope_id;
        for node in included_traverser.context.captured_nodes() {
            let mut node_copy = node.clone();
            if node_copy.scope_id.is_none() {
                node_copy.scope_id = Some(child_scope);
            }
            self.context.capture_node(node_copy);
        }

        for container in included_traverser.context.captured_containers() {
            let mut container_copy = container.clone();
            if container_copy.scope_id.is_none() {
                container_copy.scope_id = Some(child_scope);
            }
            self.context.capture_container(container_copy);
        }

        for load_node in included_traverser.context.captured_load_nodes() {
            let mut load_node_copy = load_node.clone();
            if load_node_copy.scope_id.is_none() {
                load_node_copy.scope_id = Some(child_scope);
            }
            self.context.capture_load_node(load_node_copy);
        }

        // CRITICAL: Merge global parameters from included file back to parent context
        // SetParameter actions in Python files (called from XML includes) write to the
        // child context. We must propagate them back so into_record_json() can find them.
        for (key, value) in included_traverser.context.global_parameters() {
            self.context.set_global_parameter(key, value);
        }

        // NOTE: Do NOT merge configurations from included file back to parent context.
        // In ROS 2, included files have isolated scope — their <arg> defaults and internal
        // variables should not leak to the parent. Merging them causes bugs when sibling
        // includes declare args with the same name (e.g., "node_name") but different defaults.

        Ok(())
    }
}
