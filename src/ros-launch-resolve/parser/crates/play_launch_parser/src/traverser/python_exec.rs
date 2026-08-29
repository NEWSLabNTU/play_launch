use super::{super::LaunchTraverser, include::validate_include_path};
use crate::{
    error::{ParseError, Result},
    record::{canonicalize_path, extract_package_from_path},
    substitution::{parse_substitutions, resolve_substitutions},
};
use std::{collections::HashMap, path::Path};

impl LaunchTraverser {
    pub(crate) fn execute_python_file(
        &mut self,
        path: &Path,
        args: &HashMap<String, String>,
    ) -> Result<()> {
        // The backend, resolved BEFORE any context is published: if there is
        // no Python half in this build, say so while we can still name the
        // file, rather than failing somewhere inside the executor.
        let backend = crate::python_backend::require(
            crate::python_backend::PythonNeed::LaunchFile,
            &path.display().to_string(),
        )
        .map_err(|e| ParseError::PythonError(e.to_string()))?;

        log::debug!("Executing Python file: {}", path.display());
        log::trace!("Python file arguments: {} args", args.len());

        // Write include args into context as configurations
        // (Python API reads these via with_launch_context → get_configuration)
        for (k, v) in args {
            // Parse and resolve substitutions in the value
            let resolved_value = match parse_substitutions(v) {
                Ok(subs) => match resolve_substitutions(&subs, &self.context) {
                    Ok(resolved) => {
                        log::trace!("  Resolved '{}': '{}' -> '{}'", k, v, resolved);
                        resolved
                    }
                    Err(e) => {
                        log::warn!("Failed to resolve substitutions in '{}': {}", v, e);
                        v.clone()
                    }
                },
                Err(e) => {
                    log::trace!("No substitutions in '{}': {}", v, e);
                    v.clone()
                }
            };
            self.context.set_configuration(k.clone(), resolved_value);
        }

        // Add current ROS namespace for OpaqueFunction to access
        let current_ns = self.context.current_namespace();
        if !current_ns.is_empty() && current_ns != "/" {
            self.context
                .set_configuration("ros_namespace".to_string(), current_ns.clone());
            log::debug!("Added ros_namespace='{}' to context", current_ns);
        }

        // Set the thread-local context for Python API to access (cleared on guard drop)
        let _ctx_guard = crate::bridge::LaunchContextGuard::new(&mut self.context);

        let path_str = path.to_str().ok_or_else(|| {
            ParseError::PythonError(format!("Invalid UTF-8 in path: {}", path.display()))
        })?;
        let exec_result = backend.exec_file(path_str);

        // Guard clears context on drop (including on early return/panic)
        drop(_ctx_guard);

        // Propagate execution errors
        exec_result.map_err(ParseError::PythonError)?;

        // Python API stores captures directly in self.context via thread-local
        // (SetParameter also writes global params directly to context via thread-local)
        log::debug!("After Python execution:");
        log::debug!("  Captured nodes: {}", self.context.captured_nodes().len());
        log::debug!(
            "  Captured containers: {}",
            self.context.captured_containers().len()
        );
        log::debug!(
            "  Captured load_nodes: {}",
            self.context.captured_load_nodes().len()
        );

        log::debug!(
            "Python file '{}' completed - captures stored in context via thread-local",
            path.display()
        );

        // Process includes recursively (get from context and clear)
        let includes = self.context.captured_includes().to_vec();
        // Clear captured includes to prevent reprocessing in recursive calls
        self.context.captured_includes_mut().clear();

        log::debug!(
            "Processing {} captured includes from Python file '{}', current context namespace: '{}'",
            includes.len(),
            path.display(),
            self.context.current_namespace()
        );

        for include in includes {
            log::debug!(
                "Processing Python include: {} with ROS namespace '{}'",
                include.file_path,
                include.ros_namespace
            );

            // Convert args to HashMap
            let mut include_args = args.clone();
            for (key, value) in include.args {
                include_args.insert(key, value);
            }

            // Parse and resolve substitutions in the file path
            // (Python includes may contain substitutions like $(find-pkg-share pkg)/launch/file.xml)
            let file_path_subs = parse_substitutions(&include.file_path)?;
            let file_path_str =
                resolve_substitutions(&file_path_subs, &self.context).map_err(|e| {
                    ParseError::InvalidSubstitution(format!(
                        "Failed to resolve Python include path '{}': {}",
                        include.file_path, e
                    ))
                })?;

            log::debug!(
                "Resolved Python include path: {} -> {}",
                include.file_path,
                file_path_str
            );

            // Validate include path
            let include_path = Path::new(&file_path_str);
            validate_include_path(include_path, &file_path_str)?;

            // Resolve relative paths relative to the current Python file
            let resolved_include_path = if include_path.is_relative() {
                // Resolve relative to the current Python file
                if let Some(parent_dir) = path.parent() {
                    parent_dir.join(include_path)
                } else {
                    include_path.to_path_buf()
                }
            } else {
                include_path.to_path_buf()
            };

            // Determine the ROS namespace to apply
            let ros_ns = if !include.ros_namespace.is_empty() && include.ros_namespace != "/" {
                Some(include.ros_namespace.clone())
            } else {
                None
            };

            // Process the include with namespace context
            let result =
                if let Some(ext) = resolved_include_path.extension().and_then(|s| s.to_str()) {
                    match ext {
                        "py" => {
                            // Push scope for this Python-to-Python include
                            let py_file_name = resolved_include_path
                                .file_name()
                                .and_then(|s| s.to_str())
                                .unwrap_or("unknown")
                                .to_string();
                            let py_pkg = extract_package_from_path(&resolved_include_path);
                            let py_path = canonicalize_path(&resolved_include_path);
                            let py_ns = self.context.current_namespace();
                            let child_scope_id = self.scope_table.push(
                                py_pkg,
                                py_file_name,
                                py_path,
                                py_ns,
                                include_args.clone(),
                                Some(self.current_scope_id),
                            );
                            let prev_scope_id = self.current_scope_id;
                            self.current_scope_id = child_scope_id;

                            // Track captures before execution
                            let prev_cap_nodes = self.context.captured_nodes().len();
                            let prev_cap_containers = self.context.captured_containers().len();
                            let prev_cap_load_nodes = self.context.captured_load_nodes().len();
                            let prev_rec = self.records.len();
                            let prev_cont = self.containers.len();
                            let prev_ln = self.load_nodes.len();

                            // Save scope so namespace is restored after include
                            let scope = self.context.save_scope();
                            if let Some(ref ns) = ros_ns {
                                self.context.push_namespace(ns.clone());
                            }
                            let result =
                                self.execute_python_file(&resolved_include_path, &include_args);
                            self.context.restore_scope(scope);

                            // Stamp scope on new captures/records
                            for cap in &mut self.context.captured_nodes_mut()[prev_cap_nodes..] {
                                if cap.scope_id.is_none() {
                                    cap.scope_id = Some(child_scope_id);
                                }
                            }
                            for cap in
                                &mut self.context.captured_containers_mut()[prev_cap_containers..]
                            {
                                if cap.scope_id.is_none() {
                                    cap.scope_id = Some(child_scope_id);
                                }
                            }
                            for cap in
                                &mut self.context.captured_load_nodes_mut()[prev_cap_load_nodes..]
                            {
                                if cap.scope_id.is_none() {
                                    cap.scope_id = Some(child_scope_id);
                                }
                            }
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

                            // Update scope args with all resolved configurations
                            let final_args = self.context.configurations();
                            self.scope_table.update_args(child_scope_id, final_args);

                            self.current_scope_id = prev_scope_id;
                            result
                        }
                        "xml" => {
                            // For XML includes, pass namespace directly
                            self.process_xml_include_with_namespace(
                                &resolved_include_path,
                                &include_args,
                                ros_ns.clone(),
                            )
                        }
                        "yaml" | "yml" => {
                            // Push scope for YAML include from Python
                            let yaml_file_name = resolved_include_path
                                .file_name()
                                .and_then(|s| s.to_str())
                                .unwrap_or("unknown")
                                .to_string();
                            let yaml_pkg = extract_package_from_path(&resolved_include_path);
                            let yaml_path = canonicalize_path(&resolved_include_path);
                            let yaml_ns = self.context.current_namespace();
                            let child_scope_id = self.scope_table.push(
                                yaml_pkg,
                                yaml_file_name,
                                yaml_path,
                                yaml_ns,
                                include_args.clone(),
                                Some(self.current_scope_id),
                            );
                            let prev_scope_id = self.current_scope_id;
                            self.current_scope_id = child_scope_id;

                            let result = self.process_yaml_launch_file(&resolved_include_path);

                            // Update scope args with all resolved configurations
                            let final_args = self.context.configurations();
                            self.scope_table.update_args(child_scope_id, final_args);

                            self.current_scope_id = prev_scope_id;
                            result
                        }
                        _ => {
                            log::warn!(
                                "Unknown file type for Python include: {}",
                                resolved_include_path.display()
                            );
                            Ok(())
                        }
                    }
                } else {
                    log::warn!(
                        "No file extension for Python include: {}",
                        resolved_include_path.display()
                    );
                    Ok(())
                };

            // Propagate any errors
            result?;
        }

        Ok(())
    }
}
