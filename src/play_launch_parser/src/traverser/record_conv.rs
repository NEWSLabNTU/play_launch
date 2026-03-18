use super::super::LaunchTraverser;
use crate::{error::Result, record::RecordJson};
use std::collections::HashMap;

impl LaunchTraverser {
    pub fn into_record_json(mut self) -> Result<RecordJson> {
        // Convert captures from context to records
        let captured_count = self.context.captured_nodes().len();
        log::debug!("Processing {} captured nodes from context", captured_count);
        log::debug!("Also have {} nodes in self.records", self.records.len());
        log::debug!("Scope table: {} scopes", self.scope_table.len());

        // Build global_params from context (accumulated via SetParameter actions)
        let final_global_params = self.context.global_parameters();
        let global_params_opt = if final_global_params.is_empty() {
            None
        } else {
            Some(final_global_params.into_iter().collect::<Vec<_>>())
        };

        // Convert captures to records.
        // Use capture.scope_id if set (stamped during include processing),
        // otherwise fall back to current_scope_id (root).
        let mut nodes: Vec<_> = self
            .context
            .captured_nodes()
            .iter()
            .map(|n| {
                let mut rec = n.to_record(&global_params_opt)?;
                if rec.scope.is_none() {
                    rec.scope = Some(n.scope_id.unwrap_or(self.current_scope_id));
                }
                Ok(rec)
            })
            .collect::<Result<Vec<_>>>()?;

        let mut containers: Vec<_> = self
            .context
            .captured_containers()
            .iter()
            .map(|c| {
                let mut rec = c.to_record(&global_params_opt)?;
                if rec.scope.is_none() {
                    rec.scope = Some(c.scope_id.unwrap_or(self.current_scope_id));
                }
                Ok(rec)
            })
            .collect::<Result<Vec<_>>>()?;

        let mut load_nodes: Vec<_> = self
            .context
            .captured_load_nodes()
            .iter()
            .map(|ln| {
                let mut rec = ln.to_record(&global_params_opt)?;
                if rec.scope.is_none() {
                    rec.scope = Some(ln.scope_id.unwrap_or(self.current_scope_id));
                }
                Ok(rec)
            })
            .collect::<Result<Vec<_>>>()?;

        // CRITICAL: Backfill global_params for nodes that were created before SetParameter ran
        // Some XML nodes are created early in parsing, before Python files run SetParameter actions
        log::debug!(
            "Final global_params has {} keys",
            global_params_opt.as_ref().map_or(0, |v| v.len())
        );
        if let Some(ref global_params_vec) = global_params_opt {
            // Backfill nodes from context captures
            let mut backfilled_count = 0;
            for node in nodes.iter_mut() {
                if node.global_params.is_none() {
                    node.global_params = Some(global_params_vec.clone());
                    backfilled_count += 1;
                }
            }
            log::debug!("Backfilled {} captured nodes", backfilled_count);

            // Backfill XML nodes from self.records (both global_params field and cmd)
            let mut backfilled_count = 0;
            for node in self.records.iter_mut() {
                if node.global_params.is_none() {
                    node.global_params = Some(global_params_vec.clone());
                    // Only inject -p params into cmd for ROS nodes (package is Some).
                    // Raw executables (package is None) don't understand ROS params.
                    if node.package.is_some() {
                        for (key, value) in global_params_vec {
                            node.cmd.push("-p".to_string());
                            node.cmd.push(format!(
                                "{}:={}",
                                key,
                                crate::record::generator::normalize_param_value(value)
                            ));
                        }
                    }
                    backfilled_count += 1;
                }
            }
            log::debug!(
                "Backfilled {} XML nodes from self.records",
                backfilled_count
            );

            // Backfill containers
            for container in containers.iter_mut() {
                if container.global_params.is_none() {
                    container.global_params = Some(global_params_vec.clone());
                }
            }
            for container in self.containers.iter_mut() {
                if container.global_params.is_none() {
                    container.global_params = Some(global_params_vec.clone());
                }
            }
        }

        // Merge with local vectors (from includes that haven't migrated yet)
        nodes.extend(self.records);
        containers.extend(self.containers);
        load_nodes.extend(self.load_nodes);

        Ok(RecordJson {
            node: nodes,
            container: containers,
            load_node: load_nodes,
            lifecycle_node: Vec::new(),
            file_data: HashMap::new(),
            variables: self.context.configurations(),
            scopes: self.scope_table.into_entries(),
        })
    }
}
