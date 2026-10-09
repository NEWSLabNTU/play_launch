use super::super::LaunchTraverser;
use crate::{error::Result, record::RecordJson};
use std::collections::HashMap;

impl LaunchTraverser {
    pub fn into_record_json(self) -> Result<RecordJson> {
        // Convert captures from context to records
        let captured_count = self.context.captured_nodes().len();
        log::debug!("Processing {} captured nodes from context", captured_count);
        log::debug!("Also have {} nodes in self.records", self.records.len());
        log::debug!("Scope table: {} scopes", self.scope_table.len());

        // Global parameters are POSITIONAL: launch applies the
        // `global_params` list as it stands when a node executes, so a node
        // declared before a `<set_parameter>` never sees it. Every capture and
        // record carries the set in effect where it was declared; the final
        // set is only the fallback for a capture that recorded none. This used
        // to back-fill every node created before the first `<set_parameter>`
        // with the FINAL set, and give every Python-declared node the final
        // set too.
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
                let gp = n
                    .global_params
                    .clone()
                    .or_else(|| global_params_opt.clone());
                let mut rec = n.to_record(&gp)?;
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
                let gp = c
                    .global_params
                    .clone()
                    .or_else(|| global_params_opt.clone());
                let mut rec = c.to_record(&gp)?;
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
                let gp = ln
                    .global_params
                    .clone()
                    .or_else(|| global_params_opt.clone());
                let mut rec = ln.to_record(&gp)?;
                if rec.scope.is_none() {
                    rec.scope = Some(ln.scope_id.unwrap_or(self.current_scope_id));
                }
                Ok(rec)
            })
            .collect::<Result<Vec<_>>>()?;

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
            dropped_actions: self.dropped_actions,
        })
    }
}
