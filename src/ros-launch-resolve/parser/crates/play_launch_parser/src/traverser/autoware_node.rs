//! Executing an `<autoware_node>` (#0095), for the XML and YAML frontends.

use super::super::LaunchTraverser;
use crate::{
    actions::autoware_node::{AutowareNodeAction, AutowareNodeForm},
    error::{ParseError, Result},
    record::CommandGenerator,
};

impl LaunchTraverser {
    /// Decide what the action becomes and take the path that action
    /// already takes: `<load_composable_node>`'s for a component,
    /// `<node>`'s for a process.
    pub(crate) fn execute_autoware_node(&mut self, action: &AutowareNodeAction) -> Result<()> {
        match action.resolve(&self.context)? {
            AutowareNodeForm::Component(load) => {
                for capture in load.to_captures(&self.context)? {
                    self.context.capture_load_node(capture);
                }
            }
            AutowareNodeForm::Process(node) => {
                let mut record = CommandGenerator::generate_node_record(&node, &self.context)
                    .map_err(ParseError::from)?;
                record.scope = Some(self.current_scope_id);
                self.records.push(record);
            }
        }
        Ok(())
    }
}
