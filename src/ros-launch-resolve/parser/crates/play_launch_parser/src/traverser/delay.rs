//! Where a `<timer period="N">` body's delay is recorded.
//!
//! A timer does not change WHAT its children are, only WHEN they start, so
//! the traverser walks them exactly as it walks a `<group>` body and then
//! stamps the delay on whatever they produced. Marking after the fact —
//! rather than threading a "current delay" through every record constructor
//! — is what makes `<include>`, `<node_container>` and the YAML frontend
//! inherit the behaviour for free: anything appended to the member lists
//! while the timer body was being traversed is, by construction, inside the
//! timer.
//!
//! Records arrive by two routes (local vectors for the XML/YAML frontend,
//! the context's capture lists for the Python one and for
//! `<load_composable_node>`), so a mark covers both.

use super::super::LaunchTraverser;

/// How many members existed before a timer body was traversed. Everything
/// appended past these indices belongs to the timer.
///
/// An `<include>` uses the same mark to stamp its scope on what the included
/// file produced (`traverser::include`), since it, too, runs in the
/// includer's own traverser and context.
#[derive(Clone, Copy, Debug)]
pub(crate) struct DelayMark {
    pub(super) records: usize,
    pub(super) containers: usize,
    pub(super) load_nodes: usize,
    pub(super) captured_nodes: usize,
    pub(super) captured_containers: usize,
    pub(super) captured_load_nodes: usize,
}

/// Nested timers ADD: ROS 2 starts the inner timer when the outer one fires,
/// so a 2 s timer inside a 3 s timer fires at 5 s. The inner mark is applied
/// first (innermost body unwinds first), so accumulating onto whatever is
/// already there produces exactly that.
/// Every action the XML/YAML frontends of stock `launch` and `launch_ros`
/// expose (`@expose_action`), Humble and Jazzy, read off their sources. A
/// tag outside this list and outside what this parser dispatches is not a
/// ROS 2 action at all unless a launch extension registers it, and is
/// refused (#0095). One inside it that this parser does not implement is
/// dropped and recorded, for `check` to refuse.
pub(crate) const STOCK_FRONTEND_ACTIONS: &[&str] = &[
    "append_env",
    "arg",
    "executable",
    "group",
    "include",
    "let",
    "lifecycle_node",
    "load_composable_node",
    "log",
    "node",
    "node_container",
    "pop-ros-namespace",
    "pop_ros_namespace",
    "push-ros-namespace",
    "push_ros_namespace",
    "reset",
    "reset_env",
    "ros_timer",
    "set_env",
    "set_parameter",
    "set_parameters_from_file",
    "set_remap",
    "set_ros_log_dir",
    "set_use_sim_time",
    "shutdown",
    "timer",
    "unset_env",
];

fn accumulate(slot: &mut Option<f64>, period: f64) {
    *slot = Some(slot.unwrap_or(0.0) + period);
}

impl LaunchTraverser {
    pub(crate) fn delay_mark(&self) -> DelayMark {
        DelayMark {
            records: self.records.len(),
            containers: self.containers.len(),
            load_nodes: self.load_nodes.len(),
            captured_nodes: self.context.captured_nodes().len(),
            captured_containers: self.context.captured_containers().len(),
            captured_load_nodes: self.context.captured_load_nodes().len(),
        }
    }

    pub(crate) fn apply_start_delay(&mut self, mark: DelayMark, period: f64) {
        for r in self.records.iter_mut().skip(mark.records) {
            accumulate(&mut r.start_delay_secs, period);
        }
        for c in self.containers.iter_mut().skip(mark.containers) {
            accumulate(&mut c.start_delay_secs, period);
        }
        for l in self.load_nodes.iter_mut().skip(mark.load_nodes) {
            accumulate(&mut l.start_delay_secs, period);
        }
        for n in self
            .context
            .captured_nodes_mut()
            .iter_mut()
            .skip(mark.captured_nodes)
        {
            accumulate(&mut n.start_delay_secs, period);
        }
        for c in self
            .context
            .captured_containers_mut()
            .iter_mut()
            .skip(mark.captured_containers)
        {
            accumulate(&mut c.start_delay_secs, period);
        }
        for l in self
            .context
            .captured_load_nodes_mut()
            .iter_mut()
            .skip(mark.captured_load_nodes)
        {
            accumulate(&mut l.start_delay_secs, period);
        }
    }

    /// Record an action this parser recognised as an action but does not
    /// implement. The warning stays (it is what a `RUST_LOG` reader expects),
    /// but the entry is what lets `check` refuse: a warning nobody reads is
    /// how `<timer>` dropped whole subtrees while `check` exited 0.
    ///
    /// Deduplicated per (action, file): a launch file with thirty `<log>`
    /// lines is one finding, not thirty.
    ///
    /// Only for an action stock `launch`/`launch_ros` expose
    /// ([`STOCK_FRONTEND_ACTIONS`]). Any other tag is refused, as
    /// `launch`'s frontend refuses it (`Unrecognized entity of the type`):
    /// either it is a typo, or a launch EXTENSION registers it — and an
    /// extension's action exists to start something (`<autoware_node>`,
    /// #0095), so dropping it loses nodes while `resolve` and `launch`
    /// exit 0.
    pub(crate) fn note_dropped_action(&mut self, action: &str) -> crate::error::Result<()> {
        if !STOCK_FRONTEND_ACTIONS.contains(&action) {
            return Err(crate::error::ParseError::UnknownAction {
                action: action.to_string(),
                file: self
                    .context
                    .current_file()
                    .map(|p| p.display().to_string())
                    .unwrap_or_else(|| "<unknown file>".to_string()),
            });
        }
        log::warn!("Unsupported action type: {action}");
        let entry = crate::record::DroppedAction {
            action: action.to_string(),
            file: self.context.current_file().map(|p| p.display().to_string()),
            detail: None,
        };
        self.note_dropped(entry);
        Ok(())
    }

    /// Merge one dropped-action entry (from this file or an included one).
    pub(crate) fn note_dropped(&mut self, entry: crate::record::DroppedAction) {
        if !self.dropped_actions.contains(&entry) {
            self.dropped_actions.push(entry);
        }
    }
}
