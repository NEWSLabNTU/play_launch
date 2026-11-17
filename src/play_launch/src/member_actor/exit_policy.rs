//! Which exits of a member end the launch (#0063, #0081, #0090).
//!
//! `on_exit=Shutdown()` (and its `OnProcessExit` handler spelling) ends the
//! launch on EVERY exit. An `on_exit` that is a function of the exit event
//! ends it only on the exit codes the parser found it returning a shutdown
//! for (`shutdown_on_exit_codes`). Either way the member does not respawn
//! from such an exit: launch skips a respawn once the launch is shutting
//! down, and the shutdown is what that exit starts.

use ros_launch_manifest_model::ExitCodes;
use ros_launch_resolve::ros::launch_dump::NodeRecord;

/// When a member's exit ends the launch.
#[derive(Debug, Clone, PartialEq)]
pub enum ExitPolicy {
    /// On every exit.
    Any,
    /// On these exit codes only.
    Codes(ExitCodes),
}

impl ExitPolicy {
    /// The policy a launch record declares, if any.
    pub fn of(record: &NodeRecord) -> Option<Self> {
        if record.on_exit_shutdown == Some(true) {
            return Some(ExitPolicy::Any);
        }
        record.shutdown_on_exit_codes.clone().map(ExitPolicy::Codes)
    }

    /// Whether an exit with this status ends the launch. The code is
    /// launch's `returncode`: the exit status, or `-N` for signal N. An exit
    /// whose status could not be read counts only for [`ExitPolicy::Any`].
    pub fn ends_launch(&self, exit_code: Option<i32>, signal: Option<i32>) -> bool {
        match self {
            ExitPolicy::Any => true,
            ExitPolicy::Codes(codes) => exit_code
                .or(signal.map(|s| -s))
                .is_some_and(|c| codes.contains(c)),
        }
    }

    /// How a log line names the reason.
    pub fn describe(&self) -> &'static str {
        match self {
            ExitPolicy::Any => "on_exit=Shutdown",
            ExitPolicy::Codes(_) => "an on_exit handler that shuts down on this exit code",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn codes_match_the_status_or_the_negated_signal() {
        let p = ExitPolicy::Codes(ExitCodes::Only { only: vec![-15, 2] });
        assert!(p.ends_launch(Some(2), None));
        assert!(p.ends_launch(None, Some(15)));
        assert!(!p.ends_launch(Some(1), None));
        assert!(!p.ends_launch(None, None));
        assert!(ExitPolicy::Any.ends_launch(None, None));
    }
}
