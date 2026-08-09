//! TOML parsing for the system scheduling document.

use crate::{resolve::SchedError, types::SystemSched};

/// Parse a system scheduling document from a TOML string.
pub fn parse_system_sched(input: &str) -> Result<SystemSched, SchedError> {
    toml::from_str(input).map_err(|e| SchedError::Parse(e.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_generic_head_and_platform_subtable() {
        let src = r#"
[tiers.control]
class = "real_time"
deadline_us = 50000
period_us = 20000

[tiers.control.posix]
priority = 80
sched_class = "SCHED_FIFO"
core = 1

[tiers.control.freertos]
priority = 12
stack_bytes = 8192
deadline_us = 40000

[[assign]]
tier = "control"
nodes = ["ndt_localizer", "ekf_localizer"]

[[assign]]
tier = "perception"
scope = "/perception/lidar"
"#;
        let sched = parse_system_sched(src).expect("must parse");
        let control = sched.tiers.get("control").expect("control tier");
        assert_eq!(control.class.as_deref(), Some("real_time"));
        assert_eq!(control.deadline_us, Some(50000));
        assert_eq!(control.platform("posix").unwrap().priority, 80);
        assert_eq!(
            control.platform("posix").unwrap().sched_class.as_deref(),
            Some("SCHED_FIFO")
        );
        assert_eq!(
            control.platform("freertos").unwrap().deadline_us,
            Some(40000)
        );
        assert_eq!(sched.assign.len(), 2);
        assert_eq!(
            sched.assign[0].nodes,
            vec!["ndt_localizer", "ekf_localizer"]
        );
        assert_eq!(sched.assign[1].scope.as_deref(), Some("/perception/lidar"));
    }

    #[test]
    fn priority_number_on_generic_head_is_rejected() {
        // deny_unknown_fields: a bare `priority` at the tier head must error —
        // this is what protects portability.
        let src = r#"
[tiers.control]
priority = 80
"#;
        let err = parse_system_sched(src).unwrap_err();
        assert!(matches!(err, SchedError::Parse(_)));
    }

    #[test]
    fn empty_document_is_valid_and_default() {
        let sched = parse_system_sched("").expect("empty parses");
        assert!(sched.tiers.is_empty());
        assert!(sched.assign.is_empty());
    }
}
