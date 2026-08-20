use dashmap::DashMap;
use serde::{Deserialize, Serialize};
use std::{
    collections::HashMap,
    sync::Arc,
    time::{Duration, SystemTime},
};
use tokio::sync::broadcast;

/// Capacity of the change-notification channel.
///
/// The payload is a bare ping, not the data — a lagged subscriber has missed
/// nothing it cannot recover by re-reading the registry, so this only needs to
/// be large enough that bursts do not spam the lag path.
const CHANGE_CHANNEL_CAPACITY: usize = 16;

/// Diagnostic level matching ROS2 diagnostic_msgs/DiagnosticStatus
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum DiagnosticLevel {
    Ok = 0,
    Warning = 1,
    Error = 2,
    Stale = 3,
}

impl DiagnosticLevel {
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => DiagnosticLevel::Ok,
            1 => DiagnosticLevel::Warning,
            2 => DiagnosticLevel::Error,
            3 => DiagnosticLevel::Stale,
            _ => DiagnosticLevel::Stale, // Default to STALE for unknown values
        }
    }

    /// Ordering for "worst level", which is NOT the numeric wire value.
    ///
    /// `diagnostic_msgs` numbers STALE as 3, above ERROR at 2, but a stale
    /// diagnostic is not worse news than a failing one: ERROR is a fault the
    /// system reported, STALE is the absence of a report. Ranking STALE above
    /// ERROR would let a silent publisher mask a live fault on the same node.
    pub fn severity(&self) -> u8 {
        match self {
            DiagnosticLevel::Ok => 0,
            DiagnosticLevel::Stale => 1,
            DiagnosticLevel::Warning => 2,
            DiagnosticLevel::Error => 3,
        }
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            DiagnosticLevel::Ok => "OK",
            DiagnosticLevel::Warning => "WARNING",
            DiagnosticLevel::Error => "ERROR",
            DiagnosticLevel::Stale => "STALE",
        }
    }
}

/// A single diagnostic status message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiagnosticStatus {
    pub hardware_id: String,
    pub name: String,
    pub level: DiagnosticLevel,
    pub message: String,
    pub values: HashMap<String, String>,
    #[serde(with = "humantime_serde")]
    pub timestamp: SystemTime,
}

impl DiagnosticStatus {
    pub fn key(&self) -> String {
        format!("{}/{}", self.hardware_id, self.name)
    }
}

/// Diagnostic counts by level
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct DiagnosticCounts {
    pub ok: usize,
    pub warning: usize,
    pub error: usize,
    pub stale: usize,
    pub total: usize,
}

/// Registry for storing and accessing diagnostic data.
///
/// Holds the latest status per `hardware_id/name` and nothing else. It
/// deliberately keeps no history: `DiagnosticCsvWriter` already writes every
/// accepted status to `diagnostics.csv`, which is the durable record and
/// survives the process. An in-memory `Vec` alongside it grew without bound for
/// the life of a run — one four-hour session on the golf cart logged 405,480
/// statuses, each carrying three `String`s and a `HashMap` of values — and the
/// only reader of that `Vec` was a `get_history` with no callers. Read the CSV.
#[derive(Clone)]
pub struct DiagnosticRegistry {
    // Latest status for each diagnostic (keyed by "hardware_id/name")
    diagnostics: Arc<DashMap<String, DiagnosticStatus>>,
    /// How long a diagnostic may go without an update before it reads STALE.
    /// `None` disables ageing. See [`Self::age`].
    stale_after: Option<Duration>,
    /// Pinged whenever a diagnostic's level changes, so SSE clients can be
    /// pushed to instead of polling. Carries no payload on purpose: the
    /// receiver re-reads the registry, which is always current and cannot be
    /// delivered out of order.
    changes: broadcast::Sender<()>,
}

impl DiagnosticRegistry {
    pub fn new(stale_after: Option<Duration>) -> Self {
        let (changes, _) = broadcast::channel(CHANGE_CHANNEL_CAPACITY);
        Self {
            diagnostics: Arc::new(DashMap::new()),
            stale_after,
            changes,
        }
    }

    /// Subscribe to level-change notifications.
    pub fn subscribe(&self) -> broadcast::Receiver<()> {
        self.changes.subscribe()
    }

    /// Update or insert a diagnostic status.
    ///
    /// Returns whether this changed the diagnostic's level, which the caller
    /// uses to decide whether the update may bypass rate limiting.
    pub fn update(&self, status: DiagnosticStatus) -> bool {
        let key = status.key();
        let level = status.level;

        let previous = self.diagnostics.insert(key, status);
        let changed = previous.map(|p| p.level) != Some(level);

        if changed {
            // Err only means nobody is subscribed.
            let _ = self.changes.send(());
        }

        changed
    }

    /// Apply the staleness rule to a status as it is read.
    ///
    /// A publisher that dies stops publishing; it does not announce level 3.
    /// Without this, its last status sits in the registry reading OK forever,
    /// so a crashed node and a healthy one are indistinguishable — across that
    /// same 405,480-status run, not one status arrived at STALE or WARNING.
    ///
    /// Ageing happens on read rather than in a sweeping task so that there is
    /// no window in which the stored value and the reported value disagree, and
    /// no second writer contending for the map. The cost is that a diagnostic
    /// going stale raises no change notification, which is why the SSE stream
    /// also ticks on a timer.
    fn age(&self, status: &DiagnosticStatus, now: SystemTime) -> DiagnosticStatus {
        let mut out = status.clone();
        if let Some(limit) = self.stale_after
            && out.level != DiagnosticLevel::Stale
            && now
                .duration_since(status.timestamp)
                .is_ok_and(|elapsed| elapsed > limit)
        {
            out.level = DiagnosticLevel::Stale;
        }
        out
    }

    /// Get all current diagnostics (latest status for each), aged.
    pub fn list_all(&self) -> Vec<DiagnosticStatus> {
        let now = SystemTime::now();
        self.diagnostics
            .iter()
            .map(|entry| self.age(entry.value(), now))
            .collect()
    }

    /// Get diagnostic counts by level, aged.
    /// Worst level per attributed member, for the node-card badges
    /// (phase 62 W2), plus the diagnostics that matched no member.
    ///
    /// Derived from `list_all`, which is the AGED view: staleness is applied
    /// on read, so a member whose diagnostics went silent badges stale rather
    /// than keeping the green it last published. That equivalence is the point
    /// — a badge computed from a different view than the table could disagree
    /// with it, and the table is what an operator checks second.
    pub fn badges(
        &self,
        attributor: &crate::diagnostics::attribution::Attributor,
    ) -> (std::collections::HashMap<String, DiagnosticLevel>, Vec<DiagnosticStatus>) {
        let mut worst: std::collections::HashMap<String, DiagnosticLevel> =
            std::collections::HashMap::new();
        let mut unmatched = Vec::new();
        for status in self.list_all() {
            match attributor.attribute(&status.name) {
                (Some(member), _) => {
                    worst
                        .entry(member.to_string())
                        .and_modify(|l| {
                            if status.level.severity() > l.severity() {
                                *l = status.level;
                            }
                        })
                        .or_insert(status.level);
                }
                (None, _) => unmatched.push(status),
            }
        }
        (worst, unmatched)
    }

    pub fn get_counts(&self) -> DiagnosticCounts {
        let now = SystemTime::now();
        let mut counts = DiagnosticCounts::default();

        for entry in self.diagnostics.iter() {
            counts.total += 1;
            match self.age(entry.value(), now).level {
                DiagnosticLevel::Ok => counts.ok += 1,
                DiagnosticLevel::Warning => counts.warning += 1,
                DiagnosticLevel::Error => counts.error += 1,
                DiagnosticLevel::Stale => counts.stale += 1,
            }
        }

        counts
    }
}

impl Default for DiagnosticRegistry {
    fn default() -> Self {
        Self::new(None)
    }
}

#[cfg(test)]
mod tests {

    /// STALE must not outrank ERROR, despite being the higher wire value.
    ///
    /// The failure this pins: a node with a live ERROR and a silent publisher
    /// badging "stale" and hiding the fault. `diagnostic_msgs` numbers STALE 3
    /// and ERROR 2, so the obvious `max()` on the wire value is wrong.
    #[test]
    fn error_outranks_stale_in_the_badge() {
        use crate::diagnostics::attribution::Attributor;
        let reg = DiagnosticRegistry::new(None);
        for (name, level) in [
            ("talker: heartbeat", DiagnosticLevel::Stale),
            ("talker: link", DiagnosticLevel::Error),
        ] {
            reg.update(DiagnosticStatus {
                hardware_id: "hw".into(),
                name: name.into(),
                level,
                message: String::new(),
                values: Default::default(),
                timestamp: std::time::SystemTime::now(),
            });
        }
        let a = Attributor::new(["/ns/talker"]);
        let (worst, unmatched) = reg.badges(&a);
        assert_eq!(worst.get("/ns/talker"), Some(&DiagnosticLevel::Error));
        assert!(unmatched.is_empty());
    }

    /// A diagnostic naming no member lands in the unmatched bucket rather than
    /// being dropped or attached to something plausible.
    #[test]
    fn unmatched_diagnostics_are_returned_not_discarded() {
        use crate::diagnostics::attribution::Attributor;
        let reg = DiagnosticRegistry::new(None);
        reg.update(DiagnosticStatus {
            hardware_id: "hw".into(),
            name: "vehicle_interface/can_tx".into(),
            level: DiagnosticLevel::Error,
            message: String::new(),
            values: Default::default(),
            timestamp: std::time::SystemTime::now(),
        });
        let a = Attributor::new(["/ns/talker"]);
        let (worst, unmatched) = reg.badges(&a);
        assert!(worst.is_empty(), "must not guess a member");
        assert_eq!(unmatched.len(), 1);
        assert_eq!(unmatched[0].name, "vehicle_interface/can_tx");
    }

    use super::*;

    fn status(name: &str, level: DiagnosticLevel, timestamp: SystemTime) -> DiagnosticStatus {
        DiagnosticStatus {
            hardware_id: "hw".to_string(),
            name: name.to_string(),
            level,
            message: String::new(),
            values: HashMap::new(),
            timestamp,
        }
    }

    /// A publisher that stops publishing must stop reading OK.
    ///
    /// Without ageing, a node that crashed is indistinguishable from one that
    /// is fine, because level 3 only ever arrives from a publisher that is
    /// still alive enough to declare it.
    #[test]
    fn a_diagnostic_that_stops_updating_reads_stale() {
        let registry = DiagnosticRegistry::new(Some(Duration::from_secs(10)));
        let long_ago = SystemTime::now() - Duration::from_secs(30);

        registry.update(status("fresh", DiagnosticLevel::Ok, SystemTime::now()));
        registry.update(status("dead", DiagnosticLevel::Ok, long_ago));

        let counts = registry.get_counts();
        assert_eq!(counts.total, 2);
        assert_eq!(counts.ok, 1, "the fresh one stays OK");
        assert_eq!(counts.stale, 1, "the silent one ages out");

        let dead = registry
            .list_all()
            .into_iter()
            .find(|s| s.name == "dead")
            .expect("still listed");
        assert_eq!(dead.level, DiagnosticLevel::Stale);
    }

    /// Ageing is opt-in: with no limit configured, nothing is rewritten.
    #[test]
    fn ageing_is_off_when_unconfigured() {
        let registry = DiagnosticRegistry::new(None);
        registry.update(status(
            "ancient",
            DiagnosticLevel::Ok,
            SystemTime::now() - Duration::from_secs(86_400),
        ));
        assert_eq!(registry.get_counts().ok, 1);
    }

    /// `update` reports level transitions, and only transitions — that is the
    /// signal both the debounce bypass and the SSE push are built on.
    #[test]
    fn update_reports_only_level_transitions() {
        let registry = DiagnosticRegistry::new(None);
        let now = SystemTime::now();

        assert!(
            registry.update(status("a", DiagnosticLevel::Ok, now)),
            "first sighting is a change"
        );
        assert!(
            !registry.update(status("a", DiagnosticLevel::Ok, now)),
            "same level again is not"
        );
        assert!(
            registry.update(status("a", DiagnosticLevel::Error, now)),
            "OK -> ERROR is"
        );
        assert!(
            registry.update(status("a", DiagnosticLevel::Ok, now)),
            "and so is ERROR -> OK"
        );
    }
}
