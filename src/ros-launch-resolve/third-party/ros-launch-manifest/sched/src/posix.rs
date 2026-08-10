//! Typed `posix` (Linux) scheduling placement.
//!
//! Replaces the stringly-typed trio (`sched_class: Option<String>`,
//! `priority: i64`, `core: Option<u32>`) that every consumer had to
//! re-interpret for itself. Three problems that caused, all of them real:
//!
//! - **Illegal states were representable.** A tier could carry
//!   `sched_class: "SCHED_OTHER"` next to `priority: 10` — a combination
//!   Linux cannot express, and one that was written into `system_model.yaml`
//!   and treated as fatal by strict mode.
//! - **Every new knob was another loose `Option`.** `ResolvedTier` already
//!   carries thirteen fields mixing a portable head with `posix` placement.
//! - **Unknown values failed open.** A `sched_class` typo silently became
//!   `SCHED_OTHER`, dropping a node out of real-time with no diagnostic.
//!
//! The types here are **additive**: `sched_class`/`priority`/`core` remain on
//! [`crate::ResolvedTier`] and [`crate::TierPlatformSpec`] for one release so
//! consumers can migrate without a lockstep bump.

use serde::{Deserialize, Serialize};

/// Errors building or validating a [`PosixPlacement`].
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum PosixError {
    #[error(
        "unknown POSIX scheduling class {value:?}; expected one of \
         SCHED_OTHER, SCHED_BATCH, SCHED_IDLE, SCHED_FIFO, SCHED_RR, SCHED_DEADLINE"
    )]
    UnknownSchedClass { value: String },
    #[error(
        "SCHED_DEADLINE cannot use an explicit CPU mask: a deadline thread's \
         affinity may not be narrower than the root domain it was created on, \
         so sched_setaffinity(2) returns EPERM. Use an exclusive cpuset \
         partition instead."
    )]
    DeadlineWithCpuMask,
    #[error("real-time priority {priority} is out of range {min}..={max}")]
    PriorityOutOfRange { priority: i64, min: i64, max: i64 },
    #[error("nice value {nice} is out of range -20..=19")]
    NiceOutOfRange { nice: i64 },
    #[error("uclamp range {min}..={max} is invalid (need min <= max <= 1024)")]
    UclampOutOfRange { min: u32, max: u32 },
    #[error("an empty CPU mask names no CPU to run on")]
    EmptyCpuMask,
    #[error(
        "SCHED_DEADLINE requires runtime <= deadline <= period, got {runtime_ns}/{deadline_ns}/{period_ns} ns"
    )]
    DeadlineParamsUnordered {
        runtime_ns: u64,
        deadline_ns: u64,
        period_ns: u64,
    },
}

/// The largest legal utilization-clamp value. The kernel's scale is 0..=1024,
/// not a percentage.
pub const UCLAMP_MAX: u32 = 1024;

/// Which Linux scheduling policy, naming only the parameters that policy
/// actually has.
///
/// The shape is the point: `Batch` has no priority because Linux has no such
/// thing, `Fifo` has no nice value, and `Deadline` has neither — it carries a
/// reservation instead. None of those combinations can be constructed wrongly.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "policy", rename_all = "snake_case")]
pub enum PosixSched {
    /// `SCHED_IDLE` — runs only when nothing else wants the CPU.
    Idle,
    /// `SCHED_BATCH` — CFS without the interactivity bonus, so it preempts
    /// less. For throughput work with no latency requirement.
    Batch { nice: i32 },
    /// `SCHED_OTHER` — the default CFS policy.
    Other { nice: i32 },
    /// `SCHED_FIFO` — fixed priority, runs until it blocks or is preempted.
    Fifo { priority: i32 },
    /// `SCHED_RR` — `SCHED_FIFO` plus a time slice among equal priorities.
    /// Note the slice is a global sysctl on Linux, not a per-task value.
    Rr { priority: i32 },
    /// `SCHED_DEADLINE` — a CBS reservation of `runtime` every `period`, to be
    /// completed within `deadline`. Carries no priority: deadline threads
    /// preempt every fixed-priority thread regardless of RT priority.
    Deadline {
        runtime_ns: u64,
        deadline_ns: u64,
        period_ns: u64,
        /// `SCHED_FLAG_DL_OVERRUN` — deliver `SIGXCPU` when the reservation is
        /// exceeded. Set from `deadline_policy: fault`.
        overrun: bool,
    },
}

/// The six values `sched_class` may legally take, parsed rather than guessed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PosixPolicyKind {
    Other,
    Batch,
    Idle,
    Fifo,
    Rr,
    Deadline,
}

impl PosixPolicyKind {
    /// Parse a `sched_class` string, **failing closed**.
    ///
    /// The previous behaviour mapped anything unrecognized to `SCHED_OTHER`,
    /// so `SCHED_FIF0` quietly dropped a node out of real-time. A scheduling
    /// declaration is exactly the kind of thing that must not fail open.
    pub fn parse(value: &str) -> Result<Self, PosixError> {
        match value.trim() {
            "SCHED_OTHER" | "SCHED_NORMAL" => Ok(PosixPolicyKind::Other),
            "SCHED_BATCH" => Ok(PosixPolicyKind::Batch),
            "SCHED_IDLE" => Ok(PosixPolicyKind::Idle),
            "SCHED_FIFO" => Ok(PosixPolicyKind::Fifo),
            "SCHED_RR" => Ok(PosixPolicyKind::Rr),
            "SCHED_DEADLINE" => Ok(PosixPolicyKind::Deadline),
            other => Err(PosixError::UnknownSchedClass {
                value: other.to_string(),
            }),
        }
    }

    /// The canonical spelling, for round-tripping into `system_model.yaml`.
    pub fn as_str(self) -> &'static str {
        match self {
            PosixPolicyKind::Other => "SCHED_OTHER",
            PosixPolicyKind::Batch => "SCHED_BATCH",
            PosixPolicyKind::Idle => "SCHED_IDLE",
            PosixPolicyKind::Fifo => "SCHED_FIFO",
            PosixPolicyKind::Rr => "SCHED_RR",
            PosixPolicyKind::Deadline => "SCHED_DEADLINE",
        }
    }

    /// Is this one of the real-time policies (`SCHED_FIFO`/`RR`/`DEADLINE`)?
    ///
    /// This is what decides whether `class: real_time` is written into the
    /// model's execution layer, and what `band_violations` uses to skip tiers
    /// that have no RT priority to check.
    pub fn is_real_time(self) -> bool {
        matches!(
            self,
            PosixPolicyKind::Fifo | PosixPolicyKind::Rr | PosixPolicyKind::Deadline
        )
    }
}

impl PosixSched {
    pub fn kind(&self) -> PosixPolicyKind {
        match self {
            PosixSched::Idle => PosixPolicyKind::Idle,
            PosixSched::Batch { .. } => PosixPolicyKind::Batch,
            PosixSched::Other { .. } => PosixPolicyKind::Other,
            PosixSched::Fifo { .. } => PosixPolicyKind::Fifo,
            PosixSched::Rr { .. } => PosixPolicyKind::Rr,
            PosixSched::Deadline { .. } => PosixPolicyKind::Deadline,
        }
    }

    /// The fixed priority, for the two policies that have one.
    ///
    /// `None` for `SCHED_DEADLINE` specifically because it has no priority at
    /// all — a deadline thread preempts every fixed-priority thread — which is
    /// why band checks must skip it rather than compare it as zero.
    pub fn priority(&self) -> Option<i32> {
        match self {
            PosixSched::Fifo { priority } | PosixSched::Rr { priority } => Some(*priority),
            _ => None,
        }
    }

    pub fn is_real_time(&self) -> bool {
        self.kind().is_real_time()
    }

    /// Does this policy require `SCHED_FLAG_RESET_ON_FORK`?
    ///
    /// Only `SCHED_DEADLINE`, whose threads the kernel refuses to `fork(2)`
    /// from without it (`EAGAIN`). Deliberately NOT true for `SCHED_FIFO`/`RR`:
    /// the kernel resets scheduling in `sched_fork()`, which runs for *thread*
    /// creation as well, so the flag would stop threads created after an apply
    /// sweep from inheriting the policy — leaving an arbitrary subset of a
    /// node's threads at `SCHED_OTHER`. Measured, not theorised.
    ///
    /// Expressed as a method rather than a field so it cannot be set wrongly.
    pub fn requires_reset_on_fork(&self) -> bool {
        matches!(self, PosixSched::Deadline { .. })
    }
}

/// How a placement constrains which CPUs a node may run on.
///
/// An enum rather than two optional fields because the two mechanisms are
/// mutually exclusive *and* policy-dependent: `SCHED_DEADLINE` may not use an
/// affinity mask at all.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum PosixAffinity {
    /// Leave affinity alone.
    #[default]
    Inherit,
    /// An explicit CPU mask, applied with `sched_setaffinity(2)`.
    Cpus { cpus: Vec<u32> },
    /// Membership of a cgroup v2 cpuset partition, named by path. The only
    /// form legal for `SCHED_DEADLINE`, which needs a restricted *root
    /// domain* rather than a narrowed mask.
    Cpuset { path: String },
}

/// A complete `posix` placement for one tier.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PosixPlacement {
    pub sched: PosixSched,
    #[serde(default)]
    pub affinity: PosixAffinity,
    /// Utilization clamp, `(min, max)` on the kernel's 0..=1024 scale.
    ///
    /// Note `min` is a no-op on RT policies, which already default to
    /// `1024`/`1024` and therefore already request the maximum performance
    /// point under `schedutil`. The useful RT knob is `max`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub uclamp: Option<(u32, u32)>,
}

impl PosixPlacement {
    /// Build and validate in one step. Prefer this over struct literals so no
    /// caller can skip [`PosixPlacement::validate`].
    pub fn new(sched: PosixSched, affinity: PosixAffinity) -> Result<Self, PosixError> {
        let p = PosixPlacement {
            sched,
            affinity,
            uclamp: None,
        };
        p.validate()?;
        Ok(p)
    }

    pub fn with_uclamp(mut self, min: u32, max: u32) -> Result<Self, PosixError> {
        self.uclamp = Some((min, max));
        self.validate()?;
        Ok(self)
    }

    /// Reject the combinations Linux cannot execute.
    pub fn validate(&self) -> Result<(), PosixError> {
        match &self.sched {
            PosixSched::Fifo { priority } | PosixSched::Rr { priority } => {
                let p = *priority as i64;
                if !(crate::platform::POSIX_RT_PRIORITY_MIN
                    ..=crate::platform::POSIX_RT_PRIORITY_MAX)
                    .contains(&p)
                {
                    return Err(PosixError::PriorityOutOfRange {
                        priority: p,
                        min: crate::platform::POSIX_RT_PRIORITY_MIN,
                        max: crate::platform::POSIX_RT_PRIORITY_MAX,
                    });
                }
            }
            PosixSched::Other { nice } | PosixSched::Batch { nice } => {
                if !(-20..=19).contains(nice) {
                    return Err(PosixError::NiceOutOfRange { nice: *nice as i64 });
                }
            }
            PosixSched::Deadline {
                runtime_ns,
                deadline_ns,
                period_ns,
                ..
            } => {
                // The kernel's own admission requirement. Checking it here
                // turns a late EINVAL from sched_setattr into a spec error
                // naming the node.
                if !(runtime_ns <= deadline_ns && deadline_ns <= period_ns) {
                    return Err(PosixError::DeadlineParamsUnordered {
                        runtime_ns: *runtime_ns,
                        deadline_ns: *deadline_ns,
                        period_ns: *period_ns,
                    });
                }
            }
            PosixSched::Idle => {}
        }

        match &self.affinity {
            PosixAffinity::Cpus { cpus } => {
                if cpus.is_empty() {
                    return Err(PosixError::EmptyCpuMask);
                }
                if matches!(self.sched, PosixSched::Deadline { .. }) {
                    return Err(PosixError::DeadlineWithCpuMask);
                }
            }
            PosixAffinity::Cpuset { .. } | PosixAffinity::Inherit => {}
        }

        if let Some((min, max)) = self.uclamp
            && (min > max || max > UCLAMP_MAX)
        {
            return Err(PosixError::UclampOutOfRange { min, max });
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sched_class_parsing_fails_closed() {
        assert_eq!(
            PosixPolicyKind::parse("SCHED_FIFO").unwrap(),
            PosixPolicyKind::Fifo
        );
        assert_eq!(
            PosixPolicyKind::parse(" SCHED_DEADLINE ").unwrap(),
            PosixPolicyKind::Deadline
        );
        // The old behaviour silently returned SCHED_OTHER here, dropping a
        // node out of real-time with no diagnostic at all.
        let err = PosixPolicyKind::parse("SCHED_FIF0").unwrap_err();
        assert_eq!(
            err,
            PosixError::UnknownSchedClass {
                value: "SCHED_FIF0".to_string()
            }
        );
        assert!(err.to_string().contains("SCHED_FIFO"), "{err}");
    }

    #[test]
    fn every_policy_round_trips_through_its_canonical_spelling() {
        for kind in [
            PosixPolicyKind::Other,
            PosixPolicyKind::Batch,
            PosixPolicyKind::Idle,
            PosixPolicyKind::Fifo,
            PosixPolicyKind::Rr,
            PosixPolicyKind::Deadline,
        ] {
            assert_eq!(PosixPolicyKind::parse(kind.as_str()).unwrap(), kind);
        }
    }

    #[test]
    fn deadline_has_no_priority_and_is_real_time() {
        let dl = PosixSched::Deadline {
            runtime_ns: 1,
            deadline_ns: 2,
            period_ns: 3,
            overrun: false,
        };
        // Not zero — absent. A band check that read this as 0 would flag a
        // spurious violation, which is the bug that shipped once already.
        assert_eq!(dl.priority(), None);
        assert!(dl.is_real_time());
        assert_eq!(PosixSched::Fifo { priority: 40 }.priority(), Some(40));
        assert_eq!(PosixSched::Batch { nice: 10 }.priority(), None);
        assert!(!PosixSched::Batch { nice: 10 }.is_real_time());
        assert!(!PosixSched::Idle.is_real_time());
    }

    #[test]
    fn only_deadline_requires_reset_on_fork() {
        assert!(
            PosixSched::Deadline {
                runtime_ns: 1,
                deadline_ns: 2,
                period_ns: 3,
                overrun: false,
            }
            .requires_reset_on_fork()
        );
        // Setting it on FIFO defeats PTHREAD_INHERIT_SCHED for threads created
        // after an apply sweep — measured, and the reason this is a method
        // rather than a settable field.
        assert!(!PosixSched::Fifo { priority: 40 }.requires_reset_on_fork());
        assert!(!PosixSched::Rr { priority: 40 }.requires_reset_on_fork());
        assert!(!PosixSched::Other { nice: 0 }.requires_reset_on_fork());
    }

    #[test]
    fn deadline_with_a_cpu_mask_is_rejected() {
        let err = PosixPlacement::new(
            PosixSched::Deadline {
                runtime_ns: 1_000_000,
                deadline_ns: 10_000_000,
                period_ns: 10_000_000,
                overrun: true,
            },
            PosixAffinity::Cpus { cpus: vec![2, 3] },
        )
        .unwrap_err();
        assert_eq!(err, PosixError::DeadlineWithCpuMask);
        assert!(err.to_string().contains("EPERM"), "{err}");
    }

    #[test]
    fn deadline_in_a_cpuset_is_accepted() {
        PosixPlacement::new(
            PosixSched::Deadline {
                runtime_ns: 8_000_000,
                deadline_ns: 100_000_000,
                period_ns: 100_000_000,
                overrun: true,
            },
            PosixAffinity::Cpuset {
                path: "/sys/fs/cgroup/play_launch.slice/rt-1".to_string(),
            },
        )
        .expect("a cpuset is the legal way to give a deadline task a CPU subset");
    }

    #[test]
    fn deadline_params_must_be_ordered() {
        let err = PosixPlacement::new(
            PosixSched::Deadline {
                runtime_ns: 20_000_000,
                deadline_ns: 10_000_000,
                period_ns: 10_000_000,
                overrun: false,
            },
            PosixAffinity::Inherit,
        )
        .unwrap_err();
        assert!(matches!(err, PosixError::DeadlineParamsUnordered { .. }));
    }

    #[test]
    fn out_of_range_priority_nice_and_uclamp_are_rejected() {
        assert!(matches!(
            PosixPlacement::new(PosixSched::Fifo { priority: 0 }, PosixAffinity::Inherit)
                .unwrap_err(),
            PosixError::PriorityOutOfRange { .. }
        ));
        assert!(matches!(
            PosixPlacement::new(PosixSched::Rr { priority: 100 }, PosixAffinity::Inherit)
                .unwrap_err(),
            PosixError::PriorityOutOfRange { .. }
        ));
        assert!(matches!(
            PosixPlacement::new(PosixSched::Batch { nice: 20 }, PosixAffinity::Inherit)
                .unwrap_err(),
            PosixError::NiceOutOfRange { .. }
        ));
        assert!(matches!(
            PosixPlacement::new(PosixSched::Other { nice: 0 }, PosixAffinity::Inherit)
                .unwrap()
                .with_uclamp(0, 2000)
                .unwrap_err(),
            PosixError::UclampOutOfRange { .. }
        ));
        assert!(matches!(
            PosixPlacement::new(PosixSched::Other { nice: 0 }, PosixAffinity::Inherit)
                .unwrap()
                .with_uclamp(900, 100)
                .unwrap_err(),
            PosixError::UclampOutOfRange { .. }
        ));
    }

    #[test]
    fn empty_cpu_mask_is_rejected() {
        assert_eq!(
            PosixPlacement::new(
                PosixSched::Fifo { priority: 40 },
                PosixAffinity::Cpus { cpus: vec![] }
            )
            .unwrap_err(),
            PosixError::EmptyCpuMask
        );
    }

    #[test]
    fn placement_round_trips_through_yaml() {
        let p = PosixPlacement::new(
            PosixSched::Fifo { priority: 40 },
            PosixAffinity::Cpus { cpus: vec![2, 3] },
        )
        .unwrap()
        .with_uclamp(0, 1024)
        .unwrap();
        let yaml = serde_yaml_ng::to_string(&p).expect("serialize");
        let back: PosixPlacement = serde_yaml_ng::from_str(&yaml).expect("deserialize");
        assert_eq!(p, back);
    }
}
