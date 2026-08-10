//! Read-only cgroup v2 cpuset inspection: can this process host
//! `SCHED_DEADLINE` reservations, and if not, exactly why?
//!
//! # Why this module mutates nothing
//!
//! `SCHED_DEADLINE` refuses an affinity mask narrower than the root domain the
//! thread was created on (`sched_setattr` returns `EPERM`), so confining a
//! reserved node to a CPU subset requires a restricted *root domain* — an
//! exclusive cgroup v2 cpuset partition. Three measured facts then decide the
//! whole shape of this module:
//!
//! 1. A partition only validates when its parent holds those CPUs
//!    exclusively. Nested under a systemd-managed slice (whose `cpuset.cpus`
//!    is blank, leaving `cpuset.cpus.exclusive.effective` empty) it always
//!    reads back `root invalid`. Only a **top-level** cgroup works.
//! 2. cgroup v2 migration requires write access to the *common ancestor* of
//!    source and destination. A top-level slice's common ancestor with a login
//!    session is the cgroup root, which no unprivileged user can write —
//!    and `clone3(CLONE_INTO_CGROUP)` inherits the same rule. So nothing can
//!    move itself in; a process must be **started** inside.
//! 3. Once it is started inside, there is nothing left to do. Children inherit
//!    the cgroup, so every spawned node already has `affinity == root domain`,
//!    which is precisely the precondition `SCHED_DEADLINE` wants.
//!
//! Creating a per-run sub-cgroup would add nothing and cost something: cgroup
//! v2 forbids a cgroup from holding processes *and* enabling controllers for
//! its children ("no internal processes"), so enabling `cpuset` in our own
//! cgroup would return `EBUSY` and force a `main/` sub-cgroup plus a
//! self-migration first. Provisioning is therefore the operator's job, done
//! out of band, and this module's entire contribution is to tell the truth
//! about the result.
//!
//! # The failure that motivates the readback check
//!
//! Writing `cpuset.cpus.partition = root` can **succeed** and still leave the
//! partition invalid — the kernel reports invalidity by changing what the file
//! reads back, not by failing the write. A measured probe wrote `root`, read
//! back `root invalid`, and `SCHED_DEADLINE` then *succeeded* — on the full
//! unrestricted root domain, with no isolation whatsoever. A partition that
//! fails open produces a system that looks reserved and is not, so the readback
//! is load-bearing rather than defensive.

use std::path::{Path, PathBuf};

/// Where the unified cgroup v2 hierarchy is mounted.
const CGROUP_ROOT: &str = "/sys/fs/cgroup";

/// What `cpuset.cpus.partition` says about a cgroup.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PartitionState {
    /// A valid partition root: it owns its CPUs exclusively.
    Root,
    /// The kernel accepted the write and rejected the result. The string is
    /// the kernel's own reason, e.g. `"root invalid (Invalid cpu list in
    /// cpuset.cpus.exclusive)"`.
    Invalid(String),
    /// An ordinary cgroup, sharing its parent's CPUs.
    Member,
    /// No `cpuset.cpus.partition` file — the `cpuset` controller is not
    /// enabled for this cgroup by its parent.
    NoCpusetController,
}

impl PartitionState {
    fn parse(raw: &str) -> Self {
        let raw = raw.trim();
        match raw {
            "root" => PartitionState::Root,
            "member" => PartitionState::Member,
            // "isolated" is a partition root that also disables load
            // balancing — strictly stronger than what a reservation needs, so
            // it counts as valid.
            "isolated" => PartitionState::Root,
            other if other.starts_with("root invalid") || other.starts_with("isolated invalid") => {
                PartitionState::Invalid(other.to_string())
            }
            other => PartitionState::Invalid(other.to_string()),
        }
    }

    pub fn is_valid_partition(&self) -> bool {
        matches!(self, PartitionState::Root)
    }
}

/// Whether this process can host `SCHED_DEADLINE` reservations.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ReservationReadiness {
    /// Inside a valid partition root. `cgroup` is the partition's path and
    /// `cpus` the CPUs it owns.
    Ready { cgroup: PathBuf, cpus: Vec<u32> },
    /// Not inside one. `reason` says which precondition failed, in terms the
    /// reader can act on.
    NotReady { reason: String },
}

impl ReservationReadiness {
    pub fn is_ready(&self) -> bool {
        matches!(self, ReservationReadiness::Ready { .. })
    }
}

/// This process's cgroup v2 path, as an absolute filesystem path.
///
/// `/proc/self/cgroup` under the unified hierarchy has exactly one line of the
/// form `0::/user.slice/...`. A v1-only system, or a hybrid one where the
/// process is in no v2 cgroup, yields `None`.
pub fn self_cgroup_dir() -> Option<PathBuf> {
    let content = std::fs::read_to_string("/proc/self/cgroup").ok()?;
    let rel = content
        .lines()
        .find_map(|line| line.strip_prefix("0::"))?
        .trim();
    // The path is absolute *relative to the hierarchy root*, so strip the
    // leading slash before joining or `Path::join` would discard the prefix.
    Some(Path::new(CGROUP_ROOT).join(rel.trim_start_matches('/')))
}

fn read_cgroup_file(dir: &Path, name: &str) -> Option<String> {
    std::fs::read_to_string(dir.join(name)).ok()
}

/// Read a cgroup's partition state.
pub fn partition_state(dir: &Path) -> PartitionState {
    match read_cgroup_file(dir, "cpuset.cpus.partition") {
        Some(raw) => PartitionState::parse(&raw),
        None => PartitionState::NoCpusetController,
    }
}

/// Parse a kernel CPU list — `"0-3,7"`, `"31"`, or empty.
///
/// Empty is `Some(vec![])`, not `None`: an empty `cpuset.cpus` means "inherit
/// from the parent", which is a real and different answer from "the file is
/// missing".
pub fn parse_cpu_list(raw: &str) -> Option<Vec<u32>> {
    let raw = raw.trim();
    if raw.is_empty() {
        return Some(Vec::new());
    }
    let mut cpus = Vec::new();
    for part in raw.split(',') {
        let part = part.trim();
        match part.split_once('-') {
            Some((lo, hi)) => {
                let lo: u32 = lo.trim().parse().ok()?;
                let hi: u32 = hi.trim().parse().ok()?;
                if lo > hi {
                    return None;
                }
                cpus.extend(lo..=hi);
            }
            None => cpus.push(part.parse().ok()?),
        }
    }
    cpus.sort_unstable();
    cpus.dedup();
    Some(cpus)
}

/// The CPUs a cgroup may actually run on.
pub fn effective_cpus(dir: &Path) -> Option<Vec<u32>> {
    parse_cpu_list(&read_cgroup_file(dir, "cpuset.cpus.effective")?)
}

/// Walk from this process's cgroup up to the hierarchy root, looking for a
/// valid partition root.
///
/// Ancestors are searched because being *inside* a partition is what matters,
/// not being *at* it: a process one level below a partition root is still in
/// that restricted root domain. An `Invalid` state found on the way is
/// reported immediately rather than skipped, since it is almost always the
/// thing the operator meant to create and needs to hear about.
pub fn reservation_readiness() -> ReservationReadiness {
    let Some(start) = self_cgroup_dir() else {
        return ReservationReadiness::NotReady {
            reason: "this process is not in a cgroup v2 hierarchy (no `0::` line in \
                     /proc/self/cgroup) — reservations need cgroup v2 unified mode"
                .to_string(),
        };
    };

    if !start.exists() {
        return ReservationReadiness::NotReady {
            reason: format!(
                "this process's cgroup ({}) is not readable at {CGROUP_ROOT} — is the unified \
                 hierarchy mounted there?",
                start.display()
            ),
        };
    }

    let root = Path::new(CGROUP_ROOT);
    let mut dir = start.clone();
    loop {
        match partition_state(&dir) {
            PartitionState::Root => {
                let cpus = effective_cpus(&dir).unwrap_or_default();
                return ReservationReadiness::Ready { cgroup: dir, cpus };
            }
            PartitionState::Invalid(reason) => {
                return ReservationReadiness::NotReady {
                    reason: format!(
                        "cgroup {} declares a partition but the kernel rejected it: `{reason}`. \
                         The write succeeded and the partition did NOT take effect — a task \
                         here would run on the full root domain with no isolation. A partition \
                         root must be a TOP-LEVEL cgroup (parent = the cgroup root); nested \
                         under a systemd-managed slice it is always invalid, because that \
                         slice's `cpuset.cpus` is blank and so it holds no CPUs exclusively.",
                        dir.display()
                    ),
                };
            }
            PartitionState::Member | PartitionState::NoCpusetController => {}
        }

        if dir == root {
            break;
        }
        match dir.parent() {
            Some(parent) if parent.starts_with(root) => dir = parent.to_path_buf(),
            _ => break,
        }
    }

    ReservationReadiness::NotReady {
        reason: format!(
            "no ancestor of this process's cgroup ({}) is a cpuset partition root. \
             SCHED_DEADLINE refuses an affinity mask narrower than its root domain, so a \
             reserved node must be STARTED inside a partition — it cannot migrate into one, \
             because cgroup v2 requires write access to the common ancestor and a top-level \
             slice's common ancestor with a login session is the cgroup root. Provision a \
             top-level partition as root and launch play_launch inside it.",
            start.display()
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cpu_lists_parse_in_every_kernel_spelling() {
        assert_eq!(parse_cpu_list("31"), Some(vec![31]));
        assert_eq!(parse_cpu_list("0-3"), Some(vec![0, 1, 2, 3]));
        assert_eq!(parse_cpu_list("0-1,4,6-7"), Some(vec![0, 1, 4, 6, 7]));
        assert_eq!(parse_cpu_list(" 2 , 3 "), Some(vec![2, 3]));
        assert_eq!(parse_cpu_list("0-31\n"), Some((0..=31).collect::<Vec<_>>()));
        // Empty is "inherit from parent" — a real answer, distinct from a
        // missing file, which is why this is Some(vec![]) and not None.
        assert_eq!(parse_cpu_list(""), Some(Vec::new()));
        assert_eq!(parse_cpu_list("   "), Some(Vec::new()));

        assert_eq!(parse_cpu_list("garbage"), None);
        assert_eq!(parse_cpu_list("5-1"), None, "reversed range is not a range");
        assert_eq!(parse_cpu_list("1,,2"), None);
    }

    #[test]
    fn partition_states_parse() {
        assert_eq!(PartitionState::parse("root\n"), PartitionState::Root);
        assert_eq!(PartitionState::parse("member"), PartitionState::Member);
        // `isolated` is a partition root that also disables load balancing —
        // stronger than a reservation needs, so it counts as valid.
        assert_eq!(PartitionState::parse("isolated"), PartitionState::Root);
        assert!(PartitionState::parse("root").is_valid_partition());
        assert!(!PartitionState::parse("member").is_valid_partition());
    }

    #[test]
    fn an_invalid_partition_is_never_mistaken_for_a_valid_one() {
        // This is the measured failure mode: the write succeeds, the readback
        // says invalid, and SCHED_DEADLINE then works on the FULL root domain
        // with no isolation. Treating this as valid would ship a system that
        // reports "reserved" and is not.
        let state =
            PartitionState::parse("root invalid (Invalid cpu list in cpuset.cpus.exclusive)");
        assert!(!state.is_valid_partition());
        let PartitionState::Invalid(reason) = &state else {
            panic!("expected Invalid, got {state:?}");
        };
        assert!(reason.contains("Invalid cpu list"), "{reason}");

        assert!(!PartitionState::parse("isolated invalid").is_valid_partition());
        // Anything unrecognized is invalid too — failing closed, because the
        // consequence of guessing "valid" is silent loss of isolation.
        assert!(!PartitionState::parse("something new").is_valid_partition());
    }

    #[test]
    fn readiness_explains_itself_on_an_unprovisioned_host() {
        // On any ordinary developer machine no ancestor is a partition root,
        // and the diagnostic is the entire deliverable of this module: it has
        // to say what is missing and why migration is not an option.
        let readiness = reservation_readiness();
        match readiness {
            ReservationReadiness::Ready { cpus, .. } => {
                assert!(!cpus.is_empty(), "a valid partition must own some CPUs");
            }
            ReservationReadiness::NotReady { reason } => {
                assert!(
                    reason.contains("partition"),
                    "the reason must name what is missing: {reason}"
                );
                assert!(
                    reason.len() > 80,
                    "a one-liner is indistinguishable from a broken feature: {reason}"
                );
            }
        }
    }

    #[test]
    fn self_cgroup_dir_is_under_the_hierarchy_root() {
        // Skipped rather than failed on a non-cgroup2 host.
        let Some(dir) = self_cgroup_dir() else {
            eprintln!("skipping self_cgroup_dir_is_under_the_hierarchy_root: no cgroup v2");
            return;
        };
        assert!(
            dir.starts_with(CGROUP_ROOT),
            "{} must be under {CGROUP_ROOT}",
            dir.display()
        );
    }
}
