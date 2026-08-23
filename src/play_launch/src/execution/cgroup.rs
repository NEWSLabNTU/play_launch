//! One cgroup v2 group per container, per node — phase 66 W1.
//!
//! `--container-mode isolated` forks a process per composable to buy fault
//! isolation, and in doing so has already given up everything that makes a ROS
//! container a container: shared address space, shared executor, one DDS
//! participant. cgroup v2 returns the *resource and lifecycle* half to a set of
//! separate processes.
//!
//! Three facts shape this module, each measured rather than read from
//! documentation (probes in `tmp/`, design in
//! `docs/design/cgroup-per-container.md`):
//!
//! 1. **A composable inherits its container's group at `fork()`.** play_launch
//!    never spawns a composable — `prepare_composable_node_contexts_from_model`
//!    builds LoadNode *requests*, and the C++ container fork+execs
//!    `component_node`. So placing the container process is enough to place
//!    every composable in it, and `cgroup.kill` reaches those grandchildren.
//!    Per-container grouping therefore costs zero changes to
//!    `play_launch_container`.
//!
//! 2. **Migration into a sibling group is `EPERM`; placement at birth is not.**
//!    cgroup v2 requires write access to the *common ancestor* of source and
//!    destination — the rule [`super::cpuset`] already documents for the RT
//!    partition. So the child places itself in `pre_exec`, and play_launch
//!    never moves a running process. It must also never move *itself* per
//!    spawn: a write to `cgroup.procs` moves every thread of the process, and
//!    play_launch is a tokio runtime.
//!
//! 3. **The capability cannot be read off the path or the controller list.** A
//!    login-session shell and a `systemd-run --user --scope` both report
//!    `controllers: memory pids` and differ only on whether `mkdir` succeeds:
//!
//!    ```text
//!    plain shell        /user.slice/.../session-N.scope        mkdir: EPERM
//!    systemd-run scope  /user@1000.service/app.slice/run-*     mkdir: ok
//!    ```
//!
//!    So availability is decided by attempting the thing, the same way phase 60
//!    learned that writing `partition root` can succeed and read back
//!    `root invalid`. play_launch normally starts from a terminal, which means
//!    **unavailable is the default path, not the exception** — every failure
//!    here degrades to exactly today's behaviour and says so once.

use std::{
    ffi::CString,
    path::{Path, PathBuf},
};
use tracing::{debug, info, warn};

/// Where the unified cgroup v2 hierarchy is mounted.
const CGROUP_ROOT: &str = "/sys/fs/cgroup";

/// Controllers this phase needs. `cpu` is deliberately absent: it is enabled at
/// the cgroup root and dropped at `user.slice`, so asking for it here would
/// fail the whole `subtree_control` write and take `memory` down with it. See
/// phase 66 W4 for the root-side drop-in that would grant it.
const WANTED_CONTROLLERS: &str = "+memory +pids";

/// A per-run cgroup tree, rooted at the cgroup play_launch was started in.
///
/// ```text
/// <scope>/
/// ├── supervisor/          play_launch itself
/// ├── node/<dir_name>/     one per plain <node>
/// └── container/<dir_name>/ the container process AND its composables
/// ```
#[derive(Debug)]
pub struct CgroupTree {
    /// The cgroup play_launch was started in; the parent of everything below.
    root: PathBuf,
}

/// Which kind of member a group is for. Only affects the path, so the tree is
/// legible in `systemd-cgls` and in a bug report.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GroupKind {
    Node,
    Container,
}

impl GroupKind {
    fn dir(self) -> &'static str {
        match self {
            GroupKind::Node => "node",
            GroupKind::Container => "container",
        }
    }
}

/// Read this process's cgroup v2 path from `/proc/self/cgroup`.
///
/// The v2 line is `0::/path`. A v1-only host has no such line, which is itself
/// a clean "unavailable".
fn own_cgroup_path() -> Option<PathBuf> {
    let text = std::fs::read_to_string("/proc/self/cgroup").ok()?;
    let rel = text
        .lines()
        .find_map(|line| line.strip_prefix("0::"))?
        .trim()
        .to_string();
    Some(PathBuf::from(CGROUP_ROOT).join(rel.trim_start_matches('/')))
}

impl CgroupTree {
    /// Set up the tree, or report why it is unavailable.
    ///
    /// Returns `None` for every failure. A launch must never fail because
    /// grouping was not possible — the fallback is what play_launch did before
    /// this module existed.
    pub fn probe() -> Option<Self> {
        let root = own_cgroup_path()?;
        if !root.is_dir() {
            debug!(
                "cgroups: own cgroup {} is not readable; grouping disabled",
                root.display()
            );
            return None;
        }

        // The only honest test is to try. Reading `cgroup.controllers` here
        // would report `memory pids` in both the writable and the unwritable
        // case (see the module doc).
        let probe = root.join(".play_launch_probe");
        if let Err(e) = std::fs::create_dir(&probe) {
            // `debug!`, not `info!`. This is the ORDINARY outcome — play_launch
            // is normally started from a terminal, whose session scope no
            // unprivileged user can write — and an INFO line on every launch
            // forever, about an optional feature nobody asked for, is how a
            // real signal gets scrolled past. A user who DID ask, by writing
            // `cgroups.limits`, is warned by the caller instead.
            debug!(
                "cgroups: no per-container grouping — cannot create a cgroup under {} ({}). \
                 Memory is reported per process and teardown uses the process group, as before. \
                 To enable it, start play_launch under `systemd-run --user --scope`.",
                root.display(),
                e
            );
            return None;
        }
        let _ = std::fs::remove_dir(&probe);

        let tree = CgroupTree { root };

        // A cgroup that holds processes cannot enable controllers for its
        // children, so play_launch has to step down into a leaf of its own
        // before the subtree can be configured. Order matters: move first,
        // then enable.
        let supervisor = tree.root.join("supervisor");
        if let Err(e) = std::fs::create_dir_all(&supervisor) {
            debug!("cgroups: cannot create supervisor leaf: {e}");
            return None;
        }
        if let Err(e) = std::fs::write(supervisor.join("cgroup.procs"), "0\n") {
            debug!(
                "cgroups: no per-container grouping — could not move play_launch into its own \
                 leaf ({e})"
            );
            return None;
        }
        if let Err(e) = std::fs::write(tree.root.join("cgroup.subtree_control"), WANTED_CONTROLLERS)
        {
            // Not fatal on its own: the groups still exist and `cgroup.kill`
            // still works, we just lose `memory.current`. Say which half was
            // lost rather than disabling everything.
            warn!(
                "cgroups: grouping is active but controllers could not be enabled ({e}); \
                 memory accounting will fall back to per-process sums"
            );
        }

        info!(
            "cgroups: per-container grouping active under {}",
            tree.root.display()
        );
        Some(tree)
    }

    /// The tree's root — the cgroup play_launch was started in.
    pub fn root(&self) -> &Path {
        &self.root
    }

    /// Create (or reuse) the group for one member and return its path.
    ///
    /// `dir_name` is the same log-directory name the member already uses, so a
    /// group in `systemd-cgls` lines up with a directory in `play_log/`.
    pub fn group_for(&self, kind: GroupKind, dir_name: &str) -> Option<PathBuf> {
        // `cgroup.subtree_control` is enabled ONE LEVEL AT A TIME: a controller
        // is available to a cgroup only if its PARENT enabled it. The tree here
        // is two levels deep (`<kind>/<name>`), so enabling `+memory` on the
        // root alone leaves the leaf with no controllers at all — which reads
        // back not as an error but as `memory.current: 0` on a group with seven
        // live members. Measured exactly that way before this line existed.
        let parent = self.root.join(kind.dir());
        if let Err(e) = std::fs::create_dir_all(&parent) {
            debug!("cgroups: cannot create {}: {e}", parent.display());
            return None;
        }
        let _ = std::fs::write(parent.join("cgroup.subtree_control"), WANTED_CONTROLLERS);

        let path = parent.join(sanitize(dir_name));
        match std::fs::create_dir_all(&path) {
            Ok(()) => Some(path),
            Err(e) => {
                debug!("cgroups: cannot create {}: {e}", path.display());
                None
            }
        }
    }

    /// The group's current memory charge, in bytes.
    ///
    /// This is the number that summing per-process RSS gets wrong: shared pages
    /// are counted once here and once *per process* there. Under `isolated`
    /// every composable is the same `component_node` binary plus the same
    /// rclcpp/rmw/DDS libraries, so the error scales with the composable count
    /// — measured at 2.4x across six processes sharing far less.
    pub fn memory_current(group: &Path) -> Option<u64> {
        let charged: u64 = std::fs::read_to_string(group.join("memory.current"))
            .ok()?
            .trim()
            .parse()
            .ok()?;
        // A group with members cannot really be charged nothing. Zero here
        // means the memory controller was never enabled for this level, and
        // reporting it as a memory figure would replace an over-count with a
        // far worse under-count.
        if charged == 0 {
            return None;
        }
        Some(charged)
    }

    /// The memory charge of the group `pid` belongs to, if that group is one
    /// of ours.
    ///
    /// The pid is what the monitor has; the group is what the kernel has. The
    /// check that it lies under `root` is not defensive tidiness — without it
    /// this would happily report the charge of whatever cgroup a process
    /// happened to be in, including the whole user session.
    pub fn memory_current_for_pid(root: &Path, pid: u32) -> Option<u64> {
        let text = std::fs::read_to_string(format!("/proc/{pid}/cgroup")).ok()?;
        let rel = text.lines().find_map(|l| l.strip_prefix("0::"))?.trim();
        let group = PathBuf::from(CGROUP_ROOT).join(rel.trim_start_matches('/'));
        if !group.starts_with(root) || group == root {
            return None;
        }
        Self::memory_current(&group)
    }

    /// Kill every process in the group, atomically.
    ///
    /// Reaches grandchildren, which is the point: a composable whose container
    /// died first is still a member, and a PGID kill can miss it.
    pub fn kill(group: &Path) -> bool {
        std::fs::write(group.join("cgroup.kill"), "1\n").is_ok()
    }

    /// Report every group whose `memory.events` counters moved, before the
    /// tree is removed.
    ///
    /// This is the half of W2 that is not a limit. Without it play_launch can
    /// only *infer* that a container was under memory pressure — from a node
    /// that died, or a launch that was slow — and inference is what reported a
    /// killed composable as loaded (#0019). These counters are the kernel's own
    /// record of what it did.
    ///
    /// Silent when nothing happened. A line of zeroes per container per launch
    /// is how a real signal gets scrolled past.
    pub fn report_memory_events(&self) {
        for kind in [GroupKind::Node, GroupKind::Container] {
            let parent = self.root.join(kind.dir());
            let Ok(entries) = std::fs::read_dir(&parent) else {
                continue;
            };
            for entry in entries.flatten() {
                let path = entry.path();
                let Some(ev) = MemoryEvents::read(&path) else {
                    continue;
                };
                if ev.is_quiet() {
                    continue;
                }
                let name = entry.file_name().to_string_lossy().to_string();
                // A kill is the kernel having already acted; a throttle is it
                // holding the line. Different severities, different reads.
                if ev.oom_kill > 0 || ev.max > 0 {
                    warn!("cgroups: {} {}: {}", kind.dir(), name, ev.summary());
                } else {
                    info!("cgroups: {} {}: {}", kind.dir(), name, ev.summary());
                }
            }
        }
    }

    /// Remove the tree. Best-effort: a non-empty group cannot be removed, and
    /// that is information rather than an error — it means something outlived
    /// the shutdown.
    pub fn cleanup(&self) {
        // Step back out of the supervisor leaf so it can be removed.
        let _ = std::fs::write(self.root.join("cgroup.procs"), "0\n");
        for kind in [GroupKind::Node, GroupKind::Container] {
            let parent = self.root.join(kind.dir());
            if let Ok(entries) = std::fs::read_dir(&parent) {
                for entry in entries.flatten() {
                    let _ = std::fs::remove_dir(entry.path());
                }
            }
            let _ = std::fs::remove_dir(&parent);
        }
        let _ = std::fs::remove_dir(self.root.join("supervisor"));
    }
}

impl Drop for CgroupTree {
    /// Remove the tree on any exit path.
    ///
    /// Deliberately does NOT kill: teardown is the existing descendant-kill
    /// path's job, and a W1 that started killing through a second mechanism
    /// would be a behaviour change wearing an accounting change's clothes. A
    /// group that will not remove is one whose members outlived shutdown —
    /// information, not an error.
    fn drop(&mut self) {
        // Order matters: the counters live in the directories cleanup removes.
        self.report_memory_events();
        self.cleanup();
    }
}

/// `cgroups.limits` compiled once, so a glob is parsed at startup rather than
/// per member.
///
/// An invalid pattern is reported and dropped rather than failing the launch:
/// this is an optional enhancement, and refusing to start a vehicle over a
/// typo in an optional limit is the wrong trade.
#[derive(Debug, Default)]
pub struct LimitRules {
    rules: Vec<(Vec<glob::Pattern>, CgroupLimits)>,
}

impl LimitRules {
    pub fn compile(groups: &[crate::cli::config::CgroupLimitGroup]) -> Self {
        const MB: u64 = 1024 * 1024;
        let mut rules = Vec::new();
        for (i, g) in groups.iter().enumerate() {
            let mut pats = Vec::new();
            for raw in &g.match_ {
                match glob::Pattern::new(raw) {
                    Ok(p) => pats.push(p),
                    Err(e) => warn!("cgroups.limits[{i}]: invalid glob {raw:?}: {e} — ignored"),
                }
            }
            if pats.is_empty() {
                continue;
            }
            rules.push((
                pats,
                CgroupLimits {
                    memory_high_bytes: g.memory_high_mb.map(|m| m * MB),
                    memory_max_bytes: g.memory_max_mb.map(|m| m * MB),
                    pids_max: g.pids_max,
                    oom_group: g.oom_group,
                },
            ));
        }
        Self { rules }
    }

    /// Limits for one member. FIRST match wins — a reader scanning the list top
    /// to bottom expects the specific rule above the general one to be the one
    /// that applies, and merging rules instead would make the effective limits
    /// of a node unreadable without running the program.
    pub fn for_member(&self, fqn: &str) -> CgroupLimits {
        for (pats, limits) in &self.rules {
            if pats.iter().any(|p| p.matches(fqn)) {
                return *limits;
            }
        }
        CgroupLimits::default()
    }

    pub fn is_empty(&self) -> bool {
        self.rules.is_empty()
    }

    pub fn len(&self) -> usize {
        self.rules.len()
    }
}

/// Limits to write into one group — phase 66 W2.
///
/// Resolved from `cgroups.limits` before the group is created, so the values
/// are already numbers by the time anything touches `/sys/fs/cgroup`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CgroupLimits {
    pub memory_high_bytes: Option<u64>,
    pub memory_max_bytes: Option<u64>,
    pub pids_max: Option<u64>,
    pub oom_group: Option<bool>,
}

impl CgroupLimits {
    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }

    /// Write the limits, returning what was actually applied.
    ///
    /// Each write is independent: a kernel that refuses one must not silence
    /// the others, and the caller reports what landed rather than what was
    /// asked for. Anything unset is left alone — not written as "max", which
    /// would overwrite a limit an outer cgroup imposed.
    pub fn apply(&self, group: &Path) -> Vec<String> {
        let mut applied = Vec::new();
        let mut write = |file: &str, value: String, label: String| {
            if std::fs::write(group.join(file), format!("{value}\n")).is_ok() {
                applied.push(label);
            } else {
                debug!("cgroups: could not set {file} on {}", group.display());
            }
        };
        if let Some(v) = self.memory_high_bytes {
            write("memory.high", v.to_string(), format!("memory.high={v}"));
        }
        if let Some(v) = self.memory_max_bytes {
            write("memory.max", v.to_string(), format!("memory.max={v}"));
        }
        if let Some(v) = self.pids_max {
            write("pids.max", v.to_string(), format!("pids.max={v}"));
        }
        if let Some(v) = self.oom_group {
            let raw = u8::from(v);
            write(
                "memory.oom.group",
                raw.to_string(),
                format!("oom.group={raw}"),
            );
        }
        applied
    }
}

/// The `memory.events` counters for a group.
///
/// These are FACTS the kernel keeps, which is the point: without them
/// play_launch can only infer that a container was under memory pressure, and
/// inference is what reported a killed composable as loaded (#0019).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MemoryEvents {
    /// Times the group was reclaimed for breaching `memory.high`.
    pub high: u64,
    /// Times allocation was refused at `memory.max`.
    pub max: u64,
    /// Times an OOM was declared in the group.
    pub oom: u64,
    /// Processes actually killed. With `oom.group=1` a single OOM kills every
    /// member, so this exceeds `oom` — that gap IS the container semantics
    /// being visible.
    pub oom_kill: u64,
}

impl MemoryEvents {
    pub fn read(group: &Path) -> Option<Self> {
        let text = std::fs::read_to_string(group.join("memory.events")).ok()?;
        let mut ev = MemoryEvents::default();
        for line in text.lines() {
            let mut it = line.split_whitespace();
            let (Some(key), Some(val)) = (it.next(), it.next()) else {
                continue;
            };
            let Ok(n) = val.parse::<u64>() else { continue };
            match key {
                "high" => ev.high = n,
                "max" => ev.max = n,
                "oom" => ev.oom = n,
                "oom_kill" => ev.oom_kill = n,
                _ => {}
            }
        }
        Some(ev)
    }

    /// Nothing has gone wrong yet.
    pub fn is_quiet(&self) -> bool {
        *self == Self::default()
    }

    /// A one-line report for a human. Only the non-zero counters, because a
    /// line of zeroes per container per launch is how a real signal gets
    /// scrolled past.
    pub fn summary(&self) -> String {
        let mut parts = Vec::new();
        if self.high > 0 {
            parts.push(format!("throttled {}x at memory.high", self.high));
        }
        if self.max > 0 {
            parts.push(format!("hit memory.max {}x", self.max));
        }
        if self.oom > 0 {
            parts.push(format!("{} OOM event(s)", self.oom));
        }
        if self.oom_kill > 0 {
            parts.push(format!("{} process(es) killed", self.oom_kill));
        }
        parts.join(", ")
    }
}

/// A cgroup path prepared for use after `fork()`.
///
/// The `CString` is built **before** the fork so the child does no allocation
/// and no formatting: joining is then `open`/`write`/`close`, which is
/// async-signal-safe, exactly like the `oom_score_adj` write it sits beside.
#[derive(Clone, Debug)]
pub struct CgroupHandle(CString);

impl CgroupHandle {
    pub fn new(group: &Path) -> Option<Self> {
        CString::new(group.join("cgroup.procs").as_os_str().as_encoded_bytes())
            .ok()
            .map(CgroupHandle)
    }

    /// Move the calling process into the group. Call only between `fork()` and
    /// `exec()`.
    ///
    /// Writes `"0"`, not a pid: cgroup v2 reads `0` as "the calling process",
    /// which removes the only step that would have needed integer formatting in
    /// a post-fork context.
    ///
    /// Silent on failure by design. The child is mid-spawn with no way to
    /// report, and losing its group costs accounting, never correctness.
    pub fn join(&self) {
        unsafe {
            let fd = libc::open(self.0.as_ptr(), libc::O_WRONLY | libc::O_CLOEXEC);
            if fd < 0 {
                return;
            }
            let buf = b"0\n";
            libc::write(fd, buf.as_ptr() as *const libc::c_void, buf.len());
            libc::close(fd);
        }
    }
}

/// Make a member name safe as a single path component.
///
/// A node's directory name can carry `/` from its namespace; a cgroup name
/// cannot.
fn sanitize(name: &str) -> String {
    name.replace('/', "_")
        .trim_start_matches('.')
        .replace('\0', "")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A namespaced node name must not silently create a nested cgroup — or
    /// worse, escape the tree.
    #[test]
    fn a_namespaced_name_becomes_one_path_component() {
        assert_eq!(sanitize("/sensing/lidar/driver"), "_sensing_lidar_driver");
        assert!(!sanitize("/a/b").contains('/'));
        // A leading dot would collide with the probe directory's naming.
        assert!(!sanitize("..evil").starts_with('.'));
    }

    /// The path layout is part of the contract: it is what makes a group in
    /// `systemd-cgls` line up with a directory in `play_log/`.
    #[test]
    fn group_paths_are_kind_scoped() {
        let tree = CgroupTree {
            root: PathBuf::from("/sys/fs/cgroup/test.scope"),
        };
        let node = tree.root.join(GroupKind::Node.dir()).join("talker");
        let ctr = tree.root.join(GroupKind::Container.dir()).join("talker");
        assert_ne!(
            node, ctr,
            "a node and a container may share a name and must not share a group"
        );
        assert!(node.ends_with("node/talker"));
        assert!(ctr.ends_with("container/talker"));
    }

    use crate::cli::config::CgroupLimitGroup;

    fn rule(pats: &[&str], f: impl FnOnce(&mut CgroupLimitGroup)) -> CgroupLimitGroup {
        let mut g = CgroupLimitGroup {
            match_: pats.iter().map(|s| s.to_string()).collect(),
            ..Default::default()
        };
        f(&mut g);
        g
    }

    /// First match wins. Merging instead would make a node's effective limits
    /// unreadable without running the program — you could not answer "what
    /// applies to /perception/x" by reading the file top to bottom.
    #[test]
    fn the_first_matching_rule_wins() {
        let rules = LimitRules::compile(&[
            rule(&["/perception/**"], |g| g.memory_high_mb = Some(4096)),
            rule(&["**"], |g| g.memory_high_mb = Some(512)),
        ]);
        assert_eq!(
            rules.for_member("/perception/detector").memory_high_bytes,
            Some(4096 * 1024 * 1024)
        );
        assert_eq!(
            rules.for_member("/other/node").memory_high_bytes,
            Some(512 * 1024 * 1024)
        );
        // The specific rule sets only `memory_high`; a later rule must NOT
        // contribute its own fields to the same member.
        assert_eq!(rules.for_member("/perception/detector").pids_max, None);
    }

    /// A member matching nothing gets no limits — not a default someone
    /// guessed. A wrong `memory.max` turns a slow launch into a killed one.
    #[test]
    fn an_unmatched_member_is_unlimited() {
        let rules = LimitRules::compile(&[rule(&["/sensing/**"], |g| g.memory_max_mb = Some(1))]);
        assert!(rules.for_member("/planning/x").is_empty());
    }

    /// An invalid glob is dropped with a warning, never fatal. Refusing to
    /// start a vehicle over a typo in an OPTIONAL limit is the wrong trade.
    #[test]
    fn a_bad_glob_does_not_fail_the_launch() {
        let rules = LimitRules::compile(&[
            rule(&["["], |g| g.pids_max = Some(1)),
            rule(&["/ok/**"], |g| g.pids_max = Some(64)),
        ]);
        assert_eq!(rules.for_member("/ok/node").pids_max, Some(64));
    }

    /// `oom_group` is the bit that chooses the failure model, so `false` must
    /// reach the kernel as an explicit 0 rather than being dropped as
    /// "falsy" — a group inheriting 1 from a parent would otherwise silently
    /// keep container semantics for a node that asked for isolation.
    #[test]
    fn oom_group_false_is_a_value_not_an_absence() {
        let rules = LimitRules::compile(&[rule(&["**"], |g| g.oom_group = Some(false))]);
        let l = rules.for_member("/x");
        assert_eq!(l.oom_group, Some(false));
        assert!(!l.is_empty(), "an explicit false is a setting to apply");
    }

    /// Megabytes in the config, bytes in the cgroup file. An off-by-1024 here
    /// would set a limit a thousand times too small and read as a mysterious
    /// OOM.
    #[test]
    fn megabytes_become_bytes() {
        let rules = LimitRules::compile(&[rule(&["**"], |g| {
            g.memory_high_mb = Some(1);
            g.memory_max_mb = Some(2);
        })]);
        let l = rules.for_member("/x");
        assert_eq!(l.memory_high_bytes, Some(1_048_576));
        assert_eq!(l.memory_max_bytes, Some(2_097_152));
    }

    /// Only non-zero counters are reported: a line of zeroes per container per
    /// launch is how a real signal gets scrolled past.
    #[test]
    fn memory_events_reports_only_what_happened() {
        assert!(MemoryEvents::default().is_quiet());
        assert_eq!(MemoryEvents::default().summary(), "");

        let ev = MemoryEvents {
            high: 3,
            oom_kill: 2,
            ..Default::default()
        };
        let s = ev.summary();
        assert!(s.contains("throttled 3x"), "{s}");
        assert!(s.contains("2 process(es) killed"), "{s}");
        assert!(
            !s.contains("memory.max"),
            "silent about what did not happen: {s}"
        );
    }

    /// Probing must never panic or block, whatever the host looks like. On a
    /// developer machine started from a terminal this returns `None`, which is
    /// the documented default path.
    #[test]
    fn probe_is_infallible() {
        let _ = CgroupTree::probe();
    }

    /// `/proc/self/cgroup` parsing: v2 is the `0::` line, and a v1-only host
    /// has none.
    #[test]
    fn own_cgroup_path_is_read_from_the_v2_line() {
        // Cannot assert a specific path — it differs per host and per CI — but
        // if a path comes back it must be under the v2 mount.
        if let Some(p) = own_cgroup_path() {
            assert!(p.starts_with(CGROUP_ROOT), "{}", p.display());
        }
    }
}
