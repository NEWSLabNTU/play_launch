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
            info!(
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
            info!(
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
        self.cleanup();
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
