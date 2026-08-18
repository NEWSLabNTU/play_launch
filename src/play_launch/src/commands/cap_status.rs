//! Why a helper capability is missing, not merely that it is (issue #0015).
//!
//! A file capability lives on the inode and does not survive a content
//! change. That is the kernel behaving correctly: `security.capability` is a
//! statement about a specific binary, so replacing the binary must invalidate
//! it. Anything that copied a capability forward across a rebuild would be
//! granting privilege to code nobody vetted.
//!
//! The problem was never that. It is that every message said the same thing
//! —
//!
//!     scheduling: no privilege to apply; run `play_launch setcap`
//!
//! — whether the user had never run `setcap` or had run it twenty minutes
//! earlier and rebuilt since. To someone in the second case the tool is
//! reporting that they did not do a thing they did do, so the natural reading
//! is that `setcap` did not work. It did; the binary changed.
//!
//! So `setcap` records WHAT it capped — path, content hash, when — and the
//! runtime compares. The hash is what makes the answer honest: an mtime or an
//! inode number would call a touched-but-identical file "changed", and would
//! miss a rebuild that happened to land on the same inode.
//!
//! No message here suggests `just setcap`. That recipe uses a throwaway
//! container to avoid a password prompt, which is a DEVELOPER convenience
//! resting on `docker` group membership being root-equivalent, and it does
//! not work under rootless Docker. The supported path is `play_launch setcap`.

use sha2::{Digest, Sha256};
use std::path::{Path, PathBuf};

/// Which capability a helper needs. Only used for message text.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cap {
    SysNice,
    SysPtrace,
}

impl Cap {
    pub fn name(self) -> &'static str {
        match self {
            Cap::SysNice => "cap_sys_nice",
            Cap::SysPtrace => "cap_sys_ptrace",
        }
    }

    /// What stops working without it — stated in terms of the user's run, not
    /// of the capability.
    fn consequence(self) -> &'static str {
        match self {
            Cap::SysNice => "real-time scheduling cannot be applied",
            Cap::SysPtrace => "I/O monitoring cannot read privileged processes",
        }
    }
}

/// Why the capability is or is not in place.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CapStatus {
    /// Present on the binary that will actually run.
    Present,
    /// No record that `setcap` was ever run for this path.
    NeverGranted,
    /// Granted, and the binary has been REPLACED since — a rebuild, an
    /// upgrade, a reinstall. The capability could not have survived that.
    BinaryChanged { granted: String },
    /// Granted, the binary is byte-identical, and the capability is gone
    /// anyway. Something removed it (`setcap -r`, a filesystem without
    /// xattr support, a copy onto the same path).
    Revoked { granted: String },
    /// Could not tell — `getcap` missing, or the helper could not be
    /// resolved. Reported as unknown rather than as absent, because
    /// "we cannot check" and "it is not there" call for different actions.
    Unknown { why: String },
}

impl CapStatus {
    pub fn is_present(&self) -> bool {
        matches!(self, CapStatus::Present)
    }
}

/// Where grants are recorded. Deliberately NOT next to the binary: a system
/// install is root-owned, and `setcap` run with `sudo` would leave a
/// root-owned file the unprivileged run then cannot read.
fn stamp_path() -> Option<PathBuf> {
    let base = match std::env::var_os("XDG_STATE_HOME") {
        Some(v) if !v.is_empty() => PathBuf::from(v),
        _ => {
            let home = std::env::var_os("HOME")?;
            PathBuf::from(home).join(".local/state")
        }
    };
    Some(base.join("play_launch/setcap-grants.tsv"))
}

fn hash_file(path: &Path) -> Option<String> {
    let bytes = std::fs::read(path).ok()?;
    let mut h = Sha256::new();
    h.update(&bytes);
    Some(format!("{:x}", h.finalize()))
}

/// One recorded grant: path, the hash of the binary as capped, and when.
fn read_grant(path: &Path) -> Option<(String, String)> {
    let text = std::fs::read_to_string(stamp_path()?).ok()?;
    let want = path.to_string_lossy();
    // Last entry wins — the file is append-only, so re-granting the same path
    // leaves the older rows behind it.
    text.lines()
        .rev()
        .filter_map(|line| {
            let mut f = line.split('\t');
            let p = f.next()?;
            let hash = f.next()?;
            let when = f.next().unwrap_or("an earlier run");
            (p == want).then(|| (hash.to_string(), when.to_string()))
        })
        .next()
}

/// Record that `path` was just granted a capability, so a later run can tell
/// a rebuild from a never-granted install.
///
/// Best-effort: failing to record must never fail the grant itself, which is
/// the part that matters. The cost of losing the record is a less specific
/// message later, not a broken capability.
pub fn record_grant(path: &Path) {
    let Some(stamp) = stamp_path() else {
        return;
    };
    let Some(hash) = hash_file(path) else {
        return;
    };
    if let Some(dir) = stamp.parent() {
        let _ = std::fs::create_dir_all(dir);
    }
    let when = std::process::Command::new("date")
        .arg("+%Y-%m-%d %H:%M:%S")
        .output()
        .ok()
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| s.trim().to_string())
        .unwrap_or_else(|| "an earlier run".to_string());

    use std::io::Write as _;
    if let Ok(mut f) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&stamp)
    {
        let _ = writeln!(f, "{}\t{hash}\t{when}", path.to_string_lossy());
    }
}

/// Does `path` currently carry `cap`, and if not, why not.
pub fn diagnose(path: &Path, cap: Cap) -> CapStatus {
    let output = match std::process::Command::new("getcap").arg(path).output() {
        Ok(o) if o.status.success() => o,
        Ok(_) | Err(_) => {
            return CapStatus::Unknown {
                why: "`getcap` is not available (install the `libcap2-bin` package)".to_string(),
            };
        }
    };
    let stdout = String::from_utf8_lossy(&output.stdout);
    // `getcap` prints `cap_sys_nice=ep`; `setcap` is invoked with `+ep`.
    if stdout.contains(&format!("{}=ep", cap.name()))
        || stdout.contains(&format!("{}+ep", cap.name()))
    {
        return CapStatus::Present;
    }

    match read_grant(path) {
        None => CapStatus::NeverGranted,
        Some((granted_hash, when)) => match hash_file(path) {
            Some(now) if now == granted_hash => CapStatus::Revoked { granted: when },
            Some(_) => CapStatus::BinaryChanged { granted: when },
            // Unreadable but `getcap` worked: treat as changed rather than
            // claiming it was revoked, since we cannot show it is the same
            // binary.
            None => CapStatus::BinaryChanged { granted: when },
        },
    }
}

/// The message to print for a status, as lines.
///
/// Returns empty for [`CapStatus::Present`] so a caller can log the result
/// unconditionally.
pub fn explain(status: &CapStatus, path: &Path, cap: Cap) -> Vec<String> {
    let fix = format!(
        "run `play_launch setcap` to grant {}+ep to {}",
        cap.name(),
        path.display()
    );
    match status {
        CapStatus::Present => Vec::new(),
        CapStatus::NeverGranted => vec![
            format!(
                "{} lacks {} — {}.",
                path.display(),
                cap.name(),
                cap.consequence()
            ),
            fix,
        ],
        CapStatus::BinaryChanged { granted } => vec![
            format!(
                "{} lacks {} — {}.",
                path.display(),
                cap.name(),
                cap.consequence()
            ),
            format!(
                "It WAS granted ({granted}), but the binary has been replaced since — a rebuild, \
                 upgrade or reinstall. A file capability is bound to the exact file contents and \
                 cannot survive that, by design: carrying it across would grant privilege to code \
                 that was never vetted."
            ),
            format!("This is expected after every rebuild; {fix}"),
        ],
        CapStatus::Revoked { granted } => vec![
            format!(
                "{} lacks {} — {}.",
                path.display(),
                cap.name(),
                cap.consequence()
            ),
            format!(
                "It was granted ({granted}) and the binary is UNCHANGED, so something removed it \
                 — `setcap -r`, a filesystem mounted without xattr support, or a copy over the \
                 same path."
            ),
            fix,
        ],
        CapStatus::Unknown { why } => vec![
            format!(
                "Cannot tell whether {} carries {}: {why}.",
                path.display(),
                cap.name()
            ),
            format!("{} if it turns out to be missing.", cap.consequence()),
            fix,
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The distinction this module exists for: a rebuild and a fresh install
    /// must not produce the same message, because only one of them means the
    /// user did something wrong.
    #[test]
    fn a_rebuild_and_a_fresh_install_read_differently() {
        let path = Path::new("/nonexistent/play_launch_rt_helper");

        let fresh = explain(&CapStatus::NeverGranted, path, Cap::SysNice).join(" ");
        let rebuilt = explain(
            &CapStatus::BinaryChanged {
                granted: "2026-08-18 10:00:00".to_string(),
            },
            path,
            Cap::SysNice,
        )
        .join(" ");

        assert!(fresh.contains("play_launch setcap"));
        assert!(!fresh.contains("WAS granted"), "{fresh}");

        assert!(rebuilt.contains("WAS granted"), "{rebuilt}");
        assert!(rebuilt.contains("replaced"), "{rebuilt}");
        // Says the behaviour is correct, so the user does not go looking for
        // a bug in setcap.
        assert!(rebuilt.contains("by design"), "{rebuilt}");
        assert!(
            rebuilt.contains("expected after every rebuild"),
            "{rebuilt}"
        );
    }

    /// Unknown must not masquerade as absent: "cannot check" and "not there"
    /// call for different actions.
    #[test]
    fn unknown_is_not_reported_as_missing() {
        let path = Path::new("/nonexistent/play_launch_io_helper");
        let msg = explain(
            &CapStatus::Unknown {
                why: "`getcap` is not available".to_string(),
            },
            path,
            Cap::SysPtrace,
        )
        .join(" ");
        assert!(msg.contains("Cannot tell"), "{msg}");
        assert!(!msg.starts_with("/nonexistent"), "{msg}");
    }

    /// No message may point at the developer-only Docker path.
    #[test]
    fn no_message_recommends_the_docker_shortcut() {
        let path = Path::new("/nonexistent/helper");
        for status in [
            CapStatus::NeverGranted,
            CapStatus::BinaryChanged {
                granted: "now".into(),
            },
            CapStatus::Revoked {
                granted: "now".into(),
            },
            CapStatus::Unknown { why: "x".into() },
        ] {
            let msg = explain(&status, path, Cap::SysNice).join(" ");
            assert!(!msg.contains("just setcap"), "{msg}");
            assert!(!msg.contains("docker"), "{msg}");
            assert!(msg.contains("play_launch setcap"), "{msg}");
        }
    }

    /// A present capability produces no output at all, so callers can log
    /// unconditionally.
    #[test]
    fn present_says_nothing() {
        assert!(explain(&CapStatus::Present, Path::new("/x"), Cap::SysNice).is_empty());
    }
}
