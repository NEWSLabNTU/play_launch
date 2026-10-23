//! `rlm.lock` (phase 91 E7): the extensions and profiles the contracts in a
//! directory resolved to, by version and sha256.
//!
//! A lock sits beside the contracts it covers and is written by
//! `check --update-lock`. Resolution reads it whenever it exists: for each
//! `use:` entry the locked version is chosen while it is still installed and
//! the requirement still accepts it, so installing a newer extension does not
//! move a checked contract. What the lock cannot hold is reported:
//!
//! - the locked version is gone or no longer accepted, or the file of that
//!   version changed (another sha256): `lock-drift`, a warning; under
//!   `check --locked`, `lock-mismatch`, an error;
//! - under `--locked`, a contract that uses anything with no lock beside it
//!   (`lock-missing`) or a `use:` entry the lock does not name
//!   (`lock-mismatch`).
//!
//! `--update-lock` resolves as if there were no lock (the highest accepted
//! version) and rewrites each directory's lock from what was chosen.

use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

use serde::{Deserialize, Serialize};

/// The file name of a lock.
pub const FILE: &str = "rlm.lock";

/// How resolution treats a lock (`check --locked`, `--update-lock`).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum LockMode {
    /// Prefer the locked versions; drift is a warning.
    #[default]
    Prefer,
    /// Prefer the locked versions; drift, a missing lock and an unlocked
    /// entry are errors.
    Locked,
    /// Ignore any lock; the caller rewrites it from the resolution.
    Update,
}

/// One locked resolution.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Entry {
    pub version: String,
    pub sha256: String,
    /// `extension` or `profile`.
    pub kind: String,
}

/// A lock file.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Lock {
    pub version: u32,
    pub resolved: BTreeMap<String, Entry>,
}

/// The lock beside `contract`.
pub fn path_for(contract: &Path) -> PathBuf {
    contract
        .parent()
        .unwrap_or_else(|| Path::new("."))
        .join(FILE)
}

/// Read the lock beside `contract`: `Ok(None)` when there is none.
pub fn read(contract: &Path) -> Result<Option<Lock>, String> {
    let path = path_for(contract);
    let text = match std::fs::read_to_string(&path) {
        Ok(t) => t,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(format!("{}: {e}", path.display())),
    };
    let lock: Lock = serde_yaml_ng::from_str(&text)
        .map_err(|e| format!("{} is not a lock file: {e}", path.display()))?;
    if lock.version != 1 {
        return Err(format!(
            "{}: lock version {} (this checker writes 1)",
            path.display(),
            lock.version
        ));
    }
    Ok(Some(lock))
}

/// The lock's text.
pub fn render(lock: &Lock) -> String {
    let mut out = String::from(
        "# rlm.lock: the extensions and profiles the contracts in this directory\n\
         # resolved to. Written by `play_launch check --update-lock`; `check\n\
         # --locked` refuses any other resolution. Commit it.\n",
    );
    out.push_str(&serde_yaml_ng::to_string(lock).expect("a lock serializes"));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_lock_round_trips_and_names_its_writer() {
        let lock = Lock {
            version: 1,
            resolved: BTreeMap::from([(
                "machine.iso13855".to_string(),
                Entry {
                    version: "0.1.0".into(),
                    sha256: "ab".repeat(32),
                    kind: "extension".into(),
                },
            )]),
        };
        let text = render(&lock);
        assert!(text.contains("--update-lock"));
        assert_eq!(serde_yaml_ng::from_str::<Lock>(&text).unwrap(), lock);
    }
}
