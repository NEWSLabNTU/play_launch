//! The launch-tree verbs, as library functions.
//!
//! `resolve`, `dump`, `check`, `contract eject` and `plot` are each offered by
//! TWO command-line interfaces: `play_launch` (the product a user installs,
//! which links a ROS runtime) and `ros-launch-resolve` (the developer /
//! integration binary that builds with no ROS at all). There is exactly ONE
//! implementation of each verb, and it lives here.
//!
//! # Why the verbs are in the library and not in a CLI
//!
//! They used to live in `ros-launch-resolve-cli`, which meant the only way for
//! `play_launch` to offer `resolve` was to tell users to install a second
//! binary — a developer tool that was never meant to be user-visible. Copying
//! the handlers into the second CLI instead would have left two
//! implementations to drift apart. Moving them down here gives both CLIs a
//! shared body and reduces each handler to argument mapping.
//!
//! # The rule this module obeys
//!
//! **The library must not know a CLI exists.** Every entry point takes a
//! plain `*Inputs` struct of owned values — no `clap` derives, no borrowed
//! `*Args`, no dependency on either CLI's `options` module — and no entry
//! point calls [`std::process::exit`]. [`check::run`] returns the exit code it
//! *intends*; deciding what to do with it is the caller's job.

use eyre::Context as _;
use tracing::debug;

pub mod check;
pub mod contract;
pub mod dump;
pub mod plot;
pub mod resolve;

pub use check::CheckInputs;
pub use contract::ContractEjectInputs;
pub use dump::DumpInputs;
pub use plot::PlotInputs;
pub use resolve::ResolveInputs;

/// Which launch-file frontend to parse with.
///
/// Deliberately NOT a `clap::ValueEnum`: each CLI keeps its own
/// clap-derived enum and maps into this one, so the library carries no
/// knowledge of how the choice reached it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ParserBackend {
    /// The Rust parser (default, fast, no fallback on error).
    #[default]
    Rust,
    /// The Python parser (slower, maximum compatibility).
    Python,
}

/// Maximum number of Tokio worker threads (async workload is mostly idle)
const MAX_WORKER_THREADS: usize = 8;

/// Build a Tokio multi-thread runtime with adaptive thread pool configuration.
///
/// Uses number of CPUs capped at `MAX_WORKER_THREADS` for efficiency.
/// Automatically adapts to platform: 2-core Pi, 4-core laptop, 8-core AGX Orin, 32-core server.
pub fn build_tokio_runtime() -> eyre::Result<tokio::runtime::Runtime> {
    let worker_threads = std::cmp::min(num_cpus::get(), MAX_WORKER_THREADS);
    let max_blocking = worker_threads * 2;
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(worker_threads)
        .max_blocking_threads(max_blocking)
        .thread_name("play_launch-worker")
        .enable_all()
        .build()?;
    debug!(
        "Tokio runtime created ({} worker threads, {} max blocking threads)",
        worker_threads, max_blocking
    );
    Ok(runtime)
}

/// Undo clap's positional mis-binding when the launch target is a direct PATH.
///
/// Every launch-tree verb declares two positionals — `<package_or_path>` then
/// an optional `<launch_file>` — followed by variadic `KEY:=VALUE` launch
/// arguments. When the user gives a package name that shape is right
/// (`check my_pkg my.launch.xml mode:=x`). When they give a direct path there
/// is no second positional to fill, so clap binds the FIRST `KEY:=VALUE` to
/// `launch_file` and the binding is silently lost:
///
/// ```text
/// check multihost.launch.xml host:=robot1
///       └─ package_or_path    └─ launch_file (!), never a launch argument
/// ```
///
/// A `KEY:=VALUE` is never a launch file name, so reclassifying it is
/// unambiguous. This lives here, called by every verb that takes launch
/// arguments, because it previously existed only in `resolve` — `check` was
/// missing it and validated contracts against a node set the user never asked
/// for while reporting success.
///
/// Returns the corrected `(launch_file, launch_arguments)` pair.
pub fn reclassify_launch_file_arg(
    launch_file: Option<&str>,
    launch_arguments: &[String],
) -> (Option<String>, Vec<String>) {
    match launch_file {
        Some(lf) if lf.contains(":=") => {
            let mut args = Vec::with_capacity(launch_arguments.len() + 1);
            args.push(lf.to_string());
            args.extend_from_slice(launch_arguments);
            (None, args)
        }
        other => (other.map(str::to_string), launch_arguments.to_vec()),
    }
}

/// Resolve a launch file path from a package name or a direct path.
pub fn resolve_launch_file(
    package_or_path: &str,
    launch_file: Option<&str>,
) -> eyre::Result<std::path::PathBuf> {
    use std::path::PathBuf;

    // Check if it looks like a direct file path
    if package_or_path.contains('/')
        || package_or_path.ends_with(".py")
        || package_or_path.ends_with(".xml")
        || package_or_path.ends_with(".yaml")
    {
        let path = PathBuf::from(package_or_path);
        if !path.exists() {
            return Err(eyre::eyre!("Launch file not found: {}", path.display()));
        }
        return Ok(path);
    }

    // Otherwise, treat as ROS package name
    let Some(file) = launch_file else {
        // Verb-neutral on purpose: this helper is shared by `resolve`,
        // `dump`, `check` and `contract eject`, across two CLIs. Naming one
        // verb here told a user running `check` to look at `launch` — a verb
        // `ros-launch-resolve` does not even have.
        return Err(eyre::eyre!(
            "Launch file name required when the first argument is a package name.\n\
             Give <package> <file.launch.py|xml>, or pass the path to the launch \
             file directly."
        ));
    };

    // Package name. Resolve its share directory the way ament does — first
    // matching prefix on AMENT_PREFIX_PATH wins — then search inside it.
    let Some(pkg_share) = std::env::var("AMENT_PREFIX_PATH").ok().and_then(|paths| {
        paths
            .split(':')
            .map(|prefix| PathBuf::from(prefix).join("share").join(package_or_path))
            .find(|share| share.is_dir())
    }) else {
        return Err(eyre::eyre!(
            "Package '{package_or_path}' not found.\n\
             \n\
             Make sure:\n\
             1. The ROS 2 workspace is sourced (source install/setup.bash)\n\
             2. The package is built (colcon build --packages-select {package_or_path})"
        ));
    };

    // An explicit relative path (`topics/talker_listener.launch.py`) is taken
    // as given. `ros2 launch` does not accept this form — it compares only the
    // basename — but it is unambiguous by construction and this tool has
    // accepted it for long enough that removing it would break callers.
    if file.contains('/') {
        // Both roots, as before this became recursive: `topics/x.launch.py`
        // means `launch/topics/x.launch.py` for a conventional package, and
        // `share/<pkg>/topics/x.launch.py` for one that installs elsewhere.
        // Dropping the `launch/` root broke `demo_nodes_cpp
        // topics/talker_listener.launch.py`, which is the documented form.
        for root in [pkg_share.join("launch"), pkg_share.clone()] {
            let direct = root.join(file);
            if direct.is_file() {
                return Ok(direct);
            }
        }
        return Err(eyre::eyre!(
            "Launch file '{file}' not found under {} (tried both there and \
             under its `launch/` subdirectory).",
            pkg_share.display()
        ));
    }

    // Bare filename: walk the whole share directory, matching on basename.
    // This mirrors `ros2 launch` (ros2launch/api/api.py
    // `get_share_file_path_from_package`), which is why a file under
    // `launch/topics/` is reachable by name alone — `demo_nodes_cpp
    // talker_listener.launch.py` is the canonical example, and searching only
    // `launch/` and the share root used to miss it.
    let mut matches = find_by_basename(&pkg_share, file);

    // Deterministic across filesystems: readdir order is not guaranteed, and
    // an unstable answer here would be worse than a wrong one.
    matches.sort();

    match matches.len() {
        1 => Ok(matches.into_iter().next().expect("len checked")),
        0 => Err(eyre::eyre!(
            "Launch file '{file}' was not found in the share directory of \
             package '{package_or_path}', which is at {}.\n\
             \n\
             Make sure the package is built and the launch file is installed \
             (an `install(DIRECTORY launch ...)` or `data_files` entry).",
            pkg_share.display()
        )),
        // `ros2 launch` errors here too rather than picking one. Silently
        // preferring `launch/` would make the choice invisible in the very
        // case where the user most needs to know a choice was made.
        _ => Err(eyre::eyre!(
            "Launch file '{file}' was found more than once in package \
             '{package_or_path}':\n{}\n\
             \n\
             Disambiguate by passing the path relative to the package share \
             directory, e.g. `{package_or_path} {}`.",
            matches
                .iter()
                .map(|p| format!("  {}", p.display()))
                .collect::<Vec<_>>()
                .join("\n"),
            matches[0]
                .strip_prefix(&pkg_share)
                .unwrap_or(&matches[0])
                .display()
        )),
    }
}

/// Every file under `dir` whose file name equals `name`, recursively.
///
/// Directory symlinks are not followed, matching Python's `os.walk` default
/// (`followlinks=False`) that `ros2 launch` relies on. Symlinked *files* are
/// still matched, which is what a `colcon --symlink-install` share tree is
/// made of.
fn find_by_basename(dir: &std::path::Path, name: &str) -> Vec<std::path::PathBuf> {
    let mut out = Vec::new();
    let mut stack = vec![dir.to_path_buf()];
    while let Some(current) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&current) else {
            continue; // unreadable subtree: skip, do not fail the lookup
        };
        for entry in entries.flatten() {
            let path = entry.path();
            let Ok(file_type) = entry.file_type() else {
                continue;
            };
            if file_type.is_dir() {
                stack.push(path);
            } else if entry.file_name() == name {
                out.push(path);
            }
        }
    }
    out
}

/// Parse a launch file into an in-memory [`crate::ros::launch_dump::LaunchDump`]
/// — the parser-agnostic intermediate `resolve`/`launch` build a SystemModel
/// from. Phase 47.B4/B5: no user-facing artifact results from this. The
/// Rust parser path never touches disk (parse → JSON string → deserialize,
/// entirely in memory — the same trick `resolve`'s Rust branch has used
/// since Phase 46.4). The Python parser path necessarily round-trips
/// through a private OS-temp scratch file (the PyO3 `dump_launch` bridge
/// itself only knows how to write JSON to a path) that is deleted before
/// this function returns — an interop detail, not a `record.json` companion
/// left for the user.
///
/// Returns the dump plus the resolved launch file path (used for
/// provenance hashing by callers).
pub async fn parse_to_launch_dump(
    package_or_path: &str,
    launch_file: Option<&str>,
    launch_arguments: &[String],
    parser: ParserBackend,
) -> eyre::Result<(crate::ros::launch_dump::LaunchDump, std::path::PathBuf)> {
    let launch_path = resolve_launch_file(package_or_path, launch_file)?;

    let dump = match parser {
        ParserBackend::Rust => {
            let cli_args = parse_launch_arguments(launch_arguments);
            let record =
                crate::verbs::parse_launch_file(&launch_path, cli_args).map_err(|e| {
                    // Verb- and binary-neutral: two CLIs share this, and
                    // every verb that reaches it accepts `--parser`.
                    let _ = launch_file;
                    eyre::eyre!(
                        "Rust parser error while parsing {package_or_path}: {e}\n\n\
                         Hint: if this is a parser limitation rather than a bad launch \
                         file, re-run the same command with `--parser python` (slower, \
                         maximum compatibility)."
                    )
                })?;
            let json = serde_json::to_string_pretty(&record)?;
            serde_json::from_str(&json)?
        }
        ParserBackend::Python => {
            let scratch_path = std::env::temp_dir().join(format!(
                "play_launch-parse-{}-{}.record.json",
                std::process::id(),
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_nanos())
                    .unwrap_or(0),
            ));
            let launcher = crate::python::dump_launcher::DumpLauncher::new()
                .wrap_err("Failed to initialize dump_launch. Ensure ROS workspace is sourced.")?;
            launcher
                .dump_launch(
                    package_or_path,
                    launch_file,
                    launch_arguments,
                    &scratch_path,
                )
                .await?;
            let dump =
                crate::ros::launch_dump::load_launch_dump(&scratch_path).wrap_err_with(|| {
                    format!("loading Python-parsed record {}", scratch_path.display())
                })?;
            let _ = std::fs::remove_file(&scratch_path);

            // Phase 46.5 — fail loud on a stale pre-Phase-40.1 Python install
            // (missing `ScopeOrigin.path`). Shared across every caller so
            // stale usage can never silently pass anywhere.
            crate::ros::launch_dump::ensure_python_scope_paths(&dump)?;
            dump
        }
    };

    Ok((dump, launch_path))
}

/// Parse `KEY:=VALUE` launch arguments, warning on anything that isn't.
/// Parse a launch file, with the Python backend installed.
///
/// Registration is the CALLER's job since nano-ros issue 0897 W2b — the
/// parser no longer links `libpython` itself, so somebody has to say which
/// implementation executes `.launch.py` and `$(eval ...)`. Doing it HERE
/// rather than at each verb means there is one place to change when that
/// becomes a runtime `dlopen` rather than a link.
pub fn parse_launch_file(
    path: &std::path::Path,
    args: std::collections::HashMap<String, String>,
) -> play_launch_parser::error::Result<play_launch_parser::record::RecordJson> {
    // Load the Python half against a discovered interpreter. A
    // failure here is NOT fatal: with no usable interpreter the
    // parser still resolves XML and YAML and reports
    // `PythonUnavailable` only where ROS 2 genuinely defines
    // Python — so a launch tree that uses neither still works.
    if let Err(e) = play_launch_parser_pyload::install() {
        // Record it where the USER will read it. `log::warn!` alone put the
        // one actionable sentence behind `RUST_LOG`, so the error a person
        // actually sees said "no Python backend" and stopped there — on hosts
        // with a working Python, where that reads as a build that deliberately
        // omits the feature rather than a load that failed for a nameable
        // reason.
        play_launch_parser::python_backend::set_unavailable_reason(e.to_string());
        log::warn!("Python launch support unavailable: {e}");
    }
    play_launch_parser::parse_launch_file(path, args)
}

pub fn parse_launch_arguments(args: &[String]) -> std::collections::HashMap<String, String> {
    args.iter()
        .filter_map(|arg| {
            let parts: Vec<&str> = arg.splitn(2, ":=").collect();
            if parts.len() == 2 {
                Some((parts[0].to_string(), parts[1].to_string()))
            } else {
                tracing::warn!("Ignoring invalid launch argument (expected KEY:=VALUE): {arg}");
                None
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::reclassify_launch_file_arg;

    use super::resolve_launch_file;

    // ── resolve_launch_file: recursive lookup, matching `ros2 launch` ──

    /// Serialises AMENT_PREFIX_PATH across these tests and restores it after.
    /// The variable is process-global and cargo runs tests in threads.
    struct AmentGuard {
        prev: Option<String>,
        _lock: std::sync::MutexGuard<'static, ()>,
    }

    static AMENT_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    impl AmentGuard {
        fn set(prefix: &std::path::Path) -> Self {
            let lock = AMENT_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let prev = std::env::var("AMENT_PREFIX_PATH").ok();
            unsafe { std::env::set_var("AMENT_PREFIX_PATH", prefix) };
            Self { prev, _lock: lock }
        }
    }

    impl Drop for AmentGuard {
        fn drop(&mut self) {
            match &self.prev {
                Some(v) => unsafe { std::env::set_var("AMENT_PREFIX_PATH", v) },
                None => unsafe { std::env::remove_var("AMENT_PREFIX_PATH") },
            }
        }
    }

    fn share_with(files: &[&str]) -> tempfile::TempDir {
        let tmp = tempfile::TempDir::new().unwrap();
        let share = tmp.path().join("share").join("demo_pkg");
        std::fs::create_dir_all(&share).unwrap();
        for rel in files {
            let full = share.join(rel);
            std::fs::create_dir_all(full.parent().unwrap()).unwrap();
            std::fs::write(&full, b"<launch/>").unwrap();
        }
        tmp
    }

    #[test]
    fn finds_a_launch_file_nested_below_the_launch_dir() {
        // The regression this fix exists for: `ros2 launch demo_nodes_cpp
        // talker_listener.launch.py` works because ros2 walks the whole share
        // tree; searching only `launch/` and the share root missed it.
        let tmp = share_with(&["launch/topics/talker.launch.py"]);
        let _g = AmentGuard::set(tmp.path());
        let got = resolve_launch_file("demo_pkg", Some("talker.launch.py")).unwrap();
        assert_eq!(
            got,
            tmp.path()
                .join("share/demo_pkg/launch/topics/talker.launch.py")
        );
    }

    #[test]
    fn still_finds_the_flat_launch_dir_case() {
        let tmp = share_with(&["launch/a.launch.xml"]);
        let _g = AmentGuard::set(tmp.path());
        let got = resolve_launch_file("demo_pkg", Some("a.launch.xml")).unwrap();
        assert_eq!(got, tmp.path().join("share/demo_pkg/launch/a.launch.xml"));
    }

    #[test]
    fn an_explicit_relative_path_is_taken_as_given() {
        // Not a `ros2 launch` form, but one this tool has long accepted, and
        // the documented escape from the ambiguity error below.
        let tmp = share_with(&["launch/topics/dup.launch.py", "other/dup.launch.py"]);
        let _g = AmentGuard::set(tmp.path());
        let got = resolve_launch_file("demo_pkg", Some("launch/topics/dup.launch.py")).unwrap();
        assert_eq!(
            got,
            tmp.path()
                .join("share/demo_pkg/launch/topics/dup.launch.py")
        );
    }

    #[test]
    fn an_explicit_relative_path_resolves_under_the_launch_dir_too() {
        // `demo_nodes_cpp topics/talker_listener.launch.py` — the documented
        // form, where the path is relative to `launch/`, not to the share
        // root. Making the lookup recursive briefly dropped this root.
        let tmp = share_with(&["launch/topics/talker.launch.py"]);
        let _g = AmentGuard::set(tmp.path());
        let got = resolve_launch_file("demo_pkg", Some("topics/talker.launch.py")).unwrap();
        assert_eq!(
            got,
            tmp.path()
                .join("share/demo_pkg/launch/topics/talker.launch.py")
        );
    }

    #[test]
    fn duplicate_basenames_error_rather_than_silently_picking_one() {
        // `ros2 launch` raises MultipleLaunchFilesError here. Preferring
        // `launch/` would hide the choice in exactly the case where the user
        // most needs to know one was made.
        let tmp = share_with(&["launch/dup.launch.py", "nested/deeper/dup.launch.py"]);
        let _g = AmentGuard::set(tmp.path());
        let err = resolve_launch_file("demo_pkg", Some("dup.launch.py")).unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("more than once"), "{msg}");
        // An ambiguity error that does not say what was ambiguous is useless.
        assert!(msg.contains("launch/dup.launch.py"), "{msg}");
        assert!(msg.contains("nested/deeper/dup.launch.py"), "{msg}");
    }

    #[test]
    fn a_missing_file_names_the_share_directory_that_was_searched() {
        let tmp = share_with(&[]);
        let _g = AmentGuard::set(tmp.path());
        let err = resolve_launch_file("demo_pkg", Some("nope.launch.py")).unwrap_err();
        let share = tmp.path().join("share/demo_pkg");
        assert!(err.to_string().contains(&share.display().to_string()));
    }

    #[test]
    fn a_missing_package_is_distinguished_from_a_missing_file() {
        let tmp = share_with(&[]);
        let _g = AmentGuard::set(tmp.path());
        let err = resolve_launch_file("no_such_pkg", Some("x.launch.py")).unwrap_err();
        assert!(err.to_string().contains("Package 'no_such_pkg' not found"));
    }

    fn v(items: &[&str]) -> Vec<String> {
        items.iter().map(|s| s.to_string()).collect()
    }

    /// The direct-path shape: clap put a `KEY:=VALUE` in `launch_file`.
    #[test]
    fn a_key_value_in_the_launch_file_slot_becomes_a_launch_argument() {
        let (file, args) = reclassify_launch_file_arg(Some("host:=robot1"), &v(&["mode:=fast"]));
        assert_eq!(file, None);
        // Order matters: it was the FIRST argument the user typed.
        assert_eq!(args, v(&["host:=robot1", "mode:=fast"]));
    }

    /// The package-name shape must be left exactly as clap parsed it.
    #[test]
    fn a_real_launch_file_name_is_left_alone() {
        let (file, args) = reclassify_launch_file_arg(Some("my.launch.xml"), &v(&["host:=robot1"]));
        assert_eq!(file.as_deref(), Some("my.launch.xml"));
        assert_eq!(args, v(&["host:=robot1"]));
    }

    #[test]
    fn no_launch_file_is_a_no_op() {
        let (file, args) = reclassify_launch_file_arg(None, &v(&["host:=robot1"]));
        assert_eq!(file, None);
        assert_eq!(args, v(&["host:=robot1"]));
    }
}
