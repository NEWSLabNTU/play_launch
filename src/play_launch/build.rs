fn main() {
    // Phase 85 I2: `--version` names the build, not just the crate. Done
    // before the runtime-feature early return below: every build gets it.
    embed_long_version();

    // nano-ros #285 — only the `runtime` feature links a live ROS graph.
    // Without it (`--no-default-features`) the crate is the pure resolve
    // pipeline: no rclrs, no colcon-generated message crates, and therefore
    // nothing to link against. Emitting these unconditionally made every
    // Python-free consumer fail at link with
    //   rust-lld: error: unable to find library -lplay_launch_msgs__rosidl_typesupport_c
    // even though nothing in the build referenced those symbols.
    println!("cargo:rerun-if-changed=build.rs");
    if std::env::var_os("CARGO_FEATURE_RUNTIME").is_none() {
        return;
    }

    // Add ROS library search paths from AMENT_PREFIX_PATH.
    // colcon-cargo-ros2 sources the install space before invoking cargo,
    // so AMENT_PREFIX_PATH includes both system ROS packages and
    // locally-built packages (e.g. play_launch_msgs).
    if let Ok(ament_prefix_path) = std::env::var("AMENT_PREFIX_PATH") {
        for prefix in ament_prefix_path.split(':') {
            let lib_path = std::path::Path::new(prefix).join("lib");
            if lib_path.exists() {
                println!("cargo:rustc-link-search=native={}", lib_path.display());
            }
        }
    }

    // Link against ROS 2 message libraries
    println!("cargo:rustc-link-lib=composition_interfaces__rosidl_typesupport_c");
    println!("cargo:rustc-link-lib=composition_interfaces__rosidl_generator_c");
    println!("cargo:rustc-link-lib=rcl_interfaces__rosidl_typesupport_c");
    println!("cargo:rustc-link-lib=rcl_interfaces__rosidl_generator_c");
    println!("cargo:rustc-link-lib=rosidl_runtime_c");
    println!("cargo:rustc-link-lib=play_launch_msgs__rosidl_typesupport_c");
    println!("cargo:rustc-link-lib=play_launch_msgs__rosidl_generator_c");
}

/// Phase 85 I2: `PLAY_LAUNCH_LONG_VERSION`, e.g.
/// `0.13.0 (v0.13.0-3-gabc1234, rlm v0.1.47)`.
///
/// A bare semver could not tell apart two binaries that both printed
/// `0.12.0` and disagreed on the same contract: every commit between two
/// release tags carries the old version string and a newer grammar. The git
/// describe names the commit (with `-dirty` for an uncommitted tree), and the
/// rlm tag names the grammar the checker reads. Outside a git checkout (an
/// sdist) the describe is omitted rather than invented.
fn embed_long_version() {
    let crate_version = std::env::var("CARGO_PKG_VERSION").unwrap_or_default();
    let manifest_dir = std::path::PathBuf::from(
        std::env::var("CARGO_MANIFEST_DIR").unwrap_or_else(|_| ".".to_string()),
    );

    let git = |args: &[&str]| -> Option<String> {
        let out = std::process::Command::new("git")
            .args(args)
            .current_dir(&manifest_dir)
            .output()
            .ok()?;
        if !out.status.success() {
            return None;
        }
        let s = String::from_utf8(out.stdout).ok()?.trim().to_string();
        (!s.is_empty()).then_some(s)
    };
    let describe = git(&["describe", "--tags", "--always", "--dirty"]);

    // Rebuild when the commit, the branch, a tag or the index moves.
    if let Some(git_dir) = git(&["rev-parse", "--absolute-git-dir"]) {
        let git_dir = std::path::PathBuf::from(git_dir);
        println!("cargo:rerun-if-changed={}", git_dir.join("HEAD").display());
        println!("cargo:rerun-if-changed={}", git_dir.join("index").display());
        let common = git(&["rev-parse", "--git-common-dir"])
            .map(|c| {
                let c = std::path::PathBuf::from(c);
                if c.is_absolute() { c } else { manifest_dir.join(c) }
            })
            .unwrap_or_else(|| git_dir.clone());
        println!("cargo:rerun-if-changed={}", common.join("packed-refs").display());
        println!("cargo:rerun-if-changed={}", common.join("refs/tags").display());
        if let Ok(head) = std::fs::read_to_string(git_dir.join("HEAD"))
            && let Some(r) = head.trim().strip_prefix("ref: ")
        {
            println!("cargo:rerun-if-changed={}", common.join(r).display());
        }
    }

    // The pinned `ros-launch-manifest` tag, as `just bump-manifest` writes it.
    let toml_path = manifest_dir.join("Cargo.toml");
    println!("cargo:rerun-if-changed={}", toml_path.display());
    let rlm = std::fs::read_to_string(&toml_path).ok().and_then(|toml| {
        toml.lines()
            .filter(|l| l.contains("ros-launch-manifest.git"))
            .find_map(|l| {
                let rest = &l[l.find("tag = \"")? + "tag = \"".len()..];
                Some(rest[..rest.find('"')?].to_string())
            })
    });

    let mut parts: Vec<String> = Vec::new();
    if let Some(d) = describe {
        parts.push(d);
    }
    parts.push(format!("rlm {}", rlm.as_deref().unwrap_or("unknown")));
    println!(
        "cargo:rustc-env=PLAY_LAUNCH_LONG_VERSION={crate_version} ({})",
        parts.join(", ")
    );
    println!(
        "cargo:rustc-env=PLAY_LAUNCH_RLM_TAG={}",
        rlm.as_deref().unwrap_or("unknown")
    );
}
