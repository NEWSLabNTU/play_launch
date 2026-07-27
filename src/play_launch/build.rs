fn main() {
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
