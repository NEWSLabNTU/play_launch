//! Phase 85 I3: embed the pinned `ros-launch-manifest` tag, so a contract
//! refusal can name the grammar this checker reads. The tag is read from the
//! workspace manifest, where `just bump-manifest` writes it; outside this
//! workspace (a vendored copy) it is `unknown` rather than guessed.
fn main() {
    let dir = std::path::PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let ws = dir.join("../Cargo.toml");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed={}", ws.display());
    let tag = std::fs::read_to_string(&ws)
        .ok()
        .and_then(|toml| {
            toml.lines()
                .filter(|l| l.contains("ros-launch-manifest.git"))
                .find_map(|l| {
                    let rest = &l[l.find("tag = \"")? + "tag = \"".len()..];
                    Some(rest[..rest.find('"')?].to_string())
                })
        })
        .unwrap_or_else(|| "unknown".to_string());
    println!("cargo:rustc-env=RLM_PINNED_TAG={tag}");
}
