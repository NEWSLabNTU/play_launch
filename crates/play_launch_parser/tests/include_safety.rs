//! `<include>` recursion-safety tests.
//!
//! Three scenarios:
//!
//! 1. **Cyclic include, default (`strict_includes = false`)** — the parser
//!    must log a warning and skip the cycle, returning a record with no
//!    nodes. Preserves backward-compat for every existing `play_launch`
//!    caller (Autoware's tolerant include semantics in particular).
//!
//! 2. **Cyclic include, `strict_includes = true`** — the parser must return
//!    `ParseError::CircularInclude` with a chain that names the file
//!    closing the cycle. This is the opt-in path orchestration / lint
//!    flows use.
//!
//! 3. **Depth-cap exceeded** — already exposed via `MaxIncludeDepthExceeded`
//!    but no in-tree test pinned the boundary. Lowering
//!    `max_include_depth` to 2 with a 3-level chain exercises it.

use play_launch_parser::{ParseOptions, error::ParseError, parse_launch_file_with_options};
use std::{collections::HashMap, fs};
use tempfile::TempDir;

fn write(dir: &TempDir, name: &str, contents: &str) -> std::path::PathBuf {
    let path = dir.path().join(name);
    fs::write(&path, contents).expect("write fixture file");
    path
}

/// Build a 2-file cyclic include chain in `dir`: `a.launch.xml` → `b.launch.xml`
/// → `a.launch.xml`. Returns the entry path.
fn cycle_fixture(dir: &TempDir) -> std::path::PathBuf {
    let a = dir.path().join("a.launch.xml");
    let b = dir.path().join("b.launch.xml");
    write(
        dir,
        "a.launch.xml",
        &format!("<launch>\n  <include file=\"{}\" />\n</launch>\n", b.display()),
    );
    write(
        dir,
        "b.launch.xml",
        &format!("<launch>\n  <include file=\"{}\" />\n</launch>\n", a.display()),
    );
    a
}

#[test]
fn cycle_default_is_warn_and_skip() {
    let dir = TempDir::new().unwrap();
    let entry = cycle_fixture(&dir);

    let opts = ParseOptions::default();
    let record = parse_launch_file_with_options(&entry, HashMap::new(), opts)
        .expect("default cycle behavior is warn-and-skip");
    assert_eq!(
        record.node.len(),
        0,
        "default-mode cycle should produce no nodes (the skipped branch carries none)"
    );
}

#[test]
fn cycle_strict_returns_error() {
    let dir = TempDir::new().unwrap();
    let entry = cycle_fixture(&dir);

    let opts = ParseOptions {
        strict_includes: true,
        ..ParseOptions::default()
    };
    let err = parse_launch_file_with_options(&entry, HashMap::new(), opts)
        .expect_err("strict mode must raise CircularInclude");
    match err {
        ParseError::CircularInclude { file, chain } => {
            assert!(
                file.ends_with("a.launch.xml") || file.ends_with("b.launch.xml"),
                "expected `file` to name one of the two cyclic launches; got `{file}`"
            );
            assert!(
                chain.contains(" → "),
                "expected `chain` to render the include stack with `→`; got `{chain}`"
            );
        }
        other => panic!("expected CircularInclude, got {other:?}"),
    }
}

#[test]
fn depth_cap_exceeded_returns_error() {
    let dir = TempDir::new().unwrap();
    // 3-level chain: entry → mid → leaf. The chain length is the count of
    // included files already on the stack when an inner `<include>` is
    // processed:
    //   - entry processes `<include mid>`:  chain.len() == 0
    //   - mid   processes `<include leaf>`: chain.len() == 1 (mid is on it)
    // So `max_include_depth = 1` trips at the leaf-include (the file that
    // *would* push the chain past the cap is the one named in the error).
    let leaf = dir.path().join("leaf.launch.xml");
    let mid = dir.path().join("mid.launch.xml");
    let entry = dir.path().join("entry.launch.xml");
    write(
        &dir,
        "leaf.launch.xml",
        "<launch>\n  <node pkg=\"demo\" exec=\"talker\" />\n</launch>\n",
    );
    write(
        &dir,
        "mid.launch.xml",
        &format!(
            "<launch>\n  <include file=\"{}\" />\n</launch>\n",
            leaf.display()
        ),
    );
    write(
        &dir,
        "entry.launch.xml",
        &format!(
            "<launch>\n  <include file=\"{}\" />\n</launch>\n",
            mid.display()
        ),
    );

    let opts = ParseOptions {
        max_include_depth: 1,
        ..ParseOptions::default()
    };
    let err = parse_launch_file_with_options(&entry, HashMap::new(), opts)
        .expect_err("depth-cap should reject the 2nd-level include");
    match err {
        ParseError::MaxIncludeDepthExceeded { max, file } => {
            assert_eq!(max, 1);
            assert!(
                file.ends_with("leaf.launch.xml"),
                "expected the file that closed past the cap to be `leaf.launch.xml`; got `{file}`"
            );
        }
        other => panic!("expected MaxIncludeDepthExceeded, got {other:?}"),
    }
}

#[test]
fn deep_chain_under_cap_resolves_to_leaf() {
    let dir = TempDir::new().unwrap();
    let leaf = dir.path().join("leaf.launch.xml");
    let mid = dir.path().join("mid.launch.xml");
    let entry = dir.path().join("entry.launch.xml");
    write(
        &dir,
        "leaf.launch.xml",
        "<launch>\n  <node pkg=\"demo\" exec=\"talker\" />\n</launch>\n",
    );
    write(
        &dir,
        "mid.launch.xml",
        &format!(
            "<launch>\n  <include file=\"{}\" />\n</launch>\n",
            leaf.display()
        ),
    );
    write(
        &dir,
        "entry.launch.xml",
        &format!(
            "<launch>\n  <include file=\"{}\" />\n</launch>\n",
            mid.display()
        ),
    );

    let opts = ParseOptions::default();
    let record = parse_launch_file_with_options(&entry, HashMap::new(), opts).expect("plan ok");
    assert_eq!(record.node.len(), 1, "leaf node should land");
    assert_eq!(record.node[0].executable, "talker");
}
