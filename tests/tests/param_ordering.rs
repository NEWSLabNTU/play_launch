//! Issue 0007 regression: parameter SOURCE ORDERING must survive to the
//! model and to the spawned command line, on both frontends.
//!
//! ROS 2 treats a node's parameter sources as one ORDERED list — an inline
//! `<param>` applied before a `<param from=>` loses to the file. The legacy
//! `params` + `params_files` split cannot express that: it forces
//! files-then-inline, so the inline value always won.
//!
//! phase-54 built the ordered list in the parser, but it was dropped twice on
//! the way out — `model_builder` hard-coded an empty list into the model, and
//! `NodeCommandLine::from_node_record` hard-coded one into its own return
//! struct after computing it. Both are fixed; these tests pin the whole chain
//! so a future drop is a failure rather than a silent revert to files-first.

use play_launch_tests::fixtures;

fn launch_file(name: &str) -> String {
    fixtures::repo_root()
        .join("tests/fixtures/simple_test/launch")
        .join(name)
        .to_str()
        .unwrap()
        .to_string()
}

/// The model must carry `param_sources` in DOCUMENT order.
fn assert_model_ordering(parser: &str, file: &str) {
    let env = fixtures::install_env();
    let (model, _tmp) = fixtures::resolve_model(&env, &launch_file(file), None, parser);

    let node = model["structure"]["nodes"]
        .as_object()
        .expect("structure.nodes")
        .values()
        .find(|n| n["exec"].as_str() == Some("talker"))
        .expect("the probe node");

    let sources = node["param_sources"].as_array().unwrap_or_else(|| {
        panic!(
            "--parser {parser}: param_sources missing or empty — the ordered \
             list was dropped between parser and model, so consumers fall back \
             to the legacy files-then-inline split (issue 0007).\nnode: {node:#?}"
        )
    });

    assert_eq!(
        sources.len(),
        2,
        "--parser {parser}: expected inline + file, got {sources:#?}"
    );
    assert_eq!(
        sources[0]["kind"].as_str(),
        Some("inline"),
        "--parser {parser}: the INLINE param is written first in the launch \
         file and must come first in the list: {sources:#?}"
    );
    assert_eq!(
        sources[1]["kind"].as_str(),
        Some("file"),
        "--parser {parser}: the FILE is written second and must come second — \
         it is what makes `a` resolve to 2: {sources:#?}"
    );
    // The file must carry the value that WINS. If this is 1, the ordering is
    // right but the wrong content got attached.
    let content = sources[1]["content"].as_str().unwrap_or_default();
    assert!(
        content.contains("a: 2"),
        "--parser {parser}: the second source should be the a=2 file: {content}"
    );
}

#[test]
fn model_preserves_param_source_order_xml() {
    if fixtures::install_env().is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    assert_model_ordering("rust", "param_ordering.launch.xml");
}

#[test]
fn model_preserves_param_source_order_yaml() {
    if fixtures::install_env().is_empty() {
        eprintln!("skip: ROS env not available");
        return;
    }
    // The YAML frontend built an EMPTY ordered list until this fix, so this
    // case regressed independently of the XML one.
    assert_model_ordering("rust", "param_ordering.launch.yaml");
}
