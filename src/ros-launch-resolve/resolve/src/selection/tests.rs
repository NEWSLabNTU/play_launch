use std::collections::BTreeMap;

use ros_launch_manifest_model::{NodeInstance, ParamValue, ScopeInfo, SystemModel, TopicWiring};

use super::*;

fn scope(model: &mut SystemModel, key: &str, parent: Option<&str>, args: &[(&str, &str)]) {
    let (pkg, file) = key.split('#').next().unwrap().split_once('/').unwrap();
    model.structure.scopes.insert(
        key.to_string(),
        ScopeInfo {
            parent: parent.map(str::to_string),
            package: Some(pkg.to_string()),
            file: Some(file.to_string()),
            args: args
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect::<BTreeMap<_, _>>(),
            ..Default::default()
        },
    );
}

fn node(model: &mut SystemModel, key: &str, scope: &str, f: impl FnOnce(&mut NodeInstance)) {
    let mut n = NodeInstance {
        scope: scope.to_string(),
        pkg: Some("p".into()),
        exec: Some("e".into()),
        node_name: Some(key.rsplit('/').next().unwrap().to_string()),
        ..Default::default()
    };
    f(&mut n);
    model.structure.nodes.insert(key.to_string(), n);
}

fn topic(model: &mut SystemModel, t: &str, pubs: &[&str], subs: &[&str]) {
    model.structure.topics.insert(
        t.to_string(),
        TopicWiring {
            msg_type: "m/msg/T".into(),
            publishers: pubs.iter().map(|s| s.to_string()).collect(),
            subscribers: subs.iter().map(|s| s.to_string()).collect(),
        },
    );
}

/// A small tree with the shapes Autoware has: a container declared in one
/// file loading composables from another, one file included twice that
/// differs by an argument, a lifecycle manager, a `/clock` publisher.
fn model() -> SystemModel {
    let mut m = SystemModel::default();
    scope(&mut m, "top/top.launch.xml", None, &[("x", "1")]);
    scope(
        &mut m,
        "sens/sensing.launch.xml",
        Some("top/top.launch.xml"),
        &[("x", "1")],
    );
    scope(
        &mut m,
        "perc/perception.launch.xml",
        Some("top/top.launch.xml"),
        &[("x", "1")],
    );
    scope(
        &mut m,
        "mon/monitor.launch.xml#5",
        Some("top/top.launch.xml"),
        &[("x", "1"), ("suffix", "map")],
    );
    scope(
        &mut m,
        "mon/monitor.launch.xml#6",
        Some("top/top.launch.xml"),
        &[("x", "1"), ("suffix", "pose")],
    );
    scope(
        &mut m,
        "sim/sim.launch.xml",
        Some("top/top.launch.xml"),
        &[],
    );
    scope(
        &mut m,
        "other/sensing.launch.xml",
        Some("sim/sim.launch.xml"),
        &[],
    );

    node(&mut m, "/container", "top/top.launch.xml", |_| {});
    node(&mut m, "/manager", "top/top.launch.xml", |n| {
        n.params.insert(
            "node_names".into(),
            ParamValue::StrList(vec!["perception/det".into(), "sensing/driver".into()]),
        );
    });
    node(&mut m, "/sensing/driver", "sens/sensing.launch.xml", |_| {});
    node(&mut m, "/sensing/filter", "sens/sensing.launch.xml", |n| {
        n.exec = None;
        n.plugin = Some("f::Filter".into());
        n.container = Some("/container".into());
    });
    node(
        &mut m,
        "/perception/det",
        "perc/perception.launch.xml",
        |n| {
            n.params
                .insert("use_sim_time".into(), ParamValue::Bool(true));
        },
    );
    node(&mut m, "/mon/map", "mon/monitor.launch.xml#5", |_| {});
    node(&mut m, "/mon/pose", "mon/monitor.launch.xml#6", |_| {});
    node(&mut m, "/sim/clock", "sim/sim.launch.xml", |_| {});
    node(
        &mut m,
        "/sim/fake_lidar",
        "other/sensing.launch.xml",
        |_| {},
    );

    topic(
        &mut m,
        "/sensing/points",
        &["/sensing/driver/output"],
        &["/sensing/filter/input"],
    );
    topic(
        &mut m,
        "/sensing/filtered",
        &["/sensing/filter/output"],
        &["/perception/det/input"],
    );
    topic(
        &mut m,
        "/clock",
        &["/sim/clock/clock"],
        &["/perception/det/clock"],
    );
    m
}

fn eval(m: &SystemModel, only: &[&str], exclude: &[&str]) -> eyre::Result<Selection> {
    let s = |v: &[&str]| v.iter().map(|x| x.to_string()).collect::<Vec<_>>();
    evaluate(
        m,
        &SelectionSpec::parse(&s(only), &s(exclude), false, false)?,
    )
}

#[test]
fn parse_and_canonical_form() {
    let cases = [
        ("/a/b", "node:/a/b"),
        ("node:/a/**", "node:/a/**"),
        ("sensing.launch.xml", "scope:sensing.launch.xml"),
        ("sens/sensing.launch.xml", "scope:sens/sensing.launch.xml"),
        (
            "monitor.launch.xml[suffix=map]",
            "scope:monitor.launch.xml[suffix=map]",
        ),
        ("scope:monitor.launch.xml#6", "scope:monitor.launch.xml#6"),
        (
            "monitor.launch.xml[under=top/top.launch.xml][nth=2]",
            "scope:monitor.launch.xml[under=top/top.launch.xml][nth=2]",
        ),
        ("plugin:autoware::x::*", "plugin:autoware::x::*"),
        ("container:/c", "container:/c"),
    ];
    for (input, canon) in cases {
        let s: Selector = input.parse().unwrap_or_else(|e| panic!("{input}: {e}"));
        assert_eq!(s.to_string(), canon);
        let again: Selector = canon.parse().unwrap();
        assert_eq!(again, s, "{canon} round-trips");
    }
    for bad in [
        "",
        "rviz2",
        "x.launch.xml[foo]",
        "x.launch.xml[nth=0]",
        "x.launch.xml#a",
        "scope:",
    ] {
        assert!(bad.parse::<Selector>().is_err(), "{bad:?} is refused");
    }
}

#[test]
fn only_keeps_a_subtree_and_the_container_it_needs() {
    let m = model();
    let s = eval(&m, &["sens/sensing.launch.xml"], &[]).unwrap();
    assert_eq!(s.kept, ["/container", "/sensing/driver", "/sensing/filter"]);
    assert_eq!(s.added.len(), 1);
    assert_eq!(s.added[0].container, "/container");
    assert_eq!(s.added[0].needed_by, ["/sensing/filter"]);
}

#[test]
fn no_deps_leaves_the_container_out_and_says_so() {
    let m = model();
    let spec = SelectionSpec::parse(&["sens/sensing.launch.xml".into()], &[], true, false).unwrap();
    let s = evaluate(&m, &spec).unwrap();
    assert_eq!(s.kept, ["/sensing/driver", "/sensing/filter"]);
    assert!(s.warnings.iter().any(|w| w.contains("--no-deps")));
}

#[test]
fn excluding_a_needed_container_is_an_error() {
    let m = model();
    let e = eval(&m, &["sens/sensing.launch.xml"], &["/container"]).unwrap_err();
    assert!(
        e.to_string().contains("needs its container `/container`"),
        "{e}"
    );
}

#[test]
fn a_file_in_two_packages_must_name_its_package() {
    let m = model();
    let e = eval(&m, &["sensing.launch.xml"], &[])
        .unwrap_err()
        .to_string();
    assert!(e.contains("2 packages"), "{e}");
    assert!(
        e.contains("scope:other/sensing.launch.xml") && e.contains("scope:sens/sensing.launch.xml"),
        "{e}"
    );
}

#[test]
fn a_repeated_file_must_say_which_inclusion() {
    let m = model();
    let e = eval(&m, &[], &["monitor.launch.xml"])
        .unwrap_err()
        .to_string();
    assert!(e.contains("included 2 times"), "{e}");
    // The choices are listed already disambiguated, by the argument that
    // differs, and each one evaluates.
    assert!(
        e.contains("scope:mon/monitor.launch.xml[suffix=map]"),
        "{e}"
    );
    assert!(
        e.contains("scope:mon/monitor.launch.xml[suffix=pose]"),
        "{e}"
    );
    assert!(e.contains("[*] for all 2"), "{e}");

    let s = eval(&m, &[], &["mon/monitor.launch.xml[suffix=map]"]).unwrap();
    assert_eq!(s.excluded, ["/mon/map"]);
    let s = eval(&m, &[], &["monitor.launch.xml[*]"]).unwrap();
    assert_eq!(s.excluded, ["/mon/map", "/mon/pose"]);
    let s = eval(&m, &[], &["monitor.launch.xml#6"]).unwrap();
    assert_eq!(s.excluded, ["/mon/pose"]);
    let s = eval(&m, &[], &["monitor.launch.xml[nth=2]"]).unwrap();
    assert_eq!(s.excluded, ["/mon/pose"]);
    assert!(s.warnings.iter().any(|w| w.contains("by position")));
    let s = eval(
        &m,
        &[],
        &["monitor.launch.xml[under=top/top.launch.xml][suffix=pose]"],
    )
    .unwrap();
    assert_eq!(s.excluded, ["/mon/pose"]);
}

#[test]
fn a_selector_matching_nothing_is_an_error() {
    let m = model();
    for sel in [
        "/nope",
        "nope.launch.xml",
        "monitor.launch.xml[suffix=nope]",
        "pkg:nope",
    ] {
        let e = eval(&m, &[sel], &[]).unwrap_err().to_string();
        assert!(e.contains("matches no"), "{sel}: {e}");
    }
}

#[test]
fn node_globs_keep_star_inside_a_segment() {
    let m = model();
    let s = eval(&m, &["/sensing/*"], &[]).unwrap();
    assert_eq!(s.matched[0].count, 2);
    let e = eval(&m, &["/*"], &["/sim/**"]);
    // `/*` is the root-level nodes only.
    assert_eq!(e.unwrap().matched[0].count, 2);
    let s = eval(&m, &["/**"], &[]).unwrap();
    assert_eq!(s.matched[0].count, m.structure.nodes.len());
}

#[test]
fn cut_inputs_and_the_named_traps() {
    let m = model();
    // Keep perception and the manager; the filter and the clock are cut.
    let s = eval(&m, &["perc/perception.launch.xml", "/manager"], &[]).unwrap();
    let topics: Vec<&str> = s.cut_inputs.iter().map(|c| c.topic.as_str()).collect();
    assert_eq!(topics, ["/clock", "/sensing/filtered"]);
    assert!(
        s.warnings.iter().any(|w| w.contains("use_sim_time on 1")),
        "{:?}",
        s.warnings
    );
    assert!(
        s.warnings
            .iter()
            .any(|w| w.contains("manages `/sensing/driver`")),
        "{:?}",
        s.warnings
    );
    assert!(
        !s.warnings
            .iter()
            .any(|w| w.contains("manages `/perception/det`"))
    );
    let text = render_explain(&m, &s);
    assert!(
        text.contains("ros2 bag play <bag> --topics /clock /sensing/filtered"),
        "{text}"
    );
}

#[test]
fn start_none_keeps_nothing_and_refuses_selectors() {
    let m = model();
    let s = evaluate(&m, &SelectionSpec::parse(&[], &[], false, true).unwrap()).unwrap();
    assert!(s.kept.is_empty());
    assert_eq!(s.excluded.len(), m.structure.nodes.len());
    assert!(SelectionSpec::parse(&["/a".into()], &[], false, true).is_err());
}

#[test]
fn stable_selectors_evaluate_to_their_own_inclusion() {
    let m = model();
    let tree = TreeIndex::new(&m);
    for key in m.structure.scopes.keys() {
        let sel = tree.stable_selector(key);
        let got = match_scope(&tree, &m, &sel).unwrap_or_else(|e| panic!("{key}: {sel}: {e}"));
        assert_eq!(got, [key.clone()], "{sel}");
    }
}

#[test]
fn explain_marks_the_tree() {
    let m = model();
    let s = eval(&m, &["sens/sensing.launch.xml"], &[]).unwrap();
    let text = render_explain(&m, &s);
    assert!(text.contains("[~] top/top.launch.xml  3/9"), "{text}");
    assert!(text.contains("[+] /container"), "{text}");
    assert!(text.contains("[x] sens/sensing.launch.xml  2/2"), "{text}");
    assert!(
        text.contains("[ ] mon/monitor.launch.xml  (suffix=map)  0/1"),
        "{text}"
    );
}

#[test]
fn describe_gives_selectors_that_evaluate_back_to_the_set() {
    let m = model();
    let keys: Vec<String> = m.structure.nodes.keys().cloned().collect();
    // Every subset of this small model: the description must select it
    // exactly, and use whole inclusions where it can.
    for mask in 0u32..(1 << keys.len()) {
        let kept: HashSet<String> = keys
            .iter()
            .enumerate()
            .filter(|(i, _)| mask & (1 << i) != 0)
            .map(|(_, k)| k.clone())
            .collect();
        let spec = describe(&m, &kept);
        let got = evaluate(&m, &spec).unwrap_or_else(|e| panic!("{spec:?}: {e}"));
        let got: HashSet<String> = got.kept.into_iter().collect();
        assert_eq!(got, kept, "{:?}", spec.to_flags());
    }
    let kept: HashSet<String> = ["/sensing/driver", "/sensing/filter", "/container"]
        .iter()
        .map(|s| s.to_string())
        .collect();
    assert_eq!(
        describe(&m, &kept).to_flags(),
        [
            "--only node:/container",
            "--only scope:sens/sensing.launch.xml"
        ]
    );
}
