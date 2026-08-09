//! Golden-file round-trip: the fixture must parse, survive
//! serialize→reparse unchanged, and carry the values the design doc promises.

use pretty_assertions::assert_eq;
use ros_launch_manifest_model::*;

fn golden() -> SystemModel {
    let yaml = include_str!("golden/perception.system_model.yaml");
    SystemModel::from_yaml_str(yaml).expect("golden fixture must parse")
}

#[test]
fn roundtrip_is_lossless() {
    let model = golden();
    let emitted = model.to_yaml_string().unwrap();
    let reparsed = SystemModel::from_yaml_str(&emitted).expect("re-emitted YAML must parse");
    assert_eq!(model, reparsed);
}

#[test]
fn serialization_is_deterministic() {
    let model = golden();
    assert_eq!(
        model.to_yaml_string().unwrap(),
        model.to_yaml_string().unwrap()
    );
    // BTreeMap ordering: re-emitting a reparsed model is byte-identical —
    // the property the provenance hash relies on.
    let emitted = model.to_yaml_string().unwrap();
    let again = SystemModel::from_yaml_str(&emitted)
        .unwrap()
        .to_yaml_string()
        .unwrap();
    assert_eq!(emitted, again);
}

#[test]
fn meta_carries_binding_and_provenance() {
    let m = golden().meta;
    assert_eq!(m.version, SCHEMA_VERSION);
    assert_eq!(m.args["mode"], "velodyne");
    assert_eq!(m.inputs.len(), 2);
    assert_eq!(m.inputs[0].path, "perception.yaml");
    assert_eq!(m.resolver.unwrap().tool, "play_launch");
    assert_eq!(
        m.record.as_ref().unwrap().path,
        "perception.record.json",
        "the bound spawn-info companion"
    );
    assert_eq!(
        m.diagnostics.len(),
        1,
        "warnings embed; errors refuse emission"
    );
}

#[test]
fn structure_layer_resolved_shapes() {
    let s = golden().structure;
    // scope tree with parent links
    assert_eq!(
        s.scopes["/perception/detection"].parent.as_deref(),
        Some("/perception")
    );
    // plain node vs composable vs lifecycle
    let det = &s.nodes["/perception/detection/detector"];
    assert_eq!(det.exec.as_deref(), Some("detector_node"));
    assert!(!det.lifecycle);
    // Phase 46.1b — launch spawn inputs (remaps/ros_args/respawn/env).
    assert_eq!(
        det.remaps,
        vec![Remap {
            from: "/points".to_string(),
            to: "/sensing/lidar/points".to_string(),
        }]
    );
    assert_eq!(
        det.ros_args,
        vec![
            "--log-level".to_string(),
            "detector_node:=debug".to_string()
        ]
    );
    assert_eq!(det.respawn, Some(true));
    assert_eq!(det.respawn_delay, Some(2.5));
    assert_eq!(
        det.env,
        vec![EnvVar {
            name: "CUDA_VISIBLE_DEVICES".to_string(),
            value: "0".to_string(),
        }]
    );
    assert_eq!(
        s.nodes["/sensing/imu_node"].criticality.as_deref(),
        Some("high")
    );
    // R1-M4/M6 — resolved params + lifecycle autostart
    let imu = &s.nodes["/sensing/imu_node"];
    assert_eq!(imu.params["rate_hz"], ParamValue::Int(100));
    assert_eq!(
        imu.params["frame_id"],
        ParamValue::Str("imu_link".to_string())
    );
    assert_eq!(imu.params["use_filter"], ParamValue::Bool(true));
    assert_eq!(
        imu.params["offsets"],
        ParamValue::StrList(vec!["0.1".to_string(), "0.2".to_string()])
    );
    assert_eq!(
        s.nodes["/perception/tracker"].lifecycle_autostart,
        Some(Autostart::Active)
    );
    let tracker = &s.nodes["/perception/tracker"];
    assert_eq!(tracker.plugin.as_deref(), Some("tracker::TrackerNode"));
    assert_eq!(
        tracker.container.as_deref(),
        Some("/perception/pipeline_container")
    );
    assert!(tracker.lifecycle);
    // no explicit launch-spawn-fields on this node → additive-schema defaults.
    assert!(tracker.remaps.is_empty());
    assert!(tracker.ros_args.is_empty());
    assert_eq!(tracker.respawn, None);
    assert_eq!(tracker.respawn_delay, None);
    assert!(tracker.env.is_empty());
    // wiring uses "<node FQN>/<endpoint>" refs
    assert_eq!(
        s.topics["/perception/objects"].publishers,
        vec!["/perception/detection/detector/objects"]
    );
    assert_eq!(
        s.services["/perception/trigger"].srv_type,
        "std_srvs/srv/Trigger"
    );
}

#[test]
fn contracts_layer_numbers() {
    let c = golden().contracts;
    assert_eq!(
        c.pub_endpoints["/perception/detection/detector/objects"].jitter_ms,
        Some(5.0)
    );
    // R1-M5 — per-endpoint QoS
    assert_eq!(
        c.pub_endpoints["/perception/detection/detector/objects"]
            .qos
            .as_ref()
            .unwrap()
            .reliability
            .as_deref(),
        Some("best_effort")
    );
    let sub = &c.sub_endpoints["/perception/detection/detector/pointcloud"];
    assert_eq!(sub.max_age_ms, Some(100.0));
    assert!(!sub.state);
    assert!(c.sub_endpoints["/perception/tracker/input"].state);
    assert_eq!(
        c.srv_endpoints["/perception/detection/detector/trigger"].max_response_ms,
        Some(100.0)
    );
    // node path = processing budget; scope path = E2E with correlation + drops
    assert_eq!(
        c.node_paths["/perception/detection/detector/main"].max_latency_ms,
        Some(30.0)
    );
    let e2e = &c.scope_paths["/perception/e2e"];
    assert_eq!(e2e.max_latency_ms, Some(85.0));
    assert_eq!(e2e.correlation, Some(Correlation::Timestamp));
    assert_eq!(e2e.drop.as_ref().unwrap().max_drop_rate, Some(0.08));
    // topic channel contract
    let pc = &c.topics["/sensing/pointcloud"];
    assert_eq!(pc.max_transport_ms, Some(5.0));
    assert_eq!(
        pc.qos.as_ref().unwrap().reliability.as_deref(),
        Some("best_effort")
    );
}

#[test]
fn execution_layer_slices() {
    let e = golden().execution;
    // the two consumer slices
    assert_eq!(
        e.deploy["/perception/detection/detector"].target,
        Some(Target::Linux)
    );
    assert_eq!(
        e.deploy["/sensing/imu_node"].target,
        Some(Target::Mcu {
            board: "stm32f4".into()
        })
    );
    // R1-M1/M2/M3 — deploy fields, transports, bridges, features
    let imu = &e.deploy["/sensing/imu_node"];
    assert_eq!(imu.domain, Some(7));
    assert_eq!(imu.locator.as_deref(), Some("tcp/10.0.2.1:7447"));
    assert_eq!(imu.rmw.as_deref(), Some("zenoh"));
    assert_eq!(imu.extra["optimize"], ExtraValue::Str("size".to_string()));
    assert_eq!(
        imu.extra["features"],
        ExtraValue::StrList(vec!["safety".to_string()])
    );

    let t = &e.transports[0];
    assert_eq!(t.kind, "ethernet");
    assert_eq!(t.id.as_deref(), Some("eth0"));
    assert_eq!(t.mac.as_deref(), Some("02:00:00:00:00:01"));
    assert_eq!(t.domain, Some(7));
    assert_eq!(e.bridges[0].from, "eth0");
    assert_eq!(e.bridges[0].topics, vec!["/perception/objects"]);
    assert_eq!(e.features, vec!["safety"]);

    // tier table (sched crate TierDef schema) + per-callback-group binding
    let high = &e.tiers["high"];
    assert_eq!(high.class.as_deref(), Some("real_time"));
    assert_eq!(high.deadline_us, Some(2000));
    assert_eq!(high.spin_period_us, Some(1000));
    let posix = high.posix.as_ref().unwrap();
    assert_eq!(posix.priority, 80);
    assert_eq!(posix.sched_class.as_deref(), Some("SCHED_FIFO"));
    assert_eq!(posix.core, Some(2));
    assert_eq!(high.threadx.as_ref().unwrap().preempt_threshold, Some(4));
    assert_eq!(e.bindings["/sensing/imu_node/ctrl"], "high");
}

/// Phase 46.1b backward-compat: a `NodeInstance` written before
/// remaps/ros_args/respawn/env existed (no such keys at all) must still
/// parse, with every new field defaulting to empty/`None` — the
/// additive-schema guarantee `docs/design/unified-system-model.md` promises
/// nano-ros (it vendors this crate and must keep reading old models).
#[test]
fn node_instance_without_launch_fields_parses_with_defaults() {
    let yaml = "\
scope: /perception
pkg: lidar_centerpoint
exec: detector_node
";
    let node: NodeInstance = serde_yaml_ng::from_str(yaml).unwrap();
    assert_eq!(node.exec.as_deref(), Some("detector_node"));
    assert!(node.remaps.is_empty());
    assert!(node.ros_args.is_empty());
    assert_eq!(node.respawn, None);
    assert_eq!(node.respawn_delay, None);
    assert!(node.env.is_empty());

    // and re-emitting it doesn't invent any of the new keys (no noise for
    // artifacts that never carried them).
    let re_emitted = serde_yaml_ng::to_string(&node).unwrap();
    assert!(!re_emitted.contains("remaps:"), "{re_emitted}");
    assert!(!re_emitted.contains("ros_args:"), "{re_emitted}");
    assert!(!re_emitted.contains("respawn:"), "{re_emitted}");
    assert!(!re_emitted.contains("env:"), "{re_emitted}");
}
