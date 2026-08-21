//! Phase 64 — the private control channel between play_launch and its OWN
//! container binary.
//!
//! play_launch speaks `composition_interfaces/srv/LoadNode` to a container it
//! does not own, because that is the only interface such a container has. It
//! is NOT obliged to speak it to `play_launch_container`, where both ends of
//! the conversation are this repository's code — and on an edge machine that
//! service call is the single most congested thing on the box precisely when
//! it matters: ~150 fresh processes discovering each other while tens of
//! LoadNode round-trips queue behind them. The motivating launch produced 14
//! `LoadNode service call timed out after 30s` warnings and 8
//! `ComponentEvent LOADED not received after 10s` warnings for **zero** actual
//! failures (`docs/roadmap/phase-64-isolated-container-ipc.md`).
//!
//! The transport is a `socketpair(2)` created before the container is forked,
//! so there is no path to name, no listener to bind, no discovery to wait for,
//! and no cleanup: the channel exists for exactly as long as the container
//! process does. The child's fd number is handed down in
//! [`CONTROL_FD_ENV`]; play_launch keeps `FD_CLOEXEC` set on its copy and
//! clears it only in the child being exec'd, so the fd cannot leak into an
//! unrelated node spawned concurrently.
//!
//! Framing matches the io-helper protocol already in this module: a 4-byte
//! little-endian length followed by the payload. The payload is JSON rather
//! than bincode because the far end is C++ — a hand-written encoder there has
//! to be readable by a human debugging a launch, and the traffic is a few
//! hundred bytes per composable, once.

use serde::{Deserialize, Serialize};

/// Protocol version. Bumped only for an incompatible change; both ends
/// exchange it in `Hello` and fall back to the LoadNode service on mismatch,
/// because a supervisor from wheel N will meet a container from wheel N-1.
///
/// v2 (phase 64 W2) added `query`/`status` and `cancel`, and the `phase` and
/// `cpu_ms` fields on `constructing`. The bump is deliberate even though the
/// additions are backwards-compatible on the wire: a v1 container would
/// silently ignore a `query`, and the supervisor would read that silence as
/// the very thing this version exists to stop inferring.
pub const CONTROL_PROTOCOL_VERSION: u32 = 2;

/// Environment variable naming the inherited socket fd in the container.
pub const CONTROL_FD_ENV: &str = "PLAY_LAUNCH_CONTROL_FD";

/// Refuse a frame larger than this. A load request carrying every parameter of
/// a large node is a few tens of kB; a megabyte means the stream desynced.
pub const MAX_FRAME_BYTES: usize = 8 * 1024 * 1024;

/// One ROS parameter, in the shape `rcl_interfaces/msg/Parameter` has.
///
/// Only the field matching `type_` is serialized, which keeps a load frame
/// legible and small. Type inference is NOT redone at the far end: the
/// supervisor has already run `parameter_conversion::convert_parameters_to_ros`
/// (the same call the LoadNode path makes), so the container receives exactly
/// the typed values the service path would have delivered.
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
pub struct ControlParam {
    pub name: String,
    /// `rcl_interfaces/msg/ParameterType` constant.
    #[serde(rename = "type")]
    pub type_: u8,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bool_value: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub integer_value: Option<i64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub double_value: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub string_value: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub byte_array_value: Option<Vec<u8>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bool_array_value: Option<Vec<bool>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub integer_array_value: Option<Vec<i64>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub double_array_value: Option<Vec<f64>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub string_array_value: Option<Vec<String>>,
}

impl From<&rcl_interfaces::msg::Parameter> for ControlParam {
    fn from(p: &rcl_interfaces::msg::Parameter) -> Self {
        use rcl_interfaces::msg::ParameterType as T;
        let v = &p.value;
        let mut out = ControlParam {
            name: p.name.clone(),
            type_: v.type_,
            ..Default::default()
        };
        match v.type_ {
            T::PARAMETER_BOOL => out.bool_value = Some(v.bool_value),
            T::PARAMETER_INTEGER => out.integer_value = Some(v.integer_value),
            T::PARAMETER_DOUBLE => out.double_value = Some(v.double_value),
            T::PARAMETER_STRING => out.string_value = Some(v.string_value.clone()),
            T::PARAMETER_BYTE_ARRAY => out.byte_array_value = Some(v.byte_array_value.clone()),
            T::PARAMETER_BOOL_ARRAY => out.bool_array_value = Some(v.bool_array_value.clone()),
            T::PARAMETER_INTEGER_ARRAY => {
                out.integer_array_value = Some(v.integer_array_value.clone())
            }
            T::PARAMETER_DOUBLE_ARRAY => {
                out.double_array_value = Some(v.double_array_value.clone())
            }
            T::PARAMETER_STRING_ARRAY => {
                out.string_array_value = Some(v.string_array_value.clone())
            }
            _ => {}
        }
        out
    }
}

/// Where a load is, as the container sees it. This is the fact the supervisor
/// cannot infer from silence, and the reason `query` exists.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LoadPhase {
    /// Accepted, waiting for a spawn slot (the container's memory gate holds a
    /// fork for up to 120 s). Nothing has been forked yet.
    Queued,
    /// Forked; the child's constructor is running.
    Constructing,
    /// Constructed and running.
    Loaded,
    /// Terminal failure the container already reported.
    Failed,
    /// The container has NO record of this load: it was never accepted, or
    /// everything for it is gone. The ONLY answer that permits a resend, and
    /// it is a positive statement by the process owner rather than an
    /// inference from a timeout.
    Unknown,
}

/// play_launch → container.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "t", rename_all = "snake_case")]
pub enum SupervisorMsg {
    /// Sent once, immediately after the container is spawned.
    Hello { protocol: u32, supervisor_pid: u32 },
    /// Load one composable. The container answers `Accepted` (with the
    /// pre-assigned id) or `Rejected`, and later `Loaded`/`LoadFailed`.
    Load {
        /// Correlates `Accepted`/`Rejected` back to this request. Everything
        /// after acceptance is keyed by `unique_id` instead.
        seq: u64,
        package: String,
        plugin: String,
        node_name: String,
        node_namespace: String,
        #[serde(default)]
        remap_rules: Vec<String>,
        #[serde(default)]
        parameters: Vec<ControlParam>,
        #[serde(default)]
        extra_arguments: Vec<ControlParam>,
        /// Per-node log directory (isolated mode redirects the child's
        /// stdout/stderr into it). Empty when there is none.
        #[serde(default)]
        log_dir: String,
    },
    /// "What is the state of this load?" — asked when reporting has gone
    /// quiet. Either key may be used: `unique_id` once the load was accepted,
    /// `seq` before that (the container keeps the mapping).
    Query {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        seq: Option<u64>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        unique_id: Option<u64>,
    },
    /// Stop this load and destroy whatever it created. The container answers
    /// `LoadFailed { cancelled: true }` once nothing is running for the id —
    /// which is what makes a subsequent resend safe rather than a double load.
    Cancel { unique_id: u64, reason: String },
}

/// container → play_launch.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "t", rename_all = "snake_case")]
pub enum ContainerMsg {
    /// Sent as the first thing the container does, BEFORE `rclcpp::init` — so
    /// the supervisor learns the channel is live without waiting for DDS.
    Hello {
        protocol: u32,
        pid: i32,
        /// Whether this container accepts `Load` over the socket. False for
        /// `observable`, where loading runs on the manager's executor thread
        /// and a socket-thread load would race the manager's own bookkeeping;
        /// there the socket carries status only and LoadNode stays in charge.
        loads_over_socket: bool,
    },
    /// The load request was accepted and `unique_id` pre-assigned. This is the
    /// exact information the LoadNode response carried, delivered without it.
    Accepted { seq: u64, unique_id: u64 },
    /// The load request was refused before an id was assigned (unknown plugin).
    Rejected { seq: u64, error: String },
    /// The component is constructed and running.
    Loaded {
        unique_id: u64,
        full_node_name: String,
        /// Isolated mode: the child process. 0 when there is no separate one.
        #[serde(default)]
        pid: i32,
        /// `/proc/<pid>/stat` field 22, for the PID-reuse guard on sched apply.
        #[serde(default)]
        start_time: u64,
    },
    /// The component failed to construct.
    LoadFailed {
        unique_id: u64,
        error: String,
        /// True when this failure is the confirmed completion of a `Cancel`.
        /// The supervisor treats it as "nothing is running for this id", which
        /// is the precondition for a safe resend.
        #[serde(default)]
        cancelled: bool,
    },
    /// Liveness while a constructor runs. This is what replaces the timeout
    /// heuristics: a composable that is slow is now DISTINGUISHABLE from one
    /// that is wedged, because the container says so on a channel that cannot
    /// drop it.
    Constructing {
        unique_id: u64,
        /// 0 while `phase` is `queued` — nothing has been forked yet.
        pid: i32,
        elapsed_ms: u64,
        plugin: String,
        #[serde(default = "constructing_phase")]
        phase: LoadPhase,
        /// The child's CPU time (utime+stime). Evidence for the stall check:
        /// a constructor that is burning CPU is working. The converse does NOT
        /// hold — a constructor blocked on a service that has not come up yet
        /// burns nothing while behaving correctly — so this informs a report,
        /// and only an opted-in policy acts on it.
        #[serde(default)]
        cpu_ms: u64,
    },
    /// The answer to a `Query`.
    Status {
        /// Echoed back so a probe by `seq` can be matched before an id exists.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        seq: Option<u64>,
        #[serde(default)]
        unique_id: u64,
        phase: LoadPhase,
        #[serde(default)]
        pid: i32,
        #[serde(default)]
        elapsed_ms: u64,
        #[serde(default)]
        cpu_ms: u64,
        #[serde(default)]
        plugin: String,
        /// Whether `Cancel` can act on it. False for a composable with no
        /// process of its own — killing it would take its container's
        /// siblings with it (phase 65's inline mode).
        #[serde(default)]
        cancellable: bool,
        /// Set when the load already finished; lets a `Status` resolve an
        /// entry whose `Loaded` frame was somehow never processed.
        #[serde(default)]
        full_node_name: String,
    },
    /// A loaded component was unloaded (still driven by the UnloadNode
    /// service — see the module docs on scope).
    Unloaded {
        unique_id: u64,
        #[serde(default)]
        full_node_name: String,
    },
    /// A loaded component's process died.
    Crashed {
        unique_id: u64,
        #[serde(default)]
        full_node_name: String,
        error: String,
        #[serde(default)]
        pid: i32,
    },
}

/// A `constructing` frame from a v1 container carries no phase; it was only
/// ever sent once a child existed.
fn constructing_phase() -> LoadPhase {
    LoadPhase::Constructing
}

/// Encode one message as a length-prefixed JSON frame.
pub fn encode_frame<T: Serialize>(msg: &T) -> Result<Vec<u8>, serde_json::Error> {
    let data = serde_json::to_vec(msg)?;
    let mut buf = Vec::with_capacity(4 + data.len());
    buf.extend_from_slice(&(data.len() as u32).to_le_bytes());
    buf.extend_from_slice(&data);
    Ok(buf)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_is_length_prefixed_json() {
        let msg = SupervisorMsg::Hello {
            protocol: CONTROL_PROTOCOL_VERSION,
            supervisor_pid: 7,
        };
        let frame = encode_frame(&msg).unwrap();
        let len = u32::from_le_bytes(frame[..4].try_into().unwrap()) as usize;
        assert_eq!(len, frame.len() - 4);
        let back: SupervisorMsg = serde_json::from_slice(&frame[4..]).unwrap();
        assert_eq!(back, msg);
    }

    #[test]
    fn load_frame_round_trips() {
        let msg = SupervisorMsg::Load {
            seq: 3,
            package: "pkg".into(),
            plugin: "pkg::Plugin".into(),
            node_name: "n".into(),
            node_namespace: "/ns".into(),
            remap_rules: vec!["a:=b".into()],
            parameters: vec![ControlParam {
                name: "rate".into(),
                type_: rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
                double_value: Some(10.0),
                ..Default::default()
            }],
            extra_arguments: vec![],
            log_dir: "/tmp/x".into(),
        };
        let frame = encode_frame(&msg).unwrap();
        let back: SupervisorMsg = serde_json::from_slice(&frame[4..]).unwrap();
        assert_eq!(back, msg);
    }

    /// Absent optional fields must not be emitted: the C++ end selects the
    /// value field by `type`, and a stray `string_value: null` next to a
    /// double is the kind of thing that reads as a parameter with two values.
    #[test]
    fn only_the_typed_field_is_serialized() {
        let p = ControlParam {
            name: "flag".into(),
            type_: rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
            bool_value: Some(true),
            ..Default::default()
        };
        let json = serde_json::to_string(&p).unwrap();
        assert_eq!(json, r#"{"name":"flag","type":1,"bool_value":true}"#);
    }

    /// The three W2 messages, and the one fact each carries that a timeout
    /// could not.
    #[test]
    fn query_status_and_cancel_round_trip() {
        let query = SupervisorMsg::Query {
            seq: Some(4),
            unique_id: None,
        };
        let back: SupervisorMsg =
            serde_json::from_slice(&encode_frame(&query).unwrap()[4..]).unwrap();
        assert_eq!(back, query);
        // An unset key must be ABSENT, not null: the container reads
        // "which key did you ask by" from the presence of the field.
        assert_eq!(
            serde_json::to_string(&query).unwrap(),
            r#"{"t":"query","seq":4}"#
        );

        let cancel = SupervisorMsg::Cancel {
            unique_id: 9,
            reason: "stalled".into(),
        };
        let back: SupervisorMsg =
            serde_json::from_slice(&encode_frame(&cancel).unwrap()[4..]).unwrap();
        assert_eq!(back, cancel);

        let status = ContainerMsg::Status {
            seq: None,
            unique_id: 9,
            phase: LoadPhase::Constructing,
            pid: 41213,
            elapsed_ms: 135_000,
            cpu_ms: 400,
            plugin: "pkg::Plugin".into(),
            cancellable: true,
            full_node_name: String::new(),
        };
        let back: ContainerMsg =
            serde_json::from_slice(&encode_frame(&status).unwrap()[4..]).unwrap();
        assert_eq!(back, status);
    }

    /// `unknown` is the only phase that authorises a resend, so its spelling
    /// on the wire is load-bearing.
    #[test]
    fn load_phase_spellings_are_stable() {
        for (phase, text) in [
            (LoadPhase::Queued, "\"queued\""),
            (LoadPhase::Constructing, "\"constructing\""),
            (LoadPhase::Loaded, "\"loaded\""),
            (LoadPhase::Failed, "\"failed\""),
            (LoadPhase::Unknown, "\"unknown\""),
        ] {
            assert_eq!(serde_json::to_string(&phase).unwrap(), text);
            assert_eq!(
                serde_json::from_str::<LoadPhase>(text).unwrap(),
                phase,
                "{text} must decode back"
            );
        }
    }

    /// A v1 container sends `constructing` without `phase`/`cpu_ms`, and only
    /// ever after a child existed — so the absent phase means `constructing`,
    /// never `queued`.
    #[test]
    fn v1_constructing_frame_still_decodes() {
        let back: ContainerMsg = serde_json::from_str(
            r#"{"t":"constructing","unique_id":3,"pid":7,"elapsed_ms":15000,"plugin":"p::P"}"#,
        )
        .unwrap();
        assert_eq!(
            back,
            ContainerMsg::Constructing {
                unique_id: 3,
                pid: 7,
                elapsed_ms: 15000,
                plugin: "p::P".into(),
                phase: LoadPhase::Constructing,
                cpu_ms: 0,
            }
        );
    }

    #[test]
    fn container_messages_round_trip() {
        for msg in [
            ContainerMsg::Hello {
                protocol: 1,
                pid: 4,
                loads_over_socket: true,
            },
            ContainerMsg::Accepted {
                seq: 1,
                unique_id: 2,
            },
            ContainerMsg::Loaded {
                unique_id: 2,
                full_node_name: "/ns/n".into(),
                pid: 99,
                start_time: 12345,
            },
            ContainerMsg::Constructing {
                unique_id: 2,
                pid: 99,
                elapsed_ms: 15000,
                plugin: "pkg::Plugin".into(),
                phase: LoadPhase::Constructing,
                cpu_ms: 1234,
            },
            ContainerMsg::Crashed {
                unique_id: 2,
                full_node_name: "/ns/n".into(),
                error: "killed by signal 11".into(),
                pid: 99,
            },
        ] {
            let frame = encode_frame(&msg).unwrap();
            let back: ContainerMsg = serde_json::from_slice(&frame[4..]).unwrap();
            assert_eq!(back, msg);
        }
    }

    /// The container omits fields it has nothing to say about; every such
    /// field must decode rather than fail the frame.
    #[test]
    fn optional_fields_default_when_absent() {
        let back: ContainerMsg =
            serde_json::from_str(r#"{"t":"loaded","unique_id":5,"full_node_name":"/n"}"#).unwrap();
        assert_eq!(
            back,
            ContainerMsg::Loaded {
                unique_id: 5,
                full_node_name: "/n".into(),
                pid: 0,
                start_time: 0,
            }
        );
    }
}
