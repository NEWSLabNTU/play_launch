//! Service calls on the Health page (phase 95; rlm
//! `docs/design/service-contracts.md`, "Runtime companion").
//!
//! - `call_unanswered` (warning): a client's request is still open past the
//!   client's contract `timeout:` (`contracts.cli_endpoints.<ref>.
//!   timeout_ms`), or 10 s when the contract states none. Services report
//!   nothing when a server is missing, dies with a request in hand, or a
//!   request or response is lost (failure modes F1-F3), and rclcpp's client
//!   waits forever by default, so this is the only place such a call shows.
//!   The issue closes when the request is answered, or ten minutes after it
//!   was sent (`interception::calls`), whichever comes first.

use super::{Condition, Link, Severity, Subject};
use crate::interception::{CallSnapshot, ClientRow};
use ros_launch_manifest_model::SystemModel;

/// The wait a client with no stated `timeout:` is judged by.
pub const DEFAULT_TIMEOUT_MS: f64 = 10_000.0;

/// The contract `timeout:` of the client `node` -> `service`, from the
/// model: the client endpoint the service's wiring lists under that node.
pub fn contract_timeout_ms(model: &SystemModel, node: &str, service: &str) -> Option<f64> {
    let wiring = model.structure.services.get(service)?;
    let prefix = format!("{}/", node.trim_end_matches('/'));
    wiring
        .client
        .iter()
        .filter(|r| r.starts_with(&prefix) && !r[prefix.len()..].contains('/'))
        .find_map(|r| model.contracts.cli_endpoints.get(r)?.timeout_ms)
}

pub fn call_conditions(snap: &CallSnapshot, model: &SystemModel) -> Vec<Condition> {
    snap.clients
        .iter()
        .filter_map(|c| unanswered(c, model))
        .collect()
}

fn unanswered(c: &ClientRow, model: &SystemModel) -> Option<Condition> {
    let oldest = c.oldest_open_ms?;
    let stated = c
        .node_known
        .then(|| contract_timeout_ms(model, &c.node, &c.service))
        .flatten();
    let limit = stated.unwrap_or(DEFAULT_TIMEOUT_MS);
    if oldest <= limit {
        return None;
    }
    let against = match stated {
        Some(t) => format!("the client's contract `timeout:` of {t:.0} ms"),
        None => format!(
            "{:.0} s, since its contract states no `timeout:`",
            DEFAULT_TIMEOUT_MS / 1000.0
        ),
    };
    let open = if c.open == 1 {
        "1 request".to_string()
    } else {
        format!("{} requests", c.open)
    };
    Some(Condition {
        key: format!("call_unanswered:{}:{}", c.node, c.service),
        kind: "call_unanswered",
        severity: Severity::Warning,
        subject: Subject {
            kind: "endpoint",
            name: format!("{} → {}", c.node, c.service),
        },
        title: format!("a request to {} is unanswered", c.service),
        detail: format!(
            "{open} from {} to {} open, the oldest for {:.1} s, past {against}. \
             {} of {} requests answered so far. No middleware event reports a \
             server that is missing, died holding the request, or lost the request \
             or its response; rclcpp's client waits forever unless the code gives up",
            c.node,
            c.service,
            oldest / 1000.0,
            c.answered,
            c.sent
        ),
        link: Some(Link {
            view: "nodes",
            target: None,
        }),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn row(oldest_ms: Option<f64>) -> ClientRow {
        ClientRow {
            node: "/sys/handler".into(),
            node_known: true,
            member: "handler".into(),
            service: "/comfort/operate".into(),
            sent: 3,
            answered: 2,
            open: u64::from(oldest_ms.is_some()),
            oldest_open_ms: oldest_ms,
            expired: 0,
            request_leg: None,
            round_trip: None,
        }
    }

    fn model(timeout_ms: Option<f64>) -> SystemModel {
        let mut m = SystemModel::default();
        m.structure.services.insert(
            "/comfort/operate".into(),
            ros_launch_manifest_model::ServiceWiring {
                srv_type: "std_srvs/srv/Trigger".into(),
                server: vec!["/sys/comfort_op/operate".into()],
                client: vec!["/sys/handler/operate".into()],
            },
        );
        if let Some(t) = timeout_ms {
            m.contracts.cli_endpoints.insert(
                "/sys/handler/operate".into(),
                ros_launch_manifest_model::CliContract {
                    timeout_ms: Some(t),
                    ..Default::default()
                },
            );
        }
        m
    }

    /// The contract's timeout is the threshold; 10 s without one; nothing
    /// for a client with no open request.
    #[test]
    fn an_open_request_past_the_clients_timeout_is_reported() {
        assert!(
            call_conditions(
                &CallSnapshot {
                    clients: vec![row(Some(150.0))],
                    servers: vec![]
                },
                &model(Some(100.0))
            )
            .iter()
            .any(|c| c.kind == "call_unanswered" && c.detail.contains("`timeout:` of 100 ms"))
        );
        assert!(
            call_conditions(
                &CallSnapshot {
                    clients: vec![row(Some(50.0))],
                    servers: vec![]
                },
                &model(Some(100.0))
            )
            .is_empty()
        );
        assert!(
            call_conditions(
                &CallSnapshot {
                    clients: vec![row(Some(9_000.0))],
                    servers: vec![]
                },
                &model(None)
            )
            .is_empty()
        );
        assert!(
            !call_conditions(
                &CallSnapshot {
                    clients: vec![row(Some(10_500.0))],
                    servers: vec![]
                },
                &model(None)
            )
            .is_empty()
        );
        assert!(
            call_conditions(
                &CallSnapshot {
                    clients: vec![row(None)],
                    servers: vec![]
                },
                &model(Some(1.0))
            )
            .is_empty()
        );
    }
}
