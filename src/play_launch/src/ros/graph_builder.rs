//! Runtime communication graph introspection (Phase 25).
//!
//! Queries the ROS graph via rclrs and builds serializable snapshots
//! for the web UI.

use rclrs::{
    Node, QoSDurabilityPolicy, QoSDuration, QoSHistoryPolicy, QoSLivelinessPolicy, QoSProfile,
    QoSReliabilityPolicy,
};
use serde::Serialize;
use std::collections::HashMap;

// ===== Serializable types =====

/// Full graph snapshot — all topics and services visible to the ROS node.
#[derive(Debug, Clone, Serialize)]
pub struct GraphSnapshot {
    pub topics: Vec<TopicGraph>,
    pub services: Vec<ServiceGraph>,
}

/// Per-topic graph entry.
#[derive(Debug, Clone, Serialize)]
pub struct TopicGraph {
    pub name: String,
    pub msg_type: String,
    pub publishers: Vec<EndpointNode>,
    pub subscribers: Vec<EndpointNode>,
    /// True if the topic has zero publishers or zero subscribers.
    pub dangling: bool,
}

/// A node endpoint (publisher or subscriber) on a topic.
#[derive(Debug, Clone, Serialize)]
pub struct EndpointNode {
    /// Fully-qualified ROS node name (e.g. "/ns/node_name").
    pub fqn: String,
    /// play_launch member name, if this node is managed by us.
    pub member_name: Option<String>,
    /// QoS summary (may be None if unavailable).
    pub qos: Option<QosSummary>,
}

/// Human-readable QoS summary — all fields are strings for easy JSON display.
#[derive(Debug, Clone, Serialize)]
pub struct QosSummary {
    pub reliability: String,
    pub durability: String,
    pub history: String,
    pub deadline_ms: String,
    pub lifespan_ms: String,
    pub liveliness: String,
}

/// Per-service graph entry.
#[derive(Debug, Clone, Serialize)]
pub struct ServiceGraph {
    pub name: String,
    pub srv_type: String,
    pub servers: Vec<String>,
    pub clients: Vec<String>,
}

/// Topics and services for a single node.
#[derive(Debug, Clone, Serialize)]
pub struct NodeTopics {
    pub publishers: Vec<NodeTopicEntry>,
    pub subscribers: Vec<NodeTopicEntry>,
    pub servers: Vec<NodeServiceEntry>,
    pub clients: Vec<NodeServiceEntry>,
}

/// A topic entry from a single node's perspective.
#[derive(Debug, Clone, Serialize)]
pub struct NodeTopicEntry {
    pub name: String,
    pub msg_type: String,
    pub qos: Option<QosSummary>,
    pub publisher_count: usize,
    pub subscriber_count: usize,
    pub dangling: bool,
}

/// A service entry from a single node's perspective.
#[derive(Debug, Clone, Serialize)]
pub struct NodeServiceEntry {
    pub name: String,
    pub srv_type: String,
}

// ===== QoS conversion =====

/// Convert an rclrs QoSProfile to a human-readable summary.
pub fn qos_summary(p: &QoSProfile) -> QosSummary {
    QosSummary {
        reliability: match p.reliability {
            QoSReliabilityPolicy::Reliable => "reliable".into(),
            QoSReliabilityPolicy::BestEffort => "best_effort".into(),
            _ => "system_default".into(),
        },
        durability: match p.durability {
            QoSDurabilityPolicy::TransientLocal => "transient_local".into(),
            QoSDurabilityPolicy::Volatile => "volatile".into(),
            _ => "system_default".into(),
        },
        history: match p.history {
            QoSHistoryPolicy::KeepLast { depth } => format!("keep_last({})", depth),
            QoSHistoryPolicy::KeepAll => "keep_all".into(),
            QoSHistoryPolicy::SystemDefault { depth } => format!("system_default({})", depth),
        },
        deadline_ms: duration_str(&p.deadline),
        lifespan_ms: duration_str(&p.lifespan),
        liveliness: match p.liveliness {
            QoSLivelinessPolicy::Automatic => "automatic".into(),
            QoSLivelinessPolicy::ManualByTopic => "manual_by_topic".into(),
            _ => "system_default".into(),
        },
    }
}

fn duration_str(d: &QoSDuration) -> String {
    match d {
        QoSDuration::Infinite => "infinite".into(),
        QoSDuration::SystemDefault => "default".into(),
        QoSDuration::Custom(dur) => format!("{}", dur.as_millis()),
    }
}

// ===== Graph building =====

/// Build a full graph snapshot from the given ROS node.
///
/// `fqn_to_member` maps fully-qualified ROS node names to play_launch member names.
pub fn build_graph_snapshot(
    ros_node: &Node,
    fqn_to_member: &HashMap<String, String>,
) -> eyre::Result<GraphSnapshot> {
    let topic_names_and_types = ros_node.get_topic_names_and_types()?;

    let mut topics = Vec::with_capacity(topic_names_and_types.len());
    for (topic_name, types) in &topic_names_and_types {
        let msg_type = types.first().cloned().unwrap_or_default();

        let pub_infos = ros_node
            .get_publishers_info_by_topic(topic_name)
            .unwrap_or_default();
        let sub_infos = ros_node
            .get_subscriptions_info_by_topic(topic_name)
            .unwrap_or_default();

        let publishers: Vec<EndpointNode> = pub_infos
            .iter()
            .map(|info| {
                let fqn = make_fqn(&info.node_namespace, &info.node_name);
                EndpointNode {
                    member_name: fqn_to_member.get(&fqn).cloned(),
                    fqn,
                    qos: Some(qos_summary(&info.qos_profile)),
                }
            })
            .collect();

        let subscribers: Vec<EndpointNode> = sub_infos
            .iter()
            .map(|info| {
                let fqn = make_fqn(&info.node_namespace, &info.node_name);
                EndpointNode {
                    member_name: fqn_to_member.get(&fqn).cloned(),
                    fqn,
                    qos: Some(qos_summary(&info.qos_profile)),
                }
            })
            .collect();

        let dangling = publishers.is_empty() || subscribers.is_empty();

        topics.push(TopicGraph {
            name: topic_name.clone(),
            msg_type,
            publishers,
            subscribers,
            dangling,
        });
    }

    // Sort topics by name for stable output
    topics.sort_by(|a, b| a.name.cmp(&b.name));

    // Services — filter out rcl_interfaces parameter services
    let service_names_and_types = ros_node.get_service_names_and_types()?;
    let mut services = Vec::new();
    for (svc_name, types) in &service_names_and_types {
        let srv_type = types.first().cloned().unwrap_or_default();
        if srv_type.starts_with("rcl_interfaces/srv/") {
            continue;
        }
        services.push(ServiceGraph {
            name: svc_name.clone(),
            srv_type,
            servers: Vec::new(),
            clients: Vec::new(),
        });
    }
    services.sort_by(|a, b| a.name.cmp(&b.name));

    Ok(GraphSnapshot { topics, services })
}

/// Build per-node topic/service info.
///
/// `member_name` is the play_launch member name; `fqn` is the fully-qualified
/// ROS node name; we split it into (name, namespace) for rclrs queries.
pub fn build_node_topics(ros_node: &Node, fqn: &str) -> eyre::Result<NodeTopics> {
    let (node_name, namespace) = split_fqn(fqn);

    // Publishers
    let pub_names = ros_node
        .get_publisher_names_and_types_by_node(&node_name, &namespace)
        .unwrap_or_default();
    let mut publishers = Vec::new();
    for (topic_name, types) in &pub_names {
        let msg_type = types.first().cloned().unwrap_or_default();
        let qos = ros_node
            .get_publishers_info_by_topic(topic_name)
            .ok()
            .and_then(|infos| {
                infos
                    .iter()
                    .find(|i| make_fqn(&i.node_namespace, &i.node_name) == fqn)
                    .map(|i| qos_summary(&i.qos_profile))
            });
        let pub_count = ros_node.count_publishers(topic_name).unwrap_or(0);
        let sub_count = ros_node.count_subscriptions(topic_name).unwrap_or(0);
        publishers.push(NodeTopicEntry {
            name: topic_name.clone(),
            msg_type,
            qos,
            publisher_count: pub_count,
            subscriber_count: sub_count,
            dangling: pub_count == 0 || sub_count == 0,
        });
    }

    // Subscribers
    let sub_names = ros_node
        .get_subscription_names_and_types_by_node(&node_name, &namespace)
        .unwrap_or_default();
    let mut subscribers = Vec::new();
    for (topic_name, types) in &sub_names {
        let msg_type = types.first().cloned().unwrap_or_default();
        let qos = ros_node
            .get_subscriptions_info_by_topic(topic_name)
            .ok()
            .and_then(|infos| {
                infos
                    .iter()
                    .find(|i| make_fqn(&i.node_namespace, &i.node_name) == fqn)
                    .map(|i| qos_summary(&i.qos_profile))
            });
        let pub_count = ros_node.count_publishers(topic_name).unwrap_or(0);
        let sub_count = ros_node.count_subscriptions(topic_name).unwrap_or(0);
        subscribers.push(NodeTopicEntry {
            name: topic_name.clone(),
            msg_type,
            qos,
            publisher_count: pub_count,
            subscriber_count: sub_count,
            dangling: pub_count == 0 || sub_count == 0,
        });
    }

    // Service servers
    let srv_names = ros_node
        .get_service_names_and_types_by_node(&node_name, &namespace)
        .unwrap_or_default();
    let mut servers = Vec::new();
    for (svc_name, types) in &srv_names {
        let srv_type = types.first().cloned().unwrap_or_default();
        if srv_type.starts_with("rcl_interfaces/srv/") {
            continue;
        }
        servers.push(NodeServiceEntry {
            name: svc_name.clone(),
            srv_type,
        });
    }

    // Service clients
    let cli_names = ros_node
        .get_client_names_and_types_by_node(&node_name, &namespace)
        .unwrap_or_default();
    let mut clients = Vec::new();
    for (svc_name, types) in &cli_names {
        let srv_type = types.first().cloned().unwrap_or_default();
        if srv_type.starts_with("rcl_interfaces/srv/") {
            continue;
        }
        clients.push(NodeServiceEntry {
            name: svc_name.clone(),
            srv_type,
        });
    }

    Ok(NodeTopics {
        publishers,
        subscribers,
        servers,
        clients,
    })
}

// ===== Helpers =====

/// Construct a fully-qualified node name from namespace and name.
fn make_fqn(namespace: &str, name: &str) -> String {
    if namespace == "/" {
        format!("/{}", name)
    } else {
        format!("{}/{}", namespace, name)
    }
}

/// Split a fully-qualified node name into (name, namespace).
/// E.g. "/ns1/ns2/node" → ("node", "/ns1/ns2")
///      "/node"          → ("node", "/")
fn split_fqn(fqn: &str) -> (String, String) {
    let fqn = fqn.strip_prefix('/').unwrap_or(fqn);
    match fqn.rsplit_once('/') {
        Some((ns, name)) => (name.to_string(), format!("/{}", ns)),
        None => (fqn.to_string(), "/".to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_split_fqn() {
        assert_eq!(split_fqn("/node"), ("node".into(), "/".into()));
        assert_eq!(split_fqn("/ns/node"), ("node".into(), "/ns".into()));
        assert_eq!(split_fqn("/a/b/c"), ("c".into(), "/a/b".into()));
    }

    #[test]
    fn test_make_fqn() {
        assert_eq!(make_fqn("/", "node"), "/node");
        assert_eq!(make_fqn("/ns", "node"), "/ns/node");
    }
}
