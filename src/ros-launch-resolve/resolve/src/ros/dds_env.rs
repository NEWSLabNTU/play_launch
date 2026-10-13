//! Nodes a launch file puts on a different DDS network from the rest
//! (`docs/design/dds-assist.md` item 1, phase 89 D1).
//!
//! A launch file may give one node its own `ROS_DOMAIN_ID`,
//! `RMW_IMPLEMENTATION`, `ROS_LOCALHOST_ONLY`, discovery range or DDS config
//! file (`<env>` on a node or container). That node then discovers, and is
//! discovered by, a different set of participants from every other node of
//! the launch, and nothing says so: a per-robot domain is a common pattern in
//! hand-written multi-robot launch files, and a typo in one robot's is
//! invisible until its topics are. This compares each node's effective value
//! with what the other nodes inherit from the launching environment, before
//! anything spawns. It reports; it never changes the value.

use std::collections::BTreeMap;

/// Variables that decide which participants a node can discover. A node
/// that differs on one is on another network from the rest.
pub const DISCOVERY_VARS: [&str; 5] = [
    "ROS_DOMAIN_ID",
    "RMW_IMPLEMENTATION",
    "ROS_LOCALHOST_ONLY",
    "ROS_AUTOMATIC_DISCOVERY_RANGE",
    "ROS_STATIC_PEERS",
];

/// Variables naming a DDS configuration file. A node with another one may
/// still discover the rest, but with other transports, ports or QoS.
pub const CONFIG_VARS: [&str; 4] = [
    "CYCLONEDDS_URI",
    "FASTRTPS_DEFAULT_PROFILES_FILE",
    "FASTDDS_DEFAULT_PROFILES_FILE",
    "RMW_FASTRTPS_USE_QOS_FROM_XML",
];

/// One variable set to one value by a group of nodes, differing from what
/// the rest of the launch inherits.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub struct DdsEnvSplit {
    pub var: String,
    /// The value those nodes set.
    pub value: String,
    /// What the rest inherit; `None` when the launching environment does not
    /// set it, so the middleware's default applies.
    pub inherited: Option<String>,
    /// The nodes, by name.
    pub nodes: Vec<String>,
    /// A discovery variable: the nodes cannot discover the rest at all.
    pub splits_network: bool,
}

/// What a variable means when unset, so `ROS_DOMAIN_ID=0` set on a node is
/// no split from an environment that leaves it unset.
fn default_of(var: &str) -> Option<&'static str> {
    match var {
        "ROS_DOMAIN_ID" => Some("0"),
        "ROS_LOCALHOST_ONLY" => Some("0"),
        "ROS_AUTOMATIC_DISCOVERY_RANGE" => Some("SUBNET"),
        // Humble's and Jazzy's default; Kilted's too.
        "RMW_IMPLEMENTATION" => Some("rmw_fastrtps_cpp"),
        _ => None,
    }
}

fn effective(var: &str, value: Option<&str>) -> Option<String> {
    value
        .filter(|v| !v.is_empty())
        .map(str::to_string)
        .or_else(|| default_of(var).map(str::to_string))
}

/// Group the nodes whose launch-declared environment sets a DDS variable to
/// a value other than the inherited one. `nodes` gives each node's name and
/// its `<env>` list in declaration order (the last of a repeated name wins,
/// as at spawn); `inherited` reads the launching environment.
pub fn dds_env_splits<'a>(
    nodes: impl IntoIterator<Item = (String, &'a [(String, String)])>,
    inherited: impl Fn(&str) -> Option<String>,
) -> Vec<DdsEnvSplit> {
    let mut groups: BTreeMap<(String, String), Vec<String>> = BTreeMap::new();
    for (node, env) in nodes {
        let mut last: BTreeMap<&str, &str> = BTreeMap::new();
        for (k, v) in env {
            last.insert(k.as_str(), v.as_str());
        }
        for var in DISCOVERY_VARS.iter().chain(CONFIG_VARS.iter()) {
            let Some(&v) = last.get(var) else { continue };
            let inh = inherited(var);
            if effective(var, Some(v)) == effective(var, inh.as_deref()) {
                continue;
            }
            groups
                .entry((var.to_string(), v.to_string()))
                .or_default()
                .push(node.clone());
        }
    }
    groups
        .into_iter()
        .map(|((var, value), mut nodes)| {
            nodes.sort();
            DdsEnvSplit {
                splits_network: DISCOVERY_VARS.contains(&var.as_str()),
                inherited: inherited(&var).filter(|v| !v.is_empty()),
                var,
                value,
                nodes,
            }
        })
        .collect()
}

impl DdsEnvSplit {
    /// One line saying what was found and what it does.
    pub fn message(&self) -> String {
        let rest = match &self.inherited {
            Some(v) => format!("{v} (inherited)"),
            None => match default_of(&self.var) {
                Some(d) => format!("unset, so {d}"),
                None => "unset".to_string(),
            },
        };
        let who = match self.nodes.len() {
            1 => self.nodes[0].clone(),
            n if n <= 4 => format!("{} nodes ({})", n, self.nodes.join(", ")),
            n => format!("{} nodes ({}, …)", n, self.nodes[..3].join(", ")),
        };
        let effect = if self.splits_network {
            "it cannot discover the rest of the launch, nor they it"
        } else {
            "it may discover the rest, but runs other transports, ports or QoS"
        };
        format!(
            "{who} set{} {}={} where the rest of the launch has {rest}: {effect}",
            if self.nodes.len() == 1 { "s" } else { "" },
            self.var,
            self.value
        )
    }
}

/// Every node and container of a parsed launch with its declared env.
pub fn dump_nodes(dump: &super::launch_dump::LaunchDump) -> Vec<(String, &[(String, String)])> {
    let join = |ns: &str, name: &str| {
        if name.starts_with('/') {
            name.to_string()
        } else {
            format!("{}/{}", ns.trim_end_matches('/'), name)
        }
    };
    let mut out = Vec::new();
    for n in &dump.node {
        let Some(env) = n.env.as_deref() else {
            continue;
        };
        let name = n
            .name
            .as_deref()
            .or(n.exec_name.as_deref())
            .unwrap_or(&n.executable);
        out.push((join(n.namespace.as_deref().unwrap_or(""), name), env));
    }
    for c in &dump.container {
        let Some(env) = c.env.as_deref() else {
            continue;
        };
        out.push((join(&c.namespace, &c.name), env));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn env(pairs: &[(&str, &str)]) -> Vec<(String, String)> {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect()
    }

    #[test]
    fn a_node_on_another_domain_is_reported_and_the_default_is_no_split() {
        let a = env(&[("ROS_DOMAIN_ID", "7")]);
        let b = env(&[("ROS_DOMAIN_ID", "0"), ("OTHER", "x")]);
        let c = env(&[("ROS_DOMAIN_ID", "3"), ("ROS_DOMAIN_ID", "7")]);
        let s = dds_env_splits(
            [
                ("/r2/driver".to_string(), a.as_slice()),
                ("/r1/driver".to_string(), b.as_slice()),
                ("/r2/planner".to_string(), c.as_slice()),
            ],
            |_| None,
        );
        assert_eq!(s.len(), 1, "{s:?}");
        assert_eq!(s[0].nodes, ["/r2/driver", "/r2/planner"]);
        assert!(s[0].splits_network);
        assert!(s[0].message().contains("unset, so 0"), "{}", s[0].message());
    }

    #[test]
    fn a_config_file_differs_without_splitting_the_network() {
        let a = env(&[("CYCLONEDDS_URI", "file:///other.xml")]);
        let s = dds_env_splits([("/n".to_string(), a.as_slice())], |k| {
            (k == "CYCLONEDDS_URI").then(|| "file:///main.xml".to_string())
        });
        assert_eq!(s.len(), 1);
        assert!(!s[0].splits_network);
        assert_eq!(s[0].inherited.as_deref(), Some("file:///main.xml"));
    }

    #[test]
    fn the_same_value_as_inherited_is_no_split() {
        let a = env(&[("ROS_DOMAIN_ID", "5")]);
        let s = dds_env_splits([("/n".to_string(), a.as_slice())], |k| {
            (k == "ROS_DOMAIN_ID").then(|| "5".to_string())
        });
        assert!(s.is_empty());
    }
}
