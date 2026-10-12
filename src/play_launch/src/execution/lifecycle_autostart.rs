//! Drive a lifecycle node to the state its model asks for, after it spawns
//! (play_launch phase 86 W4).
//!
//! The model's per-node `lifecycle_autostart` is the request:
//!
//! - `Active` — configure, then activate. What Jazzy `launch_ros` does for
//!   `LifecycleNode(autostart=True)`: `execute` appends a `LifecycleTransition`
//!   with `TRANSITION_CONFIGURE` then `TRANSITION_ACTIVATE`, and the event
//!   manager calls `<node>/change_state` once the service exists, waiting for
//!   it with no deadline. Activate is requested only after configure
//!   succeeded, as stock chains it on the configure transition's event.
//! - `Configure` — configure and stop. No launch file produces this; a
//!   contract or another producer can.
//! - `None`, or no value — nothing. "The launch file did not ask" is not
//!   "configure only".
//!
//! Only a node the model marks `lifecycle: true` is driven: asking a plain
//! node for `change_state` would wait forever for a service it never offers.
//! Such a request is reported once and skipped.
//!
//! The transitions are requested once per run, like stock's, which emits them
//! from `execute` and not again on respawn. Runtime enforcement's view of
//! lifecycle state is fed by the `transition_event` subscription in
//! `commands::up`, unchanged; these transitions show up there like any other.

use ros_launch_manifest_model::{Autostart, SystemModel};
use std::{sync::Arc, time::Duration};
use tokio::sync::watch;
use tracing::{debug, info, warn};

/// How often to look for the node's `change_state` service while it is not
/// up yet. Stock polls `wait_for_service(timeout_sec=1.0)`.
const SERVICE_POLL: Duration = Duration::from_millis(200);

/// A `lifecycle_msgs/msg/Transition` this module requests.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Transition {
    Configure,
    Activate,
}

impl Transition {
    /// `lifecycle_msgs/msg/Transition` id.
    pub fn id(self) -> u8 {
        match self {
            Transition::Configure => 1, // TRANSITION_CONFIGURE
            Transition::Activate => 3,  // TRANSITION_ACTIVATE
        }
    }

    fn label(self) -> &'static str {
        match self {
            Transition::Configure => "configure",
            Transition::Activate => "activate",
        }
    }
}

/// The transitions `autostart` asks for, in order.
pub fn transitions_for(autostart: Option<&Autostart>) -> &'static [Transition] {
    match autostart {
        Some(Autostart::Active) => &[Transition::Configure, Transition::Activate],
        Some(Autostart::Configure) => &[Transition::Configure],
        Some(Autostart::None) | None => &[],
    }
}

/// What to do for one model.
#[derive(Debug, Default, PartialEq, Eq)]
pub struct Plan {
    /// `(node FQN, transitions)` for every lifecycle node with a request.
    pub drive: Vec<(String, &'static [Transition])>,
    /// Nodes with a request that are not lifecycle nodes, so nothing can be
    /// asked of them.
    pub not_lifecycle: Vec<String>,
}

/// Read the requests off the model. Pure, so it is tested without ROS.
pub fn plan(model: &SystemModel) -> Plan {
    let mut out = Plan::default();
    for (fqn, inst) in &model.structure.nodes {
        let steps = transitions_for(inst.lifecycle_autostart.as_ref());
        if steps.is_empty() {
            continue;
        }
        if inst.lifecycle {
            out.drive.push((fqn.clone(), steps));
        } else {
            out.not_lifecycle.push(fqn.clone());
        }
    }
    out
}

/// Spawn one task per node in `plan`. Returns nothing to await: each task
/// ends when its node is where it was asked to be, a transition fails, or
/// the run shuts down.
pub fn spawn_all(
    plan: Plan,
    ros_node: Option<Arc<rclrs::Node>>,
    shutdown_rx: watch::Receiver<bool>,
) {
    if !plan.not_lifecycle.is_empty() {
        warn!(
            "[lifecycle] autostart requested for node(s) that are not lifecycle nodes, \
             ignored: {}",
            plan.not_lifecycle.join(", ")
        );
    }
    if plan.drive.is_empty() {
        return;
    }
    let Some(ros_node) = ros_node else {
        warn!(
            "[lifecycle] no ROS node available, so autostart cannot be driven for: {}",
            plan.drive
                .iter()
                .map(|(f, _)| f.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        );
        return;
    };
    for (fqn, steps) in plan.drive {
        tokio::spawn(drive(ros_node.clone(), fqn, steps, shutdown_rx.clone()));
    }
}

/// Wait for `<fqn>/change_state`, then request each transition in turn,
/// stopping at the first one the node refuses.
async fn drive(
    ros_node: Arc<rclrs::Node>,
    fqn: String,
    steps: &'static [Transition],
    mut shutdown_rx: watch::Receiver<bool>,
) {
    let service = format!("{}/change_state", fqn.trim_end_matches('/'));
    let client = match ros_node.create_client::<lifecycle_msgs::srv::ChangeState>(&service) {
        Ok(c) => c,
        Err(e) => {
            warn!(
                "[lifecycle] {fqn}: cannot create a {service} client ({e:#}); autostart not driven"
            );
            return;
        }
    };

    // No deadline, as stock: a node that is slow to come up is still
    // brought up, and the wait ends with the run.
    loop {
        if *shutdown_rx.borrow() {
            return;
        }
        match client.service_is_ready() {
            Ok(true) => break,
            Ok(false) => {}
            Err(e) => debug!("[lifecycle] {fqn}: readiness check failed: {e:?}"),
        }
        tokio::select! {
            _ = tokio::time::sleep(SERVICE_POLL) => {}
            _ = shutdown_rx.changed() => {}
        }
    }

    for step in steps {
        let mut request = lifecycle_msgs::srv::ChangeState_Request::default();
        request.transition.id = step.id();
        let promise: rclrs::Promise<lifecycle_msgs::srv::ChangeState_Response> =
            match client.call(&request) {
                Ok(p) => p,
                Err(e) => {
                    warn!(
                        "[lifecycle] {fqn}: {} request failed to send: {e:?}",
                        step.label()
                    );
                    return;
                }
            };
        let response = tokio::select! {
            r = promise => r,
            _ = shutdown_rx.changed() => return,
        };
        match response {
            Ok(r) if r.success => debug!("[lifecycle] {fqn}: {} succeeded", step.label()),
            Ok(_) => {
                warn!(
                    "[lifecycle] {fqn}: the node refused {}; autostart stops here",
                    step.label()
                );
                return;
            }
            Err(e) => {
                warn!("[lifecycle] {fqn}: {} got no response: {e:?}", step.label());
                return;
            }
        }
    }
    let reached = match steps.last() {
        Some(Transition::Activate) => "active",
        _ => "inactive",
    };
    info!("[lifecycle] {fqn}: autostart brought it to {reached}");
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_launch_manifest_model::NodeInstance;

    fn model(nodes: &[(&str, bool, Option<Autostart>)]) -> SystemModel {
        let mut m = SystemModel::default();
        for (fqn, lifecycle, autostart) in nodes {
            m.structure.nodes.insert(
                fqn.to_string(),
                NodeInstance {
                    lifecycle: *lifecycle,
                    lifecycle_autostart: *autostart,
                    ..Default::default()
                },
            );
        }
        m
    }

    #[test]
    fn active_configures_then_activates_and_configure_stops_after_configure() {
        assert_eq!(
            transitions_for(Some(&Autostart::Active)),
            &[Transition::Configure, Transition::Activate]
        );
        assert_eq!(
            transitions_for(Some(&Autostart::Configure)),
            &[Transition::Configure]
        );
        assert!(transitions_for(Some(&Autostart::None)).is_empty());
        assert!(transitions_for(None).is_empty());
        // lifecycle_msgs/msg/Transition ids.
        assert_eq!(Transition::Configure.id(), 1);
        assert_eq!(Transition::Activate.id(), 3);
    }

    #[test]
    fn only_lifecycle_nodes_with_a_request_are_driven() {
        let p = plan(&model(&[
            ("/active", true, Some(Autostart::Active)),
            ("/configure", true, Some(Autostart::Configure)),
            ("/none", true, Some(Autostart::None)),
            ("/unsaid", true, None),
            ("/plain_asked", false, Some(Autostart::Active)),
            ("/plain", false, None),
        ]));
        let mut drive: Vec<(&str, &[Transition])> =
            p.drive.iter().map(|(f, s)| (f.as_str(), *s)).collect();
        drive.sort();
        assert_eq!(
            drive,
            vec![
                (
                    "/active",
                    &[Transition::Configure, Transition::Activate][..]
                ),
                ("/configure", &[Transition::Configure][..]),
            ]
        );
        assert_eq!(p.not_lifecycle, vec!["/plain_asked".to_string()]);
    }
}
