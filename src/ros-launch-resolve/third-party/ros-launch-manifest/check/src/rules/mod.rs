//! Validation rules for manifest checking.

mod causal_dag;
mod chain_shape;
mod consistency;
mod dangling_entity;
mod drop_sanity;
mod endpoint_topic;
mod endpoint_unique;
mod explicit_trigger;
mod inherited_rate;
mod once_durability;
mod qos_compat;
mod qos_match;
mod queue_drain_rate;
mod rate_hierarchy;
mod satisfiability;
mod scope_budget;
mod service_type;
mod service_wiring;
mod state_consistency;
mod sync_feasibility;
mod wiring;

use crate::{check::CheckContext, graph::DataflowGraph};
use ros_launch_manifest_types::Manifest;

/// A validation rule that checks one aspect of the manifest.
pub trait ValidationRule: Send + Sync {
    fn id(&self) -> &str;
    fn check(&self, manifest: &Manifest, graph: &DataflowGraph, ctx: &mut CheckContext);
}

/// Default set of validation rules.
pub fn default_rules() -> Vec<Box<dyn ValidationRule>> {
    vec![
        Box::new(endpoint_unique::EndpointUniqueRule),
        Box::new(wiring::WiringRule),
        Box::new(qos_compat::QosCompatRule),
        Box::new(qos_match::QosMatchRule),
        Box::new(rate_hierarchy::RateHierarchyRule),
        Box::new(scope_budget::ScopeBudgetRule),
        Box::new(causal_dag::CausalDagRule),
        Box::new(drop_sanity::DropSanityRule),
        Box::new(service_wiring::ServiceWiringRule),
        Box::new(service_type::ServiceTypeRule),
        Box::new(dangling_entity::DanglingEntityRule),
        Box::new(satisfiability::SatisfiabilityRule),
        Box::new(consistency::ConsistencyRule),
        Box::new(state_consistency::StateConsistencyRule),
        // Vocabulary v2 rules (Phase 44.2).
        Box::new(explicit_trigger::ExplicitTriggerRule),
        Box::new(inherited_rate::InheritedRateRule),
        Box::new(once_durability::OnceDurabilityRule),
        Box::new(sync_feasibility::SyncFeasibilityRule),
        Box::new(queue_drain_rate::QueueDrainRateRule),
        Box::new(chain_shape::ChainShapeRule),
    ]
}
