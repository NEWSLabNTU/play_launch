//! Rule: verify no valid arg combination produces dangling entities.
//!
//! Uses Z3 SMT solver to check whether any valid arg assignment makes a
//! conditional topic/service/action structurally invalid (0 publishers,
//! 0 servers). Also detects unreachable nodes (condition always false).

use super::ValidationRule;
use crate::{CheckContext, graph::DataflowGraph};
use ros_launch_manifest_types::Manifest;
use std::collections::{HashMap, HashSet};
use z3::ast::Ast;

pub struct SatisfiabilityRule;

impl ValidationRule for SatisfiabilityRule {
    fn id(&self) -> &str {
        "satisfiability"
    }

    fn check(&self, manifest: &Manifest, _graph: &DataflowGraph, ctx: &mut CheckContext) {
        let finite_args: Vec<(&str, Vec<&str>)> = manifest
            .args
            .iter()
            .filter_map(|(name, decl)| decl.valid_values().map(|vals| (name.as_str(), vals)))
            .collect();

        if finite_args.is_empty() {
            return;
        }

        let cfg = z3::Config::new();
        let z3_ctx = z3::Context::new(&cfg);

        // Create Z3 enum sorts and variables
        let mut arg_vars: HashMap<&str, ArgVar> = HashMap::new();
        for (name, values) in &finite_args {
            let z3_names: Vec<z3::Symbol> = values
                .iter()
                .map(|v| z3::Symbol::String(v.to_string()))
                .collect();
            let (sort, consts, _) =
                z3::Sort::enumeration(&z3_ctx, z3::Symbol::String(name.to_string()), &z3_names);
            let var = z3::ast::Datatype::new_const(&z3_ctx, *name, &sort);
            let const_map: HashMap<String, z3::ast::Datatype> = values
                .iter()
                .zip(consts.iter())
                .map(|(v, c)| {
                    let val = c.apply(&[]);
                    let dt = val.as_datatype().expect("enum const should be datatype");
                    (v.to_string(), dt)
                })
                .collect();
            arg_vars.insert(
                name,
                ArgVar {
                    var,
                    consts: const_map,
                },
            );
        }

        // Collect conditional node names — refs to these are optional
        let conditional_nodes: HashSet<&str> = manifest
            .nodes
            .iter()
            .filter(|(_, n)| n.if_condition.is_some() || n.unless_condition.is_some())
            .map(|(name, _)| name.as_str())
            .collect();

        // Check topics with all-conditional publishers
        for (topic_name, topic) in &manifest.topics {
            if topic.subscribers.is_empty() {
                continue;
            }
            // Skip if all subscribers are state-only (polled, non-required) —
            // 0 publishers is harmless when no subscriber needs causal data.
            if all_state_only_subs(&topic.subscribers, manifest) {
                continue;
            }
            let conditional_pubs: Vec<&str> = topic
                .publishers
                .iter()
                .filter(|r| is_conditional_ref(r, &conditional_nodes))
                .map(|s| s.as_str())
                .collect();
            if conditional_pubs.is_empty()
                || topic
                    .publishers
                    .iter()
                    .any(|r| !is_conditional_ref(r, &conditional_nodes))
            {
                continue;
            }
            check_all_filtered(
                &z3_ctx,
                &arg_vars,
                manifest,
                &conditional_pubs,
                &format!("topic '{topic_name}' has 0 publishers"),
                ctx,
                self.id(),
            );
        }

        // Check services with all-conditional servers
        for (svc_name, svc) in &manifest.services {
            if svc.client.is_empty() {
                continue;
            }
            let conditional_servers: Vec<&str> = svc
                .server
                .iter()
                .filter(|r| is_conditional_ref(r, &conditional_nodes))
                .map(|s| s.as_str())
                .collect();
            if conditional_servers.is_empty()
                || svc
                    .server
                    .iter()
                    .any(|r| !is_conditional_ref(r, &conditional_nodes))
            {
                continue;
            }
            check_all_filtered(
                &z3_ctx,
                &arg_vars,
                manifest,
                &conditional_servers,
                &format!("service '{svc_name}' has 0 servers"),
                ctx,
                self.id(),
            );
        }

        // Check actions with all-conditional servers
        for (act_name, act) in &manifest.actions {
            if act.client.is_empty() {
                continue;
            }
            let conditional_servers: Vec<&str> = act
                .server
                .iter()
                .filter(|r| is_conditional_ref(r, &conditional_nodes))
                .map(|s| s.as_str())
                .collect();
            if conditional_servers.is_empty()
                || act
                    .server
                    .iter()
                    .any(|r| !is_conditional_ref(r, &conditional_nodes))
            {
                continue;
            }
            check_all_filtered(
                &z3_ctx,
                &arg_vars,
                manifest,
                &conditional_servers,
                &format!("action '{act_name}' has 0 servers"),
                ctx,
                self.id(),
            );
        }

        // Detect unreachable nodes
        for (node_name, node) in &manifest.nodes {
            if node.if_condition.is_none() && node.unless_condition.is_none() {
                continue;
            }
            match build_node_condition(&z3_ctx, &arg_vars, node) {
                Some(active) => {
                    let solver = z3::Solver::new(&z3_ctx);
                    solver.assert(&active);
                    if solver.check() == z3::SatResult::Unsat {
                        ctx.warning(
                            self.id(),
                            &format!("nodes.{node_name}"),
                            format!("node '{node_name}' is unreachable — its condition is false for all valid arg values"),
                        );
                    }
                }
                None => {
                    // Condition references a value not in any arg's domain
                    // (e.g., bool == 'wtf') — always false, node is unreachable
                    ctx.warning(
                        self.id(),
                        &format!("nodes.{node_name}"),
                        format!("node '{node_name}' is unreachable — condition references invalid arg value"),
                    );
                }
            }
        }
    }
}

struct ArgVar<'a> {
    var: z3::ast::Datatype<'a>,
    consts: HashMap<String, z3::ast::Datatype<'a>>,
}

fn check_all_filtered<'a>(
    z3_ctx: &'a z3::Context,
    arg_vars: &HashMap<&str, ArgVar<'a>>,
    manifest: &Manifest,
    optional_refs: &[&str],
    description: &str,
    ctx: &mut CheckContext,
    rule_id: &str,
) {
    let solver = z3::Solver::new(z3_ctx);
    for ep_ref in optional_refs {
        let node_name = match ep_ref.split_once('/') {
            Some((n, _)) => n,
            None => continue,
        };
        if let Some(node) = manifest.nodes.get(node_name)
            && let Some(active) = build_node_condition(z3_ctx, arg_vars, node)
        {
            solver.assert(&active.not());
        }
    }
    if solver.check() == z3::SatResult::Sat {
        let model = solver.get_model().unwrap();
        let mut parts = Vec::new();
        for (&name, av) in arg_vars {
            if let Some(val) = model.eval(&av.var, true) {
                parts.push(format!("{name}={val}"));
            }
        }
        parts.sort();
        ctx.error(
            rule_id,
            "args",
            format!("{description} when {}", parts.join(", ")),
        );
    }
}

fn build_node_condition<'a>(
    z3_ctx: &'a z3::Context,
    arg_vars: &HashMap<&str, ArgVar<'a>>,
    node: &ros_launch_manifest_types::NodeDecl,
) -> Option<z3::ast::Bool<'a>> {
    if let Some(ref cond) = node.if_condition {
        translate_condition(z3_ctx, arg_vars, cond)
    } else if let Some(ref cond) = node.unless_condition {
        translate_condition(z3_ctx, arg_vars, cond).map(|e| e.not())
    } else {
        None
    }
}

fn translate_condition<'a>(
    z3_ctx: &'a z3::Context,
    arg_vars: &HashMap<&str, ArgVar<'a>>,
    condition: &str,
) -> Option<z3::ast::Bool<'a>> {
    let trimmed = condition.trim();

    if let Some(pos) = find_top_level(trimmed, " and ") {
        let left = translate_condition(z3_ctx, arg_vars, &trimmed[..pos])?;
        let right = translate_condition(z3_ctx, arg_vars, &trimmed[pos + 5..])?;
        return Some(z3::ast::Bool::and(z3_ctx, &[&left, &right]));
    }
    if let Some(pos) = find_top_level(trimmed, " or ") {
        let left = translate_condition(z3_ctx, arg_vars, &trimmed[..pos])?;
        let right = translate_condition(z3_ctx, arg_vars, &trimmed[pos + 4..])?;
        return Some(z3::ast::Bool::or(z3_ctx, &[&left, &right]));
    }
    if let Some(pos) = trimmed.find(" == ") {
        let var_name = extract_var(trimmed[..pos].trim())?;
        let literal = extract_literal(trimmed[pos + 4..].trim());
        let av = arg_vars.get(var_name)?;
        let c = av.consts.get(literal)?;
        return Some(av.var._eq(c));
    }
    if let Some(pos) = trimmed.find(" != ") {
        let var_name = extract_var(trimmed[..pos].trim())?;
        let literal = extract_literal(trimmed[pos + 4..].trim());
        let av = arg_vars.get(var_name)?;
        let c = av.consts.get(literal)?;
        return Some(av.var._eq(c).not());
    }

    // Boolean shorthand: $(var x) → x == "true"
    let var_name = extract_var(trimmed)?;
    let av = arg_vars.get(var_name)?;
    let true_c = av.consts.get("true")?;
    Some(av.var._eq(true_c))
}

fn find_top_level(s: &str, op: &str) -> Option<usize> {
    let mut depth = 0u32;
    let mut in_q = false;
    for (i, b) in s.bytes().enumerate() {
        match b {
            b'(' if !in_q => depth += 1,
            b')' if !in_q => depth = depth.saturating_sub(1),
            b'\'' | b'"' => in_q = !in_q,
            _ => {}
        }
        if depth == 0 && !in_q && s[i..].starts_with(op) {
            return Some(i);
        }
    }
    None
}

fn extract_var(s: &str) -> Option<&str> {
    let t = s.trim();
    t.strip_prefix("$(var ")
        .and_then(|inner| inner.strip_suffix(')'))
        .map(|v| v.trim())
        .or(Some(t))
}

fn extract_literal(s: &str) -> &str {
    let t = s.trim();
    t.strip_prefix('\'')
        .and_then(|s| s.strip_suffix('\''))
        .or_else(|| t.strip_prefix('"').and_then(|s| s.strip_suffix('"')))
        .unwrap_or(t)
}

/// Check if a ref points to a node that has a condition (if:/unless:).
fn is_conditional_ref(r: &str, conditional_nodes: &HashSet<&str>) -> bool {
    r.split_once('/')
        .is_some_and(|(node, _)| conditional_nodes.contains(node))
}

/// Check if all subscribers of a topic are state-only (state: true, not required: true).
/// If so, 0 publishers is harmless — polled subscribers just read nothing.
fn all_state_only_subs(subscribers: &[String], manifest: &Manifest) -> bool {
    subscribers.iter().all(|sub_ref| {
        if let Some((node_name, ep_name)) = sub_ref.split_once('/')
            && let Some(node) = manifest.nodes.get(node_name)
            && let Some(ep) = node.subscribers.get(ep_name)
        {
            return ep.state.unwrap_or(false) && !ep.required.unwrap_or(false);
        }
        // Can't resolve — assume it needs data (conservative)
        false
    })
}
