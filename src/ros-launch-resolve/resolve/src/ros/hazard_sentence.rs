//! A hazard as one sentence, and its requirement in every finding about it
//! (rlm v0.3.13, play_launch phase 100 F3).
//!
//! `hazards.<h>.requirement: { id, source }` names the requirement of the
//! hazard analysis the hazard implements. The checker carries it two ways:
//!
//! - `check --explain` prints each hazard as one sentence: what is watched,
//!   for which fault classes, the requirement it implements, the bound it
//!   must meet, the levels it falls through with the time charged for each,
//!   and what each level commands;
//! - every diagnostic about a hazard of a scope gains `[requirement <id>]`,
//!   so a finding can be traced to the analysis without opening the
//!   contract.

use std::fmt::Write as _;

use ros_launch_manifest_check::Diagnostic;
use ros_launch_manifest_types::{CompareOp, PredicateValue, ValuePredicate};

use super::manifest_loader::{LevelRole, ManifestIndex, ResolvedHazard};

fn op_symbol(op: CompareOp) -> &'static str {
    match op {
        CompareOp::Equals => "==",
        CompareOp::NotEquals => "!=",
        CompareOp::Lt => "<",
        CompareOp::Le => "<=",
        CompareOp::Gt => ">",
        CompareOp::Ge => ">=",
    }
}

/// A predicate as a reader writes it: `linear.x == 0`.
pub fn predicate_text(p: &ValuePredicate) -> String {
    let v = match &p.value {
        PredicateValue::Bool(b) => b.to_string(),
        PredicateValue::Int(i) => i.to_string(),
        PredicateValue::Float(f) => f.to_string(),
        PredicateValue::Constant(c) => c.clone(),
        PredicateValue::Str(s) => format!("\"{s}\""),
        PredicateValue::Read(r) => r.clone(),
    };
    format!("{} {} {v}", p.field, op_symbol(p.op))
}

/// The tag a diagnostic about `h` carries.
fn tag(h: &ResolvedHazard) -> Option<String> {
    h.decl
        .requirement
        .as_ref()
        .map(|r| format!("[requirement {}]", r.id))
}

/// Is `d` about hazard `name`? Its path is under `hazards.<name>`, or its
/// message names the hazard the way every hazard rule does.
fn is_about(d: &Diagnostic, name: &str) -> bool {
    d.path == format!("hazards.{name}")
        || d.path.starts_with(&format!("hazards.{name}."))
        || d.message.contains(&format!("hazard '{name}'"))
}

fn tag_list(diags: &mut [Diagnostic], name: &str, tag: &str) {
    for d in diags.iter_mut() {
        if is_about(d, name) && !d.message.contains(tag) {
            d.message.push(' ');
            d.message.push_str(tag);
        }
    }
}

/// Append each hazard's requirement to every diagnostic about it. A
/// scope's own diagnostics are matched within the scope; a cross-scope one
/// (which carries no scope) is matched by name. Idempotent, so `check` can
/// call it again after the diagnostics it adds itself.
pub fn tag_requirements(index: &mut ManifestIndex) {
    let tagged: Vec<(usize, String, String)> = index
        .hazards
        .iter()
        .filter_map(|h| tag(h).map(|t| (h.scope_id, h.name.clone(), t)))
        .collect();
    for (scope, name, t) in &tagged {
        if let Some(m) = index.manifests.get_mut(scope) {
            tag_list(&mut m.diagnostics, name, t);
        }
        tag_list(&mut index.merge_diagnostics, name, t);
    }
}

/// `check --explain`'s hazard sentences.
pub fn render_hazards(index: &ManifestIndex) -> String {
    let mut out = String::new();
    for h in &index.hazards {
        out.push_str("  ");
        out.push_str(&sentence(index, h));
        out.push('\n');
    }
    if out.is_empty() {
        return out;
    }
    format!("\n-- Hazards (--explain) --\n{out}")
}

fn join_and(items: &[String]) -> String {
    match items.len() {
        0 => String::new(),
        1 => items[0].clone(),
        n => format!("{} and {}", items[..n - 1].join(", "), items[n - 1]),
    }
}

/// One hazard, in one sentence (a few clauses, one line).
pub fn sentence(index: &ManifestIndex, h: &ResolvedHazard) -> String {
    let d = &h.decl;
    let mut s = format!("hazard '{}'", h.name);
    match &d.requirement {
        Some(r) => match &r.source {
            Some(src) => write!(s, " implements {} ({src})", r.id).unwrap(),
            None => write!(s, " implements {}", r.id).unwrap(),
        },
        None => s.push_str(" names no requirement"),
    }
    if let Some(i) = &d.integrity {
        write!(s, " at {i}").unwrap();
    }
    // What is watched, and for which classes.
    let watched: Vec<String> = h
        .guards
        .iter()
        .map(|g| {
            if g.all_of && g.members.len() > 1 {
                format!("all of {{{}}}", g.members.join(", "))
            } else {
                g.members.join(", ")
            }
        })
        .collect();
    // An omitted `on:` is rlm's default classes (v0.3.15), named as such.
    let classes = d
        .classes()
        .iter()
        .map(|k| format!("{k:?}").to_lowercase())
        .collect::<Vec<_>>()
        .join(", ");
    if watched.is_empty() {
        s.push_str(": it watches nothing");
    } else {
        write!(s, ": it watches {} for {classes}", watched.join(" or ")).unwrap();
    }
    if let Some(w) = &d.when {
        write!(s, " (when {})", predicate_text(w)).unwrap();
    }
    // The bound.
    match (d.ftti.as_ref().and_then(|b| b.ms()), &d.deadline_by) {
        (Some(f), Some(c)) => write!(
            s,
            "; the system must reach a safe state within {f:.2}ms and meet {c}"
        )
        .unwrap(),
        (Some(f), None) => {
            write!(s, "; the system must reach a safe state within {f:.2}ms").unwrap()
        }
        (None, Some(c)) => write!(s, "; the system must meet {c}").unwrap(),
        (None, None) => s.push_str("; it states no bound"),
    }
    // The levels it falls through, with what was charged.
    let rows: Vec<String> = index
        .budgets
        .iter()
        .filter(|r| r.scope_id == h.scope_id && r.hazard == h.name)
        // A skipped level is not reached; an overrun row is the case where a
        // supervised budget is exceeded, which the budget table lists.
        .filter(|r| !matches!(r.role, LevelRole::Skipped | LevelRole::Overrun))
        .map(|r| {
            let role = match r.role {
                LevelRole::Mrc => " (minimal risk condition)",
                LevelRole::Dwell => " (dwell)",
                _ => "",
            };
            match r.total_ms {
                Some(t) => format!("'{}'{role} in {t:.2}ms", r.rung),
                None => format!("'{}'{role} in an unknown time", r.rung),
            }
        })
        .collect();
    let mode = |name: &str| {
        index
            .modes
            .iter()
            .find(|m| m.scope_id == h.scope_id && m.name == name)
    };
    let how = match d.entry() {
        Some(e) if mode(e).is_some() => format!("; it enters mode '{e}'"),
        Some(e) => format!("; it reacts by '{e}'"),
        None => "; it names no reaction".to_string(),
    };
    s.push_str(&how);
    if !rows.is_empty() {
        write!(s, ", reaching {}", join_and(&rows)).unwrap();
    }
    // What each level of its sequence commands.
    if let Some(entry) = d.entry().and_then(|e| mode(e)) {
        let chain = std::iter::once(&entry.name).chain(entry.decl.fallback.iter());
        for name in chain {
            let Some(m) = mode(name) else { continue };
            // A level this hazard's fault removes is never reached.
            let skipped = index.budgets.iter().any(|r| {
                r.scope_id == h.scope_id
                    && r.hazard == h.name
                    && r.rung == *name
                    && r.role == LevelRole::Skipped
            });
            if skipped {
                continue;
            }
            for (topic, preds) in &m.commands {
                let p: Vec<String> = preds.iter().map(predicate_text).collect();
                write!(s, ". At '{}', {topic} must carry {}", m.name, join_and(&p)).unwrap();
            }
        }
    }
    s.push('.');
    s
}
