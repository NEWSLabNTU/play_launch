//! `$(var ...)` substitution engine for manifest args.
//!
//! Resolves `$(var name)` references in all string fields of a manifest
//! using a provided args map. Runs before static checks and namespace resolution.

use crate::types::ArgDecl;
use std::collections::{BTreeMap, HashMap};

/// Error during substitution.
#[derive(Debug, thiserror::Error)]
pub enum SubstError {
    #[error("unresolved variable: $(var {name}) — not in args")]
    Unresolved { name: String },
    #[error("required arg '{name}' has no default and was not provided")]
    RequiredArgMissing { name: String },
    #[error("arg '{name}' has invalid value '{value}' — expected one of {expected}")]
    InvalidArgValue {
        name: String,
        value: String,
        expected: String,
    },
    #[error("malformed substitution: {expr}")]
    Malformed { expr: String },
}

/// Resolve manifest args from caller-provided scope args.
///
/// All manifest args are mandatory. Errors if any declared arg is missing
/// from `caller_args`. Validates values against type constraints (bool, choices).
/// Caller args not declared in the manifest are passed through.
pub fn resolve_args(
    manifest_args: &BTreeMap<String, ArgDecl>,
    caller_args: &HashMap<String, String>,
) -> Result<HashMap<String, String>, SubstError> {
    let mut resolved = HashMap::new();

    for (name, decl) in manifest_args {
        if let Some(value) = caller_args.get(name) {
            // Validate against type constraint
            if let Some(valid) = decl.valid_values()
                && !valid.contains(&value.as_str())
            {
                return Err(SubstError::InvalidArgValue {
                    name: name.clone(),
                    value: value.clone(),
                    expected: format!("{:?}", valid),
                });
            }
            resolved.insert(name.clone(), value.clone());
        } else {
            return Err(SubstError::RequiredArgMissing { name: name.clone() });
        }
    }

    // Also include caller args not declared in manifest (pass-through)
    for (name, value) in caller_args {
        if !resolved.contains_key(name) {
            resolved.insert(name.clone(), value.clone());
        }
    }

    Ok(resolved)
}

/// Replace all `$(var name)` in a string with values from the args map.
pub fn substitute_str(s: &str, args: &HashMap<String, String>) -> Result<String, SubstError> {
    let mut result = String::with_capacity(s.len());
    let mut remaining = s;

    while let Some(start) = remaining.find("$(var ") {
        result.push_str(&remaining[..start]);
        let after_prefix = &remaining[start + 6..]; // skip "$(var "
        let end = after_prefix
            .find(')')
            .ok_or_else(|| SubstError::Malformed {
                expr: remaining[start..].to_string(),
            })?;
        let var_name = after_prefix[..end].trim();
        let value = args.get(var_name).ok_or_else(|| SubstError::Unresolved {
            name: var_name.to_string(),
        })?;
        result.push_str(value);
        remaining = &after_prefix[end + 1..];
    }

    result.push_str(remaining);
    Ok(result)
}

/// Substitute all `$(var ...)` in a Vec<String>.
pub fn substitute_vec(
    v: &[String],
    args: &HashMap<String, String>,
) -> Result<Vec<String>, SubstError> {
    v.iter().map(|s| substitute_str(s, args)).collect()
}

/// Substitute all `$(var ...)` in an Option<String>.
pub fn substitute_opt(
    o: &Option<String>,
    args: &HashMap<String, String>,
) -> Result<Option<String>, SubstError> {
    match o {
        Some(s) => Ok(Some(substitute_str(s, args)?)),
        None => Ok(None),
    }
}

/// Substitute all `$(var ...)` references in a manifest's string fields.
///
/// This walks all topics, services, imports, exports, and endpoint references.
/// Does NOT modify node/topic keys (they are manifest-local identifiers).
pub fn substitute_manifest(
    manifest: &crate::Manifest,
    args: &HashMap<String, String>,
) -> Result<crate::Manifest, SubstError> {
    let mut m = manifest.clone();

    // Nodes: conditions
    for node in m.nodes.values_mut() {
        node.if_condition = substitute_opt(&node.if_condition, args)?;
        node.unless_condition = substitute_opt(&node.unless_condition, args)?;
        // Node-level paths: conditions + fields
        for path in node.paths.values_mut() {
            path.if_condition = substitute_opt(&path.if_condition, args)?;
            path.unless_condition = substitute_opt(&path.unless_condition, args)?;
            path.input = substitute_vec(&path.input, args)?;
            path.output = substitute_vec(&path.output, args)?;
            path.correlation = substitute_opt(&path.correlation, args)?;
        }
    }

    // Topics: conditions + fields
    for topic in m.topics.values_mut() {
        topic.if_condition = substitute_opt(&topic.if_condition, args)?;
        topic.unless_condition = substitute_opt(&topic.unless_condition, args)?;
        topic.msg_type = substitute_str(&topic.msg_type, args)?;
        topic.publishers = substitute_vec(&topic.publishers, args)?;
        topic.subscribers = substitute_vec(&topic.subscribers, args)?;
    }

    // Services: conditions + fields
    for svc in m.services.values_mut() {
        svc.if_condition = substitute_opt(&svc.if_condition, args)?;
        svc.unless_condition = substitute_opt(&svc.unless_condition, args)?;
        svc.srv_type = substitute_str(&svc.srv_type, args)?;
        svc.server = substitute_vec(&svc.server, args)?;
        svc.client = substitute_vec(&svc.client, args)?;
    }

    // Actions: conditions + fields
    for act in m.actions.values_mut() {
        act.if_condition = substitute_opt(&act.if_condition, args)?;
        act.unless_condition = substitute_opt(&act.unless_condition, args)?;
        act.action_type = substitute_str(&act.action_type, args)?;
        act.server = substitute_vec(&act.server, args)?;
        act.client = substitute_vec(&act.client, args)?;
    }

    // Scope paths: conditions + fields
    for path in m.paths.values_mut() {
        path.if_condition = substitute_opt(&path.if_condition, args)?;
        path.unless_condition = substitute_opt(&path.unless_condition, args)?;
        path.input = substitute_vec(&path.input, args)?;
        path.output = substitute_vec(&path.output, args)?;
        path.correlation = substitute_opt(&path.correlation, args)?;
    }

    // Includes: external manifest path
    for include in m.includes.values_mut() {
        if let crate::IncludeDecl::External { manifest } = include {
            *manifest = substitute_str(manifest, args)?;
        }
    }

    // External topics: type field
    for ext in m.external_topics.values_mut() {
        if let Some(t) = &ext.msg_type {
            ext.msg_type = Some(substitute_str(t, args)?);
        }
    }

    Ok(m)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_substitute_str_basic() {
        let args = HashMap::from([("name".into(), "hello".into())]);
        assert_eq!(substitute_str("$(var name)", &args).unwrap(), "hello");
        assert_eq!(
            substitute_str("prefix_$(var name)_suffix", &args).unwrap(),
            "prefix_hello_suffix"
        );
    }

    #[test]
    fn test_substitute_str_multiple() {
        let args = HashMap::from([("a".into(), "X".into()), ("b".into(), "Y".into())]);
        assert_eq!(substitute_str("$(var a)/$(var b)", &args).unwrap(), "X/Y");
    }

    #[test]
    fn test_substitute_str_no_vars() {
        let args = HashMap::new();
        assert_eq!(
            substitute_str("plain string", &args).unwrap(),
            "plain string"
        );
    }

    #[test]
    fn test_substitute_str_unresolved() {
        let args = HashMap::new();
        let err = substitute_str("$(var missing)", &args).unwrap_err();
        assert!(matches!(err, SubstError::Unresolved { .. }));
    }

    #[test]
    fn test_substitute_str_malformed() {
        let args = HashMap::new();
        let err = substitute_str("$(var unclosed", &args).unwrap_err();
        assert!(matches!(err, SubstError::Malformed { .. }));
    }

    #[test]
    fn test_resolve_args_all_present() {
        let manifest_args =
            BTreeMap::from([("a".into(), ArgDecl::String), ("b".into(), ArgDecl::String)]);
        let caller_args =
            HashMap::from([("a".into(), "val_a".into()), ("b".into(), "val_b".into())]);
        let resolved = resolve_args(&manifest_args, &caller_args).unwrap();
        assert_eq!(resolved["a"], "val_a");
        assert_eq!(resolved["b"], "val_b");
    }

    #[test]
    fn test_resolve_args_missing() {
        let manifest_args = BTreeMap::from([("req".into(), ArgDecl::String)]);
        let caller_args = HashMap::new();
        let err = resolve_args(&manifest_args, &caller_args).unwrap_err();
        assert!(matches!(err, SubstError::RequiredArgMissing { .. }));
    }

    #[test]
    fn test_resolve_args_passthrough() {
        let manifest_args = BTreeMap::new();
        let caller_args = HashMap::from([("extra".into(), "val".into())]);
        let resolved = resolve_args(&manifest_args, &caller_args).unwrap();
        assert_eq!(resolved["extra"], "val");
    }

    #[test]
    fn test_resolve_args_bool_valid() {
        let manifest_args = BTreeMap::from([("flag".into(), ArgDecl::Bool)]);
        let caller_args = HashMap::from([("flag".into(), "true".into())]);
        let resolved = resolve_args(&manifest_args, &caller_args).unwrap();
        assert_eq!(resolved["flag"], "true");
    }

    #[test]
    fn test_resolve_args_bool_invalid() {
        let manifest_args = BTreeMap::from([("flag".into(), ArgDecl::Bool)]);
        let caller_args = HashMap::from([("flag".into(), "yes".into())]);
        let err = resolve_args(&manifest_args, &caller_args).unwrap_err();
        assert!(matches!(err, SubstError::InvalidArgValue { .. }));
    }

    #[test]
    fn test_resolve_args_choices_valid() {
        let manifest_args = BTreeMap::from([(
            "mode".into(),
            ArgDecl::Choices(vec!["ndt".into(), "eagleye".into()]),
        )]);
        let caller_args = HashMap::from([("mode".into(), "ndt".into())]);
        let resolved = resolve_args(&manifest_args, &caller_args).unwrap();
        assert_eq!(resolved["mode"], "ndt");
    }

    #[test]
    fn test_resolve_args_choices_invalid() {
        let manifest_args = BTreeMap::from([(
            "mode".into(),
            ArgDecl::Choices(vec!["ndt".into(), "eagleye".into()]),
        )]);
        let caller_args = HashMap::from([("mode".into(), "gnss".into())]);
        let err = resolve_args(&manifest_args, &caller_args).unwrap_err();
        assert!(matches!(err, SubstError::InvalidArgValue { .. }));
    }

    #[test]
    fn test_substitute_manifest_topics() {
        // Parse manifest with args (map form — keys only)
        let yaml = r#"
args:
  topic_name:

version: 1
nodes:
  planner:
    sub: [objects]
topics:
  objects:
    type: autoware_perception_msgs/msg/PredictedObjects
    sub: [planner/objects]
"#;
        let manifest = crate::parse_manifest_str(yaml).unwrap();
        assert_eq!(manifest.args.len(), 1);
        assert!(manifest.args.contains_key("topic_name"));

        // Parse with list form
        let yaml_list = r#"
args: [topic_name, other_arg]
version: 1
"#;
        let m2 = crate::parse_manifest_str(yaml_list).unwrap();
        assert_eq!(m2.args.len(), 2);
        assert!(m2.args.contains_key("topic_name"));
        assert!(m2.args.contains_key("other_arg"));

        // Substitution on a manifest with $(var ...) in fields
        let yaml_with_var = r#"
args:
  topic_name:

version: 1
topics:
  input:
    type: $(var topic_name)
    pub: []
    sub: []
"#;
        let m = crate::parse_manifest_str(yaml_with_var).unwrap();
        let args = HashMap::from([("topic_name".into(), "/resolved/topic".into())]);
        let substituted = substitute_manifest(&m, &args).unwrap();
        assert_eq!(substituted.topics["input"].msg_type, "/resolved/topic");
    }

    #[test]
    fn test_parse_typed_args() {
        let yaml = r#"
args:
  free_arg:
  bool_arg:
    type: bool
  enum_arg:
    choices: [ndt, eagleye, gnss]
version: 1
"#;
        let m = crate::parse_manifest_str(yaml).unwrap();
        assert_eq!(m.args.len(), 3);
        assert!(matches!(m.args["free_arg"], ArgDecl::String));
        assert!(matches!(m.args["bool_arg"], ArgDecl::Bool));
        match &m.args["enum_arg"] {
            ArgDecl::Choices(v) => assert_eq!(v, &vec!["ndt", "eagleye", "gnss"]),
            _ => panic!("expected Choices"),
        }
    }
}
