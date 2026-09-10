//! Launch parameter values checked against the contract's declarations
//! (nano-ros phase 446 W2).
//!
//! A node whose contract carries a `params:` section declares every
//! parameter it has, by name and ROS 2 type. What happens to a launch value
//! that disagrees depends on how the value reached the node:
//!
//! - A value addressed to the node BY NAME -- an inline `<param>`, a global
//!   parameter, or a parameter-file section whose key is the node's FQN or
//!   bare name -- that names an undeclared parameter is an ERROR. Someone
//!   meant this node, so `upate_rate` is a typo, and left alone it reaches a
//!   node that ignores it in silence.
//! - The same value reaching the node through a WILDCARD key (`/**`, `/*`,
//!   `/**/foo`, any key with a `*`) is a WARNING. rclcpp ignores an
//!   undeclared override, and shared files (Autoware's
//!   `vehicle_info.param.yaml`) are loaded into many nodes that each read a
//!   subset of them.
//! - A value for a DECLARED parameter that does not parse as the declared
//!   type is an ERROR whatever the key: it throws at declare time, at
//!   runtime, far from the file that set it.
//!
//! A node with no `params:` in its contract is not checked: presence of
//! `contracts.node_params.<fqn>` is the declaration. A node whose contract
//! says `params: {}` (phase 446 F1) has an EMPTY entry: it declares no
//! parameters, so every launch value for it is undeclared, under the same
//! rules (the implicit and `qos_overrides.*` exemptions still apply).

use ros_launch_manifest_model::{
    ParamSource, ParamType, ParamValue, SystemModel, param_file_values,
};

/// Parameters rclcpp declares on EVERY node, whatever its code does. A
/// contract describes the node's own code, so it need not list them.
const IMPLICIT: &[(&str, ParamType)] = &[
    ("use_sim_time", ParamType::Bool),
    ("start_type_description_service", ParamType::Bool),
];

/// The QoS override parameters (`qos_overrides.<topic>.<entity>.<policy>`).
/// This resolver derives some of them itself from contract deadline and
/// liveliness declarations, and rclcpp declares them on demand for an entity
/// created with `QosOverridingOptions` -- never through the node's own
/// parameter declarations.
const QOS_OVERRIDES_PREFIX: &str = "qos_overrides.";

/// What the check found: `errors` refuse the model, `warnings` go into
/// `meta.diagnostics`. One line each.
#[derive(Debug, Default, PartialEq)]
pub struct ParamFindings {
    pub errors: Vec<String>,
    pub warnings: Vec<String>,
}

/// One launch value and where it came from.
struct Value {
    name: String,
    value: ParamValue,
    /// `set by ...`, for the message.
    source: String,
    /// It reached the node through a wildcard param-file key.
    wildcard: bool,
}

/// The values one parameter file gives `fqn`, each tagged with the section
/// key that matched. Section matching is the manifest crate's own
/// (`param_file_values`), applied to one section at a time, so this cannot
/// disagree with what a spawn or a bake would apply.
fn file_values(content: &str, fqn: &str, launch: &str, out: &mut Vec<Value>) {
    let Ok(serde_yaml_ng::Value::Mapping(sections)) =
        serde_yaml_ng::from_str::<serde_yaml_ng::Value>(content)
    else {
        return;
    };
    for (key, body) in sections {
        let Some(k) = key.as_str().map(str::to_string) else {
            continue;
        };
        let mut one = serde_yaml_ng::Mapping::new();
        one.insert(key, body);
        let Ok(section) = serde_yaml_ng::to_string(&one) else {
            continue;
        };
        let wildcard = k.contains('*');
        let kind = if wildcard { "wildcard key" } else { "key" };
        let source = format!("{kind} `{k}` of a parameter file loaded by launch file {launch}");
        out.extend(
            param_file_values(&section, fqn)
                .into_iter()
                .map(|(name, value)| Value {
                    name,
                    value,
                    source: source.clone(),
                    wildcard,
                }),
        );
    }
}

/// Every disagreement between a declaring node's launch values and its
/// declarations.
pub fn check_declared_params(model: &SystemModel) -> ParamFindings {
    let mut findings = ParamFindings::default();
    for (fqn, declared) in &model.contracts.node_params {
        let Some(inst) = model.structure.nodes.get(fqn) else {
            continue;
        };
        let scope = model.structure.scopes.get(&inst.scope);
        let launch = scope
            .and_then(|s| s.file.clone())
            .unwrap_or_else(|| inst.scope.clone());
        let contract = scope
            .and_then(|s| s.manifest.clone())
            .map(|m| format!("its contract ({m})"))
            .unwrap_or_else(|| "its contract".to_string());

        let inline = |name: &String, value: &ParamValue| Value {
            name: name.clone(),
            value: value.clone(),
            source: format!("launch file {launch}"),
            wildcard: false,
        };
        let mut values: Vec<Value> = Vec::new();
        // The ordered list is authoritative when present (phase 54); older
        // models carry only the legacy split views.
        if inst.param_sources.is_empty() {
            for content in &inst.params_files {
                file_values(content, fqn, &launch, &mut values);
            }
            values.extend(inst.params.iter().map(|(n, v)| inline(n, v)));
        } else {
            for src in &inst.param_sources {
                match src {
                    ParamSource::Inline { name, value } => values.push(inline(name, value)),
                    ParamSource::File { content } => {
                        file_values(content, fqn, &launch, &mut values)
                    }
                }
            }
        }

        for Value {
            name,
            value,
            source,
            wildcard,
        } in values
        {
            if name.starts_with(QOS_OVERRIDES_PREFIX) {
                continue;
            }
            let ty = declared
                .get(&name)
                .map(|d| d.ty)
                .or_else(|| IMPLICIT.iter().find(|(n, _)| *n == name).map(|(_, t)| *t));
            match ty {
                None if wildcard => findings.warnings.push(format!(
                    "param-undeclared: node `{fqn}`: parameter `{name}`, set by {source}, is \
                     not declared in {contract}; the node ignores it"
                )),
                None => findings.errors.push(format!(
                    "node `{fqn}`: parameter `{name}`, set by {source}, is not declared in \
                     {contract}; {}",
                    if declared.is_empty() {
                        "it declares no parameters (`params: {}`)".to_string()
                    } else {
                        format!(
                            "it declares: {}",
                            declared.keys().cloned().collect::<Vec<_>>().join(", ")
                        )
                    }
                )),
                Some(ty) => {
                    if let Err(found) = ty.check(&value) {
                        findings.errors.push(format!(
                            "node `{fqn}`: parameter `{name}`, set by {source}, is declared \
                             `{ty}` in {contract} but {found}"
                        ));
                    }
                }
            }
        }
    }
    findings
}

#[cfg(test)]
mod tests {
    use super::*;
    use ros_launch_manifest_model::{NodeInstance, ParamContract, ScopeInfo};
    use std::collections::BTreeMap;

    const NODE: &str = "/system/mrm_handler";

    /// One node in `launch/island.launch.xml`, declaring two parameters.
    fn model_with(sources: Vec<ParamSource>) -> SystemModel {
        let mut m = SystemModel::default();
        m.structure.scopes.insert(
            "island".to_string(),
            ScopeInfo {
                file: Some("island.launch.xml".to_string()),
                manifest: Some("island.contract.yaml".to_string()),
                ..Default::default()
            },
        );
        m.structure.nodes.insert(
            NODE.to_string(),
            NodeInstance {
                scope: "island".to_string(),
                param_sources: sources,
                ..Default::default()
            },
        );
        m.contracts.node_params.insert(
            NODE.to_string(),
            BTreeMap::from([
                (
                    "update_rate".to_string(),
                    ParamContract {
                        ty: ParamType::Integer,
                    },
                ),
                (
                    "timeout".to_string(),
                    ParamContract {
                        ty: ParamType::Double,
                    },
                ),
            ]),
        );
        m
    }

    /// The errors, asserting there are some.
    fn errors_of(m: &SystemModel) -> Vec<String> {
        let f = check_declared_params(m);
        assert!(!f.errors.is_empty(), "expected errors, got {f:?}");
        f.errors
    }

    fn file(content: &str) -> ParamSource {
        ParamSource::File {
            content: content.to_string(),
        }
    }

    fn inline(name: &str, value: ParamValue) -> ParamSource {
        ParamSource::Inline {
            name: name.to_string(),
            value,
        }
    }

    #[test]
    fn declared_values_of_the_declared_type_pass() {
        let m = model_with(vec![
            inline("update_rate", ParamValue::Int(10)),
            ParamSource::File {
                content: "/**:\n  ros__parameters:\n    timeout: 0.5\n    use_sim_time: false\n"
                    .to_string(),
            },
            inline(
                "qos_overrides./out.publisher.deadline",
                ParamValue::Int(100),
            ),
        ]);
        assert_eq!(check_declared_params(&m), ParamFindings::default());
    }

    #[test]
    fn an_undeclared_name_is_refused_naming_node_parameter_and_file() {
        let m = model_with(vec![inline("update_rat", ParamValue::Int(10))]);
        let errors = errors_of(&m);
        assert_eq!(errors.len(), 1, "{errors:?}");
        let e = &errors[0];
        for part in [
            "node `/system/mrm_handler`",
            "parameter `update_rat`",
            "launch file island.launch.xml",
            "not declared",
            "island.contract.yaml",
            "timeout, update_rate",
        ] {
            assert!(e.contains(part), "missing `{part}` in: {e}");
        }
    }

    #[test]
    fn a_mistyped_value_is_refused_naming_node_parameter_and_file() {
        let m = model_with(vec![ParamSource::File {
            content: "/system/mrm_handler:\n  ros__parameters:\n    timeout: 5\n".to_string(),
        }]);
        let errors = errors_of(&m);
        assert_eq!(errors.len(), 1, "{errors:?}");
        let e = &errors[0];
        for part in [
            "node `/system/mrm_handler`",
            "parameter `timeout`",
            "a parameter file loaded by launch file island.launch.xml",
            "declared `double`",
            "got an integer (5)",
            "5.0",
        ] {
            assert!(e.contains(part), "missing `{part}` in: {e}");
        }
    }

    /// A parameter file's section for ANOTHER node is not this node's value.
    #[test]
    fn another_nodes_section_is_not_checked_against_this_node() {
        let m = model_with(vec![ParamSource::File {
            content: "/system/other:\n  ros__parameters:\n    anything: true\n".to_string(),
        }]);
        assert_eq!(check_declared_params(&m), ParamFindings::default());
    }

    /// The legacy split views (models that predate the ordered list) are
    /// checked too.
    #[test]
    fn legacy_views_are_checked() {
        let mut m = model_with(Vec::new());
        let inst = m.structure.nodes.get_mut(NODE).unwrap();
        inst.params
            .insert("update_rate".to_string(), ParamValue::Str("fast".into()));
        let errors = errors_of(&m);
        assert!(errors[0].contains("got a string (\"fast\")"), "{errors:?}");
    }

    /// No `params:` in the contract means no check: today's behaviour.
    #[test]
    fn a_node_without_declarations_is_not_checked() {
        let mut m = model_with(vec![inline("whatever", ParamValue::Int(1))]);
        m.contracts.node_params.clear();
        assert_eq!(check_declared_params(&m), ParamFindings::default());
    }

    /// Option B: a shared file's undeclared key reaches the node through a
    /// wildcard, and rclcpp ignores it -- a warning, not a refusal.
    #[test]
    fn an_undeclared_name_under_a_wildcard_key_is_a_warning() {
        for key in ["/**", "/**/mrm_handler", "/*/mrm_handler"] {
            let m = model_with(vec![file(&format!(
                "{key}:\n  ros__parameters:\n    wheel_base: 2.7\n    timeout: 0.5\n"
            ))]);
            let f = check_declared_params(&m);
            assert!(f.errors.is_empty(), "{key}: {f:?}");
            assert_eq!(f.warnings.len(), 1, "{key}: {f:?}");
            let w = &f.warnings[0];
            for part in [
                "node `/system/mrm_handler`",
                "parameter `wheel_base`",
                &format!("wildcard key `{key}`"),
                "launch file island.launch.xml",
                "not declared",
                "island.contract.yaml",
                "ignores it",
            ] {
                assert!(w.contains(part), "missing `{part}` in: {w}");
            }
        }
    }

    /// The same key under the node's own name is addressed to it: a typo.
    #[test]
    fn an_undeclared_name_under_the_nodes_own_key_is_refused() {
        for key in ["/system/mrm_handler", "mrm_handler"] {
            let m = model_with(vec![file(&format!(
                "{key}:\n  ros__parameters:\n    upate_rate: 10\n"
            ))]);
            let f = check_declared_params(&m);
            assert!(f.warnings.is_empty(), "{key}: {f:?}");
            assert_eq!(f.errors.len(), 1, "{key}: {f:?}");
            let e = &f.errors[0];
            for part in [
                "node `/system/mrm_handler`",
                "parameter `upate_rate`",
                &format!("key `{key}`"),
                "launch file island.launch.xml",
                "not declared",
                "timeout, update_rate",
            ] {
                assert!(e.contains(part), "missing `{part}` in: {e}");
            }
            assert!(!e.contains("wildcard"), "{e}");
        }
    }

    /// One file, two sections: each value is judged by its own key.
    #[test]
    fn each_section_of_one_file_is_judged_by_its_own_key() {
        let m = model_with(vec![file(
            "/**:\n  ros__parameters:\n    shared: 1\n\
             /system/mrm_handler:\n  ros__parameters:\n    upate_rate: 10\n",
        )]);
        let f = check_declared_params(&m);
        assert_eq!((f.errors.len(), f.warnings.len()), (1, 1), "{f:?}");
        assert!(f.errors[0].contains("`upate_rate`"), "{f:?}");
        assert!(f.warnings[0].contains("`shared`"), "{f:?}");
    }

    /// An inline `<param>` on the node is addressed to it by name.
    #[test]
    fn an_undeclared_inline_name_is_refused() {
        let m = model_with(vec![inline("upate_rate", ParamValue::Int(10))]);
        let f = check_declared_params(&m);
        assert!(f.warnings.is_empty(), "{f:?}");
        assert_eq!(f.errors.len(), 1, "{f:?}");
        assert!(f.errors[0].contains("parameter `upate_rate`"), "{f:?}");
    }

    /// `params: {}` declares no parameters (phase 446 F1): a value addressed
    /// to the node by name is undeclared, and an error that says so.
    #[test]
    fn an_inline_value_on_an_empty_declaration_is_refused() {
        let mut m = model_with(vec![inline("update_rate", ParamValue::Int(10))]);
        m.contracts
            .node_params
            .insert(NODE.to_string(), BTreeMap::new());
        let f = check_declared_params(&m);
        assert!(f.warnings.is_empty(), "{f:?}");
        assert_eq!(f.errors.len(), 1, "{f:?}");
        let e = &f.errors[0];
        for part in [
            "node `/system/mrm_handler`",
            "parameter `update_rate`",
            "launch file island.launch.xml",
            "not declared",
            "it declares no parameters (`params: {}`)",
        ] {
            assert!(e.contains(part), "missing `{part}` in: {e}");
        }
    }

    /// The same node under a wildcard key only warns, and the exemptions
    /// hold: an empty declaration is the option-B rule with nothing declared.
    #[test]
    fn a_wildcard_value_on_an_empty_declaration_warns() {
        let mut m = model_with(vec![
            file("/**:\n  ros__parameters:\n    wheel_base: 2.7\n    use_sim_time: true\n"),
            inline(
                "qos_overrides./out.publisher.deadline",
                ParamValue::Int(100),
            ),
        ]);
        m.contracts
            .node_params
            .insert(NODE.to_string(), BTreeMap::new());
        let f = check_declared_params(&m);
        assert!(f.errors.is_empty(), "{f:?}");
        assert_eq!(f.warnings.len(), 1, "{f:?}");
        let w = &f.warnings[0];
        for part in ["parameter `wheel_base`", "wildcard key `/**`", "ignores it"] {
            assert!(w.contains(part), "missing `{part}` in: {w}");
        }
    }

    /// A declared parameter of the wrong type is wrong wherever it came from.
    #[test]
    fn a_mistyped_value_under_a_wildcard_key_is_refused() {
        let m = model_with(vec![file("/**:\n  ros__parameters:\n    timeout: 5\n")]);
        let f = check_declared_params(&m);
        assert!(f.warnings.is_empty(), "{f:?}");
        assert_eq!(f.errors.len(), 1, "{f:?}");
        let e = &f.errors[0];
        for part in [
            "parameter `timeout`",
            "wildcard key `/**`",
            "declared `double`",
            "5.0",
        ] {
            assert!(e.contains(part), "missing `{part}` in: {e}");
        }
    }
}
