//! Launch parameter values checked against the contract's declarations
//! (nano-ros phase 446 W2).
//!
//! A node whose contract carries a `params:` section declares every
//! parameter it has, by name and ROS 2 type. A launch value for a name it
//! does not declare, or one that does not parse as the declared type, is a
//! resolution error naming the node, the parameter and where the value came
//! from. Left alone, the first reaches a node that ignores an undeclared
//! override in silence, and the second throws only when the code declares
//! the parameter -- at runtime, far from the file that set it.
//!
//! A node with no `params:` in its contract is not checked: presence of
//! `contracts.node_params.<fqn>` is the declaration.

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

/// Every disagreement between a declaring node's launch values and its
/// declarations, one line each. `Ok` when there is none.
pub fn check_declared_params(model: &SystemModel) -> Result<(), Vec<String>> {
    let mut errors = Vec::new();
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

        let inline = format!("launch file {launch}");
        let file = format!("a parameter file loaded by launch file {launch}");
        let mut values: Vec<(String, ParamValue, &str)> = Vec::new();
        // The ordered list is authoritative when present (phase 54); older
        // models carry only the legacy split views.
        if inst.param_sources.is_empty() {
            for content in &inst.params_files {
                values.extend(
                    param_file_values(content, fqn)
                        .into_iter()
                        .map(|(n, v)| (n, v, file.as_str())),
                );
            }
            values.extend(
                inst.params
                    .iter()
                    .map(|(n, v)| (n.clone(), v.clone(), inline.as_str())),
            );
        } else {
            for src in &inst.param_sources {
                match src {
                    ParamSource::Inline { name, value } => {
                        values.push((name.clone(), value.clone(), inline.as_str()));
                    }
                    ParamSource::File { content } => values.extend(
                        param_file_values(content, fqn)
                            .into_iter()
                            .map(|(n, v)| (n, v, file.as_str())),
                    ),
                }
            }
        }

        for (name, value, source) in values {
            if name.starts_with(QOS_OVERRIDES_PREFIX) {
                continue;
            }
            let ty = declared
                .get(&name)
                .map(|d| d.ty)
                .or_else(|| IMPLICIT.iter().find(|(n, _)| *n == name).map(|(_, t)| *t));
            match ty {
                None => errors.push(format!(
                    "node `{fqn}`: parameter `{name}`, set by {source}, is not declared in \
                     {contract}; it declares: {}",
                    declared.keys().cloned().collect::<Vec<_>>().join(", ")
                )),
                Some(ty) => {
                    if let Err(found) = ty.check(&value) {
                        errors.push(format!(
                            "node `{fqn}`: parameter `{name}`, set by {source}, is declared \
                             `{ty}` in {contract} but {found}"
                        ));
                    }
                }
            }
        }
    }
    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors)
    }
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
        assert_eq!(check_declared_params(&m), Ok(()));
    }

    #[test]
    fn an_undeclared_name_is_refused_naming_node_parameter_and_file() {
        let m = model_with(vec![inline("update_rat", ParamValue::Int(10))]);
        let errors = check_declared_params(&m).unwrap_err();
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
        let errors = check_declared_params(&m).unwrap_err();
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
        assert_eq!(check_declared_params(&m), Ok(()));
    }

    /// The legacy split views (models that predate the ordered list) are
    /// checked too.
    #[test]
    fn legacy_views_are_checked() {
        let mut m = model_with(Vec::new());
        let inst = m.structure.nodes.get_mut(NODE).unwrap();
        inst.params
            .insert("update_rate".to_string(), ParamValue::Str("fast".into()));
        let errors = check_declared_params(&m).unwrap_err();
        assert!(errors[0].contains("got a string (\"fast\")"), "{errors:?}");
    }

    /// No `params:` in the contract means no check: today's behaviour.
    #[test]
    fn a_node_without_declarations_is_not_checked() {
        let mut m = model_with(vec![inline("whatever", ParamValue::Int(1))]);
        m.contracts.node_params.clear();
        assert_eq!(check_declared_params(&m), Ok(()));
    }
}
