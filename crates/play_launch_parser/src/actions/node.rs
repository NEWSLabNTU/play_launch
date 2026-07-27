//! Node action implementation

use crate::{
    captures::NodeCapture,
    error::{ParseError, Result},
    substitution::{LaunchContext, Substitution, parse_substitutions, resolve_substitutions},
    xml::{Entity, EntityExt, XmlEntity},
};

/// phase-54 (issue 0007) — one parameter source, in document order.
///
/// Mirrors ROS 2's model: `launch_ros`'s `parse_nested_parameters` appends one
/// entry per `<param>` child (a `ParameterFile` for `from=`, a dict for
/// `name=`) into a single ordered list, and `execute` emits them in that
/// order. Kind carries no precedence — only position does.
#[derive(Debug, Clone)]
pub enum ParamSourceSpec {
    /// `<param name= value=/>` — an inline key/value.
    Inline(Parameter),
    /// `<param from=/>` — a parameter file path (substitutions unresolved).
    File(Vec<Substitution>),
}

/// Node action representing a ROS 2 node
#[derive(Debug, Clone)]
pub struct NodeAction {
    pub package: Vec<Substitution>,
    pub executable: Vec<Substitution>,
    pub name: Option<Vec<Substitution>>,
    pub namespace: Option<Vec<Substitution>>,
    pub parameters: Vec<Parameter>,
    pub param_files: Vec<Vec<Substitution>>,
    /// phase-54 (issue 0007) — the ORDERED parameter-source list, one element
    /// per `<param>` child in document order. ROS 2 has no "inline beats
    /// file" rule: `launch_ros` materializes an inline dict into a temp params
    /// file and emits `--params-file`/`-p` in list order, so POSITION is the
    /// only precedence. `parameters` / `param_files` above are the legacy
    /// split views, kept until every consumer reads this vec.
    pub param_sources: Vec<ParamSourceSpec>,
    pub remappings: Vec<Remapping>,
    pub environment: Vec<(String, String)>,
    pub args: Option<Vec<Substitution>>,
    pub output: Option<String>,
    pub respawn: Option<Vec<Substitution>>,
    pub respawn_delay: Option<Vec<Substitution>>,
    /// `<node machine="…">` — target host for ROS 2 multi-host launch.
    pub machine: Option<Vec<Substitution>>,
}

impl NodeAction {
    pub fn from_entity(entity: &XmlEntity) -> Result<Self> {
        let package =
            entity
                .required_attr_str("pkg")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "node".to_string(),
                    attribute: "pkg".to_string(),
                })?;
        let package = parse_substitutions(&package)?;

        let executable =
            entity
                .required_attr_str("exec")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "node".to_string(),
                    attribute: "exec".to_string(),
                })?;
        let executable = parse_substitutions(&executable)?;

        let name = entity
            .optional_attr_str("name")?
            .map(|s| parse_substitutions(&s))
            .transpose()?;

        let namespace = entity
            .optional_attr_str("namespace")?
            .map(|s| parse_substitutions(&s))
            .transpose()?;

        // `<node machine="…">` — ROS 2 multi-host launch target host.
        let machine = entity
            .optional_attr_str("machine")?
            .map(|s| parse_substitutions(&s))
            .transpose()?;

        // Parse children for params, remaps, env
        let mut parameters = Vec::new();
        let mut param_files = Vec::new();
        // phase-54 — the ordered view; the two vecs above are legacy splits.
        let mut param_sources: Vec<ParamSourceSpec> = Vec::new();
        let mut remappings = Vec::new();
        let mut environment = Vec::new();

        for child in entity.children() {
            match child.type_name() {
                "param" => {
                    // Check if this is a parameter file reference
                    if let Some(from_attr) = child.optional_attr_str("from")? {
                        // This is a parameter file
                        let subs = parse_substitutions(&from_attr)?;
                        param_files.push(subs.clone());
                        param_sources.push(ParamSourceSpec::File(subs));
                    } else {
                        // This is an inline parameter
                        let param = Parameter::from_entity(&child)?;
                        parameters.push(param.clone());
                        param_sources.push(ParamSourceSpec::Inline(param));
                    }
                }
                "remap" => remappings.push(Remapping::from_entity(&child)?),
                "env" => environment.push(parse_env(&child)?),
                "composable_node" | "composable-node" => {
                    // Composable nodes are children of node_container
                    // For now, we log and skip them (not yet fully supported)
                    log::debug!("Found composable_node in container (not yet fully supported)");
                }
                other => {
                    return Err(ParseError::UnexpectedElement {
                        parent: "node".to_string(),
                        child: other.to_string(),
                    });
                }
            }
        }

        // Parse args attribute (command-line arguments before --ros-args)
        let args = entity
            .optional_attr_str("args")?
            .map(|s| parse_substitutions(&s))
            .transpose()?;

        Ok(Self {
            package,
            executable,
            name,
            namespace,
            parameters,
            param_files,
            param_sources,
            remappings,
            environment,
            args,
            output: entity.optional_attr("output")?,
            respawn: entity
                .optional_attr_str("respawn")?
                .map(|s| parse_substitutions(&s))
                .transpose()?,
            respawn_delay: entity
                .optional_attr_str("respawn_delay")?
                .map(|s| parse_substitutions(&s))
                .transpose()?,
            machine,
        })
    }

    /// Convert NodeAction to NodeCapture by resolving substitutions
    pub fn to_capture(&self, context: &LaunchContext) -> Result<NodeCapture> {
        // Resolve package and executable (required)
        let package = resolve_substitutions(&self.package, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        let executable = resolve_substitutions(&self.executable, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

        // Resolve optional name and namespace
        let name = self
            .name
            .as_ref()
            .map(|n| resolve_substitutions(n, context))
            .transpose()
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

        let namespace = self
            .namespace
            .as_ref()
            .map(|ns| resolve_substitutions(ns, context))
            .transpose()
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

        // Resolve `machine` (multi-host target host).
        let machine = self
            .machine
            .as_ref()
            .map(|m| resolve_substitutions(m, context))
            .transpose()
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

        // Resolve parameters
        let parameters: Vec<(String, String)> = self
            .parameters
            .iter()
            .map(|p| {
                let value = resolve_substitutions(&p.value, context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                Ok((p.name.clone(), value))
            })
            .collect::<Result<Vec<_>>>()?;

        // Resolve parameter files
        let params_files: Vec<String> = self
            .param_files
            .iter()
            .map(|pf| {
                resolve_substitutions(pf, context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))
            })
            .collect::<Result<Vec<_>>>()?;

        // Resolve remappings
        let remappings: Vec<(String, String)> = self
            .remappings
            .iter()
            .map(|r| {
                let from = resolve_substitutions(&r.from, context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                let to = resolve_substitutions(&r.to, context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                Ok((from, to))
            })
            .collect::<Result<Vec<_>>>()?;

        // Resolve args attribute (command-line arguments before --ros-args)
        let arguments = if let Some(ref args_subs) = self.args {
            let resolved = resolve_substitutions(args_subs, context)
                .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
            // Split by whitespace to get individual arguments (matches Python behavior)
            resolved.split_whitespace().map(|s| s.to_string()).collect()
        } else {
            Vec::new()
        };

        // phase-54 (issue 0007) — resolve the ORDERED source list. One element
        // per `<param>` in document order; kind carries no precedence.
        let param_sources: Vec<crate::record::types::ParamSource> = self
            .param_sources
            .iter()
            .map(|src| match src {
                ParamSourceSpec::Inline(p) => {
                    let value = resolve_substitutions(&p.value, context)
                        .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                    Ok(crate::record::types::ParamSource::Inline {
                        name: p.name.clone(),
                        value,
                    })
                }
                ParamSourceSpec::File(subs) => {
                    // The capture carries the resolved PATH; the record
                    // generator swaps in the file's resolved contents (and
                    // expands launch temp files into Inline entries).
                    let path = resolve_substitutions(subs, context)
                        .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                    Ok(crate::record::types::ParamSource::File { content: path })
                }
            })
            .collect::<Result<Vec<_>>>()?;

        Ok(NodeCapture {
            package,
            executable,
            name,
            namespace,
            machine,
            parameters,
            params_files,
            param_sources,
            remappings,
            arguments,
            env_vars: self.environment.clone(),
            scope_id: None,
        })
    }
}

#[derive(Debug, Clone)]
pub struct Parameter {
    pub name: String,
    pub value: Vec<Substitution>,
}

impl Parameter {
    pub fn from_entity(entity: &XmlEntity) -> Result<Self> {
        let value_str: String =
            entity
                .required_attr("value")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "param".to_string(),
                    attribute: "value".to_string(),
                })?;

        Ok(Self {
            name: entity
                .required_attr("name")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "param".to_string(),
                    attribute: "name".to_string(),
                })?,
            value: parse_substitutions(&value_str)?,
        })
    }
}

#[derive(Debug, Clone)]
pub struct Remapping {
    pub from: Vec<Substitution>,
    pub to: Vec<Substitution>,
}

impl Remapping {
    pub fn from_entity(entity: &XmlEntity) -> Result<Self> {
        let from_str: String =
            entity
                .required_attr("from")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "remap".to_string(),
                    attribute: "from".to_string(),
                })?;

        let to_str: String =
            entity
                .required_attr("to")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "remap".to_string(),
                    attribute: "to".to_string(),
                })?;

        Ok(Self {
            from: parse_substitutions(&from_str)?,
            to: parse_substitutions(&to_str)?,
        })
    }
}

fn parse_env(entity: &XmlEntity) -> Result<(String, String)> {
    let name: String =
        entity
            .required_attr("name")?
            .ok_or_else(|| ParseError::MissingAttribute {
                element: "env".to_string(),
                attribute: "name".to_string(),
            })?;
    let value: String =
        entity
            .required_attr("value")?
            .ok_or_else(|| ParseError::MissingAttribute {
                element: "env".to_string(),
                attribute: "value".to_string(),
            })?;
    Ok((name, value))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xml::parse_xml_string;

    #[test]
    fn test_parse_simple_node() {
        let xml = r#"<node pkg="demo_nodes_cpp" exec="talker" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let node = NodeAction::from_entity(&entity).unwrap();

        assert_eq!(node.package.len(), 1);
        assert_eq!(node.executable.len(), 1);
        assert!(node.name.is_none());
        assert!(node.namespace.is_none());
    }

    #[test]
    fn test_parse_node_with_name() {
        let xml = r#"<node pkg="demo_nodes_cpp" exec="talker" name="my_talker" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let node = NodeAction::from_entity(&entity).unwrap();

        assert!(node.name.is_some());
    }

    #[test]
    fn test_parse_node_with_param() {
        let xml = r#"<node pkg="demo" exec="node">
            <param name="rate" value="10.0" />
        </node>"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let node = NodeAction::from_entity(&entity).unwrap();

        assert_eq!(node.parameters.len(), 1);
        assert_eq!(node.parameters[0].name, "rate");
        assert_eq!(
            node.parameters[0].value,
            vec![Substitution::Text("10.0".to_string())]
        );
    }

    #[test]
    fn test_parse_node_with_remap() {
        let xml = r#"<node pkg="demo" exec="node">
            <remap from="chatter" to="/chat" />
        </node>"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let node = NodeAction::from_entity(&entity).unwrap();

        assert_eq!(node.remappings.len(), 1);
        assert_eq!(
            node.remappings[0].from,
            vec![Substitution::Text("chatter".to_string())]
        );
        assert_eq!(
            node.remappings[0].to,
            vec![Substitution::Text("/chat".to_string())]
        );
    }

    #[test]
    fn test_parse_node_with_env() {
        let xml = r#"<node pkg="demo" exec="node">
            <env name="MY_VAR" value="my_value" />
        </node>"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let node = NodeAction::from_entity(&entity).unwrap();

        assert_eq!(node.environment.len(), 1);
        assert_eq!(node.environment[0].0, "MY_VAR");
        assert_eq!(node.environment[0].1, "my_value");
    }

    #[test]
    fn test_parse_node_with_param_file() {
        let xml = r#"<node pkg="demo" exec="node">
            <param from="/path/to/params.yaml" />
        </node>"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let node = NodeAction::from_entity(&entity).unwrap();

        assert_eq!(node.param_files.len(), 1);
        assert_eq!(
            node.param_files[0],
            vec![Substitution::Text("/path/to/params.yaml".to_string())]
        );
    }

    #[test]
    fn test_parse_node_with_mixed_params() {
        let xml = r#"<node pkg="demo" exec="node">
            <param name="inline_param" value="inline_value" />
            <param from="/path/to/params.yaml" />
            <param name="another_param" value="another_value" />
        </node>"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let node = NodeAction::from_entity(&entity).unwrap();

        assert_eq!(node.parameters.len(), 2);
        assert_eq!(node.param_files.len(), 1);
        assert_eq!(node.parameters[0].name, "inline_param");
        assert_eq!(node.parameters[1].name, "another_param");
    }

    #[test]
    fn test_parse_node_with_param_file_substitution() {
        let xml = r#"<node pkg="demo" exec="node">
            <param from="$(dirname)/params.yaml" />
        </node>"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let node = NodeAction::from_entity(&entity).unwrap();

        assert_eq!(node.param_files.len(), 1);
        assert_eq!(node.param_files[0].len(), 2);
        assert_eq!(node.param_files[0][0], Substitution::Dirname);
        assert_eq!(
            node.param_files[0][1],
            Substitution::Text("/params.yaml".to_string())
        );
    }
}

#[cfg(test)]
mod param_source_order_tests {
    use super::*;

    /// phase-54 (issue 0007) — sibling `<param>` children keep DOCUMENT ORDER
    /// in `param_sources`, so a file written after an inline value can win.
    /// ROS 2 has no "inline beats file" rule: `launch_ros` materializes an
    /// inline dict into a temp params file and emits `--params-file`/`-p` in
    /// list order, making position the only precedence.
    #[test]
    fn param_sources_preserve_document_order() {
        let xml = r#"<node pkg="p" exec="e">
            <param name="a" value="1"/>
            <param from="/cfg/a_is_2.yaml"/>
            <param name="b" value="3"/>
        </node>"#;
        let doc = roxmltree::Document::parse(xml).expect("xml parses");
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let node = NodeAction::from_entity(&entity).expect("node parses");

        assert_eq!(node.param_sources.len(), 3, "{:?}", node.param_sources);
        assert!(
            matches!(&node.param_sources[0], ParamSourceSpec::Inline(p) if p.name == "a"),
            "{:?}",
            node.param_sources[0]
        );
        assert!(
            matches!(&node.param_sources[1], ParamSourceSpec::File(_)),
            "the file must keep its position AFTER the inline `a`: {:?}",
            node.param_sources[1]
        );
        assert!(
            matches!(&node.param_sources[2], ParamSourceSpec::Inline(p) if p.name == "b"),
            "{:?}",
            node.param_sources[2]
        );

        // The legacy split views stay populated for back-compat.
        assert_eq!(node.parameters.len(), 2);
        assert_eq!(node.param_files.len(), 1);
    }
}
