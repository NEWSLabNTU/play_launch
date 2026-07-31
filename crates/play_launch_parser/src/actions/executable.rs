//! Executable action implementation

use crate::{
    captures::NodeCapture,
    error::{ParseError, Result},
    substitution::{LaunchContext, Substitution, parse_substitutions, resolve_substitutions},
    xml::{Entity, EntityExt, XmlEntity},
};

/// Executable action for launching non-ROS executables
#[derive(Debug, Clone)]
pub struct ExecutableAction {
    pub cmd: Vec<Substitution>,
    pub cwd: Option<Vec<Substitution>>,
    pub name: Option<Vec<Substitution>>,
    pub shell: bool,
    pub output: Option<String>,
    pub environment: Vec<(String, String)>,
    pub arguments: Vec<Vec<Substitution>>,
}

impl ExecutableAction {
    pub fn from_entity(entity: &XmlEntity) -> Result<Self> {
        let cmd_str =
            entity
                .required_attr_str("cmd")?
                .ok_or_else(|| ParseError::MissingAttribute {
                    element: "executable".to_string(),
                    attribute: "cmd".to_string(),
                })?;
        let cmd = parse_substitutions(&cmd_str)?;

        let cwd = entity
            .optional_attr_str("cwd")?
            .map(|s| parse_substitutions(&s))
            .transpose()?;

        let name = entity
            .optional_attr_str("name")?
            .map(|s| parse_substitutions(&s))
            .transpose()?;

        let shell: bool = entity.optional_attr("shell")?.unwrap_or(false);
        let output: Option<String> = entity.optional_attr("output")?;

        // Parse child elements (env and arg)
        let mut environment = Vec::new();
        let mut arguments = Vec::new();

        for child in entity.children() {
            // Child elements never reach `traverse_entity` — validate here.
            // `<arg>` under `<executable>` is a Rust-only extension (real
            // ROS 2 rejects the element as a child of `<executable>`
            // entirely) with its own narrow attribute set (`value` only —
            // see the `executable-arg` spec in `xml/attr_spec.rs`), distinct
            // from top-level `<arg>` declarations. Route it through that
            // dedicated key instead of the generic `"arg"` spec
            // `validate_attrs` would derive from `child.type_name()`.
            if child.type_name() == "arg" {
                let names: Vec<&str> = child.attributes().into_iter().map(|(k, _)| k).collect();
                crate::xml::attr_spec::validate_named("executable-arg", &names)?;
            } else {
                crate::xml::attr_spec::validate_attrs(&child)?;
            }
            match child.type_name() {
                "env" => {
                    let name: String = child.required_attr_str("name")?.ok_or_else(|| {
                        ParseError::MissingAttribute {
                            element: "env".to_string(),
                            attribute: "name".to_string(),
                        }
                    })?;
                    let value: String = child.required_attr_str("value")?.ok_or_else(|| {
                        ParseError::MissingAttribute {
                            element: "env".to_string(),
                            attribute: "value".to_string(),
                        }
                    })?;
                    environment.push((name, value));
                }
                "arg" => {
                    let value_str: String = child.required_attr_str("value")?.ok_or_else(|| {
                        ParseError::MissingAttribute {
                            element: "arg".to_string(),
                            attribute: "value".to_string(),
                        }
                    })?;
                    arguments.push(parse_substitutions(&value_str)?);
                }
                other => {
                    return Err(ParseError::UnexpectedElement {
                        parent: "executable".to_string(),
                        child: other.to_string(),
                    });
                }
            }
        }

        Ok(Self {
            cmd,
            cwd,
            name,
            shell,
            output,
            environment,
            arguments,
        })
    }

    /// Convert ExecutableAction to NodeCapture by resolving substitutions
    /// Executables are treated as nodes without a package
    pub fn to_capture(&self, context: &LaunchContext) -> Result<NodeCapture> {
        // Resolve cmd (executable)
        let executable = resolve_substitutions(&self.cmd, context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

        // Resolve optional name
        let name = self
            .name
            .as_ref()
            .map(|n| resolve_substitutions(n, context))
            .transpose()
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;

        // Resolve arguments
        let arguments: Vec<String> = self
            .arguments
            .iter()
            .map(|arg| {
                resolve_substitutions(arg, context)
                    .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))
            })
            .collect::<Result<Vec<_>>>()?;

        Ok(NodeCapture {
            package: String::new(), // Executables don't have packages
            executable,
            name,
            namespace: None, // Executables don't use namespaces
            parameters: Vec::new(),
            params_files: Vec::new(),
            param_sources: Vec::new(),
            remappings: Vec::new(),
            arguments,
            env_vars: self.environment.clone(),
            scope_id: None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xml::parse_xml_string;

    #[test]
    fn test_parse_simple_executable() {
        let xml = r#"<executable cmd="rosbag" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let exec = ExecutableAction::from_entity(&entity).unwrap();

        assert_eq!(exec.cmd, vec![Substitution::Text("rosbag".to_string())]);
        assert!(exec.cwd.is_none());
        assert!(!exec.shell);
    }

    #[test]
    fn test_parse_executable_with_args() {
        let xml = r#"<executable cmd="rosbag">
            <arg value="record" />
            <arg value="-a" />
        </executable>"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let exec = ExecutableAction::from_entity(&entity).unwrap();

        assert_eq!(exec.arguments.len(), 2);
    }

    #[test]
    fn test_parse_executable_with_env() {
        let xml = r#"<executable cmd="rviz2">
            <env name="DISPLAY" value=":0" />
        </executable>"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let exec = ExecutableAction::from_entity(&entity).unwrap();

        assert_eq!(exec.environment.len(), 1);
        assert_eq!(exec.environment[0].0, "DISPLAY");
        assert_eq!(exec.environment[0].1, ":0");
    }

    #[test]
    fn test_parse_executable_with_cwd() {
        let xml = r#"<executable cmd="ls" cwd="/tmp" shell="true" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let exec = ExecutableAction::from_entity(&entity).unwrap();

        assert!(exec.cwd.is_some());
        assert!(exec.shell);
    }
}
