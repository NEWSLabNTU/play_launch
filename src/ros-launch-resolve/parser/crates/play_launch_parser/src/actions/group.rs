//! Group action implementation

use crate::{
    error::{ParseError, Result},
    substitution::{
        LaunchContext, Substitution,
        context::{ConfigurationSnapshot, ScopeSnapshot},
        parse_substitutions, resolve_substitutions,
    },
    xml::{Entity, XmlEntity},
};

/// Group action for scoping namespaces and parameters
#[derive(Debug, Clone)]
pub struct GroupAction {
    pub namespace: Option<Vec<Substitution>>,
    /// Whether this group creates an isolated scope (default: true).
    /// When false, namespace/env changes leak to subsequent siblings.
    pub scoped: bool,
    /// `forwarding` (default true): whether a scoped group's body sees the
    /// configurations and environment from outside it. `false` resets both
    /// on entry, keeping only `keep`.
    pub forwarding: bool,
    /// `<keep name value>` children: configurations set on entry — the only
    /// ones that survive `forwarding="false"`.
    pub keep: Vec<(Vec<Substitution>, Vec<Substitution>)>,
}

/// What [`GroupAction::enter`] saved, for [`GroupAction::leave`].
pub type GroupScope = Option<(ScopeSnapshot, ConfigurationSnapshot)>;

/// A `data_type=bool` attribute, as `launch` coerces it.
pub fn bool_attribute(name: &str, value: &str) -> Result<bool> {
    crate::param_value::bool_attr(value).map_err(|_| ParseError::TypeCoercion {
        attribute: name.to_string(),
        value: value.to_string(),
        expected_type: "bool",
    })
}

impl GroupAction {
    pub fn from_entity(entity: &XmlEntity) -> Result<Self> {
        // `<group ns="…">` / `<group namespace="…">` — the launch-XML frontend
        // sugar for a leading `<push-ros-namespace>` inside the group (the IR
        // builder pushes `namespace` onto the stack for the group's body).
        // ROS 2's launch_xml, nano-ros's `nros-launch-parser` (RFC-0024), and
        // real Autoware/nav2 launch files all accept it, so honoring it keeps
        // the resolved model's node FQNs consistent across both runtimes
        // (previously this was dropped → `/alpha/talker` resolved as
        // `/talker`). `ns` takes precedence; `namespace` is the long spelling.
        let namespace = entity
            .optional_attr_str("ns")?
            .or(entity.optional_attr_str("namespace")?)
            .map(|s| parse_substitutions(&s))
            .transpose()?;

        let scoped = entity
            .optional_attr_str("scoped")?
            .map(|s| bool_attribute("scoped", &s))
            .transpose()?
            .unwrap_or(true);
        let forwarding = entity
            .optional_attr_str("forwarding")?
            .map(|s| bool_attribute("forwarding", &s))
            .transpose()?
            .unwrap_or(true);
        let keep = entity
            .children()
            .filter(|c| c.type_name() == "keep")
            .map(|c| {
                Ok((
                    parse_substitutions(&c.required_attr_str("name")?.unwrap_or_default())?,
                    parse_substitutions(&c.required_attr_str("value")?.unwrap_or_default())?,
                ))
            })
            .collect::<Result<Vec<_>>>()?;

        Ok(Self {
            namespace,
            scoped,
            forwarding,
            keep,
        })
    }

    /// `GroupAction.get_sub_entities`' prologue. Scoped: push the
    /// configurations and environment, then either set `keep` (forwarding)
    /// or reset the environment and every configuration but `keep`.
    /// Unscoped: just set `keep`.
    pub fn enter(&self, context: &mut LaunchContext) -> Result<GroupScope> {
        let scope = self
            .scoped
            .then(|| (context.save_scope(), context.push_launch_configurations()));
        if self.scoped && !self.forwarding {
            context.reset_environment();
            let keep = self.evaluate_keep(context)?;
            context.reset_launch_configurations(keep);
        } else {
            for (name, value) in &self.keep {
                let name = resolve(name, context)?;
                let value = resolve(value, context)?;
                context.set_configuration_literal(name, value);
            }
        }
        Ok(scope)
    }

    /// The epilogue: `PopEnvironment`, `PopLaunchConfigurations`.
    pub fn leave(context: &mut LaunchContext, scope: GroupScope) {
        if let Some((saved, configurations)) = scope {
            context.restore_scope(saved);
            context.pop_launch_configurations(configurations);
        }
    }

    /// `ResetLaunchConfigurations` performs every `keep` before clearing.
    fn evaluate_keep(&self, context: &LaunchContext) -> Result<Vec<(String, String)>> {
        self.keep
            .iter()
            .map(|(n, v)| Ok((resolve(n, context)?, resolve(v, context)?)))
            .collect()
    }
}

fn resolve(subs: &[Substitution], context: &LaunchContext) -> Result<String> {
    resolve_substitutions(subs, context).map_err(|e| ParseError::InvalidSubstitution(e.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xml::parse_xml_string;

    #[test]
    fn test_parse_group_ns_attr_honored() {
        // `<group ns="…">` is the launch-XML sugar for a leading
        // push-ros-namespace; the ns attribute must be captured (the IR
        // builder pushes it onto the namespace stack for the body).
        let xml = r#"<group ns="/my_namespace" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let group = GroupAction::from_entity(&entity).unwrap();

        assert!(
            group.namespace.is_some(),
            "ns attribute must be captured, not ignored"
        );
    }

    #[test]
    fn test_parse_group_namespace_long_attr_honored() {
        let xml = r#"<group namespace="robot1" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let group = GroupAction::from_entity(&entity).unwrap();
        assert!(
            group.namespace.is_some(),
            "namespace attribute must be captured"
        );
    }

    #[test]
    fn test_parse_group_scoped_false() {
        let xml = r#"<group scoped="false" />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let group = GroupAction::from_entity(&entity).unwrap();

        assert!(!group.scoped, "scoped=false should be parsed");
    }

    #[test]
    fn test_parse_group_scoped_default() {
        let xml = r#"<group />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let group = GroupAction::from_entity(&entity).unwrap();

        assert!(group.scoped, "default scoped should be true");
    }

    #[test]
    fn test_parse_group_without_namespace() {
        let xml = r#"<group />"#;
        parse_xml_string(xml).unwrap();
        let doc = roxmltree::Document::parse(xml).unwrap();
        let entity = crate::xml::XmlEntity::new(doc.root_element());
        let group = GroupAction::from_entity(&entity).unwrap();

        assert!(group.namespace.is_none());
    }
}
