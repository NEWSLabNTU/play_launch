use super::super::LaunchTraverser;
use crate::{
    actions::IncludeAction,
    error::{ParseError, Result},
    file_cache::read_file_cached,
    record::{canonicalize_path, extract_package_from_path},
    substitution::{resolve_substitutions, types::Substitution},
    xml,
};
use std::path::Path;

/// Validate that an include path is a launch file (by extension).
/// Prevents including arbitrary files like `/etc/passwd`.
pub(crate) fn validate_include_path(path: &Path, original: &str) -> Result<()> {
    let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("");
    match ext {
        "xml" | "py" | "yaml" | "yml" => Ok(()),
        _ => Err(ParseError::IoError(std::io::Error::new(
            std::io::ErrorKind::PermissionDenied,
            format!(
                "Include path '{}' has unexpected extension '.{}'. \
                 Only .xml, .py, .yaml, .yml launch files can be included.",
                original, ext
            ),
        ))),
    }
}

/// An include's arguments: as the XML/YAML frontends parse them (resolved
/// here, in order), or already resolved by the Python half.
#[derive(Clone, Copy)]
pub(crate) enum IncludeArgs<'a> {
    Subs(&'a [(String, Vec<Substitution>)]),
    Resolved(&'a [(String, String)]),
}

impl IncludeArgs<'_> {
    fn names(&self) -> Vec<String> {
        match self {
            Self::Subs(a) => a.iter().map(|(n, _)| n.clone()).collect(),
            Self::Resolved(a) => a.iter().map(|(n, _)| n.clone()).collect(),
        }
    }

    fn is_empty(&self) -> bool {
        match self {
            Self::Subs(a) => a.is_empty(),
            Self::Resolved(a) => a.is_empty(),
        }
    }
}

impl LaunchTraverser {
    pub(crate) fn process_include(&mut self, include: &IncludeAction) -> Result<()> {
        // `IncludeLaunchDescription.execute` resolves the file BEFORE it sets
        // any argument.
        let file_path_str = resolve_substitutions(&include.file, &self.context)
            .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
        self.process_include_target(&file_path_str, IncludeArgs::Subs(&include.args))
    }

    /// Set the include's arguments in the CURRENT context, in order — launch
    /// returns them as `SetLaunchConfiguration` actions ahead of the
    /// description, so a later one can read an earlier one, and they persist
    /// afterwards. Returns them resolved.
    fn set_include_args(&mut self, args: IncludeArgs) -> Result<Vec<(String, String)>> {
        let mut out = Vec::new();
        match args {
            IncludeArgs::Subs(args) => {
                for (key, value_subs) in args {
                    let value = resolve_substitutions(value_subs, &self.context)
                        .map_err(|e| ParseError::InvalidSubstitution(e.to_string()))?;
                    log::trace!("  Include arg: {} = {}", key, value);
                    self.context
                        .set_configuration_literal(key.clone(), value.clone());
                    out.push((key.clone(), value));
                }
            }
            IncludeArgs::Resolved(args) => {
                for (key, value) in args {
                    self.context
                        .set_configuration_literal(key.clone(), value.clone());
                    out.push((key.clone(), value.clone()));
                }
            }
        }
        Ok(out)
    }

    /// Include the launch file at `file_path_str` (already resolved), passing
    /// it `args`. The one path every frontend's include takes.
    pub(crate) fn process_include_target(
        &mut self,
        file_path_str: &str,
        args: IncludeArgs,
    ) -> Result<()> {
        let file_path = Path::new(file_path_str);

        log::trace!("Processing include: {}", file_path_str);

        // Validate the include path has a launch file extension
        validate_include_path(file_path, file_path_str)?;

        // Resolve relative paths relative to the current launch file
        let resolved_path = if file_path.is_relative() {
            if let Some(current_file) = self.context.current_file() {
                if let Some(parent_dir) = current_file.parent() {
                    parent_dir.join(file_path)
                } else {
                    file_path.to_path_buf()
                }
            } else {
                file_path.to_path_buf()
            }
        } else {
            file_path.to_path_buf()
        };

        // Canonicalize path for circular include detection
        let canonical_path = resolved_path
            .canonicalize()
            .unwrap_or_else(|_| resolved_path.clone());

        // Check for circular includes in the current include chain. The
        // default behavior is "warn and skip" to preserve compatibility with
        // every existing `play_launch` caller (Autoware in particular ships
        // launches that re-include `_common.launch.xml` from sibling
        // packages); strict callers (orchestration planners, lint passes) opt
        // in to a hard error via `ParseOptions.strict_includes` to surface a
        // diagnostic instead of silently dropping the branch.
        if self.include_chain.contains(&canonical_path) {
            if self.strict_includes {
                let chain = self
                    .include_chain
                    .iter()
                    .map(|p| p.display().to_string())
                    .collect::<Vec<_>>()
                    .join(" → ");
                return Err(ParseError::CircularInclude {
                    file: canonical_path.display().to_string(),
                    chain,
                });
            }
            log::warn!("Circular include detected: {}", canonical_path.display());
            return Ok(()); // Skip circular includes
        }

        // Check include depth limit
        if self.include_chain.len() >= self.max_include_depth {
            return Err(ParseError::MaxIncludeDepthExceeded {
                max: self.max_include_depth,
                file: canonical_path.display().to_string(),
            });
        }

        log::debug!("Including launch file: {}", resolved_path.display());
        if !args.is_empty() {
            log::trace!("Include args: {:?}", args.names());
        }

        let given = args.names();

        // A `.launch.py` target is executed by the Python frontend, in this
        // same context (an include scopes nothing).
        if resolved_path.extension().and_then(|s| s.to_str()) == Some("py") {
            log::debug!("Including Python launch file: {}", resolved_path.display());

            self.set_include_args(args)?;

            let py_file_name = resolved_path
                .file_name()
                .and_then(|s| s.to_str())
                .unwrap_or("unknown")
                .to_string();
            let child_scope_id = self.scope_table.push(
                extract_package_from_path(&resolved_path),
                py_file_name,
                canonicalize_path(&resolved_path),
                self.context.current_namespace(),
                self.context.configurations(),
                Some(self.current_scope_id),
            );
            let prev_scope_id = std::mem::replace(&mut self.current_scope_id, child_scope_id);
            let mark = self.delay_mark();
            self.include_chain.push(canonical_path);

            // Issue 0030: the file's own declarations are what the include has
            // to have passed; they come back with the execution, so the check
            // follows it.
            let result = self
                .execute_python_file(&resolved_path)
                .and_then(|declared| {
                    check_required_include_args(
                        &py_required_args(&declared),
                        &given,
                        &resolved_path,
                    )
                });

            self.include_chain.pop();
            // Records from a file the `.launch.py` included already carry
            // that include's scope; only the rest are this file's.
            self.stamp_scope_since(mark, child_scope_id);
            let final_args = self.context.configurations();
            self.scope_table.update_args(child_scope_id, final_args);
            self.current_scope_id = prev_scope_id;
            return result;
        }

        // XML and YAML targets share one path, because `launch` gives them one
        // semantics: `IncludeLaunchDescription.execute` returns
        // `[SetLaunchConfiguration(name, value) for each <arg>, description]`,
        // and the description's entities then run in the SAME context as the
        // include. Nothing is scoped by the include itself — only a scoped
        // `<group>` around it pushes and pops launch configurations. So:
        //
        // - the include's arguments are set in the current context, in order
        //   (a later one can read an earlier one);
        // - the included file's `<arg>` defaults apply only where nothing is
        //   set, so a passed value wins and an earlier sibling's value
        //   persists into a later sibling that does not pass one;
        // - whatever the included file declares or `<let>`s stays visible to
        //   the includer afterwards.
        let is_yaml = matches!(
            resolved_path.extension().and_then(|s| s.to_str()),
            Some("yaml" | "yml")
        );

        // Parsed up front: launch checks the include's required arguments
        // against the included description before anything runs.
        let xml_content = if is_yaml {
            None
        } else {
            Some(read_file_cached(&resolved_path)?)
        };
        let xml_doc = xml_content
            .as_deref()
            .map(roxmltree::Document::parse)
            .transpose()?;

        // What launch demands of an include (issues 0029/0030): every
        // argument the included file declares without a default, outside any
        // condition and outside any nested include, must be named among THIS
        // include's own arguments. The scope does not count, which is why
        // this is checked against the include's own arguments and not the
        // context.
        let required = match &xml_doc {
            Some(doc) => xml_required_args(&xml::XmlEntity::new(doc.root_element())),
            None => {
                log::debug!("Including YAML launch file: {}", resolved_path.display());
                yaml_required_args(&resolved_path)?
            }
        };
        check_required_include_args(&required, &given, &resolved_path)?;

        self.set_include_args(args)?;

        // Push a new scope for this include
        let include_file_name = resolved_path
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("unknown")
            .to_string();
        let child_scope_id = self.scope_table.push(
            extract_package_from_path(&resolved_path),
            include_file_name,
            canonicalize_path(&resolved_path),
            self.context.current_namespace(),
            self.context.configurations(),
            Some(self.current_scope_id),
        );
        let prev_scope_id = std::mem::replace(&mut self.current_scope_id, child_scope_id);
        let prev_file = self.context.current_file().cloned();
        let mark = self.delay_mark();
        self.include_chain.push(canonical_path);

        let result = match &xml_doc {
            Some(doc) => {
                self.context.set_current_file(resolved_path.clone());
                self.traverse_entity(&xml::XmlEntity::new(doc.root_element()))
            }
            None => self.process_yaml_launch_file(&resolved_path),
        };

        self.include_chain.pop();
        match prev_file {
            Some(prev) => self.context.set_current_file(prev),
            None => self.context.clear_current_file(),
        }
        // Everything the file produced that no nested include already
        // claimed belongs to this scope.
        self.stamp_scope_since(mark, child_scope_id);
        // The scope's args are every configuration in effect once the file
        // has run, including the defaults its `<arg>`s supplied.
        let final_args = self.context.configurations();
        self.scope_table.update_args(child_scope_id, final_args);
        self.current_scope_id = prev_scope_id;

        result
    }

    /// Stamp `scope_id` on every record and capture appended since `mark`
    /// that does not carry a scope yet. Innermost includes stamp first, so
    /// an entry already stamped belongs to a nested include and is kept.
    fn stamp_scope_since(&mut self, mark: super::delay::DelayMark, scope_id: usize) {
        for rec in &mut self.records[mark.records..] {
            rec.scope.get_or_insert(scope_id);
        }
        for rec in &mut self.containers[mark.containers..] {
            rec.scope.get_or_insert(scope_id);
        }
        for rec in &mut self.load_nodes[mark.load_nodes..] {
            rec.scope.get_or_insert(scope_id);
        }
        for cap in &mut self.context.captured_nodes_mut()[mark.captured_nodes..] {
            cap.scope_id.get_or_insert(scope_id);
        }
        for cap in &mut self.context.captured_containers_mut()[mark.captured_containers..] {
            cap.scope_id.get_or_insert(scope_id);
        }
        for cap in &mut self.context.captured_load_nodes_mut()[mark.captured_load_nodes..] {
            cap.scope_id.get_or_insert(scope_id);
        }
    }
}

/// `(name, description)` of every argument an included file declares without a
/// default and unconditionally — what `launch` calls a required argument of the
/// included description.
///
/// Walks the tree the way `launch`'s `get_launch_arguments` does: a
/// declaration under an element carrying `if=` or `unless=` is *conditionally
/// included* and not demanded at include time (it is checked when and if it
/// executes); a nested `<include>` is its own description, and its arguments
/// are that include's to satisfy.
pub(crate) fn xml_required_args(root: &xml::XmlEntity) -> Vec<(String, String)> {
    use crate::xml::Entity;
    let mut out = Vec::new();
    fn walk(entity: &xml::XmlEntity, out: &mut Vec<(String, String)>) {
        for child in entity.children() {
            let conditional = matches!(child.optional_attr_str("if"), Ok(Some(_)))
                || matches!(child.optional_attr_str("unless"), Ok(Some(_)));
            match child.type_name() {
                "arg" => {
                    if conditional {
                        continue;
                    }
                    let name = match child.optional_attr_str("name") {
                        Ok(Some(name)) => name,
                        _ => continue,
                    };
                    let has_default = matches!(child.optional_attr_str("default"), Ok(Some(_)));
                    if !has_default {
                        let description = child
                            .optional_attr_str("description")
                            .ok()
                            .flatten()
                            .unwrap_or_else(|| "no description given".to_string());
                        out.push((name, description));
                    }
                }
                "include" => {}
                _ => {
                    if !conditional {
                        walk(&child, out);
                    }
                }
            }
        }
    }
    walk(root, &mut out);
    out
}

/// The YAML frontend's equivalent of [`xml_required_args`]: `- arg:` entries
/// of the `launch:` list without a `default`, recursing into `group:` children
/// that carry no `if`/`unless`, and never into an `include:`.
pub(crate) fn yaml_required_args(path: &Path) -> Result<Vec<(String, String)>> {
    use serde_yaml_ng::Value;
    let content = read_file_cached(path)?;
    let yaml: Value = serde_yaml_ng::from_str(&content)
        .map_err(|e| ParseError::InvalidSubstitution(format!("Invalid YAML: {}", e)))?;
    let mut out = Vec::new();
    fn walk(items: &[Value], out: &mut Vec<(String, String)>) {
        for item in items {
            let Some(map) = item.as_mapping() else {
                continue;
            };
            let Some((key, body)) = map.iter().next() else {
                continue;
            };
            let Some(body) = body.as_mapping() else {
                continue;
            };
            let conditional = body.contains_key(Value::String("if".into()))
                || body.contains_key(Value::String("unless".into()));
            match key.as_str() {
                Some("arg") => {
                    if conditional || body.contains_key(Value::String("default".into())) {
                        continue;
                    }
                    let Some(name) = body
                        .get(Value::String("name".into()))
                        .and_then(Value::as_str)
                    else {
                        continue;
                    };
                    let description = body
                        .get(Value::String("description".into()))
                        .and_then(Value::as_str)
                        .unwrap_or("no description given");
                    out.push((name.to_string(), description.to_string()));
                }
                Some("group") => {
                    if !conditional
                        && let Some(children) = body
                            .get(Value::String("children".into()))
                            .and_then(Value::as_sequence)
                    {
                        walk(children, out);
                    }
                }
                _ => {}
            }
        }
    }
    if let Some(list) = yaml.get("launch").and_then(Value::as_sequence) {
        walk(list, &mut out);
    }
    Ok(out)
}

/// What an included `.launch.py` requires of its include (issue 0030): the
/// declarations it constructed without a default, outside any
/// `OpaqueFunction` — the ones launch's `get_launch_arguments` can see.
pub(crate) fn py_required_args(
    declared: &[crate::captures::DeclaredArgumentCapture],
) -> Vec<(String, String)> {
    declared
        .iter()
        .filter(|d| !d.has_default && !d.opaque)
        .map(|d| {
            (
                d.name.clone(),
                d.description
                    .clone()
                    .unwrap_or_else(|| "no description given".to_string()),
            )
        })
        .collect()
}

/// Refuse the include if any required argument is not among its own `<arg>`s.
/// `given` is the include's own argument names, in order, and nothing else.
pub(crate) fn check_required_include_args(
    required: &[(String, String)],
    given: &[String],
    file: &Path,
) -> Result<()> {
    for (name, description) in required {
        if !given.iter().any(|given_name| given_name == name) {
            return Err(ParseError::MissingIncludeArgument {
                name: name.clone(),
                description: description.clone(),
                given: given.join(", "),
                file: file.display().to_string(),
            });
        }
    }
    Ok(())
}
