//! `export` — write part of a launch tree as a launch file (phase 106 W5).
//!
//! Three forms (`docs/design/launch-extraction.md`):
//! - **command**: one node's command line and its parameter files;
//! - **self-contained**: one flat `.launch.xml`, an element per member, with
//!   a parameter file per parameter source, nothing evaluated;
//! - **include**: `<include>`s of the original files wherever an include's
//!   whole subtree is selected, flat elements for the rest, so the result
//!   follows later edits inside the included files.
//!
//! Every file form is verified the way the parsers are: the written file is
//! resolved again, and its members must match the selection's. An
//! `<include>` whose members come out different is opened (written flat
//! one level down) and the export runs again, so the file never starts
//! something other than what was picked; at worst it follows fewer edits.

use std::{
    collections::{BTreeMap, BTreeSet, HashSet},
    fmt::Write as _,
    path::{Path, PathBuf},
};

use eyre::{Context, Result, bail};
use ros_launch_manifest_model::{NodeInstance, OnExit, ParamSource, ParamValue, SystemModel};

use crate::{
    model::{ModelBuildInputs, build_checked_model},
    ros::launch_dump::LaunchDump,
    selection::{self, SelectionSpec, TreeIndex, shell_quote},
    verbs::ParserBackend,
};

/// Which file to write.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Form {
    Command,
    SelfContained,
    Include,
}

impl Form {
    pub fn as_str(self) -> &'static str {
        match self {
            Form::Command => "command",
            Form::SelfContained => "self-contained",
            Form::Include => "include",
        }
    }
}

/// Everything `export` reads.
pub struct ExportInputs {
    /// Package name or launch file path (absent with `model`).
    pub package_or_path: Option<String>,
    pub launch_file: Option<String>,
    pub launch_arguments: Vec<String>,
    pub parser: ParserBackend,
    pub contracts: Option<PathBuf>,
    pub no_provider_contracts: bool,
    pub sched: Option<PathBuf>,
    pub target: String,
    pub process_per_composable: bool,
    /// Export from a saved SystemModel instead of a launch file.
    pub model: Option<PathBuf>,
    pub selection: SelectionSpec,
    pub form: Form,
    pub out_dir: PathBuf,
    pub name: Option<String>,
    /// The command that made this export, recorded in the file's header so
    /// `--again` can run it again. Empty: no header command.
    pub source_command: Vec<String>,
}

/// What `export` wrote and found.
#[derive(Debug, Default)]
pub struct ExportReport {
    /// The launch file (file forms) or the command file (command form).
    pub file: PathBuf,
    /// Ready commands to run the result.
    pub run: Vec<String>,
    /// Fields the launch format could not say, per member.
    pub lost: Vec<String>,
    /// Includes the export had to open, each with the first difference.
    pub opened: Vec<String>,
    /// Differences verification found that opening could not remove.
    pub differences: Vec<String>,
}

/// Resolve the source, evaluate the selection, write the export.
pub async fn export(inputs: ExportInputs) -> Result<ExportReport> {
    let (model, dump) = match &inputs.model {
        Some(path) => {
            if inputs.form == Form::Include {
                bail!(
                    "the include form needs the launch files, and a saved model has none of \
                     their structure; export from the launch command, or use --form \
                     self-contained"
                );
            }
            let text = std::fs::read_to_string(path)
                .wrap_err_with(|| format!("reading {}", path.display()))?;
            (SystemModel::from_yaml_str(&text)?, None)
        }
        None => {
            let pkg = inputs
                .package_or_path
                .as_deref()
                .ok_or_else(|| eyre::eyre!("export needs a launch file or --model"))?;
            let (launch_file, launch_arguments) = super::reclassify_launch_file_arg(
                inputs.launch_file.as_deref(),
                &inputs.launch_arguments,
            );
            let (dump, launch_path) = super::parse_to_launch_dump(
                pkg,
                launch_file.as_deref(),
                &launch_arguments,
                inputs.parser,
            )
            .await?;
            let model = build_checked_model(ModelBuildInputs {
                dump: &dump,
                launch_path: Some(&launch_path),
                bringup_root: None,
                arg_binding: super::parse_launch_arguments(&launch_arguments)
                    .into_iter()
                    .collect(),
                contracts: inputs.contracts.as_deref(),
                no_provider_contracts: inputs.no_provider_contracts,
                sched: inputs.sched.as_deref(),
                system: None,
                target: &inputs.target,
                explain: false,
                process_per_composable: inputs.process_per_composable,
            })?;
            (model, Some(dump))
        }
    };
    let sel = selection::evaluate(&model, &inputs.selection)?;
    if sel.kept.is_empty() {
        bail!("the selection keeps nothing; there is nothing to export");
    }
    std::fs::create_dir_all(&inputs.out_dir)
        .wrap_err_with(|| format!("creating {}", inputs.out_dir.display()))?;
    let name = inputs
        .name
        .clone()
        .unwrap_or_else(|| default_name(&inputs.selection));
    let kept: Vec<String> = sel.kept.clone();

    if inputs.form == Form::Command {
        return export_command(&model, &kept, &inputs.out_dir, &name);
    }

    let file = inputs.out_dir.join(format!("{name}.launch.xml"));
    let params_dir = inputs.out_dir.join("params");
    let top_args = super::parse_launch_arguments(&inputs.launch_arguments)
        .into_iter()
        .collect::<BTreeMap<_, _>>();
    let header = header(&inputs, &kept);

    let mut report = ExportReport {
        file: file.clone(),
        ..Default::default()
    };
    let mut opened: BTreeSet<String> = BTreeSet::new();
    for _ in 0..64 {
        if params_dir.exists() {
            std::fs::remove_dir_all(&params_dir)?;
        }
        let mut w = Writer {
            model: &model,
            out_dir: &inputs.out_dir,
            params_dir: &params_dir,
            lost: Vec::new(),
            param_files: 0,
            includes: BTreeMap::new(),
        };
        let body = match (inputs.form, dump.as_ref()) {
            (Form::Include, Some(dump)) => w.include_form(dump, &kept, &opened, &top_args)?,
            _ => w.flat(&kept, &kept)?,
        };
        let mut xml = String::new();
        xml.push_str("<?xml version=\"1.0\"?>\n");
        xml.push_str(&header);
        xml.push_str("<launch>\n");
        if inputs.form == Form::Include {
            for (k, v) in &top_args {
                let _ = writeln!(xml, "  <arg name=\"{}\" default=\"{}\"/>", esc(k), esc(v));
            }
        }
        xml.push_str(&body);
        xml.push_str("</launch>\n");
        std::fs::write(&file, &xml).wrap_err_with(|| format!("writing {}", file.display()))?;

        let differences = verify(&file, &model, &kept, inputs.parser).await?;
        if differences.is_empty() || inputs.form != Form::Include {
            report.lost = w.lost;
            report.differences = differences.into_iter().map(|(_, d)| d).collect();
            break;
        }
        // Open the outermost include holding a member that came out wrong.
        let mut opened_one = false;
        for (key, diff) in &differences {
            if let Some(scope) = w
                .includes
                .iter()
                .find(|(_, members)| members.contains(key))
                .map(|(s, _)| s.clone())
            {
                if opened.insert(scope.clone()) {
                    report.opened.push(format!("{scope}: {diff}"));
                    opened_one = true;
                    break;
                }
            }
        }
        if !opened_one {
            report.lost = w.lost;
            report.differences = differences.into_iter().map(|(_, d)| d).collect();
            break;
        }
    }

    let shown = shell_quote(&file.display().to_string());
    report.run = vec![
        format!("play_launch launch {shown}"),
        format!("ros2 launch {shown}"),
    ];
    Ok(report)
}

/// `selection` from the first `--only`, else `subset`.
fn default_name(spec: &SelectionSpec) -> String {
    let base = spec
        .only
        .first()
        .map(|s| {
            let p = s.pattern.rsplit('/').next().unwrap_or(&s.pattern);
            p.split('.').next().unwrap_or(p).to_string()
        })
        .filter(|s| {
            !s.is_empty()
                && s.chars()
                    .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
        })
        .unwrap_or_else(|| "subset".to_string());
    format!("{base}_export")
}

fn header(inputs: &ExportInputs, kept: &[String]) -> String {
    let mut h = format!(
        "<!-- Exported by play_launch {} (form: {}): {} member(s).\n",
        crate::producer::get().1,
        inputs.form.as_str(),
        kept.len()
    );
    if !inputs.source_command.is_empty() {
        let cmd: Vec<String> = inputs
            .source_command
            .iter()
            .map(|w| shell_quote(w))
            .collect();
        let _ = writeln!(h, "     Source: {}", esc_comment(&cmd.join(" ")));
        // A comment cannot hold a double hyphen, so the flag is named in words.
        h.push_str(
            "     To regenerate it after the original launch files change, give this file\n",
        );
        h.push_str("     to `play_launch export` as the value of its `again` option.\n");
    }
    if inputs.form == Form::SelfContained {
        h.push_str("     A snapshot: later edits to the original launch files do not reach it.\n");
    }
    h.push_str("-->\n");
    h
}

/// XML attribute escaping.
fn esc(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('"', "&quot;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

fn esc_comment(s: &str) -> String {
    s.replace("--", "- -")
}

fn namespace_of(key: &str) -> &str {
    match key.rfind('/') {
        Some(0) | None => "/",
        Some(i) => &key[..i],
    }
}

/// A YAML value for one parameter.
fn yaml_value(v: &ParamValue) -> serde_yaml_ng::Value {
    use serde_yaml_ng::Value;
    match v {
        ParamValue::Bool(b) => Value::Bool(*b),
        ParamValue::Int(i) => Value::Number((*i).into()),
        ParamValue::Float(f) => Value::Number((*f).into()),
        ParamValue::Str(s) => Value::String(s.clone()),
        ParamValue::StrList(items) => Value::Sequence(
            items
                .iter()
                .map(|e| serde_yaml_ng::from_str(e).unwrap_or_else(|_| Value::String(e.clone())))
                .collect(),
        ),
    }
}

fn inline_yaml(params: &[(String, ParamValue)]) -> Result<String> {
    use serde_yaml_ng::{Mapping, Value};
    let mut p = Mapping::new();
    for (k, v) in params {
        p.insert(Value::String(k.clone()), yaml_value(v));
    }
    let mut rp = Mapping::new();
    rp.insert(Value::String("ros__parameters".into()), Value::Mapping(p));
    let mut root = Mapping::new();
    root.insert(Value::String("/**".into()), Value::Mapping(rp));
    Ok(serde_yaml_ng::to_string(&Value::Mapping(root))?)
}

struct Writer<'a> {
    model: &'a SystemModel,
    out_dir: &'a Path,
    params_dir: &'a Path,
    lost: Vec<String>,
    param_files: usize,
    /// Model scope key -> the members an emitted `<include>` starts.
    includes: BTreeMap<String, Vec<String>>,
}

impl Writer<'_> {
    /// Write one parameter file; returns its `$(dirname)` path.
    fn param_file(&mut self, key: &str, content: &str) -> Result<String> {
        std::fs::create_dir_all(self.params_dir)?;
        self.param_files += 1;
        let stem: String = key
            .trim_start_matches('/')
            .chars()
            .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
            .collect();
        let name = format!("{:03}_{stem}.yaml", self.param_files);
        std::fs::write(self.params_dir.join(&name), content)?;
        let rel = self
            .params_dir
            .strip_prefix(self.out_dir)
            .unwrap_or(self.params_dir)
            .join(name);
        Ok(format!("$(dirname)/{}", rel.display()))
    }

    /// `<param from=…/>` lines for a member, in the order ROS applies them.
    fn params(&mut self, key: &str, inst: &NodeInstance, pad: &str) -> Result<String> {
        let mut out = String::new();
        let mut inline: Vec<(String, ParamValue)> = Vec::new();
        let flush = |w: &mut Self,
                     inline: &mut Vec<(String, ParamValue)>,
                     out: &mut String|
         -> Result<()> {
            if !inline.is_empty() {
                let path = w.param_file(key, &inline_yaml(inline)?)?;
                let _ = writeln!(out, "{pad}<param from=\"{}\"/>", esc(&path));
                inline.clear();
            }
            Ok(())
        };
        if inst.param_sources.is_empty() {
            for c in &inst.params_files {
                let path = self.param_file(key, c)?;
                let _ = writeln!(out, "{pad}<param from=\"{}\"/>", esc(&path));
            }
            inline.extend(inst.params.iter().map(|(k, v)| (k.clone(), v.clone())));
        } else {
            for s in &inst.param_sources {
                match s {
                    ParamSource::Inline { name, value } => {
                        inline.push((name.clone(), value.clone()))
                    }
                    ParamSource::File { content } => {
                        flush(self, &mut inline, &mut out)?;
                        let path = self.param_file(key, content)?;
                        let _ = writeln!(out, "{pad}<param from=\"{}\"/>", esc(&path));
                    }
                }
            }
        }
        flush(self, &mut inline, &mut out)?;
        Ok(out)
    }

    /// Fields XML cannot say, noted as a comment on the member and in the
    /// report (design 3.1).
    fn lost_fields(&mut self, key: &str, inst: &NodeInstance, pad: &str) -> String {
        let mut lost = Vec::new();
        if inst.sigterm_timeout.is_some() || inst.sigkill_timeout.is_some() {
            lost.push(format!(
                "sigterm_timeout={:?} sigkill_timeout={:?}",
                inst.sigterm_timeout, inst.sigkill_timeout
            ));
        }
        if let Some(codes) = &inst.shutdown_on_exit_codes {
            lost.push(format!("ends the launch on exit codes {codes:?}"));
        }
        if !inst.env_unset.is_empty() {
            lost.push(format!("unsets {}", inst.env_unset.join(", ")));
        }
        if let Some(a) = &inst.lifecycle_autostart {
            lost.push(format!("lifecycle autostart {a:?}"));
        }
        if lost.is_empty() {
            return String::new();
        }
        for l in &lost {
            self.lost.push(format!("{key}: {l}"));
        }
        format!(
            "{pad}<!-- not expressible in XML: {} -->\n",
            esc_comment(&lost.join("; "))
        )
    }

    fn remaps_env(inst: &NodeInstance, pad: &str) -> String {
        let mut out = String::new();
        for r in &inst.remaps {
            let _ = writeln!(
                out,
                "{pad}<remap from=\"{}\" to=\"{}\"/>",
                esc(&r.from),
                esc(&r.to)
            );
        }
        for e in &inst.env {
            let _ = writeln!(
                out,
                "{pad}<env name=\"{}\" value=\"{}\"/>",
                esc(&e.name),
                esc(&e.value)
            );
        }
        out
    }

    /// The process attributes shared by `<node>` and `<node_container>`.
    fn process_attrs(key: &str, inst: &NodeInstance) -> String {
        let mut a = String::new();
        let _ = write!(
            a,
            " pkg=\"{}\" exec=\"{}\"",
            esc(inst.pkg.as_deref().unwrap_or_default()),
            esc(inst.exec.as_deref().unwrap_or_default())
        );
        if let Some(n) = &inst.node_name {
            let _ = write!(a, " name=\"{}\"", esc(n.rsplit('/').next().unwrap_or(n)));
        }
        let _ = write!(a, " namespace=\"{}\"", esc(namespace_of(key)));
        let mut args: Vec<String> = inst.args.iter().map(|w| shell_quote(w)).collect();
        if !inst.ros_args.is_empty() {
            args.push("--ros-args".into());
            args.extend(inst.ros_args.iter().map(|w| shell_quote(w)));
        }
        if !args.is_empty() {
            let _ = write!(a, " args=\"{}\"", esc(&args.join(" ")));
        }
        if let Some(p) = &inst.launch_prefix {
            let _ = write!(a, " launch-prefix=\"{}\"", esc(p));
        }
        if let Some(c) = &inst.cwd {
            let _ = write!(a, " cwd=\"{}\"", esc(c));
        }
        if inst.respawn == Some(true) {
            a.push_str(" respawn=\"true\"");
            if let Some(d) = inst.respawn_delay {
                let _ = write!(a, " respawn_delay=\"{d}\"");
            }
        }
        if inst.on_exit == Some(OnExit::Shutdown) {
            a.push_str(" on_exit=\"shutdown\"");
        }
        a.push_str(" output=\"screen\"");
        a
    }

    /// One member as flat XML. Composables are written by their container,
    /// or as a `<load_composable_node>` when the container is not written.
    fn member(&mut self, key: &str, written: &HashSet<&str>, pad: &str) -> Result<String> {
        let inst = &self.model.structure.nodes[key];
        let mut out = String::new();
        let inner = format!("{pad}  ");
        if inst.plugin.is_some() {
            let container = inst.container.as_deref().unwrap_or_default();
            if written.contains(container) {
                return Ok(out);
            }
            let _ = writeln!(
                out,
                "{pad}<load_composable_node target=\"{}\">",
                esc(container)
            );
            out.push_str(&self.composable(key, &inner)?);
            let _ = writeln!(out, "{pad}</load_composable_node>");
            return Ok(self.timer(inst, out, pad));
        }
        out.push_str(&self.lost_fields(key, inst, pad));
        if inst.pkg.is_none() && !inst.raw_cmd.is_empty() {
            let _ = write!(
                out,
                "{pad}<executable cmd=\"{}\"",
                esc(&inst.raw_cmd.join(" "))
            );
            if let Some(c) = &inst.cwd {
                let _ = write!(out, " cwd=\"{}\"", esc(c));
            }
            if inst.respawn == Some(true) {
                out.push_str(" respawn=\"true\"");
            }
            out.push_str(" output=\"screen\">\n");
            out.push_str(&Self::remaps_env(inst, &inner));
            let _ = writeln!(out, "{pad}</executable>");
            return Ok(self.timer(inst, out, pad));
        }
        let tag = if inst.is_container {
            "node_container"
        } else {
            "node"
        };
        let _ = writeln!(out, "{pad}<{tag}{}>", Self::process_attrs(key, inst));
        out.push_str(&self.params(key, inst, &inner)?);
        out.push_str(&Self::remaps_env(inst, &inner));
        if inst.is_container {
            let composables: Vec<String> = self
                .model
                .structure
                .nodes
                .iter()
                .filter(|(k, n)| {
                    n.container.as_deref() == Some(key) && written.contains(k.as_str())
                })
                .map(|(k, _)| k.clone())
                .collect();
            for c in composables {
                out.push_str(&self.composable(&c, &inner)?);
            }
        }
        let _ = writeln!(out, "{pad}</{tag}>");
        Ok(self.timer(inst, out, pad))
    }

    fn composable(&mut self, key: &str, pad: &str) -> Result<String> {
        let inst = &self.model.structure.nodes[key];
        let mut out = String::new();
        let inner = format!("{pad}  ");
        let name = inst
            .node_name
            .clone()
            .unwrap_or_else(|| key.rsplit('/').next().unwrap_or(key).to_string());
        let _ = writeln!(
            out,
            "{pad}<composable_node pkg=\"{}\" plugin=\"{}\" name=\"{}\" namespace=\"{}\">",
            esc(inst.pkg.as_deref().unwrap_or_default()),
            esc(inst.plugin.as_deref().unwrap_or_default()),
            esc(name.rsplit('/').next().unwrap_or(&name)),
            esc(namespace_of(key))
        );
        out.push_str(&self.params(key, inst, &inner)?);
        for r in &inst.remaps {
            let _ = writeln!(
                out,
                "{inner}<remap from=\"{}\" to=\"{}\"/>",
                esc(&r.from),
                esc(&r.to)
            );
        }
        for (k, v) in &inst.extra_args {
            let _ = writeln!(
                out,
                "{inner}<extra_arg name=\"{}\" value=\"{}\"/>",
                esc(k),
                esc(v)
            );
        }
        let _ = writeln!(out, "{pad}</composable_node>");
        Ok(out)
    }

    fn timer(&self, inst: &NodeInstance, body: String, pad: &str) -> String {
        match inst.start_delay_secs {
            Some(d) if d > 0.0 => {
                let indented: String = body.lines().map(|l| format!("  {l}\n")).collect();
                format!("{pad}<timer period=\"{d}\">\n{indented}{pad}</timer>\n")
            }
            _ => body,
        }
    }

    /// Every member of `keys` flat; `written` are all the members the file
    /// writes (a composable nests under its container when that is one).
    fn flat(&mut self, keys: &[String], written: &[String]) -> Result<String> {
        let written: HashSet<&str> = written.iter().map(|s| s.as_str()).collect();
        let mut out = String::new();
        for k in keys {
            out.push_str(&self.member(k, &written, "  ")?);
        }
        Ok(out)
    }

    /// `<include>` wherever a scope's whole subtree is kept, flat members
    /// elsewhere (design 4.1).
    fn include_form(
        &mut self,
        dump: &LaunchDump,
        kept: &[String],
        opened: &BTreeSet<String>,
        top_args: &BTreeMap<String, String>,
    ) -> Result<String> {
        let tree = TreeIndex::new(self.model);
        let kept_set: HashSet<&str> = kept.iter().map(|s| s.as_str()).collect();
        let dump_of = crate::ros::model_builder::scope_keys_by_key(dump);
        let mut flat_keys: Vec<String> = Vec::new();
        let mut includes: Vec<String> = Vec::new();
        fn walk(
            tree: &TreeIndex<'_>,
            key: &str,
            kept: &HashSet<&str>,
            opened: &BTreeSet<String>,
            flat: &mut Vec<String>,
            includes: &mut Vec<String>,
        ) {
            let all = tree.subtree_nodes(key);
            let n = all.iter().filter(|k| kept.contains(k.as_str())).count();
            if n == 0 {
                return;
            }
            if n == all.len() && !opened.contains(key) && tree.parent_of(key).is_some() {
                includes.push(key.to_string());
                return;
            }
            for k in tree.direct.get(key).into_iter().flatten() {
                if kept.contains(k.as_str()) {
                    flat.push(k.clone());
                }
            }
            for c in tree.children.get(key).into_iter().flatten() {
                walk(tree, c, kept, opened, flat, includes);
            }
        }
        for r in &tree.roots {
            walk(&tree, r, &kept_set, opened, &mut flat_keys, &mut includes);
        }
        // Members whose scope is not in the table go flat.
        let covered: HashSet<String> = includes
            .iter()
            .flat_map(|s| tree.subtree_nodes(s))
            .chain(flat_keys.iter().cloned())
            .collect();
        for k in kept.iter().filter(|k| !covered.contains(*k)) {
            flat_keys.push(k.clone());
        }

        let mut out = String::new();
        // A flat composable nests under its container only when that is
        // written flat too.
        let flat_written: Vec<String> = flat_keys.clone();
        out.push_str(&self.flat(&flat_keys, &flat_written)?);
        for scope in &includes {
            let Some(entry) = dump_of
                .get(scope)
                .and_then(|id| dump.scopes.iter().find(|s| s.id == *id))
            else {
                // No dump entry: write its members flat.
                let members = tree.subtree_nodes(scope);
                out.push_str(&self.flat(&members, &members)?);
                continue;
            };
            let origin = entry.origin.as_ref();
            let pkg = origin.and_then(|o| o.pkg.clone());
            let path = origin.and_then(|o| o.path.clone());
            let file_attr = match (&pkg, &path) {
                (Some(pkg), Some(p)) => {
                    let marker = format!("/share/{pkg}/");
                    match p.find(&marker) {
                        Some(i) => format!("$(find-pkg-share {pkg})/{}", &p[i + marker.len()..]),
                        None => p.clone(),
                    }
                }
                (_, Some(path)) => path.clone(),
                _ => tree.file_of(scope),
            };
            let ns = entry.ns.trim();
            out.push_str("  <group>\n");
            if !ns.is_empty() && ns != "/" {
                let _ = writeln!(out, "    <push-ros-namespace namespace=\"{}\"/>", esc(ns));
            }
            // What the include saw from before it: an included file sees
            // the global parameters, remaps and environment, not only its
            // arguments. The group scopes them to this include.
            if let Some(ctx) = &entry.entry_context {
                for (name, value) in &ctx.global_params {
                    let _ = writeln!(
                        out,
                        "    <set_parameter name=\"{}\" value=\"{}\"/>",
                        esc(name),
                        esc(value)
                    );
                }
                for (from, to) in &ctx.remaps {
                    let _ = writeln!(
                        out,
                        "    <set_remap from=\"{}\" to=\"{}\"/>",
                        esc(from),
                        esc(to)
                    );
                }
                for (name, value) in &ctx.env {
                    match value {
                        Some(v) => {
                            let _ = writeln!(
                                out,
                                "    <set_env name=\"{}\" value=\"{}\"/>",
                                esc(name),
                                esc(v)
                            );
                        }
                        None => {
                            let _ = writeln!(out, "    <unset_env name=\"{}\"/>", esc(name));
                        }
                    }
                }
            }
            let _ = writeln!(out, "    <include file=\"{}\">", esc(&file_attr));
            let mut args: Vec<(&String, &String)> = entry.args.iter().collect();
            args.sort();
            for (k, v) in args {
                let value = if top_args.get(k) == Some(v) {
                    format!("$(var {k})")
                } else {
                    esc(v)
                };
                let _ = writeln!(out, "      <arg name=\"{}\" value=\"{}\"/>", esc(k), value);
            }
            out.push_str("    </include>\n  </group>\n");
            self.includes
                .insert(scope.clone(), tree.subtree_nodes(scope));
        }
        Ok(out)
    }
}

/// The parts of a member that must survive an export, comparable across
/// the original and the re-resolved model.
fn fingerprint(key: &str, n: &NodeInstance) -> String {
    let mut args = n.args.clone();
    if !n.ros_args.is_empty() {
        args.push("--ros-args".into());
        args.extend(n.ros_args.iter().cloned());
    }
    let params: BTreeMap<String, ParamValue> = n.resolved_params(key);
    format!(
        "pkg={:?} exec={:?} plugin={:?} container={:?} args={:?} params={:?} remaps={:?} env={:?} \
         prefix={:?} cwd={:?} respawn={:?} cmd={:?} extra={:?} delay={:?}",
        n.pkg,
        n.exec,
        n.plugin,
        n.container,
        args,
        params,
        n.remaps
            .iter()
            .map(|r| (&r.from, &r.to))
            .collect::<Vec<_>>(),
        n.env
            .iter()
            .map(|e| (&e.name, &e.value))
            .collect::<BTreeMap<_, _>>(),
        n.launch_prefix,
        n.cwd,
        n.respawn.unwrap_or(false),
        n.raw_cmd,
        n.extra_args,
        n.start_delay_secs,
    )
}

/// Resolve the written file and compare its members with the selection's:
/// `(member, difference)` for each that differs, is missing or is extra.
async fn verify(
    file: &Path,
    model: &SystemModel,
    kept: &[String],
    parser: ParserBackend,
) -> Result<Vec<(String, String)>> {
    let path = file.display().to_string();
    let (dump, launch_path) = super::parse_to_launch_dump(&path, None, &[], parser)
        .await
        .wrap_err_with(|| format!("resolving the exported {}", file.display()))?;
    let again = build_checked_model(ModelBuildInputs {
        dump: &dump,
        launch_path: Some(&launch_path),
        bringup_root: None,
        arg_binding: BTreeMap::new(),
        contracts: None,
        no_provider_contracts: true,
        sched: None,
        system: None,
        target: "posix",
        explain: false,
        process_per_composable: false,
    })?;
    let mut out = Vec::new();
    let kept_set: HashSet<&str> = kept.iter().map(|s| s.as_str()).collect();
    for k in kept {
        let want = fingerprint(k, &model.structure.nodes[k]);
        match again.structure.nodes.get(k) {
            None => out.push((
                k.clone(),
                format!("{k} is not started by the exported file"),
            )),
            Some(n) => {
                let got = fingerprint(k, n);
                if got != want {
                    out.push((
                        k.clone(),
                        format!("{k} differs: expected {want}, got {got}"),
                    ));
                }
            }
        }
    }
    for k in again
        .structure
        .nodes
        .keys()
        .filter(|k| !kept_set.contains(k.as_str()))
    {
        out.push((
            k.clone(),
            format!("{k} is started by the exported file but was not selected"),
        ));
    }
    // An extra member belongs to whichever include started it; attribute it
    // to the include that holds a kept member of the same namespace, else
    // leave it for the report.
    Ok(out)
}

/// The command form: one node's command line and its parameter files, or
/// for one composable, its container's command and a `ros2 component load`.
fn export_command(
    model: &SystemModel,
    kept: &[String],
    out_dir: &Path,
    name: &str,
) -> Result<ExportReport> {
    let refuse = || {
        eyre::eyre!(
            "the command form writes one node's command; the selection keeps {} member(s) \
             ({}), select exactly one plain node, container or composable",
            kept.len(),
            kept.iter().take(5).cloned().collect::<Vec<_>>().join(", ")
        )
    };
    let composables: Vec<&String> = kept
        .iter()
        .filter(|k| model.structure.nodes[*k].plugin.is_some())
        .collect();
    if let [comp] = composables.as_slice() {
        let container = model.structure.nodes[*comp].container.clone();
        let others: Vec<&String> = kept.iter().filter(|k| k != comp).collect();
        let with_container = match others.as_slice() {
            [] => false,
            [c] if Some(c.as_str()) == container.as_deref() => true,
            _ => return Err(refuse()),
        };
        return export_composable_command(model, comp, with_container, out_dir, name);
    }
    if !composables.is_empty() {
        return Err(refuse());
    }
    let key = match kept {
        [one] => one,
        _ => return Err(refuse()),
    };
    let (prelude, cmd) = run_line(model, key, out_dir)?;
    let file = out_dir.join(format!("{name}.sh"));
    let mut text = String::from("#!/bin/sh\n# Exported by play_launch: one node's command line.\n");
    text.push_str(&prelude);
    text.push_str(&cmd);
    text.push('\n');
    write_script(&file, &text)?;
    Ok(ExportReport {
        file,
        run: vec![cmd],
        ..Default::default()
    })
}

/// A plain node's or container's command line: shell lines to run before it
/// (`cd`, `unset`) and the command itself.
fn run_line(model: &SystemModel, key: &str, out_dir: &Path) -> Result<(String, String)> {
    let inst = &model.structure.nodes[key];
    if inst.pkg.is_none() {
        return Ok((String::new(), inst.raw_cmd.join(" ")));
    }
    let params_dir = out_dir.join("params");
    let mut w = Writer {
        model,
        out_dir,
        params_dir: &params_dir,
        lost: Vec::new(),
        param_files: 0,
        includes: BTreeMap::new(),
    };
    let xml = w.params(key, inst, "")?;
    let files: Vec<String> = xml
        .lines()
        .filter_map(|l| l.split('"').nth(1))
        .map(|p| {
            out_dir
                .join(p.trim_start_matches("$(dirname)/"))
                .display()
                .to_string()
        })
        .collect();
    let mut words: Vec<String> = Vec::new();
    for e in &inst.env {
        words.push(format!("{}={}", e.name, e.value));
    }
    if let Some(p) = &inst.launch_prefix {
        words.extend(p.split_whitespace().map(str::to_string));
    }
    words.extend([
        "ros2".to_string(),
        "run".to_string(),
        inst.pkg.clone().unwrap_or_default(),
        inst.exec.clone().unwrap_or_default(),
    ]);
    words.extend(inst.args.iter().cloned());
    words.push("--ros-args".into());
    if let Some(n) = &inst.node_name {
        words.push("-r".into());
        words.push(format!("__node:={}", n.rsplit('/').next().unwrap_or(n)));
    }
    let ns = namespace_of(key);
    if ns != "/" {
        words.push("-r".into());
        words.push(format!("__ns:={ns}"));
    }
    for f in &files {
        words.push("--params-file".into());
        words.push(f.clone());
    }
    for r in &inst.remaps {
        words.push("-r".into());
        words.push(format!("{}:={}", r.from, r.to));
    }
    words.extend(inst.ros_args.iter().cloned());
    let cmd: Vec<String> = words.iter().map(|w| shell_quote(w)).collect();
    let mut prelude = String::new();
    if let Some(d) = &inst.cwd {
        let _ = writeln!(prelude, "cd {}", shell_quote(d));
    }
    if !inst.env_unset.is_empty() {
        let _ = writeln!(prelude, "unset {}", inst.env_unset.join(" "));
    }
    Ok((prelude, cmd.join(" ")))
}

/// Write an exported shell script, executable.
fn write_script(file: &Path, text: &str) -> Result<()> {
    use std::os::unix::fs::PermissionsExt;
    std::fs::write(file, text).wrap_err_with(|| format!("writing {}", file.display()))?;
    std::fs::set_permissions(file, std::fs::Permissions::from_mode(0o755))?;
    Ok(())
}

/// A parameter value as the YAML text `ros2 component load -p` reads back
/// as the same value.
fn param_text(v: &ParamValue) -> String {
    match v {
        ParamValue::StrList(l) => format!("[{}]", l.join(", ")),
        other => serde_yaml_ng::to_string(&yaml_value(other))
            .map(|s| s.trim_end().to_string())
            .unwrap_or_else(|_| other.to_bake_string()),
    }
}

/// One composable: `ros2 component load` with its name, namespace, every
/// resolved parameter, remaps and extra arguments; when its container is in
/// the selection, the script starts it first and waits for its load
/// service, else it loads into a container already running.
fn export_composable_command(
    model: &SystemModel,
    key: &str,
    with_container: bool,
    out_dir: &Path,
    name: &str,
) -> Result<ExportReport> {
    let inst = &model.structure.nodes[key];
    let container = inst
        .container
        .clone()
        .ok_or_else(|| eyre::eyre!("{key} names no container"))?;
    let node_name = inst
        .node_name
        .clone()
        .unwrap_or_else(|| key.rsplit('/').next().unwrap_or(key).to_string());
    let mut words: Vec<String> = vec![
        "ros2".into(),
        "component".into(),
        "load".into(),
        "--no-daemon".into(),
        container.clone(),
        inst.pkg.clone().unwrap_or_default(),
        inst.plugin.clone().unwrap_or_default(),
        "-n".into(),
        node_name
            .rsplit('/')
            .next()
            .unwrap_or(&node_name)
            .to_string(),
    ];
    let ns = namespace_of(key);
    if ns != "/" {
        words.push("--node-namespace".into());
        words.push(ns.to_string());
    }
    for (k, v) in inst.resolved_params(key) {
        words.push("-p".into());
        words.push(format!("{k}:={}", param_text(&v)));
    }
    for r in &inst.remaps {
        words.push("-r".into());
        words.push(format!("{}:={}", r.from, r.to));
    }
    for (k, v) in &inst.extra_args {
        words.push("-e".into());
        words.push(format!("{k}:={v}"));
    }
    let load: Vec<String> = words.iter().map(|w| shell_quote(w)).collect();
    let load = load.join(" ");

    let mut text = String::from(
        "#!/bin/sh\n# Exported by play_launch: one composable node, loaded with \
         `ros2 component load`.\n",
    );
    let mut run = Vec::new();
    if with_container {
        let (prelude, cmd) = run_line(model, &container, out_dir)?;
        text.push_str(&prelude);
        let _ = writeln!(text, "{cmd} &");
        text.push_str("CONTAINER=$!\n");
        text.push_str("trap 'kill -INT $CONTAINER 2>/dev/null; wait $CONTAINER' EXIT INT TERM\n");
        // `ros2 component load` refuses, loading nothing, until it finds
        // the container; a service listing would need its own discovery
        // wait, and reads empty for seconds after the node is listed.
        let _ = writeln!(
            text,
            "until {load}; do\n  kill -0 $CONTAINER 2>/dev/null || exit 1\n  sleep 1\ndone"
        );
        text.push_str("wait $CONTAINER\n");
    } else {
        let _ = writeln!(text, "# Loads into {container}, which must be running.");
        let _ = writeln!(text, "{load}");
    }
    let file = out_dir.join(format!("{name}.sh"));
    write_script(&file, &text)?;
    run.push(if with_container {
        file.display().to_string()
    } else {
        load
    });
    Ok(ExportReport {
        file,
        run,
        ..Default::default()
    })
}

/// The command recorded in an exported file's header, as words.
pub fn header_command(file: &Path) -> Result<Vec<String>> {
    let text =
        std::fs::read_to_string(file).wrap_err_with(|| format!("reading {}", file.display()))?;
    let line = text
        .lines()
        .take(20)
        .find_map(|l| l.trim().strip_prefix("Source: "))
        .ok_or_else(|| {
            eyre::eyre!(
                "{} has no `Source:` line in its header; it was not exported by play_launch, or \
                 its header was edited",
                file.display()
            )
        })?;
    Ok(split_shell(&line.replace("- -", "--")))
}

/// Split words the way [`shell_quote`] joined them.
fn split_shell(s: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let mut started = false;
    let mut chars = s.chars().peekable();
    while let Some(c) = chars.next() {
        match c {
            '\'' => {
                started = true;
                for q in chars.by_ref() {
                    if q == '\'' {
                        break;
                    }
                    cur.push(q);
                }
            }
            '\\' => {
                started = true;
                if let Some(n) = chars.next() {
                    cur.push(n);
                }
            }
            c if c.is_whitespace() => {
                if started {
                    out.push(std::mem::take(&mut cur));
                    started = false;
                }
            }
            c => {
                started = true;
                cur.push(c);
            }
        }
    }
    if started {
        out.push(cur);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shell_words_round_trip() {
        let words = vec![
            "play_launch".to_string(),
            "export".into(),
            "--only".into(),
            "scope:x.launch.xml[part=a]".into(),
            "it's".into(),
            "a b".into(),
        ];
        let joined: Vec<String> = words.iter().map(|w| shell_quote(w)).collect();
        assert_eq!(split_shell(&joined.join(" ")), words);
    }

    #[test]
    fn inline_parameters_keep_their_types() {
        let y = inline_yaml(&[
            ("a".into(), ParamValue::Str("1.0".into())),
            ("b".into(), ParamValue::Float(1.0)),
            (
                "c".into(),
                ParamValue::StrList(vec!["'5'".into(), "5".into()]),
            ),
            ("d".into(), ParamValue::Bool(true)),
        ])
        .unwrap();
        let v: serde_yaml_ng::Value = serde_yaml_ng::from_str(&y).unwrap();
        let p = &v["/**"]["ros__parameters"];
        assert_eq!(p["a"], serde_yaml_ng::Value::String("1.0".into()));
        assert!(p["b"].is_f64());
        assert!(p["c"][0].is_string() && p["c"][1].is_u64());
        assert_eq!(p["d"], serde_yaml_ng::Value::Bool(true));
    }
}
