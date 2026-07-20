use super::node_cmdline::NodeCommandLine;
use crate::{
    cli::options::ContainerMode,
    ros::launch_dump::{ComposableNodeRecord, LaunchDump, NodeRecord},
};
use eyre::bail;
use rayon::prelude::*;
use ros_launch_manifest_model as model;
use serde::Serialize;
use std::{
    collections::HashMap,
    fs,
    fs::File,
    io::prelude::*,
    path::{Path, PathBuf},
    process::Stdio,
};

/// Metadata for a regular ROS node
#[derive(Debug, Serialize)]
struct NodeMetadata {
    #[serde(rename = "type")]
    node_type: String,
    package: Option<String>,
    executable: String,
    exec_name: Option<String>,
    name: Option<String>,
    namespace: Option<String>,
    is_container: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    container_full_name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    duplicate_index: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    note: Option<String>,
}

/// Helper to deduplicate names and assign numeric suffixes
fn build_name_map(names: Vec<String>) -> HashMap<String, Vec<String>> {
    let mut name_counts: HashMap<String, usize> = HashMap::new();
    let mut result: HashMap<String, Vec<String>> = HashMap::new();

    for name in names {
        let count = name_counts.entry(name.clone()).or_insert(0);
        *count += 1;

        let unique_name = if *count == 1 {
            name.clone()
        } else {
            format!("{}_{}", name, count)
        };

        result.entry(name.clone()).or_default().push(unique_name);
    }

    result
}

/// A set of composable node contexts belonging to the same node
/// container.
pub struct ComposableNodeContextSet {
    pub load_node_contexts: Vec<ComposableNodeContext>,
}

/// The context contains all essential data to load a ROS composable
/// node into a node container.
pub struct ComposableNodeContext {
    #[allow(dead_code)]
    pub log_name: String,
    pub output_dir: PathBuf,
    pub record: ComposableNodeRecord,
    /// The model's `structure.nodes` FQN this context was built from
    /// (Phase 46.3b) — `None` for record-sourced contexts. Lets the model
    /// spawn path resolve sched-plan bindings (`SchedPlan::for_fqn`)
    /// directly from the already-known, single-source-of-truth FQN instead
    /// of recomputing it via `sched_loader::fqn_for(&launch_dump, ...)`
    /// (which needs a `LaunchDump` numeric scope id this context doesn't
    /// carry).
    pub model_fqn: Option<String>,
}

/// The context contains all essential data to execute a ROS node.
pub struct NodeContext {
    pub record: NodeRecord,
    pub cmdline: NodeCommandLine,
    pub output_dir: PathBuf,
    /// See [`ComposableNodeContext::model_fqn`].
    pub model_fqn: Option<String>,
}

impl NodeContext {
    pub fn to_exec_context(&self, pgid: Option<i32>) -> eyre::Result<ExecutionContext> {
        let NodeContext {
            record,
            cmdline,
            output_dir,
            model_fqn: _,
        } = self;

        // Raw executables (from <executable> tags) have no package or exec_name.
        // Only require these fields for regular ROS nodes.
        if record.package.is_some() && record.exec_name.is_none() {
            bail!(r#"expect the "exec_name" field but not found"#);
        }

        let stdout_path = output_dir.join("out");
        let stderr_path = output_dir.join("err");
        let cmdline_path = output_dir.join("cmdline");

        fs::create_dir_all(output_dir)?;

        {
            let mut cmdline_file = File::create(cmdline_path)?;
            cmdline_file.write_all(&cmdline.to_shell(false))?;
        }

        let stdout_file = File::create(stdout_path)?;
        let stderr_file = File::create(&stderr_path)?;

        let mut command: tokio::process::Command = cmdline.to_command(false, pgid).into();

        command.kill_on_drop(true);
        command.stdin(Stdio::null());
        command.stdout(stdout_file);
        command.stderr(stderr_file);

        // Extract directory name from output_dir for log_name
        let dir_name = output_dir
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("unknown");
        let log_name = format!("NODE '{}'", dir_name);
        Ok(ExecutionContext {
            log_name,
            output_dir: output_dir.to_path_buf(),
            command,
        })
    }
}

/// The context contains all essential data to execute a node
/// container.
pub struct NodeContainerContext {
    pub node_context: NodeContext,
}

/// Essential data for a process execution.
pub struct ExecutionContext {
    pub log_name: String,
    pub output_dir: PathBuf,
    pub command: tokio::process::Command,
}

/// Load regular node records from the dump (containers are handled
/// separately). The one record-path (LaunchDump-sourced) spawn builder
/// still live — `run` (single-node dump+replay) uses it; `replay`/`launch`
/// spawn from the SystemModel via `prepare_node_contexts_from_model`.
pub fn prepare_node_contexts(
    launch_dump: &LaunchDump,
    node_log_dir: &Path,
) -> eyre::Result<Vec<NodeContext>> {
    // First pass: Collect node names for deduplication
    let node_names: Vec<String> = launch_dump
        .node
        .iter()
        .map(|record| {
            // Use node name if available, otherwise fall back to exec_name
            record
                .name
                .clone()
                .or_else(|| record.exec_name.clone())
                .unwrap_or_else(|| "unknown".to_string())
        })
        .collect();

    let name_map = build_name_map(node_names);
    let mut name_indices: HashMap<String, usize> = HashMap::new();

    // Build a vector of (record, dir_name) pairs first
    let mut record_dirs: Vec<(&NodeRecord, String)> = Vec::new();
    for record in &launch_dump.node {
        let base_name = record
            .name
            .as_deref()
            .or(record.exec_name.as_deref())
            .unwrap_or("unknown");
        let index = name_indices.entry(base_name.to_string()).or_insert(0);
        let unique_names = name_map.get(base_name).unwrap();
        let dir_name = unique_names[*index].clone();
        *index += 1;
        record_dirs.push((record, dir_name));
    }

    // Now process in parallel with pre-computed directory names
    let node_contexts: Result<Vec<_>, _> = record_dirs
        .par_iter()
        .map(|(record, dir_name)| {
            let Some(exec_name) = &record.exec_name else {
                bail!(r#"expect the "exec_name" field but not found"#);
            };

            let base_name = record
                .name
                .as_deref()
                .or(Some(exec_name.as_str()))
                .unwrap_or("unknown");

            // Create flat directory structure
            let output_dir = node_log_dir.join(dir_name);
            let params_files_dir = output_dir.join("params_files");
            fs::create_dir_all(&params_files_dir)?;

            // Build the command line.
            let cmdline = NodeCommandLine::from_node_record(
                record,
                &params_files_dir,
                &launch_dump.variables,
            )?;

            // Create metadata
            let duplicate_index = if dir_name.contains('_') && dir_name != base_name {
                // Extract index from name like "glog_component_2"
                dir_name
                    .rsplit('_')
                    .next()
                    .and_then(|s| s.parse::<usize>().ok())
            } else {
                None
            };

            let metadata = NodeMetadata {
                node_type: "node".to_string(),
                package: record.package.clone(),
                executable: record.executable.clone(),
                exec_name: record.exec_name.clone(),
                name: record.name.clone(),
                namespace: record.namespace.clone(),
                is_container: false,
                container_full_name: None,
                duplicate_index,
                note: if record.name.is_none() {
                    Some("name was null, using exec_name as directory name".to_string())
                } else {
                    None
                },
            };

            // Write metadata.json
            let metadata_path = output_dir.join("metadata.json");
            let metadata_json = serde_json::to_string_pretty(&metadata)?;
            fs::write(metadata_path, metadata_json)?;

            eyre::Ok(NodeContext {
                record: (*record).clone(),
                cmdline,
                output_dir,
                model_fqn: None,
            })
        })
        .collect();

    node_contexts
}

// ---------------------------------------------------------------------
// Phase 46.3b — spawn context from the SystemModel
// ---------------------------------------------------------------------
//
// Phase 47.B3/B4: `replay`/`launch` spawn exclusively from the SystemModel.
// The container/composable record-path builders (LaunchDump-sourced) were
// removed once the record.json artifact was retired and their only remaining
// caller (the static-equivalence test) went with it. `prepare_node_contexts`
// (above) is the one record-path builder still live — `run` (single-node
// dump+replay) uses it.
//
// The functions below build the spawn context from `model::SystemModel`:
// same argv-assembly pipeline (`NodeCommandLine::from_node_record`, via the
// `node_cmdline::node_record_from_instance` adapter), same output-dir/
// metadata.json conventions. See `.superpowers/sdd/p46-w3-analysis.md` and
// the 46.3b report for the full design.

/// Sequential dedup pass shared by the three model-sourced builders below:
/// mirrors the record path's two-phase `build_name_map` dedup (`{base}`,
/// `{base}_2`, `{base}_3`, ... on collision), keyed by the FQN's last path
/// segment instead of `NodeRecord.name`.
fn dedup_model_dir_names(entries: &[(&String, &model::NodeInstance)]) -> Vec<String> {
    let base_names: Vec<String> = entries
        .iter()
        .map(|(fqn, _)| super::node_cmdline::split_model_fqn(fqn).1.to_string())
        .collect();
    let name_map = build_name_map(base_names.clone());
    let mut name_indices: HashMap<String, usize> = HashMap::new();
    base_names
        .iter()
        .map(|base_name| {
            let index = name_indices.entry(base_name.clone()).or_insert(0);
            let unique_names = name_map.get(base_name).unwrap();
            let dir_name = unique_names[*index].clone();
            *index += 1;
            dir_name
        })
        .collect()
}

/// Model-sourced equivalent of [`prepare_node_contexts`]: regular `<node>`
/// instances — every `structure.nodes` entry that is neither a composable
/// (`plugin.is_some()`) nor a container (`is_container`). Classification is
/// the model's own `is_container` bit (46.3b), populated by
/// `model_builder` from the record's `dump.container[]` array — NOT
/// inferred by reverse-resolving composable `container=` references, which
/// could misclassify a regular node whose name collides with a dangling/
/// ambiguous composable target and then corrupt its executable via the
/// `--container-mode` override.
pub fn prepare_node_contexts_from_model(
    system_model: &model::SystemModel,
    node_log_dir: &Path,
) -> eyre::Result<Vec<NodeContext>> {
    let entries: Vec<(&String, &model::NodeInstance)> = system_model
        .structure
        .nodes
        .iter()
        .filter(|(_, inst)| inst.plugin.is_none() && !inst.is_container)
        .collect();
    let dir_names = dedup_model_dir_names(&entries);

    let mut node_contexts = Vec::with_capacity(entries.len());
    for ((fqn, inst), dir_name) in entries.iter().zip(dir_names.iter()) {
        let output_dir = node_log_dir.join(dir_name);
        let params_files_dir = output_dir.join("params_files");
        fs::create_dir_all(&params_files_dir)?;

        let record = super::node_cmdline::node_record_from_instance(fqn, inst);
        let cmdline =
            NodeCommandLine::from_node_record(&record, &params_files_dir, &HashMap::new())?;

        let metadata = NodeMetadata {
            node_type: "node".to_string(),
            package: record.package.clone(),
            executable: record.executable.clone(),
            exec_name: record.exec_name.clone(),
            name: record.name.clone(),
            namespace: record.namespace.clone(),
            is_container: false,
            container_full_name: None,
            duplicate_index: None,
            note: None,
        };
        let metadata_path = output_dir.join("metadata.json");
        fs::write(metadata_path, serde_json::to_string_pretty(&metadata)?)?;

        node_contexts.push(NodeContext {
            record,
            cmdline,
            output_dir,
            model_fqn: Some((*fqn).clone()),
        });
    }

    Ok(node_contexts)
}

/// Container spawn contexts from the SystemModel's `structure.nodes`
/// (`is_container` entries): applies the `--container-mode` package/
/// executable override (`play_launch_container` + `--isolated`/
/// `--use_multi_threaded_executor` for `Isolated`/`Observable`, stock
/// passthrough for `Stock`).
pub fn prepare_container_contexts_from_model(
    system_model: &model::SystemModel,
    container_log_dir: &Path,
    container_mode: ContainerMode,
) -> eyre::Result<Vec<NodeContainerContext>> {
    let entries: Vec<(&String, &model::NodeInstance)> = system_model
        .structure
        .nodes
        .iter()
        .filter(|(_, inst)| inst.is_container)
        .collect();
    let dir_names = dedup_model_dir_names(&entries);

    let mut container_contexts = Vec::with_capacity(entries.len());
    for ((fqn, inst), dir_name) in entries.iter().zip(dir_names.iter()) {
        let mut record = super::node_cmdline::node_record_from_instance(fqn, inst);

        // `node_record_from_instance` collapses a root-level FQN's
        // namespace to `None` (omitting `__ns`) to match the common
        // "no `namespace=` attribute" case for regular `<node>`s, where
        // `NodeRecord.namespace` is genuinely `Option`. Containers are
        // different: `NodeContainerRecord.namespace: String` is NOT
        // optional (the record-sourced path always wrapped it `Some(...)`),
        // so `__ns` is emitted for EVERY container, including root-
        // namespaced ones (`-r __ns:=/`). Empirically found on Autoware's
        // `pointcloud_container` (root-namespaced): dropping `__ns:=/` for a
        // root container is wrong. Force `Some` here.
        record.namespace = Some(record.namespace.unwrap_or_else(|| "/".to_string()));

        // `--container-mode` package/executable override.
        let (package, executable, mut extra_args) = match container_mode {
            ContainerMode::Stock => (
                record.package.clone().unwrap_or_default(),
                record.executable.clone(),
                Vec::<String>::new(),
            ),
            ContainerMode::Observable | ContainerMode::Isolated => {
                let mut args = Vec::new();
                if record.executable == "component_container_mt" {
                    args.push("--use_multi_threaded_executor".to_string());
                }
                if container_mode == ContainerMode::Isolated {
                    args.push("--isolated".to_string());
                }
                (
                    "play_launch_container".to_string(),
                    "component_container".to_string(),
                    args,
                )
            }
        };
        let final_args = if extra_args.is_empty() {
            record.args.clone()
        } else {
            if let Some(mut existing) = record.args.clone() {
                extra_args.append(&mut existing);
            }
            Some(extra_args)
        };
        record.package = Some(package);
        record.executable = executable;
        record.args = final_args;

        let output_dir = container_log_dir.join(dir_name);
        let params_files_dir = output_dir.join("params_files");
        fs::create_dir_all(&params_files_dir)?;

        let cmdline =
            NodeCommandLine::from_node_record(&record, &params_files_dir, &HashMap::new())?;

        let metadata = NodeMetadata {
            node_type: "container".to_string(),
            package: record.package.clone(),
            executable: record.executable.clone(),
            exec_name: record.exec_name.clone(),
            name: record.name.clone(),
            namespace: record.namespace.clone(),
            is_container: true,
            container_full_name: Some((*fqn).clone()),
            duplicate_index: None,
            note: None,
        };
        let metadata_path = output_dir.join("metadata.json");
        fs::write(metadata_path, serde_json::to_string_pretty(&metadata)?)?;

        container_contexts.push(NodeContainerContext {
            node_context: NodeContext {
                record,
                cmdline,
                output_dir,
                model_fqn: Some((*fqn).clone()),
            },
        });
    }

    Ok(container_contexts)
}

/// Composable-node LoadNode-request contexts from the SystemModel: every
/// `structure.nodes` entry carrying a `plugin`. Composable nodes have no
/// argv/process of their own (`NodeCommandLine` is not involved) — they're
/// LoadNode requests only; GAP-6's `extra_args` (⊃ `use_intra_process_comms`)
/// rides straight through.
pub fn prepare_composable_node_contexts_from_model(
    system_model: &model::SystemModel,
    load_node_log_dir: &Path,
) -> eyre::Result<ComposableNodeContextSet> {
    let entries: Vec<(&String, &model::NodeInstance)> = system_model
        .structure
        .nodes
        .iter()
        .filter(|(_, inst)| inst.plugin.is_some())
        .collect();
    let dir_names = dedup_model_dir_names(&entries);

    let mut load_node_contexts = Vec::with_capacity(entries.len());
    for ((fqn, inst), dir_name) in entries.iter().zip(dir_names.iter()) {
        let (ns, name) = super::node_cmdline::split_model_fqn(fqn);
        let namespace = if ns.is_empty() {
            "/".to_string()
        } else {
            ns.to_string()
        };

        let params: Vec<(String, String)> = inst
            .params
            .iter()
            .map(|(k, v)| {
                (
                    k.clone(),
                    super::node_cmdline::param_value_to_record_string(v),
                )
            })
            .collect();
        let remaps: Vec<(String, String)> = inst
            .remaps
            .iter()
            .map(|r| (r.from.clone(), r.to.clone()))
            .collect();
        let env = (!inst.env.is_empty()).then(|| {
            inst.env
                .iter()
                .map(|e| (e.name.clone(), e.value.clone()))
                .collect()
        });

        let record = ComposableNodeRecord {
            package: inst.pkg.clone().unwrap_or_default(),
            plugin: inst.plugin.clone().unwrap_or_default(),
            target_container_name: inst.container.clone().unwrap_or_default(),
            node_name: name.to_string(),
            namespace,
            log_level: None,
            remaps,
            params,
            extra_args: inst
                .extra_args
                .iter()
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect(),
            env,
            scope: None,
        };

        let output_dir = load_node_log_dir.join(dir_name);
        let log_name = format!("COMPOSABLE_NODE '{}'", dir_name);

        load_node_contexts.push(ComposableNodeContext {
            record,
            log_name,
            output_dir,
            model_fqn: Some((*fqn).clone()),
        });
    }

    Ok(ComposableNodeContextSet { load_node_contexts })
}
