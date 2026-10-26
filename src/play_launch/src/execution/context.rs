use super::node_cmdline::NodeCommandLine;
use crate::cli::options::ContainerMode;
use eyre::bail;
use rayon::prelude::*;
use ros_launch_manifest_model as model;
use ros_launch_resolve::ros::launch_dump::{ComposableNodeRecord, LaunchDump, NodeRecord};
use serde::Serialize;
use std::{
    collections::HashMap,
    fs,
    fs::File,
    io::prelude::*,
    path::{Path, PathBuf},
    process::Stdio,
};
use tracing::{debug, info, warn};

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

/// The cgroup handle for one member, or `None` when grouping is unavailable.
///
/// Failing to create a group is never fatal: the member spawns ungrouped, and
/// the only loss is accounting.
fn member_cgroup(
    tree: Option<&super::cgroup::CgroupTree>,
    kind: super::cgroup::GroupKind,
    dir_name: &str,
    fqn: &str,
    limits: Option<&super::cgroup::LimitRules>,
) -> Option<super::cgroup::CgroupHandle> {
    let path = tree?.group_for(kind, dir_name)?;

    // Phase 66 W2 — limits are written BEFORE the member is forked into the
    // group. A limit applied afterwards is a limit the process spent some
    // window without, and for `memory.max` that window is exactly when a
    // constructor allocates most.
    if let Some(rules) = limits {
        let l = rules.for_member(fqn);
        if !l.is_empty() {
            let applied = l.apply(&path);
            if applied.is_empty() {
                warn!("cgroups: no limit could be set for {fqn}");
            } else {
                info!("cgroups: {fqn}: {}", applied.join(", "));
            }
        }
    }
    super::cgroup::CgroupHandle::new(&path)
}

/// The context contains all essential data to execute a ROS node.
pub struct NodeContext {
    pub record: NodeRecord,
    pub cmdline: NodeCommandLine,
    pub output_dir: PathBuf,
    /// See [`ComposableNodeContext::model_fqn`].
    pub model_fqn: Option<String>,
    /// Phase 66 — the cgroup this member joins between `fork()` and `exec()`.
    /// `None` whenever grouping is unavailable, which is the ordinary case:
    /// play_launch started from a terminal cannot create cgroups.
    ///
    /// A container's handle covers its composables too — they are forked by
    /// the container and inherit its group.
    pub cgroup: Option<super::cgroup::CgroupHandle>,
}

impl NodeContext {
    pub fn to_exec_context(&self, pgid: Option<i32>) -> eyre::Result<ExecutionContext> {
        let NodeContext {
            record,
            cmdline,
            output_dir,
            model_fqn: _,
            cgroup,
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

        // A respawn (or a restart from the web UI) comes back through here
        // with the previous process's files still in place. `File::create`
        // below truncates, so without this the log that explains WHY the
        // process died was destroyed by the act of restarting it.
        match rotate_previous_incarnation(output_dir) {
            Ok(Some(n)) => debug!(
                "{}: previous process's logs kept as out.{n}/err.{n}",
                output_dir.display()
            ),
            Ok(None) => {}
            Err(e) => warn!(
                "{}: could not keep the previous process's logs ({e}); they are overwritten",
                output_dir.display()
            ),
        }

        {
            let mut cmdline_file = File::create(cmdline_path)?;
            cmdline_file.write_all(&cmdline.to_shell(false))?;
        }

        let stdout_file = File::create(stdout_path)?;
        let stderr_file = File::create(&stderr_path)?;

        let mut command: tokio::process::Command =
            cmdline.to_command(false, pgid, cgroup.clone()).into();

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

/// The files in a member's log directory that describe ONE process: its
/// output, its PID and its exit status. `cmdline`, `metadata.json`,
/// `metrics.csv` and `params_files/` describe the member and are shared by
/// every process it runs.
pub const INCARNATION_FILES: [&str; 4] = ["out", "err", "pid", "status"];

/// Keep the previous process's [`INCARNATION_FILES`] before a new one starts.
///
/// The live process always writes the plain names (`out`, `err`, ...), so
/// `play_log/latest/node/<name>/err` keeps naming the current process for
/// every reader — the web UI, the Health page, `tail -F`. A process that ran
/// before it is renamed to numbered siblings, numbered in the order the
/// processes started: `err.1` is the first process, `err.2` the second, and
/// the plain `err` the newest. Nothing is renumbered later, so a path handed
/// out once stays correct.
///
/// Returns the number given to the previous process, or `None` when there
/// was none (the first spawn: no `out` or `err` yet). A file the previous
/// process did not leave (`status` after a spawn failure, say) is simply
/// absent from its set.
///
/// Rename, never copy: a grandchild that inherited the old descriptors and
/// outlived its parent keeps writing into the renamed file, where its lines
/// belong.
pub fn rotate_previous_incarnation(output_dir: &Path) -> std::io::Result<Option<u32>> {
    if !output_dir.join("out").exists() && !output_dir.join("err").exists() {
        return Ok(None);
    }
    let taken = |n: u32| {
        INCARNATION_FILES
            .iter()
            .any(|f| output_dir.join(format!("{f}.{n}")).exists())
    };
    let n = (1..=u32::MAX)
        .find(|&n| !taken(n))
        .ok_or_else(|| std::io::Error::other("no free incarnation number"))?;
    for f in INCARNATION_FILES {
        let src = output_dir.join(f);
        if src.exists() {
            fs::rename(&src, output_dir.join(format!("{f}.{n}")))?;
        }
    }
    Ok(Some(n))
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
                // `run` spawns a single node and builds no cgroup tree.
                cgroup: None,
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
    cgroups: Option<&super::cgroup::CgroupTree>,
    limits: Option<&super::cgroup::LimitRules>,
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
            cgroup: member_cgroup(
                cgroups,
                super::cgroup::GroupKind::Node,
                dir_name,
                fqn,
                limits,
            ),
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
    cgroups: Option<&super::cgroup::CgroupTree>,
    limits: Option<&super::cgroup::LimitRules>,
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
            ContainerMode::Observable | ContainerMode::Isolated | ContainerMode::CloneVm => {
                let mut args = Vec::new();
                if record.executable == "component_container_mt" {
                    args.push("--use_multi_threaded_executor".to_string());
                }
                if container_mode == ContainerMode::Isolated {
                    args.push("--isolated".to_string());
                }
                // Every clone child spins its own SingleThreadedExecutor, so
                // --use_multi_threaded_executor above governs the manager's own
                // services only. Left alone rather than suppressed: a stock
                // component_container_mt should still behave like one.
                if container_mode == ContainerMode::CloneVm {
                    args.push("--clone-vm".to_string());
                }
                (
                    "play_launch_container".to_string(),
                    "component_container".to_string(),
                    args,
                )
            }
        };
        // A launch file may already spell an isolation flag into the
        // container's `args` -- play_launch's own fixtures do. `--container-mode`
        // is the operator asking for something else on this run, and the
        // operator wins, so drop the launch file's choice rather than emit both
        // and have the container refuse the pair.
        let launch_args = record.args.clone().map(|args| {
            args.into_iter()
                .filter(|a| {
                    !(container_mode != ContainerMode::Stock
                        && matches!(a.as_str(), "--isolated" | "--clone-vm"))
                })
                .collect::<Vec<_>>()
        });

        let final_args = if extra_args.is_empty() {
            launch_args
        } else {
            if let Some(mut existing) = launch_args {
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
                cgroup: member_cgroup(
                    cgroups,
                    super::cgroup::GroupKind::Container,
                    dir_name,
                    fqn,
                    limits,
                ),
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
            config_seq: Default::default(),
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
            // The launch `<timer>` delay. For a composable this is the wait
            // before its LoadNode request, which the container actor holds
            // back (`ComposableNodeMetadata::start_after`) rather than a wait
            // before a spawn — a composable has no process of its own.
            start_delay_secs: inst.start_delay_secs,
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

#[cfg(test)]
mod incarnation_tests {
    use super::*;

    fn read(dir: &Path, f: &str) -> String {
        fs::read_to_string(dir.join(f)).unwrap()
    }

    /// The first spawn finds nothing to keep and renames nothing — a member
    /// that never restarts has exactly the layout it always had.
    #[test]
    fn the_first_spawn_keeps_nothing() {
        let tmp = tempfile::TempDir::new().unwrap();
        let dir = tmp.path();
        fs::write(dir.join("cmdline"), "x").unwrap();
        assert_eq!(rotate_previous_incarnation(dir).unwrap(), None);
        let mut names: Vec<String> = fs::read_dir(dir)
            .unwrap()
            .map(|e| e.unwrap().file_name().into_string().unwrap())
            .collect();
        names.sort();
        assert_eq!(names, ["cmdline"]);
    }

    /// Each restart moves the dead process's files to the next free number,
    /// in start order, and leaves the plain names free for the new one.
    #[test]
    fn each_restart_keeps_the_previous_process_under_the_next_number() {
        let tmp = tempfile::TempDir::new().unwrap();
        let dir = tmp.path();
        for life in 1..=3 {
            if life > 1 {
                assert_eq!(rotate_previous_incarnation(dir).unwrap(), Some(life - 1));
            }
            for f in INCARNATION_FILES {
                fs::write(dir.join(f), format!("{f} of life {life}")).unwrap();
            }
            fs::write(dir.join("cmdline"), "the same command").unwrap();
        }
        for life in 1..=2 {
            for f in INCARNATION_FILES {
                assert_eq!(
                    read(dir, &format!("{f}.{life}")),
                    format!("{f} of life {life}")
                );
            }
        }
        for f in INCARNATION_FILES {
            assert_eq!(read(dir, f), format!("{f} of life 3"));
        }
        assert!(
            !dir.join("cmdline.1").exists(),
            "cmdline is per member, not per process"
        );
    }

    /// A process that died before it could write some of its files (a spawn
    /// failure leaves `err` and no `pid`) keeps what it has, and the numbering
    /// does not skip.
    #[test]
    fn a_partial_set_is_kept_as_it_is() {
        let tmp = tempfile::TempDir::new().unwrap();
        let dir = tmp.path();
        fs::write(dir.join("err"), "spawn failed").unwrap();
        assert_eq!(rotate_previous_incarnation(dir).unwrap(), Some(1));
        assert_eq!(read(dir, "err.1"), "spawn failed");
        assert!(!dir.join("pid.1").exists());
        fs::write(dir.join("out"), "").unwrap();
        assert_eq!(rotate_previous_incarnation(dir).unwrap(), Some(2));
        assert!(dir.join("out.2").exists());
    }

    /// The whole path a respawn takes: `to_exec_context` twice on one
    /// context. The second must not truncate what the first process wrote.
    #[test]
    fn to_exec_context_twice_keeps_the_first_processs_log() {
        let tmp = tempfile::TempDir::new().unwrap();
        let dir = tmp.path().join("node/talker");
        let record: NodeRecord = serde_json::from_value(serde_json::json!({
            "executable": "true",
            "cmd": ["true"],
            "params_files": [],
        }))
        .unwrap();
        let ctx = NodeContext {
            cmdline: NodeCommandLine::from_node_record(&record, &dir, &Default::default()).unwrap(),
            record,
            output_dir: dir.clone(),
            model_fqn: None,
            cgroup: None,
        };
        ctx.to_exec_context(None).unwrap();
        fs::write(dir.join("err"), "why the first process died\n").unwrap();
        ctx.to_exec_context(None).unwrap();
        assert_eq!(read(&dir, "err.1"), "why the first process died\n");
        assert_eq!(read(&dir, "err"), "", "the live process gets a fresh err");
        assert!(dir.join("out.1").exists() && dir.join("out").exists());
    }
}
