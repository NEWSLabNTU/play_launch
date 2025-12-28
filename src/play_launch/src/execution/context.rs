use super::node_cmdline::NodeCommandLine;
use crate::ros::launch_dump::{ComposableNodeRecord, LaunchDump, NodeRecord};
use eyre::bail;
use rayon::prelude::*;
use serde::Serialize;
use std::{
    collections::{HashMap, HashSet},
    fs,
    fs::File,
    io::prelude::*,
    path::{Path, PathBuf},
    process::Stdio,
};
use tokio::process::Command;

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

/// Metadata for a composable node
#[derive(Debug, Serialize)]
struct ComposableNodeMetadata {
    #[serde(rename = "type")]
    node_type: String,
    package: String,
    plugin: String,
    node_name: String,
    namespace: String,
    target_container_name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    target_container_node_name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    duplicate_index: Option<usize>,
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

/// ROS node contexts classified into disjoint sets.
pub struct NodeContextClasses {
    pub container_contexts: Vec<NodeContainerContext>,
    pub non_container_node_contexts: Vec<NodeContext>,
}

/// A set of composable node contexts belonging to the same node
/// container.
pub struct ComposableNodeContextSet {
    pub load_node_contexts: Vec<ComposableNodeContext>,
}

/// The context contains all essential data to load a ROS composable
/// node into a node container.
pub struct ComposableNodeContext {
    pub log_name: String,
    pub output_dir: PathBuf,
    pub record: ComposableNodeRecord,
}

#[allow(dead_code)] // Kept for potential future standalone loading
impl ComposableNodeContext {
    pub fn to_load_node_command(&self, round: usize) -> eyre::Result<Command> {
        let ComposableNodeContext {
            output_dir, record, ..
        } = self;

        let command = record.to_command(false);
        let stdout_path = output_dir.join(format!("out.{round}"));
        let stderr_path = output_dir.join(format!("err.{round}"));
        let cmdline_path = output_dir.join(format!("cmdline.{round}"));

        fs::create_dir_all(output_dir)?;

        {
            let mut cmdline_file = File::create(cmdline_path)?;
            cmdline_file.write_all(&record.to_shell(false))?;
        }

        let stdout_file = File::create(stdout_path)?;
        let stderr_file = File::create(&stderr_path)?;

        let mut command: Command = command.into();

        // Create a new process group to ensure all child processes are killed together
        #[cfg(unix)]
        unsafe {
            command.pre_exec(|| {
                libc::setpgid(0, 0);
                Ok(())
            });
        }

        command.kill_on_drop(true);
        command.stdin(Stdio::null());
        command.stdout(stdout_file);
        command.stderr(stderr_file);

        Ok(command)
    }

    pub fn to_standalone_node_command(&self, pgid: Option<i32>) -> eyre::Result<Command> {
        let ComposableNodeContext {
            output_dir, record, ..
        } = self;

        let command = record.to_command(true);
        let stdout_path = output_dir.join("out");
        let stderr_path = output_dir.join("err");
        let cmdline_path = output_dir.join("cmdline");

        fs::create_dir_all(output_dir)?;

        {
            let mut cmdline_file = File::create(cmdline_path)?;
            cmdline_file.write_all(&record.to_shell(true))?;
        }

        let stdout_file = File::create(stdout_path)?;
        let stderr_file = File::create(&stderr_path)?;

        let mut command: Command = command.into();

        // Set process group: join shared PGID if provided, otherwise create new process group
        #[cfg(unix)]
        {
            #[allow(unused_imports)]
            use std::os::unix::process::CommandExt;
            if let Some(pgid) = pgid {
                command.process_group(pgid);
            } else {
                command.process_group(0);
            }
        }

        command.kill_on_drop(true);
        command.stdin(Stdio::null());
        command.stdout(stdout_file);
        command.stderr(stderr_file);

        Ok(command)
    }
}

/// The context contains all essential data to execute a ROS node.
pub struct NodeContext {
    pub record: NodeRecord,
    pub cmdline: NodeCommandLine,
    pub output_dir: PathBuf,
}

impl NodeContext {
    pub fn to_exec_context(&self, pgid: Option<i32>) -> eyre::Result<ExecutionContext> {
        let NodeContext {
            record,
            cmdline,
            output_dir,
        } = self;

        let Some(_exec_name) = &record.exec_name else {
            bail!(r#"expect the "exec_name" field but not found"#);
        };
        let Some(_package) = &record.package else {
            bail!(r#"expect the "package" field but not found"#);
        };

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
    pub node_container_name: String,
    pub node_context: NodeContext,
}

/// Essential data for a process execution.
pub struct ExecutionContext {
    pub log_name: String,
    pub output_dir: PathBuf,
    pub command: tokio::process::Command,
}

/// Load and classify node records in the dump by its kind.
pub fn prepare_node_contexts(
    launch_dump: &LaunchDump,
    node_log_dir: &Path,
    container_names: &HashSet<String>,
) -> eyre::Result<NodeContextClasses> {
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
            let Some(_package) = &record.package else {
                bail!(r#"expect the "package" field but not found"#);
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
            let cmdline = NodeCommandLine::from_node_record(record, &params_files_dir)?;

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
                is_container: false, // Will be updated later for containers
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
            })
        })
        .collect();
    let node_contexts = node_contexts?;

    // Classify the node contexts by checking if it's a node container
    // or not.
    let (container_contexts, non_container_node_contexts): (Vec<_>, Vec<_>) =
        node_contexts.into_par_iter().partition_map(|context| {
            use rayon::iter::Either;

            let NodeContext {
                record: NodeRecord {
                    name, namespace, ..
                },
                ..
            } = &context;

            // Check if the package/node_name is a known node
            // container name.
            let container_name = {
                let (Some(namespace), Some(name)) = (namespace, name) else {
                    return Either::Right(context);
                };

                // Handle namespace ending with '/' to avoid double slashes
                let container_name = if namespace.ends_with('/') {
                    format!("{namespace}{name}")
                } else {
                    format!("{namespace}/{name}")
                };
                if !container_names.contains(&container_name) {
                    return Either::Right(context);
                };
                container_name
            };
            let container_context = NodeContainerContext {
                node_container_name: container_name,
                node_context: context,
            };
            Either::Left(container_context)
        });

    // Update metadata for containers to reflect they are containers
    for container_ctx in &container_contexts {
        let NodeContext { output_dir, .. } = &container_ctx.node_context;

        // Read existing metadata
        let metadata_path = output_dir.join("metadata.json");
        if metadata_path.exists() {
            if let Ok(metadata_str) = fs::read_to_string(&metadata_path) {
                if let Ok(mut metadata) = serde_json::from_str::<serde_json::Value>(&metadata_str) {
                    // Update fields
                    metadata["type"] = serde_json::json!("container");
                    metadata["is_container"] = serde_json::json!(true);
                    metadata["container_full_name"] =
                        serde_json::json!(&container_ctx.node_container_name);

                    // Write back
                    if let Ok(updated_json) = serde_json::to_string_pretty(&metadata) {
                        let _ = fs::write(&metadata_path, updated_json);
                    }
                }
            }
        }
    }

    Ok(NodeContextClasses {
        container_contexts,
        non_container_node_contexts,
    })
}

/// Load composable node records in the dump.
pub fn prepare_composable_node_contexts(
    launch_dump: &LaunchDump,
    load_node_log_dir: &Path,
) -> eyre::Result<ComposableNodeContextSet> {
    // First pass: Collect node names for deduplication
    let node_names: Vec<String> = launch_dump
        .load_node
        .iter()
        .map(|record| record.node_name.clone())
        .collect();

    let name_map = build_name_map(node_names);
    let mut name_indices: HashMap<String, usize> = HashMap::new();

    // Build a vector of (record, dir_name) pairs first
    let mut record_dirs: Vec<(&ComposableNodeRecord, String)> = Vec::new();
    for record in &launch_dump.load_node {
        let base_name = &record.node_name;
        let index = name_indices.entry(base_name.clone()).or_insert(0);
        let unique_names = name_map.get(base_name).unwrap();
        let dir_name = unique_names[*index].clone();
        *index += 1;
        record_dirs.push((record, dir_name));
    }

    // Now process in parallel with pre-computed directory names
    let load_node_contexts: Result<Vec<_>, _> = record_dirs
        .par_iter()
        .map(|(record, dir_name)| {
            let ComposableNodeRecord {
                package,
                plugin,
                target_container_name,
                node_name,
                namespace,
                ..
            } = record;

            // Create flat directory structure
            let output_dir = load_node_log_dir.join(dir_name);
            fs::create_dir_all(&output_dir)?;

            // Extract container node name from target_container_name
            // e.g., "/control/control_container" -> "control_container"
            let target_container_node_name = target_container_name
                .rsplit('/')
                .next()
                .filter(|s| !s.is_empty())
                .map(|s| s.to_string());

            // Create metadata
            let duplicate_index = if dir_name.contains('_') && dir_name != node_name {
                // Extract index from name like "glog_component_2"
                dir_name
                    .rsplit('_')
                    .next()
                    .and_then(|s| s.parse::<usize>().ok())
            } else {
                None
            };

            let metadata = ComposableNodeMetadata {
                node_type: "composable_node".to_string(),
                package: package.clone(),
                plugin: plugin.clone(),
                node_name: node_name.clone(),
                namespace: namespace.clone(),
                target_container_name: target_container_name.clone(),
                target_container_node_name,
                duplicate_index,
            };

            // Write metadata.json
            let metadata_path = output_dir.join("metadata.json");
            let metadata_json = serde_json::to_string_pretty(&metadata)?;
            fs::write(metadata_path, metadata_json)?;

            let log_name = format!("COMPOSABLE_NODE '{}'", dir_name);
            eyre::Ok(ComposableNodeContext {
                record: (*record).clone(),
                log_name,
                output_dir,
            })
        })
        .collect();
    let load_node_contexts = load_node_contexts?;

    Ok(ComposableNodeContextSet { load_node_contexts })
}
