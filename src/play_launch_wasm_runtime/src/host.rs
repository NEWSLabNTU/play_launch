//! LaunchHost: runtime state for WASM execution, with builder pattern for records.

use play_launch_parser::record::{
    build_ros_command, normalize_param_value, resolve_exec_path, ComposableNodeContainerRecord,
    LoadNodeRecord, NodeRecord, RecordJson,
};
use play_launch_parser::substitution::LaunchContext;
use play_launch_wasm_common::memory::BUMP_BASE;
use std::collections::HashMap;

/// Scope snapshot saved/restored during group and include processing.
#[derive(Clone)]
pub struct ScopeSnapshot {
    pub namespace_depth: usize,
    pub remapping_count: usize,
}

/// Builder for node records (SpawnNode).
#[derive(Default)]
pub struct NodeBuilder {
    pub package: Option<String>,
    pub executable: Option<String>,
    pub name: Option<String>,
    pub namespace: Option<String>,
    pub params: Vec<(String, String)>,
    pub param_files: Vec<String>,
    pub remaps: Vec<(String, String)>,
    pub env: Vec<(String, String)>,
    pub args: Option<String>,
    pub respawn: Option<String>,
    pub respawn_delay: Option<String>,
}

/// Builder for executable records (SpawnExecutable).
#[derive(Default)]
pub struct ExecBuilder {
    pub cmd: Option<String>,
    pub name: Option<String>,
    pub args: Vec<String>,
    pub env: Vec<(String, String)>,
}

/// Builder for container records (SpawnContainer).
#[derive(Default)]
pub struct ContainerBuilder {
    pub package: Option<String>,
    pub executable: Option<String>,
    pub name: Option<String>,
    pub namespace: Option<String>,
    pub args: Option<String>,
}

/// Builder for composable node declarations.
#[derive(Default)]
pub struct CompNodeBuilder {
    pub package: Option<String>,
    pub plugin: Option<String>,
    pub name: Option<String>,
    pub namespace: Option<String>,
    pub params: Vec<(String, String)>,
    pub remaps: Vec<(String, String)>,
    pub extra_args: Vec<(String, String)>,
}

/// Builder for load composable node actions.
pub struct LoadNodeBuilder {
    pub target: String,
}

/// Main host state passed through wasmtime::Store.
pub struct LaunchHost {
    pub context: LaunchContext,
    pub node_builder: Option<NodeBuilder>,
    pub exec_builder: Option<ExecBuilder>,
    pub container_builder: Option<ContainerBuilder>,
    pub load_node_builder: Option<LoadNodeBuilder>,
    pub comp_node_builder: Option<CompNodeBuilder>,
    pub records: Vec<NodeRecord>,
    pub containers: Vec<ComposableNodeContainerRecord>,
    pub load_nodes: Vec<LoadNodeRecord>,
    pub bump_offset: u32,
    pub scope_stack: Vec<ScopeSnapshot>,
    /// Composable nodes accumulated during container/load_node building
    pub pending_comp_nodes: Vec<CompNodeBuilder>,
}

impl LaunchHost {
    pub fn new(cli_args: HashMap<String, String>) -> Self {
        let mut context = LaunchContext::new();
        for (k, v) in &cli_args {
            context.set_configuration(k.clone(), v.clone());
        }
        Self {
            context,
            node_builder: None,
            exec_builder: None,
            container_builder: None,
            load_node_builder: None,
            comp_node_builder: None,
            records: Vec::new(),
            containers: Vec::new(),
            load_nodes: Vec::new(),
            bump_offset: BUMP_BASE,
            scope_stack: Vec::new(),
            pending_comp_nodes: Vec::new(),
        }
    }

    /// Collect global parameters from context, normalizing values.
    fn collect_global_params(&self) -> Option<Vec<(String, String)>> {
        let map = self.context.global_parameters();
        if map.is_empty() {
            None
        } else {
            Some(
                map.into_iter()
                    .map(|(k, v)| (k, normalize_param_value(&v)))
                    .collect(),
            )
        }
    }

    /// Collect context environment merged with builder-specific env vars.
    fn collect_merged_env(&self, builder_env: &[(String, String)]) -> Option<Vec<(String, String)>> {
        let mut merged = self.context.environment();
        for (k, v) in builder_env {
            merged.insert(k.clone(), v.clone());
        }
        if merged.is_empty() {
            None
        } else {
            Some(merged.into_iter().collect())
        }
    }

    /// Finalize a node builder into a NodeRecord.
    pub fn end_node(&mut self) -> anyhow::Result<()> {
        let builder = self
            .node_builder
            .take()
            .ok_or_else(|| anyhow::anyhow!("end_node called with no active node builder"))?;

        let package = builder
            .package
            .ok_or_else(|| anyhow::anyhow!("Node missing package"))?;
        let executable = builder
            .executable
            .ok_or_else(|| anyhow::anyhow!("Node missing executable"))?;

        let exec_path = resolve_exec_path(&package, &executable);

        // Resolve namespace from builder or context
        let namespace = if let Some(ns) = &builder.namespace {
            if ns.starts_with('/') {
                if ns == "/" { None } else { Some(ns.clone()) }
            } else if ns.is_empty() {
                let current = self.context.current_namespace();
                if current == "/" { None } else { Some(current) }
            } else {
                let current = self.context.current_namespace();
                let full = if current == "/" {
                    format!("/{ns}")
                } else {
                    format!("{current}/{ns}")
                };
                Some(full)
            }
        } else {
            let current = self.context.current_namespace();
            if current == "/" { None } else { Some(current) }
        };

        let global_params = self.collect_global_params();

        // Get global remappings + node remaps
        let mut remaps = self.context.remappings();
        remaps.extend(builder.remaps);

        // Normalize node params
        let params: Vec<(String, String)> = builder
            .params
            .iter()
            .map(|(k, v)| (k.clone(), normalize_param_value(v)))
            .collect();

        let env = self.collect_merged_env(&builder.env);

        // Parse args
        let args: Option<Vec<String>> = builder.args.as_ref().map(|a| {
            a.split_whitespace().map(|s| s.to_string()).collect()
        }).filter(|v: &Vec<String>| !v.is_empty());

        // Build command
        let empty_gp = Vec::new();
        let gp = global_params.as_deref().unwrap_or(&empty_gp);
        let empty_args = Vec::new();
        let arg_list = args.as_deref().unwrap_or(&empty_args);

        let cmd = build_ros_command(
            &exec_path,
            builder.name.as_deref(),
            namespace.as_deref(),
            gp,
            &params,
            &builder.param_files,
            &remaps,
            arg_list,
        );

        let record = NodeRecord {
            args,
            cmd,
            env,
            exec_name: Some(executable.clone()),
            executable,
            global_params,
            name: builder.name,
            namespace,
            package: Some(package),
            params,
            params_files: builder.param_files,
            remaps,
            respawn: builder.respawn.as_ref().and_then(|s| s.parse::<bool>().ok()),
            respawn_delay: builder.respawn_delay.as_ref().and_then(|s| s.parse::<f64>().ok()),
            ros_args: None,
        };

        self.records.push(record);
        Ok(())
    }

    /// Finalize an executable builder into a NodeRecord.
    pub fn end_executable(&mut self) -> anyhow::Result<()> {
        let builder = self
            .exec_builder
            .take()
            .ok_or_else(|| anyhow::anyhow!("end_executable with no active builder"))?;

        let cmd_str = builder
            .cmd
            .ok_or_else(|| anyhow::anyhow!("Executable missing cmd"))?;

        let mut cmd = vec![cmd_str.clone()];
        cmd.extend(builder.args.iter().cloned());

        let name = builder.name.or_else(|| Some(cmd_str.clone()));

        let env = self.collect_merged_env(&builder.env);
        let global_params = self.collect_global_params();

        let args_list = if builder.args.is_empty() {
            None
        } else {
            Some(builder.args)
        };

        let record = NodeRecord {
            args: args_list,
            cmd,
            env,
            exec_name: name.clone(),
            executable: cmd_str,
            global_params,
            name,
            namespace: None,
            package: None,
            params: Vec::new(),
            params_files: Vec::new(),
            remaps: Vec::new(),
            respawn: None,
            respawn_delay: None,
            ros_args: None,
        };

        self.records.push(record);
        Ok(())
    }

    /// Finalize a container builder into container + load_node records.
    pub fn end_container(&mut self) -> anyhow::Result<()> {
        let builder = self
            .container_builder
            .take()
            .ok_or_else(|| anyhow::anyhow!("end_container with no active builder"))?;

        let package = builder.package.unwrap_or_default();
        let executable = builder.executable.unwrap_or_default();
        let name = builder.name.unwrap_or_default();

        let namespace = if let Some(ns) = &builder.namespace {
            if ns.starts_with('/') {
                ns.clone()
            } else if ns.is_empty() {
                self.context.current_namespace()
            } else {
                let current = self.context.current_namespace();
                if current == "/" {
                    format!("/{ns}")
                } else {
                    format!("{current}/{ns}")
                }
            }
        } else {
            self.context.current_namespace()
        };

        let exec_path = resolve_exec_path(&package, &executable);

        let global_params = self.collect_global_params();

        // Args
        let args: Option<Vec<String>> = builder.args.as_ref().map(|a| {
            a.split_whitespace().map(|s| s.to_string()).collect()
        }).filter(|v: &Vec<String>| !v.is_empty());

        let empty_gp = Vec::new();
        let gp = global_params.as_deref().unwrap_or(&empty_gp);
        let empty_args = Vec::new();
        let arg_list = args.as_deref().unwrap_or(&empty_args);

        let cmd = build_ros_command(
            &exec_path,
            Some(&name),
            Some(&namespace),
            gp,
            &[],
            &[],
            &[],
            arg_list,
        );

        let env = self.collect_merged_env(&[]);

        let container_record = ComposableNodeContainerRecord {
            args,
            cmd,
            env,
            exec_name: Some(executable.clone()),
            executable,
            global_params,
            name: name.clone(),
            namespace: namespace.clone(),
            package,
            params: Vec::new(),
            params_files: Vec::new(),
            remaps: Vec::new(),
            respawn: None,
            respawn_delay: None,
            ros_args: None,
        };

        self.containers.push(container_record);

        // Convert pending composable nodes to LoadNodeRecords
        let comp_nodes = std::mem::take(&mut self.pending_comp_nodes);
        for comp in comp_nodes {
            let target_name = format!("/{namespace}/{name}").replace("//", "/");
            let load = composable_to_load_record(comp, &target_name, &namespace);
            self.load_nodes.push(load);
        }

        Ok(())
    }

    /// Finalize a composable node builder, adding it to pending_comp_nodes.
    pub fn end_composable_node(&mut self) -> anyhow::Result<()> {
        let builder = self
            .comp_node_builder
            .take()
            .ok_or_else(|| anyhow::anyhow!("end_composable_node with no active builder"))?;
        self.pending_comp_nodes.push(builder);
        Ok(())
    }

    /// Finalize load_node builder, converting pending comp nodes to LoadNodeRecords.
    pub fn end_load_node(&mut self) -> anyhow::Result<()> {
        let builder = self
            .load_node_builder
            .take()
            .ok_or_else(|| anyhow::anyhow!("end_load_node with no active builder"))?;

        let namespace = self.context.current_namespace();
        let comp_nodes = std::mem::take(&mut self.pending_comp_nodes);
        for comp in comp_nodes {
            let load = composable_to_load_record(comp, &builder.target, &namespace);
            self.load_nodes.push(load);
        }

        Ok(())
    }

    /// Convert accumulated state into a RecordJson.
    pub fn into_record_json(self) -> RecordJson {
        RecordJson {
            container: self.containers,
            file_data: HashMap::new(),
            lifecycle_node: Vec::new(),
            load_node: self.load_nodes,
            node: self.records,
            variables: self.context.configurations(),
        }
    }
}

/// Convert a completed CompNodeBuilder into a LoadNodeRecord.
fn composable_to_load_record(
    comp: CompNodeBuilder,
    target_container: &str,
    default_namespace: &str,
) -> LoadNodeRecord {
    let namespace = if let Some(ns) = comp.namespace {
        ns
    } else {
        // Extract namespace from target container path (e.g. "/test/my_container" → "/test")
        extract_namespace_from_target(target_container)
            .unwrap_or_else(|| default_namespace.to_string())
    };
    let params: Vec<(String, String)> = comp
        .params
        .into_iter()
        .map(|(k, v)| (k, normalize_param_value(&v)))
        .collect();

    LoadNodeRecord {
        env: None,
        extra_args: comp.extra_args.into_iter().collect(),
        log_level: None,
        namespace,
        node_name: comp.name.unwrap_or_default(),
        package: comp.package.unwrap_or_default(),
        params,
        plugin: comp.plugin.unwrap_or_default(),
        remaps: comp.remaps,
        target_container_name: target_container.to_string(),
    }
}

/// Extract the namespace portion from a target container path.
/// E.g. "/test/my_container" → "/test", "/my_container" → "/".
fn extract_namespace_from_target(target: &str) -> Option<String> {
    if let Some(last_slash_idx) = target.rfind('/') {
        if last_slash_idx == 0 {
            Some("/".to_string())
        } else {
            Some(target[..last_slash_idx].to_string())
        }
    } else {
        None
    }
}
