//! Launch context for managing configurations and entity captures
//!
//! This is the unified context used by both XML and Python launch file parsers.
//! It combines substitution resolution (scope chain) with entity capture storage.

use crate::{
    captures::{ContainerCapture, DeclaredArgumentCapture, LoadNodeCapture, NodeCapture},
    substitution::{
        parser::parse_substitutions,
        types::{Substitution, resolve_substitutions},
    },
};
use indexmap::IndexMap;
use std::{cell::Cell, collections::HashMap, path::PathBuf, sync::Arc};

/// Maximum recursion depth for variable resolution to prevent stack overflow
const MAX_RESOLUTION_DEPTH: usize = 20;

// Thread-local storage for tracking resolution depth
thread_local! {
    static RESOLUTION_DEPTH: Cell<usize> = const { Cell::new(0) };
}

/// RAII guard that increments the resolution depth on creation and restores on drop.
/// Ensures the depth counter is always restored even on panic or early return.
struct ResolutionDepthGuard {
    previous: usize,
}

impl ResolutionDepthGuard {
    /// Increment the resolution depth. Returns `None` if max depth exceeded.
    fn try_new() -> Option<Self> {
        RESOLUTION_DEPTH.with(|depth| {
            let current = depth.get();
            if current >= MAX_RESOLUTION_DEPTH {
                None
            } else {
                depth.set(current + 1);
                Some(Self { previous: current })
            }
        })
    }
}

impl Drop for ResolutionDepthGuard {
    fn drop(&mut self) {
        RESOLUTION_DEPTH.with(|depth| {
            depth.set(self.previous);
        });
    }
}

/// Snapshot of the namespace depth for save/restore.
/// Used by `save_scope()` / `restore_scope()` to ensure cleanup on early returns.
pub struct ScopeSnapshot {
    namespace_depth: usize,
}

/// What a scoped group saves on entry and puts back on exit: launch's
/// `PushLaunchConfigurations` + `PushEnvironment`, which `GroupAction` wraps
/// around its body when `scoped` (the default). Only the LOCAL maps are held —
/// a parent scope is immutable, so restoring the local layer restores all of it.
///
/// The namespace and the two global lists are launch configurations too
/// (`ros_namespace`, `ros_remaps`, `global_params`), so they are saved here
/// with the rest. The lists are saved BY REFERENCE, as launch saves them: see
/// [`SharedLists`].
pub struct ConfigurationSnapshot {
    configurations: HashMap<String, Vec<Substitution>>,
    environment: HashMap<String, String>,
    namespace_stack: Vec<String>,
    remap_list: Option<usize>,
    param_list: Option<usize>,
}

/// The two launch configurations `launch_ros` keeps as Python LISTS, held by
/// reference: `ros_remaps` (`<set_remap>`, `SetRemap`) and `global_params`
/// (`<set_parameter>`, `SetParameter`).
///
/// `PushLaunchConfigurations` saves a SHALLOW copy of the configuration dict,
/// and `SetRemap.execute` does `remaps = configs.get('ros_remaps', [])`,
/// `remaps.append(...)`, `configs['ros_remaps'] = remaps`. So a `<set_remap>`
/// inside a scoped group appends to the very list the saved copy holds when
/// one existed before the group — and survives the group's pop — but creates a
/// fresh list, popped with the group, when none did. `ros2 launch` behaves this
/// way on Humble and its nodes run with those remaps; restoring a count on pop,
/// as this context used to, matched neither case.
///
/// Emulated with an arena: each list is an index, a group saves and restores
/// the index, and an append writes through it.
#[derive(Debug, Clone, Default)]
struct SharedLists {
    remaps: Vec<Vec<(String, String)>>,
    params: Vec<Vec<(String, String)>>,
}

/// Metadata for a declared argument
#[derive(Debug, Clone)]
pub struct ArgumentMetadata {
    pub name: String,
    pub default: Option<String>,
    pub description: Option<String>,
    pub choices: Option<Vec<String>>,
}

/// Frozen parent scope - immutable, shared via Arc
/// Used to create a scope chain without expensive cloning
#[derive(Debug, Clone)]
struct ParentScope {
    configurations: HashMap<String, Vec<Substitution>>,
    environment: HashMap<String, String>,
    declared_arguments: HashMap<String, ArgumentMetadata>,
    /// Chain to grandparent scope
    parent: Option<Arc<ParentScope>>,
}

/// Launch context holding configurations, state, and entity captures
///
/// Uses hybrid Arc + Local pattern: parent scope is shared (Arc), local scope is owned.
/// This makes child context creation O(1) instead of O(n).
///
/// Entity captures (nodes, containers, load_nodes, includes) are always local —
/// they are not inherited by child contexts. Child contexts capture locally,
/// and callers merge captures back to the parent after processing includes.
#[derive(Debug, Clone)]
pub struct LaunchContext {
    /// Parent scope (shared, immutable via Arc)
    parent: Option<Arc<ParentScope>>,

    /// Local scope (owned, mutable, initially empty for children)
    /// Store configurations as parsed substitutions for lazy evaluation
    local_configurations: HashMap<String, Vec<Substitution>>,
    local_environment: HashMap<String, String>,
    local_declared_arguments: HashMap<String, ArgumentMetadata>,
    /// Every `ros_remaps` / `global_params` list ever created, and which one
    /// is current (`None`: the configuration is unset). See [`SharedLists`].
    lists: SharedLists,
    remap_list: Option<usize>,
    param_list: Option<usize>,

    /// Always local (not inherited)
    current_file: Option<PathBuf>,
    namespace_stack: Vec<String>,

    /// Entity captures — always local, never inherited by child()
    captured_nodes: Vec<NodeCapture>,
    captured_containers: Vec<ContainerCapture>,
    captured_load_nodes: Vec<LoadNodeCapture>,
    /// `DeclareLaunchArgument`s a `.launch.py` constructed (issue 0030).
    captured_declarations: Vec<DeclaredArgumentCapture>,
    /// How many `OpaqueFunction` executions are on the stack; a declaration
    /// captured while this is non-zero is opaque to launch's include check.
    opaque_depth: usize,
    /// Actions the Python frontend recognised but could not model, as
    /// `(action, detail)`. Drained by `execute_python_file`.
    unsupported_actions: Vec<(String, Option<String>)>,
}

impl LaunchContext {
    pub fn new() -> Self {
        Self {
            parent: None,
            local_configurations: HashMap::new(),
            local_environment: HashMap::new(),
            local_declared_arguments: HashMap::new(),
            lists: SharedLists::default(),
            remap_list: None,
            param_list: None,
            current_file: None,
            namespace_stack: vec!["/".to_string()], // Start with root namespace
            captured_nodes: Vec::new(),
            captured_containers: Vec::new(),
            captured_load_nodes: Vec::new(),
            unsupported_actions: Vec::new(),
            captured_declarations: Vec::new(),
            opaque_depth: 0,
        }
    }

    /// Create a child context with current scope frozen as parent
    /// This is O(1) for Arc clone vs O(n) for full HashMap clone
    /// Enables efficient scope chaining for includes
    pub fn child(&self) -> Self {
        // Freeze current local scope and make it the parent
        let parent = ParentScope {
            configurations: self.local_configurations.clone(),
            environment: self.local_environment.clone(),
            declared_arguments: self.local_declared_arguments.clone(),
            parent: self.parent.clone(), // Arc clone - cheap!
        };

        Self {
            parent: Some(Arc::new(parent)),
            local_configurations: HashMap::new(), // Empty local scope
            local_environment: HashMap::new(),
            local_declared_arguments: HashMap::new(),
            // The lists are held by reference in launch; a child sees the
            // same ones (its own copy of them, since nothing writes back).
            lists: self.lists.clone(),
            remap_list: self.remap_list,
            param_list: self.param_list,
            current_file: None,
            namespace_stack: self.namespace_stack.clone(), // Small vec, acceptable to clone
            // Captures are always local — child starts empty
            captured_nodes: Vec::new(),
            captured_containers: Vec::new(),
            captured_load_nodes: Vec::new(),
            unsupported_actions: Vec::new(),
            captured_declarations: Vec::new(),
            opaque_depth: 0,
        }
    }

    /// Record the launch file currently being parsed.
    ///
    /// The path is absolutized here — once, at the single place every frontend
    /// (XML, YAML, IR, the Python-execution path) funnels through — so that
    /// `$(dirname)`, `$(filename)` and every include resolved against this
    /// file's directory agree with `launch`, which takes `os.path.abspath` of
    /// the launch file's location BEFORE `os.path.dirname`
    /// (`IncludeLaunchDescription._get_launch_file_directory()`). Storing the
    /// path as typed made `$(dirname)` the empty string for a bare filename
    /// and `"."` for `./f.launch.xml` (issue 0034).
    pub fn set_current_file(&mut self, path: PathBuf) {
        self.current_file = Some(crate::record::absolute_path(&path));
    }

    /// Forget the current file — used to restore "no file" after a nested
    /// execution that set one.
    pub fn clear_current_file(&mut self) {
        self.current_file = None;
    }

    pub fn current_file(&self) -> Option<&PathBuf> {
        self.current_file.as_ref()
    }

    pub fn current_dir(&self) -> Option<PathBuf> {
        self.current_file
            .as_ref()
            .and_then(|p| p.parent().map(|p| p.to_path_buf()))
    }

    pub fn current_filename(&self) -> Option<String> {
        self.current_file
            .as_ref()
            .and_then(|p| p.file_name())
            .and_then(|n| n.to_str())
            .map(|s| s.to_string())
    }

    /// Set a configuration value by parsing it as substitutions
    /// This allows variables to contain nested substitutions that are resolved lazily
    /// Always modifies local scope only (doesn't affect parent)
    pub fn set_configuration(&mut self, name: String, value: String) {
        // Parse the value as substitutions
        match parse_substitutions(&value) {
            Ok(subs) => {
                self.local_configurations.insert(name, subs);
            }
            Err(_) => {
                // If parsing fails, store as literal text
                self.local_configurations
                    .insert(name, vec![Substitution::Text(value)]);
            }
        }
    }

    /// Set a configuration to an already-resolved value, stored verbatim.
    ///
    /// `launch` stores the RESULT of performing a substitution, so a value
    /// that happens to contain `$(` text (an escaped `\$(var x)`, an
    /// environment variable, a command's output) is data, not a substitution
    /// to perform again on every read. Values crossing from the Python half
    /// are already resolved, and go through here.
    pub fn set_configuration_literal(&mut self, name: String, value: String) {
        self.local_configurations
            .insert(name, vec![Substitution::Text(value)]);
    }

    /// Whether the configuration is set at all — present, whatever it
    /// resolves to.
    pub fn has_configuration(&self, name: &str) -> bool {
        self.get_configuration_raw(name).is_some()
    }

    /// launch's `UnsetLaunchConfiguration`.
    pub fn unset_configuration(&mut self, name: &str) {
        self.local_configurations.remove(name);
    }

    /// Get a configuration value by resolving its stored substitutions
    /// This resolves any nested substitutions at reference time
    /// If resolution fails, returns None (variable exists but can't be resolved)
    /// Walks parent chain: local first, then parent, then grandparent, etc.
    pub fn get_configuration(&self, name: &str) -> Option<String> {
        // 1. Check local scope first (fast path - O(1))
        if let Some(subs) = self.local_configurations.get(name) {
            return resolve_substitutions(subs, self).ok();
        }

        // 2. Walk parent chain (depth ~5-10 for Autoware)
        let mut current = &self.parent;
        while let Some(parent) = current {
            if let Some(subs) = parent.configurations.get(name) {
                return resolve_substitutions(subs, self).ok();
            }
            current = &parent.parent;
        }

        None
    }

    /// Get a configuration value by resolving its substitutions, with fallback
    /// If resolution fails, constructs an unresolved string representation
    /// This is used for lenient resolution where we want a value even if some
    /// substitutions can't be resolved (e.g., for static analysis)
    ///
    /// To prevent infinite recursion from circular references, this uses a
    /// resolution depth tracker stored in thread-local storage
    /// Walks parent chain: local first, then parent, then grandparent, etc.
    pub fn get_configuration_lenient(&self, name: &str) -> Option<String> {
        // 1. Check local scope first
        if let Some(subs) = self.local_configurations.get(name) {
            return Some(match ResolutionDepthGuard::try_new() {
                None => reconstruct_substitution_string(subs),
                Some(_guard) => resolve_substitutions(subs, self)
                    .unwrap_or_else(|_| reconstruct_substitution_string(subs)),
            });
        }

        // 2. Walk parent chain
        let mut current = &self.parent;
        while let Some(parent) = current {
            if let Some(subs) = parent.configurations.get(name) {
                return Some(match ResolutionDepthGuard::try_new() {
                    None => reconstruct_substitution_string(subs),
                    Some(_guard) => resolve_substitutions(subs, self)
                        .unwrap_or_else(|_| reconstruct_substitution_string(subs)),
                });
            }
            current = &parent.parent;
        }

        None
    }

    /// Get the raw substitutions for a configuration (for debugging/testing)
    /// Walks parent chain: local first, then parent, then grandparent, etc.
    pub fn get_configuration_raw(&self, name: &str) -> Option<&Vec<Substitution>> {
        // 1. Check local scope first
        if let Some(subs) = self.local_configurations.get(name) {
            return Some(subs);
        }

        // 2. Walk parent chain
        let mut current = &self.parent;
        while let Some(parent) = current {
            if let Some(subs) = parent.configurations.get(name) {
                return Some(subs);
            }
            current = &parent.parent;
        }

        None
    }

    pub fn configurations(&self) -> HashMap<String, String> {
        // Return resolved configurations from entire scope chain
        // Walk from root to local, so local values override parent values
        let mut resolved = HashMap::new();

        // 1. Collect all parent scopes into a vec (to iterate from root to child)
        let mut scopes = Vec::new();
        let mut current = &self.parent;
        while let Some(parent) = current {
            scopes.push(parent);
            current = &parent.parent;
        }

        // 2. Apply parent scopes from root to immediate parent
        for parent in scopes.iter().rev() {
            for (key, subs) in &parent.configurations {
                if let Ok(value) = resolve_substitutions(subs, self) {
                    resolved.insert(key.clone(), value);
                }
            }
        }

        // 3. Apply local scope (overrides parent)
        for (key, subs) in &self.local_configurations {
            if let Ok(value) = resolve_substitutions(subs, self) {
                resolved.insert(key.clone(), value);
            }
        }

        resolved
    }

    /// Set environment variable in local scope only
    pub fn set_environment_variable(&mut self, name: String, value: String) {
        self.local_environment.insert(name, value);
    }

    /// Unset environment variable in local scope only
    pub fn unset_environment_variable(&mut self, name: &str) {
        self.local_environment.remove(name);
    }

    /// launch's `PushEnvironment`: what [`Self::pop_environment`] restores.
    pub fn push_environment(&self) -> HashMap<String, String> {
        self.local_environment.clone()
    }

    /// launch's `PopEnvironment`.
    pub fn pop_environment(&mut self, saved: HashMap<String, String>) {
        self.local_environment = saved;
    }

    /// launch's `ResetEnvironment`: back to the environment the launch
    /// started with, i.e. nothing set on top of it.
    pub fn reset_environment(&mut self) {
        self.local_environment.clear();
    }

    /// Get environment variable, walking parent chain
    pub fn get_environment_variable(&self, name: &str) -> Option<String> {
        // 1. Check local scope first
        if let Some(value) = self.local_environment.get(name) {
            return Some(value.clone());
        }

        // 2. Walk parent chain
        let mut current = &self.parent;
        while let Some(parent) = current {
            if let Some(value) = parent.environment.get(name) {
                return Some(value.clone());
            }
            current = &parent.parent;
        }

        None
    }

    /// Get all environment variables from entire scope chain
    pub fn environment(&self) -> HashMap<String, String> {
        // Walk from root to local, so local values override parent values
        let mut result = HashMap::new();

        // 1. Collect all parent scopes
        let mut scopes = Vec::new();
        let mut current = &self.parent;
        while let Some(parent) = current {
            scopes.push(parent);
            current = &parent.parent;
        }

        // 2. Apply parent scopes from root to immediate parent
        for parent in scopes.iter().rev() {
            result.extend(
                parent
                    .environment
                    .iter()
                    .map(|(k, v)| (k.clone(), v.clone())),
            );
        }

        // 3. Apply local scope (overrides parent)
        result.extend(
            self.local_environment
                .iter()
                .map(|(k, v)| (k.clone(), v.clone())),
        );

        result
    }

    /// Append a global topic remapping (`<set_remap>`, `SetRemap`), the way
    /// `SetRemap.execute` does: to the current `ros_remaps` list, creating it
    /// if unset. See [`SharedLists`] for why that distinction matters.
    pub fn add_remapping(&mut self, from: String, to: String) {
        let id = match self.remap_list {
            Some(id) => id,
            None => {
                self.lists.remaps.push(Vec::new());
                let id = self.lists.remaps.len() - 1;
                self.remap_list = Some(id);
                id
            }
        };
        self.lists.remaps[id].push((from, to));
    }

    /// The global remappings in effect, in the order they were set.
    pub fn remappings(&self) -> Vec<(String, String)> {
        self.remap_list
            .map(|id| self.lists.remaps[id].clone())
            .unwrap_or_default()
    }

    /// Save the current namespace depth.
    /// Use with `restore_scope()` to ensure cleanup even on early returns.
    pub fn save_scope(&self) -> ScopeSnapshot {
        ScopeSnapshot {
            namespace_depth: self.namespace_depth(),
        }
    }

    /// Restore the namespace depth from a previously saved snapshot.
    pub fn restore_scope(&mut self, snapshot: ScopeSnapshot) {
        self.restore_namespace_depth(snapshot.namespace_depth);
    }

    /// Save the launch configurations and environment, as launch's
    /// `PushLaunchConfigurations` / `PushEnvironment` do at the top of a
    /// scoped `GroupAction`. Pair with [`Self::pop_launch_configurations`].
    ///
    /// The group is the ONLY thing that scopes configurations in launch: an
    /// `<include>` sets its arguments in the current context and runs the
    /// included description there (`IncludeLaunchDescription.execute`
    /// returns `[SetLaunchConfiguration(..)..., description]`), so whatever
    /// the included file declares or `<let>`s is visible to the includer's
    /// later actions unless a group around the include pops it.
    pub fn push_launch_configurations(&self) -> ConfigurationSnapshot {
        ConfigurationSnapshot {
            configurations: self.local_configurations.clone(),
            environment: self.local_environment.clone(),
            namespace_stack: self.namespace_stack.clone(),
            remap_list: self.remap_list,
            param_list: self.param_list,
        }
    }

    /// launch's `ResetLaunchConfigurations`: every configuration is cleared —
    /// the namespace and both global lists with them, since they are
    /// configurations too — and only `keep` is set again. `GroupAction`
    /// emits it inside its push/pop when `forwarding=False`.
    pub fn reset_launch_configurations(&mut self, keep: Vec<(String, String)>) {
        self.local_configurations.clear();
        self.parent = None;
        self.namespace_stack = vec!["/".to_string()];
        self.remap_list = None;
        self.param_list = None;
        for (k, v) in keep {
            self.set_configuration_literal(k, v);
        }
    }

    /// Take over the configurations and environment a [`Self::child`] set in
    /// its own local scope, as if they had been set here.
    ///
    /// For a traversal that must run in a child context for reasons other
    /// than scoping (the Python→XML include re-prefixes namespaces on what the
    /// child produced) but whose configuration effects `launch` does not
    /// scope.
    pub fn adopt_configurations_from(&mut self, child: &LaunchContext) {
        self.local_configurations.extend(
            child
                .local_configurations
                .iter()
                .map(|(k, v)| (k.clone(), v.clone())),
        );
        self.local_environment.extend(
            child
                .local_environment
                .iter()
                .map(|(k, v)| (k.clone(), v.clone())),
        );
    }

    /// Put back what [`Self::push_launch_configurations`] saved, discarding
    /// every configuration and environment change made since.
    pub fn pop_launch_configurations(&mut self, snapshot: ConfigurationSnapshot) {
        self.local_configurations = snapshot.configurations;
        self.local_environment = snapshot.environment;
        self.namespace_stack = snapshot.namespace_stack;
        self.remap_list = snapshot.remap_list;
        self.param_list = snapshot.param_list;
    }

    /// Declare argument in local scope only
    pub fn declare_argument(&mut self, metadata: ArgumentMetadata) {
        self.local_declared_arguments
            .insert(metadata.name.clone(), metadata);
    }

    /// Get argument metadata, walking parent chain
    pub fn get_argument_metadata(&self, name: &str) -> Option<&ArgumentMetadata> {
        // 1. Check local scope first
        if let Some(metadata) = self.local_declared_arguments.get(name) {
            return Some(metadata);
        }

        // 2. Walk parent chain
        let mut current = &self.parent;
        while let Some(parent) = current {
            if let Some(metadata) = parent.declared_arguments.get(name) {
                return Some(metadata);
            }
            current = &parent.parent;
        }

        None
    }

    /// Get all declared arguments from entire scope chain
    pub fn declared_arguments(&self) -> HashMap<String, ArgumentMetadata> {
        // Walk from root to local, so local values override parent values
        let mut result = HashMap::new();

        // 1. Collect all parent scopes
        let mut scopes = Vec::new();
        let mut current = &self.parent;
        while let Some(parent) = current {
            scopes.push(parent);
            current = &parent.parent;
        }

        // 2. Apply parent scopes from root to immediate parent
        for parent in scopes.iter().rev() {
            result.extend(
                parent
                    .declared_arguments
                    .iter()
                    .map(|(k, v)| (k.clone(), v.clone())),
            );
        }

        // 3. Apply local scope (overrides parent)
        result.extend(
            self.local_declared_arguments
                .iter()
                .map(|(k, v)| (k.clone(), v.clone())),
        );

        result
    }

    /// Append a global parameter (`<set_parameter>`, `SetParameter`) to the
    /// current `global_params` list, creating it if unset — see
    /// [`SharedLists`].
    pub fn set_global_parameter(&mut self, name: String, value: String) {
        let id = match self.param_list {
            Some(id) => id,
            None => {
                self.lists.params.push(Vec::new());
                let id = self.lists.params.len() - 1;
                self.param_list = Some(id);
                id
            }
        };
        self.lists.params[id].push((name, value));
    }

    /// The value a global parameter has now: the last one set.
    pub fn get_global_parameter(&self, name: &str) -> Option<String> {
        self.param_list.and_then(|id| {
            self.lists.params[id]
                .iter()
                .rev()
                .find(|(k, _)| k == name)
                .map(|(_, v)| v.clone())
        })
    }

    /// The global parameters in effect: first-set order, last value wins.
    pub fn global_parameters(&self) -> IndexMap<String, String> {
        let mut result = IndexMap::new();
        if let Some(id) = self.param_list {
            for (k, v) in &self.lists.params[id] {
                result.insert(k.clone(), v.clone());
            }
        }
        result
    }

    // ========== The state a `.launch.py` exchanges (C ABI 7) ==========

    /// Which global lists are current, as a reference point for the next
    /// [`Self::export_state`] / [`Self::import_state`] — see
    /// [`crate::exchange::SharedListState::same`].
    pub fn list_sync(&self) -> crate::exchange::ListSync {
        crate::exchange::ListSync {
            remap: self.remap_list,
            param: self.param_list,
        }
    }

    /// The state a `.launch.py` reads and writes, for sending across the
    /// Python boundary. `since` is this side's reference point for whether a
    /// global list is the same one the other side last saw.
    pub fn export_state(&self, since: &crate::exchange::ListSync) -> crate::exchange::ContextState {
        use crate::exchange::SharedListState;
        let list =
            |current: Option<usize>, synced: Option<usize>, lists: &[Vec<(String, String)>]| {
                current.map(|id| SharedListState {
                    items: lists[id].clone(),
                    same: synced == Some(id),
                })
            };
        crate::exchange::ContextState {
            configurations: self.configurations().into_iter().collect(),
            namespace_stack: self.namespace_stack.clone(),
            remaps: list(self.remap_list, since.remap, &self.lists.remaps),
            global_parameters: list(self.param_list, since.param, &self.lists.params),
            environment: self.environment().into_iter().collect(),
            current_file: self
                .current_file
                .as_ref()
                .and_then(|p| p.to_str().map(String::from)),
        }
    }

    /// Take over a state the other side sent. An include scopes nothing, so
    /// what arrives replaces what is here: every configuration, the namespace,
    /// the environment and both global lists. `since` is this side's
    /// reference point from when it last sent its state; a list the sender
    /// marks `same` is written through the list that was current then (and
    /// that a group's snapshot may still hold), anything else starts a new one.
    /// Returns the new reference point.
    pub fn import_state(
        &mut self,
        state: &crate::exchange::ContextState,
        since: &crate::exchange::ListSync,
    ) -> crate::exchange::ListSync {
        fn adopt(
            lists: &mut Vec<Vec<(String, String)>>,
            incoming: &Option<crate::exchange::SharedListState>,
            synced: Option<usize>,
        ) -> Option<usize> {
            let incoming = incoming.as_ref()?;
            match synced {
                Some(id) if incoming.same => {
                    lists[id] = incoming.items.clone();
                    Some(id)
                }
                _ => {
                    lists.push(incoming.items.clone());
                    Some(lists.len() - 1)
                }
            }
        }
        self.parent = None;
        self.local_configurations.clear();
        for (k, v) in &state.configurations {
            self.set_configuration_literal(k.clone(), v.clone());
        }
        self.namespace_stack = if state.namespace_stack.is_empty() {
            vec!["/".to_string()]
        } else {
            state.namespace_stack.clone()
        };
        self.remap_list = adopt(&mut self.lists.remaps, &state.remaps, since.remap);
        self.param_list = adopt(
            &mut self.lists.params,
            &state.global_parameters,
            since.param,
        );
        self.local_environment = state.environment.clone().into_iter().collect();
        if let Some(f) = &state.current_file {
            self.current_file = Some(PathBuf::from(f));
        }
        self.list_sync()
    }

    /// Push a namespace onto the stack
    pub fn push_namespace(&mut self, namespace: String) {
        let trimmed = namespace.trim();

        if trimmed.is_empty() || trimmed == "/" {
            // Empty or root namespace - don't change the stack
            return;
        }

        // Check if absolute (starts with /) BEFORE normalization
        let is_absolute = trimmed.starts_with('/');

        // Normalize the namespace
        let normalized = normalize_namespace(trimmed);

        if normalized.is_empty() || normalized == "/" {
            return;
        }

        // Get current namespace
        let current = self.current_namespace();

        // Combine namespaces
        let new_ns = if is_absolute {
            // Absolute namespace - use as-is
            normalized
        } else {
            // Relative namespace - append to current
            if current == "/" {
                format!("/{}", normalized)
            } else {
                format!("{}/{}", current, normalized)
            }
        };

        self.namespace_stack.push(new_ns);
    }

    /// Pop a namespace from the stack
    pub fn pop_namespace(&mut self) {
        // Never pop the root namespace
        if self.namespace_stack.len() > 1 {
            self.namespace_stack.pop();
        }
    }

    /// Get the current namespace stack depth
    /// Used to restore namespace state when exiting scopes
    pub fn namespace_depth(&self) -> usize {
        self.namespace_stack.len()
    }

    /// Restore namespace stack to a specific depth
    /// Used to clean up all namespace pushes within a scope
    pub fn restore_namespace_depth(&mut self, depth: usize) {
        // Never shrink below 1 (root namespace must always exist)
        let target_depth = depth.max(1);
        while self.namespace_stack.len() > target_depth {
            self.namespace_stack.pop();
        }
    }

    /// Get the current namespace
    pub fn current_namespace(&self) -> String {
        self.namespace_stack
            .last()
            .cloned()
            .unwrap_or_else(|| "/".to_string())
    }

    /// Get a clone of the namespace stack (for context synchronization)
    pub fn namespace_stack(&self) -> Vec<String> {
        self.namespace_stack.clone()
    }

    /// Set the namespace stack directly (for context synchronization)
    pub fn set_namespace_stack(&mut self, stack: Vec<String>) {
        self.namespace_stack = stack;
    }

    // ========== Entity Capture Methods ==========

    /// Record an action the PYTHON frontend recognised but could not model.
    ///
    /// The Python half runs inside the interpreter with only a thread-local
    /// pointer back here, so this is its one channel for saying "I saw
    /// something I could not represent". The traverser drains it in
    /// `execute_python_file` and stamps the launch file onto each entry.
    pub fn note_unsupported_action(&mut self, action: String, detail: Option<String>) {
        self.unsupported_actions.push((action, detail));
    }

    /// Take (and clear) what the Python frontend could not model.
    pub fn take_unsupported_actions(&mut self) -> Vec<(String, Option<String>)> {
        std::mem::take(&mut self.unsupported_actions)
    }

    /// Capture a node definition
    pub fn capture_node(&mut self, node: NodeCapture) {
        self.captured_nodes.push(node);
    }

    /// Capture a container definition
    pub fn capture_container(&mut self, container: ContainerCapture) {
        self.captured_containers.push(container);
    }

    /// Capture a composable node load operation
    pub fn capture_load_node(&mut self, load_node: LoadNodeCapture) {
        self.captured_load_nodes.push(load_node);
    }

    /// Get captured nodes
    pub fn captured_nodes(&self) -> &[NodeCapture] {
        &self.captured_nodes
    }

    /// Get captured containers
    pub fn captured_containers(&self) -> &[ContainerCapture] {
        &self.captured_containers
    }

    /// Get captured load nodes
    pub fn captured_load_nodes(&self) -> &[LoadNodeCapture] {
        &self.captured_load_nodes
    }

    /// Get mutable reference to captured nodes
    pub fn captured_nodes_mut(&mut self) -> &mut Vec<NodeCapture> {
        &mut self.captured_nodes
    }

    /// Get mutable reference to captured containers
    pub fn captured_containers_mut(&mut self) -> &mut Vec<ContainerCapture> {
        &mut self.captured_containers
    }

    /// Get mutable reference to captured load nodes
    pub fn captured_load_nodes_mut(&mut self) -> &mut Vec<LoadNodeCapture> {
        &mut self.captured_load_nodes
    }

    pub fn capture_declaration(&mut self, declaration: DeclaredArgumentCapture) {
        self.captured_declarations.push(declaration);
    }

    pub fn captured_declarations(&self) -> &[DeclaredArgumentCapture] {
        &self.captured_declarations
    }

    pub fn captured_declarations_mut(&mut self) -> &mut Vec<DeclaredArgumentCapture> {
        &mut self.captured_declarations
    }

    pub fn enter_opaque_function(&mut self) {
        self.opaque_depth += 1;
    }

    pub fn leave_opaque_function(&mut self) {
        self.opaque_depth = self.opaque_depth.saturating_sub(1);
    }

    pub fn in_opaque_function(&self) -> bool {
        self.opaque_depth > 0
    }
}

/// Reconstruct the original string representation of substitutions
/// Used when resolution fails but we still want a string value
fn reconstruct_substitution_string(subs: &[Substitution]) -> String {
    let mut result = String::new();
    for sub in subs {
        match sub {
            Substitution::Text(s) => result.push_str(s),
            Substitution::LaunchConfiguration(name_subs) => {
                result.push_str("$(var ");
                result.push_str(&reconstruct_substitution_string(name_subs));
                result.push(')');
            }
            Substitution::EnvironmentVariable { name, default } => {
                result.push_str("$(env ");
                result.push_str(&reconstruct_substitution_string(name));
                if let Some(def) = default {
                    result.push(' ');
                    result.push_str(&reconstruct_substitution_string(def));
                }
                result.push(')');
            }
            Substitution::OptionalEnvironmentVariable { name, default } => {
                result.push_str("$(optenv ");
                result.push_str(&reconstruct_substitution_string(name));
                if let Some(def) = default {
                    result.push(' ');
                    result.push_str(&reconstruct_substitution_string(def));
                }
                result.push(')');
            }
            Substitution::Command { cmd, error_mode } => {
                result.push_str("$(command ");
                result.push_str(&reconstruct_substitution_string(cmd));
                // Include error mode if not default (Strict)
                if *error_mode != crate::substitution::types::CommandErrorMode::Strict {
                    result.push_str(" '");
                    result.push_str(match error_mode {
                        crate::substitution::types::CommandErrorMode::Warn => "warn",
                        crate::substitution::types::CommandErrorMode::Ignore => "ignore",
                        crate::substitution::types::CommandErrorMode::Strict => "fail",
                        crate::substitution::types::CommandErrorMode::Capture => "capture",
                    });
                    result.push('\'');
                }
                result.push(')');
            }
            Substitution::FindPackageShare(pkg) => {
                result.push_str("$(find-pkg-share ");
                result.push_str(&reconstruct_substitution_string(pkg));
                result.push(')');
            }
            Substitution::Dirname => result.push_str("$(dirname)"),
            Substitution::Filename => result.push_str("$(filename)"),
            Substitution::Anon(name) => {
                result.push_str("$(anon ");
                result.push_str(&reconstruct_substitution_string(name));
                result.push(')');
            }
            Substitution::Eval(expr) => {
                result.push_str("$(eval ");
                result.push_str(&reconstruct_substitution_string(expr));
                result.push(')');
            }
            Substitution::Call { name, args } => {
                result.push_str("$(");
                result.push_str(name);
                for arg in args {
                    result.push(' ');
                    result.push_str(&reconstruct_substitution_string(arg));
                }
                result.push(')');
            }
        }
    }
    result
}

/// Normalize a namespace string
fn normalize_namespace(ns: &str) -> String {
    let trimmed = ns.trim();

    if trimmed.is_empty() {
        return String::new();
    }

    // Remove trailing slashes

    trimmed.trim_end_matches('/').to_string()
}

impl Default for LaunchContext {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_context() {
        let context = LaunchContext::new();
        assert!(context.get_configuration("any").is_none());
    }

    #[test]
    fn test_set_and_get() {
        let mut context = LaunchContext::new();
        context.set_configuration("key".to_string(), "value".to_string());
        assert_eq!(context.get_configuration("key"), Some("value".to_string()));
    }

    #[test]
    fn test_override_configuration() {
        let mut context = LaunchContext::new();
        context.set_configuration("key".to_string(), "value1".to_string());
        context.set_configuration("key".to_string(), "value2".to_string());
        assert_eq!(context.get_configuration("key"), Some("value2".to_string()));
    }

    #[test]
    fn test_namespace_default() {
        let context = LaunchContext::new();
        assert_eq!(context.current_namespace(), "/");
    }

    #[test]
    fn test_push_namespace_relative() {
        let mut context = LaunchContext::new();
        context.push_namespace("ns1".to_string());
        assert_eq!(context.current_namespace(), "/ns1");

        context.push_namespace("ns2".to_string());
        assert_eq!(context.current_namespace(), "/ns1/ns2");
    }

    #[test]
    fn test_push_namespace_absolute() {
        let mut context = LaunchContext::new();
        context.push_namespace("ns1".to_string());
        assert_eq!(context.current_namespace(), "/ns1");

        // Absolute namespace overrides current
        context.push_namespace("/other".to_string());
        assert_eq!(context.current_namespace(), "/other");
    }

    #[test]
    fn test_pop_namespace() {
        let mut context = LaunchContext::new();
        context.push_namespace("ns1".to_string());
        context.push_namespace("ns2".to_string());
        assert_eq!(context.current_namespace(), "/ns1/ns2");

        context.pop_namespace();
        assert_eq!(context.current_namespace(), "/ns1");

        context.pop_namespace();
        assert_eq!(context.current_namespace(), "/");

        // Can't pop root
        context.pop_namespace();
        assert_eq!(context.current_namespace(), "/");
    }

    #[test]
    fn test_namespace_normalization() {
        let mut context = LaunchContext::new();

        // Trailing slashes should be removed
        context.push_namespace("ns1/".to_string());
        assert_eq!(context.current_namespace(), "/ns1");
        context.pop_namespace();

        // Leading slash makes it absolute
        context.push_namespace("/absolute".to_string());
        assert_eq!(context.current_namespace(), "/absolute");
        context.pop_namespace();

        // Empty namespace doesn't change stack
        context.push_namespace("".to_string());
        assert_eq!(context.current_namespace(), "/");

        // Root namespace doesn't change stack
        context.push_namespace("/".to_string());
        assert_eq!(context.current_namespace(), "/");
    }

    #[test]
    fn test_nested_namespaces() {
        let mut context = LaunchContext::new();

        context.push_namespace("robot1".to_string());
        assert_eq!(context.current_namespace(), "/robot1");

        context.push_namespace("sensors".to_string());
        assert_eq!(context.current_namespace(), "/robot1/sensors");

        context.push_namespace("camera".to_string());
        assert_eq!(context.current_namespace(), "/robot1/sensors/camera");

        context.pop_namespace();
        assert_eq!(context.current_namespace(), "/robot1/sensors");

        context.pop_namespace();
        assert_eq!(context.current_namespace(), "/robot1");

        context.pop_namespace();
        assert_eq!(context.current_namespace(), "/");
    }

    #[test]
    fn test_set_environment_variable() {
        let mut context = LaunchContext::new();
        context.set_environment_variable("MY_VAR".to_string(), "my_value".to_string());
        assert_eq!(
            context.get_environment_variable("MY_VAR"),
            Some("my_value".to_string())
        );
    }

    #[test]
    fn test_unset_environment_variable() {
        let mut context = LaunchContext::new();
        context.set_environment_variable("MY_VAR".to_string(), "my_value".to_string());
        assert_eq!(
            context.get_environment_variable("MY_VAR"),
            Some("my_value".to_string())
        );

        context.unset_environment_variable("MY_VAR");
        assert_eq!(context.get_environment_variable("MY_VAR"), None);
    }

    #[test]
    fn test_get_nonexistent_environment_variable() {
        let context = LaunchContext::new();
        assert_eq!(context.get_environment_variable("NONEXISTENT"), None);
    }

    #[test]
    fn test_override_environment_variable() {
        let mut context = LaunchContext::new();
        context.set_environment_variable("MY_VAR".to_string(), "value1".to_string());
        context.set_environment_variable("MY_VAR".to_string(), "value2".to_string());
        assert_eq!(
            context.get_environment_variable("MY_VAR"),
            Some("value2".to_string())
        );
    }

    #[test]
    fn test_declare_argument() {
        let mut context = LaunchContext::new();
        let metadata = ArgumentMetadata {
            name: "my_arg".to_string(),
            default: Some("default_value".to_string()),
            description: Some("Test argument".to_string()),
            choices: None,
        };

        context.declare_argument(metadata);

        let retrieved = context.get_argument_metadata("my_arg").unwrap();
        assert_eq!(retrieved.name, "my_arg");
        assert_eq!(retrieved.default, Some("default_value".to_string()));
        assert_eq!(retrieved.description, Some("Test argument".to_string()));
    }

    #[test]
    fn test_get_nonexistent_argument() {
        let context = LaunchContext::new();
        assert!(context.get_argument_metadata("nonexistent").is_none());
    }

    #[test]
    fn test_declare_argument_with_choices() {
        let mut context = LaunchContext::new();
        let metadata = ArgumentMetadata {
            name: "mode".to_string(),
            default: Some("fast".to_string()),
            description: None,
            choices: Some(vec!["fast".to_string(), "slow".to_string()]),
        };

        context.declare_argument(metadata);

        let retrieved = context.get_argument_metadata("mode").unwrap();
        assert_eq!(
            retrieved.choices,
            Some(vec!["fast".to_string(), "slow".to_string()])
        );
    }

    #[test]
    fn test_set_global_parameter() {
        let mut context = LaunchContext::new();
        context.set_global_parameter("use_sim_time".to_string(), "true".to_string());
        assert_eq!(
            context.get_global_parameter("use_sim_time"),
            Some("true".to_string())
        );
    }

    #[test]
    fn test_get_nonexistent_global_parameter() {
        let context = LaunchContext::new();
        assert_eq!(context.get_global_parameter("nonexistent"), None);
    }

    #[test]
    fn test_override_global_parameter() {
        let mut context = LaunchContext::new();
        context.set_global_parameter("param".to_string(), "value1".to_string());
        context.set_global_parameter("param".to_string(), "value2".to_string());
        assert_eq!(
            context.get_global_parameter("param"),
            Some("value2".to_string())
        );
    }

    // ========== Entity Capture Tests ==========

    #[test]
    fn test_capture_node() {
        let mut context = LaunchContext::new();

        let node = NodeCapture {
            start_delay_secs: None,
            package: "pkg".to_string(),
            executable: "exec".to_string(),
            name: Some("node1".to_string()),
            namespace: Some("/ns".to_string()),
            parameters: Vec::new(),
            params_files: Vec::new(),
            param_sources: Vec::new(),
            remappings: Vec::new(),
            arguments: Vec::new(),
            ros_arguments: Vec::new(),
            env_vars: Vec::new(),
            scope_id: None,
            ..Default::default()
        };

        context.capture_node(node);
        assert_eq!(context.captured_nodes().len(), 1);
        assert_eq!(context.captured_nodes()[0].package, "pkg");
    }

    #[test]
    fn test_capture_multiple_entities() {
        let mut context = LaunchContext::new();

        context.capture_node(NodeCapture {
            start_delay_secs: None,
            package: "pkg1".to_string(),
            executable: "exec1".to_string(),
            name: None,
            namespace: None,
            parameters: Vec::new(),
            params_files: Vec::new(),
            param_sources: Vec::new(),
            remappings: Vec::new(),
            arguments: Vec::new(),
            ros_arguments: Vec::new(),
            env_vars: Vec::new(),
            scope_id: None,
            ..Default::default()
        });

        context.capture_node(NodeCapture {
            start_delay_secs: None,
            package: "pkg2".to_string(),
            executable: "exec2".to_string(),
            name: None,
            namespace: None,
            parameters: Vec::new(),
            params_files: Vec::new(),
            param_sources: Vec::new(),
            remappings: Vec::new(),
            arguments: Vec::new(),
            ros_arguments: Vec::new(),
            env_vars: Vec::new(),
            scope_id: None,
            ..Default::default()
        });

        assert_eq!(context.captured_nodes().len(), 2);
    }

    #[test]
    fn test_capture_container() {
        let mut context = LaunchContext::new();

        context.capture_container(ContainerCapture {
            start_delay_secs: None,
            name: "my_container".to_string(),
            namespace: "/ns".to_string(),
            package: Some("rclcpp_components".to_string()),
            executable: Some("component_container".to_string()),
            cmd: Vec::new(),
            ros_arguments: Vec::new(),
            scope_id: None,
            ..Default::default()
        });

        assert_eq!(context.captured_containers().len(), 1);
        assert_eq!(context.captured_containers()[0].name, "my_container");
    }

    #[test]
    fn test_capture_load_node() {
        let mut context = LaunchContext::new();

        context.capture_load_node(LoadNodeCapture {
            start_delay_secs: None,
            package: "pkg".to_string(),
            plugin: "pkg::MyNode".to_string(),
            target_container_name: "/my_container".to_string(),
            node_name: "my_node".to_string(),
            namespace: "/ns".to_string(),
            parameters: vec![("key".to_string(), "value".to_string())],
            remappings: Vec::new(),
            extra_args: Default::default(),
            scope_id: None,
            ..Default::default()
        });

        assert_eq!(context.captured_load_nodes().len(), 1);
        assert_eq!(context.captured_load_nodes()[0].node_name, "my_node");
    }

    #[test]
    fn test_child_does_not_inherit_captures() {
        let mut context = LaunchContext::new();

        // Add captures to parent
        context.capture_node(NodeCapture {
            start_delay_secs: None,
            package: "parent_pkg".to_string(),
            executable: "parent_exec".to_string(),
            name: None,
            namespace: None,
            parameters: Vec::new(),
            params_files: Vec::new(),
            param_sources: Vec::new(),
            remappings: Vec::new(),
            arguments: Vec::new(),
            ros_arguments: Vec::new(),
            env_vars: Vec::new(),
            scope_id: None,
            ..Default::default()
        });
        assert_eq!(context.captured_nodes().len(), 1);

        // Create child — captures should NOT be inherited
        let child = context.child();
        assert_eq!(child.captured_nodes().len(), 0);
        assert_eq!(child.captured_containers().len(), 0);
        assert_eq!(child.captured_load_nodes().len(), 0);

        // Parent captures still exist
        assert_eq!(context.captured_nodes().len(), 1);
    }
}
