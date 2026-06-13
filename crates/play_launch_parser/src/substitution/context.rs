//! Launch context for managing configurations and entity captures
//!
//! This is the unified context used by both XML and Python launch file parsers.
//! It combines substitution resolution (scope chain) with entity capture storage.

use crate::{
    captures::{ContainerCapture, IncludeCapture, LoadNodeCapture, NodeCapture},
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

/// Snapshot of scoped state (namespace depth + remapping count) for save/restore.
/// Used by `save_scope()` / `restore_scope()` to ensure cleanup on early returns.
pub struct ScopeSnapshot {
    namespace_depth: usize,
    remapping_count: usize,
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
    global_parameters: IndexMap<String, String>,
    remappings: Vec<(String, String)>,
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
    local_global_parameters: IndexMap<String, String>,
    /// Local topic remappings (from -> to)
    local_remappings: Vec<(String, String)>,

    /// Always local (not inherited)
    current_file: Option<PathBuf>,
    namespace_stack: Vec<String>,

    /// Entity captures — always local, never inherited by child()
    captured_nodes: Vec<NodeCapture>,
    captured_containers: Vec<ContainerCapture>,
    captured_load_nodes: Vec<LoadNodeCapture>,
    captured_includes: Vec<IncludeCapture>,
}

impl LaunchContext {
    pub fn new() -> Self {
        Self {
            parent: None,
            local_configurations: HashMap::new(),
            local_environment: HashMap::new(),
            local_declared_arguments: HashMap::new(),
            local_global_parameters: IndexMap::new(),
            local_remappings: Vec::new(),
            current_file: None,
            namespace_stack: vec!["/".to_string()], // Start with root namespace
            captured_nodes: Vec::new(),
            captured_containers: Vec::new(),
            captured_load_nodes: Vec::new(),
            captured_includes: Vec::new(),
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
            global_parameters: self.local_global_parameters.clone(),
            remappings: self.local_remappings.clone(),
            parent: self.parent.clone(), // Arc clone - cheap!
        };

        Self {
            parent: Some(Arc::new(parent)),
            local_configurations: HashMap::new(), // Empty local scope
            local_environment: HashMap::new(),
            local_declared_arguments: HashMap::new(),
            local_global_parameters: IndexMap::new(),
            local_remappings: Vec::new(),
            current_file: None,
            namespace_stack: self.namespace_stack.clone(), // Small vec, acceptable to clone
            // Captures are always local — child starts empty
            captured_nodes: Vec::new(),
            captured_containers: Vec::new(),
            captured_load_nodes: Vec::new(),
            captured_includes: Vec::new(),
        }
    }

    pub fn set_current_file(&mut self, path: PathBuf) {
        self.current_file = Some(path);
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

    /// Add a global topic remapping to local scope
    pub fn add_remapping(&mut self, from: String, to: String) {
        self.local_remappings.push((from, to));
    }

    /// Get all global remappings from entire scope chain
    pub fn remappings(&self) -> Vec<(String, String)> {
        // Collect remappings from parent chain, then local
        let mut result = Vec::new();

        // 1. Collect all parent scopes
        let mut scopes = Vec::new();
        let mut current = &self.parent;
        while let Some(parent) = current {
            scopes.push(parent);
            current = &parent.parent;
        }

        // 2. Apply parent remappings from root to immediate parent
        for parent in scopes.iter().rev() {
            result.extend_from_slice(&parent.remappings);
        }

        // 3. Apply local remappings
        result.extend_from_slice(&self.local_remappings);

        result
    }

    /// Get current count of local remappings (for scope restoration)
    pub fn remapping_count(&self) -> usize {
        self.local_remappings.len()
    }

    /// Restore local remappings to a specific count
    /// Used to clean up all remappings added within a scope (e.g., group)
    pub fn restore_remapping_count(&mut self, count: usize) {
        self.local_remappings.truncate(count);
    }

    /// Save the current scope state (namespace depth + remapping count).
    /// Use with `restore_scope()` to ensure cleanup even on early returns.
    pub fn save_scope(&self) -> ScopeSnapshot {
        ScopeSnapshot {
            namespace_depth: self.namespace_depth(),
            remapping_count: self.remapping_count(),
        }
    }

    /// Restore scope state from a previously saved snapshot.
    pub fn restore_scope(&mut self, snapshot: ScopeSnapshot) {
        self.restore_namespace_depth(snapshot.namespace_depth);
        self.restore_remapping_count(snapshot.remapping_count);
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

    /// Set global parameter in local scope only
    pub fn set_global_parameter(&mut self, name: String, value: String) {
        self.local_global_parameters.insert(name, value);
    }

    /// Get global parameter, walking parent chain
    pub fn get_global_parameter(&self, name: &str) -> Option<String> {
        // 1. Check local scope first
        if let Some(value) = self.local_global_parameters.get(name) {
            return Some(value.clone());
        }

        // 2. Walk parent chain
        let mut current = &self.parent;
        while let Some(parent) = current {
            if let Some(value) = parent.global_parameters.get(name) {
                return Some(value.clone());
            }
            current = &parent.parent;
        }

        None
    }

    /// Get all global parameters from entire scope chain
    pub fn global_parameters(&self) -> IndexMap<String, String> {
        // Walk from root to local, so local values override parent values
        let mut result = IndexMap::new();

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
                    .global_parameters
                    .iter()
                    .map(|(k, v)| (k.clone(), v.clone())),
            );
        }

        // 3. Apply local scope (overrides parent)
        result.extend(
            self.local_global_parameters
                .iter()
                .map(|(k, v)| (k.clone(), v.clone())),
        );

        result
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

    /// Capture an include operation
    pub fn capture_include(&mut self, include: IncludeCapture) {
        self.captured_includes.push(include);
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

    /// Get captured includes
    pub fn captured_includes(&self) -> &[IncludeCapture] {
        &self.captured_includes
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

    /// Get mutable reference to captured includes
    pub fn captured_includes_mut(&mut self) -> &mut Vec<IncludeCapture> {
        &mut self.captured_includes
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
                        crate::substitution::types::CommandErrorMode::Strict => "strict",
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
            package: "pkg".to_string(),
            executable: "exec".to_string(),
            name: Some("node1".to_string()),
            namespace: Some("/ns".to_string()),
            machine: None,
            parameters: Vec::new(),
            params_files: Vec::new(),
            remappings: Vec::new(),
            arguments: Vec::new(),
            env_vars: Vec::new(),
            scope_id: None,
        };

        context.capture_node(node);
        assert_eq!(context.captured_nodes().len(), 1);
        assert_eq!(context.captured_nodes()[0].package, "pkg");
    }

    #[test]
    fn test_capture_multiple_entities() {
        let mut context = LaunchContext::new();

        context.capture_node(NodeCapture {
            package: "pkg1".to_string(),
            executable: "exec1".to_string(),
            name: None,
            namespace: None,
            machine: None,
            parameters: Vec::new(),
            params_files: Vec::new(),
            remappings: Vec::new(),
            arguments: Vec::new(),
            env_vars: Vec::new(),
            scope_id: None,
        });

        context.capture_node(NodeCapture {
            package: "pkg2".to_string(),
            executable: "exec2".to_string(),
            name: None,
            namespace: None,
            machine: None,
            parameters: Vec::new(),
            params_files: Vec::new(),
            remappings: Vec::new(),
            arguments: Vec::new(),
            env_vars: Vec::new(),
            scope_id: None,
        });

        assert_eq!(context.captured_nodes().len(), 2);
    }

    #[test]
    fn test_capture_container() {
        let mut context = LaunchContext::new();

        context.capture_container(ContainerCapture {
            name: "my_container".to_string(),
            namespace: "/ns".to_string(),
            package: Some("rclcpp_components".to_string()),
            executable: Some("component_container".to_string()),
            cmd: Vec::new(),
            scope_id: None,
        });

        assert_eq!(context.captured_containers().len(), 1);
        assert_eq!(context.captured_containers()[0].name, "my_container");
    }

    #[test]
    fn test_capture_load_node() {
        let mut context = LaunchContext::new();

        context.capture_load_node(LoadNodeCapture {
            package: "pkg".to_string(),
            plugin: "pkg::MyNode".to_string(),
            target_container_name: "/my_container".to_string(),
            node_name: "my_node".to_string(),
            namespace: "/ns".to_string(),
            parameters: vec![("key".to_string(), "value".to_string())],
            remappings: Vec::new(),
            scope_id: None,
        });

        assert_eq!(context.captured_load_nodes().len(), 1);
        assert_eq!(context.captured_load_nodes()[0].node_name, "my_node");
    }

    #[test]
    fn test_capture_include() {
        let mut context = LaunchContext::new();

        context.capture_include(IncludeCapture {
            file_path: "/path/to/file.launch.xml".to_string(),
            args: vec![("arg1".to_string(), "val1".to_string())],
            ros_namespace: "/ns".to_string(),
        });

        assert_eq!(context.captured_includes().len(), 1);
        assert_eq!(
            context.captured_includes()[0].file_path,
            "/path/to/file.launch.xml"
        );
    }

    #[test]
    fn test_child_does_not_inherit_captures() {
        let mut context = LaunchContext::new();

        // Add captures to parent
        context.capture_node(NodeCapture {
            package: "parent_pkg".to_string(),
            executable: "parent_exec".to_string(),
            name: None,
            namespace: None,
            machine: None,
            parameters: Vec::new(),
            params_files: Vec::new(),
            remappings: Vec::new(),
            arguments: Vec::new(),
            env_vars: Vec::new(),
            scope_id: None,
        });
        assert_eq!(context.captured_nodes().len(), 1);

        // Create child — captures should NOT be inherited
        let child = context.child();
        assert_eq!(child.captured_nodes().len(), 0);
        assert_eq!(child.captured_containers().len(), 0);
        assert_eq!(child.captured_load_nodes().len(), 0);
        assert_eq!(child.captured_includes().len(), 0);

        // Parent captures still exist
        assert_eq!(context.captured_nodes().len(), 1);
    }

    #[test]
    fn test_captured_includes_mut_clear() {
        let mut context = LaunchContext::new();

        context.capture_include(IncludeCapture {
            file_path: "file1.xml".to_string(),
            args: Vec::new(),
            ros_namespace: String::new(),
        });
        context.capture_include(IncludeCapture {
            file_path: "file2.xml".to_string(),
            args: Vec::new(),
            ros_namespace: String::new(),
        });
        assert_eq!(context.captured_includes().len(), 2);

        // Clear includes (used after processing)
        context.captured_includes_mut().clear();
        assert_eq!(context.captured_includes().len(), 0);
    }
}
