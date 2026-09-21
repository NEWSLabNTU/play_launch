//! Record module for generating record.json

pub mod generator;
pub mod types;

pub use generator::{
    CommandGenerator, build_ros_command, merge_params_with_global, normalize_param_value,
    resolve_exec_path,
};
pub use types::{
    ComposableNodeContainerRecord, DroppedAction, LoadNodeRecord, NodeRecord, RecordJson,
    ScopeEntry, ScopeOrigin, ScopeTable, absolute_path, canonicalize_path,
    extract_package_from_path,
};
