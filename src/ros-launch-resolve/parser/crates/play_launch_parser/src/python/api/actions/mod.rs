//! Mock `launch.actions` module classes

#![allow(non_local_definitions)] // pyo3 macros generate non-local impls

mod configuration;
mod declare_argument;
mod group;
mod include;
mod opaque_function;
mod simple_actions;

pub use configuration::{
    AppendEnvironmentVariable, PopEnvironment, PopLaunchConfigurations, PushEnvironment,
    PushLaunchConfigurations, RegisterEventHandler, ResetEnvironment, ResetLaunchConfigurations,
    SetLaunchConfiguration, Shutdown, UnsetLaunchConfiguration,
};
pub use declare_argument::DeclareLaunchArgument;
pub use group::GroupAction;
pub use include::IncludeLaunchDescription;
pub use opaque_function::OpaqueFunction;
pub use simple_actions::{
    ExecuteLocal, ExecuteProcess, LogInfo, OpaqueCoroutine, SetEnvironmentVariable, TimerAction,
    UnsetEnvironmentVariable,
};
