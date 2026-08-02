//! Mock Python API for ROS 2 launch system

pub mod actions;
pub mod conditions;
pub mod event_handlers;
pub mod launch;
pub mod launch_description_sources;
pub mod launch_ros;
pub mod parameter_descriptions;
pub mod some_substitutions_type;
pub mod substitutions;
pub mod utilities;
pub mod utils;

use pyo3::{PyTypeInfo, prelude::*};

/// Register all mock Python modules in sys.modules
///
/// This creates mock versions of:
/// - `launch`
/// - `launch.actions`
/// - `launch.substitutions`
/// - `launch.conditions`
/// - `launch_ros`
/// - `launch_ros.actions`
/// - `launch_ros.descriptions`
/// - `launch_ros.parameter_descriptions`
pub fn register_modules(py: Python) -> PyResult<()> {
    // CRITICAL: Clear any existing launch.* modules from sys.modules first
    // This prevents stale module objects from being cached
    py.run(
        c"import sys\nfor key in list(sys.modules.keys()):\n    if key.startswith(('launch', 'launch_ros', 'launch_xml')):\n        del sys.modules[key]",
        None,
        None,
    )?;

    // Create launch module
    let launch_mod = PyModule::new(py, "launch")?;
    launch_mod.add_class::<launch::LaunchDescription>()?;

    // Add SomeSubstitutionsType_types_tuple to launch module (for top-level imports)
    let str_type = pyo3::types::PyString::type_object(py);
    let tuple_type = pyo3::types::PyTuple::type_object(py);
    let list_type = pyo3::types::PyList::type_object(py);
    let types_tuple = pyo3::types::PyTuple::new(py, [&str_type, &tuple_type, &list_type])?;
    launch_mod.add("SomeSubstitutionsType_types_tuple", &types_tuple)?;
    // Also add SomeSubstitutionsType as None (it's a type alias used for type hints)
    launch_mod.add("SomeSubstitutionsType", py.None())?;

    // Create launch.actions submodule
    let launch_actions = PyModule::new(py, "launch.actions")?;
    launch_actions.add_class::<actions::DeclareLaunchArgument>()?;
    launch_actions.add_class::<actions::LogInfo>()?;
    launch_actions.add_class::<actions::SetEnvironmentVariable>()?;
    launch_actions.add_class::<actions::UnsetEnvironmentVariable>()?;
    launch_actions.add_class::<actions::GroupAction>()?;
    launch_actions.add_class::<actions::ExecuteProcess>()?;
    launch_actions.add_class::<actions::ExecuteLocal>()?;
    launch_actions.add_class::<actions::TimerAction>()?;
    launch_actions.add_class::<actions::OpaqueCoroutine>()?;
    launch_actions.add_class::<actions::Shutdown>()?;
    launch_actions.add_class::<actions::OpaqueFunction>()?;
    launch_actions.add_class::<actions::IncludeLaunchDescription>()?;
    launch_actions.add_class::<actions::SetLaunchConfiguration>()?;
    launch_actions.add_class::<actions::RegisterEventHandler>()?;
    launch_actions.add_class::<actions::PushEnvironment>()?;
    launch_actions.add_class::<actions::PopEnvironment>()?;
    launch_actions.add_class::<actions::ResetEnvironment>()?;
    launch_actions.add_class::<actions::AppendEnvironmentVariable>()?;
    launch_actions.add_class::<actions::PushLaunchConfigurations>()?;
    launch_actions.add_class::<actions::PopLaunchConfigurations>()?;
    launch_actions.add_class::<actions::ResetLaunchConfigurations>()?;
    launch_actions.add_class::<actions::UnsetLaunchConfiguration>()?;

    // EmitEvent and launch.events.Shutdown are defined as pure Python classes
    // so that Python subclasses can call super().__init__() normally.
    // PyO3 #[pyclass] maps #[new] to __new__, not __init__, which breaks
    // the super().__init__(**kwargs) pattern.
    let locals = pyo3::types::PyDict::new(py);
    py.run(
        c"
class EmitEvent:
    def __init__(self, *, event=None, **kwargs):
        pass
    def __repr__(self):
        return 'EmitEvent(...)'

class _ShutdownEvent:
    def __init__(self, *args, **kwargs):
        pass
    def __repr__(self):
        return 'Shutdown()'
",
        None,
        Some(&locals),
    )?;
    let emit_event_class = locals.get_item("EmitEvent")?.unwrap();
    launch_actions.add("EmitEvent", &emit_event_class)?;

    // Create launch.events submodule (stub event classes for import compatibility)
    // Used by launch files that import:
    //   from launch.events import Shutdown as ShutdownEvent
    let launch_events = PyModule::new(py, "launch.events")?;
    let shutdown_event_class = locals.get_item("_ShutdownEvent")?.unwrap();
    launch_events.add("Shutdown", &shutdown_event_class)?;
    // Set __path__ so Python recognizes launch.events as a package (enables submodule imports)
    launch_events.setattr(
        "__path__",
        pyo3::types::PyList::new(py, Vec::<String>::new())?,
    )?;

    // Create launch.events.process submodule with ProcessExited stub
    let launch_events_process = PyModule::new(py, "launch.events.process")?;
    py.run(
        c"
class ProcessExited:
    def __init__(self, *args, **kwargs):
        pass
",
        None,
        Some(&locals),
    )?;
    let process_exited_class = locals.get_item("ProcessExited")?.unwrap();
    launch_events_process.add("ProcessExited", &process_exited_class)?;
    launch_events.add_submodule(&launch_events_process)?;

    // Create launch.launch_context submodule with LaunchContext stub
    let launch_context_mod = PyModule::new(py, "launch.launch_context")?;
    py.run(
        c"
class LaunchContext:
    def __init__(self, *args, **kwargs):
        pass
",
        None,
        Some(&locals),
    )?;
    let launch_context_class = locals.get_item("LaunchContext")?.unwrap();
    launch_context_mod.add("LaunchContext", &launch_context_class)?;

    // Create launch.substitutions submodule
    let launch_subs = PyModule::new(py, "launch.substitutions")?;
    launch_subs.add_class::<substitutions::LaunchConfiguration>()?;
    launch_subs.add_class::<substitutions::TextSubstitution>()?;
    launch_subs.add_class::<substitutions::PathJoinSubstitution>()?;
    launch_subs.add_class::<substitutions::FindPackageShare>()?;
    launch_subs.add_class::<substitutions::EnvironmentVariable>()?;
    launch_subs.add_class::<substitutions::ThisLaunchFileDir>()?;
    launch_subs.add_class::<substitutions::PythonExpression>()?;
    launch_subs.add_class::<substitutions::Command>()?;
    launch_subs.add_class::<substitutions::NotSubstitution>()?;
    launch_subs.add_class::<substitutions::AndSubstitution>()?;
    launch_subs.add_class::<substitutions::OrSubstitution>()?;
    launch_subs.add_class::<substitutions::EqualsSubstitution>()?;
    launch_subs.add_class::<substitutions::NotEqualsSubstitution>()?;
    launch_subs.add_class::<substitutions::IfElseSubstitution>()?;
    launch_subs.add_class::<substitutions::FileContent>()?;
    launch_subs.add_class::<substitutions::AnonName>()?;
    launch_subs.add_class::<substitutions::BooleanSubstitution>()?;
    launch_subs.add_class::<substitutions::FindExecutable>()?;
    launch_subs.add_class::<substitutions::LaunchLogDir>()?;
    launch_subs.add_class::<substitutions::ThisLaunchFile>()?;

    // Create launch.conditions submodule
    let launch_conditions = PyModule::new(py, "launch.conditions")?;
    launch_conditions.add_class::<conditions::IfCondition>()?;
    launch_conditions.add_class::<conditions::UnlessCondition>()?;
    launch_conditions.add_class::<conditions::LaunchConfigurationEquals>()?;
    launch_conditions.add_class::<conditions::LaunchConfigurationNotEquals>()?;

    // Create launch.event_handlers submodule
    let launch_event_handlers = PyModule::new(py, "launch.event_handlers")?;
    launch_event_handlers.add_class::<event_handlers::OnProcessStart>()?;
    launch_event_handlers.add_class::<event_handlers::OnProcessExit>()?;
    launch_event_handlers.add_class::<event_handlers::OnStateTransition>()?;
    launch_event_handlers.add_class::<event_handlers::OnShutdown>()?;

    // Create launch.launch_description_sources submodule
    let launch_sources = PyModule::new(py, "launch.launch_description_sources")?;
    launch_sources.add_class::<launch_description_sources::PythonLaunchDescriptionSource>()?;
    launch_sources.add_class::<launch_description_sources::XMLLaunchDescriptionSource>()?;
    launch_sources.add_class::<launch_description_sources::YAMLLaunchDescriptionSource>()?;
    launch_sources.add_class::<launch_description_sources::FrontendLaunchDescriptionSource>()?;
    launch_sources.add_class::<launch_description_sources::AnyLaunchDescriptionSource>()?;

    // Create launch.frontend submodule with stub classes for import compatibility
    let launch_frontend = PyModule::new(py, "launch.frontend")?;
    // Add stub Entity class - not used in our static parsing but needed for imports
    let entity_class = py.eval(c"type('Entity', (), {})", None, None)?;
    launch_frontend.add("Entity", &entity_class)?;
    // Add stub Parser class
    let parser_class = py.eval(c"type('Parser', (), {})", None, None)?;
    launch_frontend.add("Parser", &parser_class)?;
    // Add stub functions
    launch_frontend.add("expose_action", py.None())?;
    launch_frontend.add("expose_substitution", py.None())?;

    // Create launch.frontend.type_utils submodule with stub functions
    let launch_frontend_type_utils = PyModule::new(py, "launch.frontend.type_utils")?;
    // Add stub functions that return None - not used in static parsing
    launch_frontend_type_utils.add("check_is_list_entity", py.None())?;
    launch_frontend_type_utils.add("get_data_type", py.None())?;
    launch_frontend_type_utils.add("is_iterable", py.None())?;
    launch_frontend.add_submodule(&launch_frontend_type_utils)?;

    // Create launch.utilities submodule (empty placeholder for imports)
    let launch_utilities = PyModule::new(py, "launch.utilities")?;
    // Create launch.utilities.type_utils submodule with common type aliases
    let launch_utilities_type_utils = PyModule::new(py, "launch.utilities.type_utils")?;
    // Add all type aliases as None - these are just for type hints at runtime
    launch_utilities_type_utils.add("ScalarTypesType", py.None())?;
    launch_utilities_type_utils.add("ListTypesType", py.None())?;
    launch_utilities_type_utils.add("AllowedTypesType", py.None())?;
    launch_utilities_type_utils.add("ScalarValueType", py.None())?;
    launch_utilities_type_utils.add("ListValueType", py.None())?;
    launch_utilities_type_utils.add("SequenceValueType", py.None())?;
    launch_utilities_type_utils.add("AllowedValueType", py.None())?;
    launch_utilities_type_utils.add("SomeScalarType", py.None())?;
    launch_utilities_type_utils.add("SomeSequenceType", py.None())?;
    launch_utilities_type_utils.add("SomeValueType", py.None())?;
    launch_utilities_type_utils.add("NormalizedScalarType", py.None())?;
    launch_utilities_type_utils.add("NormalizedSequenceType", py.None())?;
    launch_utilities_type_utils.add("NormalizedValueType", py.None())?;
    launch_utilities_type_utils.add("StrSomeScalarType", py.None())?;
    launch_utilities_type_utils.add("StrSomeSequenceType", py.None())?;
    launch_utilities_type_utils.add("StrSomeValueType", py.None())?;
    // Add stub functions
    launch_utilities_type_utils.add("get_typed_value", py.None())?;
    launch_utilities_type_utils.add("is_typing_list", py.None())?;
    launch_utilities_type_utils.add("is_valid_scalar_type", py.None())?;
    launch_utilities_type_utils.add("extract_type", py.None())?;
    launch_utilities_type_utils.add("is_instance_of_valid_type", py.None())?;
    launch_utilities_type_utils.add("is_instance_of", py.None())?;
    launch_utilities_type_utils.add("coerce_to_type", py.None())?;
    launch_utilities_type_utils.add("coerce_list", py.None())?;
    launch_utilities_type_utils.add("is_substitution", py.None())?;
    launch_utilities_type_utils.add("normalize_typed_substitution", py.None())?;
    launch_utilities_type_utils.add("is_normalized_substitution", py.None())?;
    launch_utilities_type_utils.add("perform_typed_substitution", py.None())?;
    launch_utilities.add_submodule(&launch_utilities_type_utils)?;

    // Add submodules to parent module as attributes
    launch_mod.add_submodule(&launch_actions)?;
    launch_mod.add_submodule(&launch_events)?;
    launch_mod.add_submodule(&launch_subs)?;
    launch_mod.add_submodule(&launch_conditions)?;
    launch_mod.add_submodule(&launch_event_handlers)?;
    launch_mod.add_submodule(&launch_sources)?;
    launch_mod.add_submodule(&launch_frontend)?;
    launch_mod.add_submodule(&launch_utilities)?;

    // Create launch_ros module
    let launch_ros_mod = PyModule::new(py, "launch_ros")?;

    // Create launch_ros.actions submodule
    let launch_ros_actions = PyModule::new(py, "launch_ros.actions")?;
    launch_ros_actions.add_class::<launch_ros::Node>()?;
    launch_ros_actions.add_class::<launch_ros::ComposableNodeContainer>()?;
    launch_ros_actions.add_class::<launch_ros::SetParameter>()?;
    launch_ros_actions.add_class::<launch_ros::SetParametersFromFile>()?;
    launch_ros_actions.add_class::<launch_ros::LifecycleNode>()?;
    launch_ros_actions.add_class::<launch_ros::LifecycleTransition>()?;
    launch_ros_actions.add_class::<launch_ros::PushRosNamespace>()?;
    launch_ros_actions.add_class::<launch_ros::PopRosNamespace>()?;
    launch_ros_actions.add_class::<launch_ros::LoadComposableNodes>()?;
    launch_ros_actions.add_class::<launch_ros::RosTimer>()?;
    launch_ros_actions.add_class::<launch_ros::SetUseSimTime>()?;
    launch_ros_actions.add_class::<launch_ros::SetRemap>()?;
    launch_ros_actions.add_class::<launch_ros::SetROSLogDir>()?;

    // Create launch_ros.descriptions submodule
    let launch_ros_desc = PyModule::new(py, "launch_ros.descriptions")?;
    launch_ros_desc.add_class::<launch_ros::ComposableNode>()?;

    // Create launch_ros.substitutions submodule
    let launch_ros_subs = PyModule::new(py, "launch_ros.substitutions")?;
    launch_ros_subs.add_class::<substitutions::FindPackageShare>()?;
    launch_ros_subs.add_class::<substitutions::ExecutableInPackage>()?;
    launch_ros_subs.add_class::<substitutions::FindPackage>()?;
    launch_ros_subs.add_class::<substitutions::Parameter>()?;

    // Create launch_ros.parameter_descriptions submodule
    let launch_ros_param_desc = PyModule::new(py, "launch_ros.parameter_descriptions")?;
    launch_ros_param_desc.add_class::<parameter_descriptions::ParameterFile>()?;
    launch_ros_param_desc.add_class::<parameter_descriptions::ParameterValue>()?;

    // Create launch_ros.utilities submodule
    let launch_ros_utilities = PyModule::new(py, "launch_ros.utilities")?;
    launch_ros_utilities.add_function(wrap_pyfunction!(
        utilities::make_namespace_absolute,
        &launch_ros_utilities
    )?)?;
    launch_ros_utilities.add_function(wrap_pyfunction!(
        utilities::prefix_namespace,
        &launch_ros_utilities
    )?)?;

    // Add submodules to parent module as attributes
    launch_ros_mod.add_submodule(&launch_ros_actions)?;
    launch_ros_mod.add_submodule(&launch_ros_desc)?;
    launch_ros_mod.add_submodule(&launch_ros_subs)?;
    launch_ros_mod.add_submodule(&launch_ros_param_desc)?;
    launch_ros_mod.add_submodule(&launch_ros_utilities)?;

    // Also set as direct attributes for Python's attribute access
    launch_ros_mod.add("actions", &launch_ros_actions)?;
    launch_ros_mod.add("descriptions", &launch_ros_desc)?;
    launch_ros_mod.add("substitutions", &launch_ros_subs)?;
    launch_ros_mod.add("parameter_descriptions", &launch_ros_param_desc)?;
    launch_ros_mod.add("utilities", &launch_ros_utilities)?;

    // Register in sys.modules
    let sys = py.import("sys")?;
    let modules = sys.getattr("modules")?;

    // IMPORTANT: Remove any existing launch modules to prevent conflicts with real ROS packages
    // Our mocks must take precedence over any installed launch packages
    let module_names_to_remove = vec![
        "launch",
        "launch.actions",
        "launch.substitutions",
        "launch.conditions",
        "launch.event_handlers",
        "launch.launch_description_sources",
        "launch.frontend",
        "launch.frontend.type_utils",
        "launch.utilities",
        "launch.utilities.type_utils",
        "launch.some_substitutions_type",
        "launch.events",
        "launch.events.process",
        "launch.launch_context",
        "launch_ros",
        "launch_ros.actions",
        "launch_ros.descriptions",
        "launch_ros.substitutions",
        "launch_ros.parameter_descriptions",
        "launch_ros.utilities",
        "launch_xml",
        "launch_xml.launch_description_sources",
        // Also remove internal modules
        "launch.actions.group_action",
        "launch.actions.opaque_function",
    ];

    for name in &module_names_to_remove {
        if let Ok(true) = modules.contains(name) {
            modules.del_item(name).ok(); // Ignore errors if module doesn't exist
        }
    }

    modules.set_item("launch", &launch_mod)?;
    modules.set_item("launch.actions", &launch_actions)?;
    modules.set_item("launch.events", &launch_events)?;
    modules.set_item("launch.events.process", &launch_events_process)?;
    modules.set_item("launch.launch_context", &launch_context_mod)?;
    modules.set_item("launch.substitutions", &launch_subs)?;
    modules.set_item("launch.conditions", &launch_conditions)?;
    modules.set_item("launch.event_handlers", &launch_event_handlers)?;
    modules.set_item("launch.launch_description_sources", &launch_sources)?;
    modules.set_item("launch.frontend", &launch_frontend)?;
    modules.set_item("launch.frontend.type_utils", &launch_frontend_type_utils)?;
    modules.set_item("launch.utilities", &launch_utilities)?;
    modules.set_item("launch.utilities.type_utils", &launch_utilities_type_utils)?;
    modules.set_item("launch_ros", &launch_ros_mod)?;
    modules.set_item("launch_ros.actions", &launch_ros_actions)?;
    modules.set_item("launch_ros.descriptions", &launch_ros_desc)?;
    modules.set_item("launch_ros.substitutions", &launch_ros_subs)?;
    modules.set_item("launch_ros.parameter_descriptions", &launch_ros_param_desc)?;
    modules.set_item("launch_ros.utilities", &launch_ros_utilities)?;

    // Create and register launch.some_substitutions_type submodule
    let launch_some_subs_type = some_substitutions_type::register_module(py)?;
    launch_mod.add_submodule(&launch_some_subs_type)?;
    modules.set_item("launch.some_substitutions_type", &launch_some_subs_type)?;

    // Create launch_xml module
    let launch_xml_mod = PyModule::new(py, "launch_xml")?;

    // Create launch_xml.launch_description_sources submodule
    let launch_xml_sources = PyModule::new(py, "launch_xml.launch_description_sources")?;
    launch_xml_sources.add_class::<launch_description_sources::XMLLaunchDescriptionSource>()?;
    launch_xml_mod.add_submodule(&launch_xml_sources)?;

    // Register launch_xml in sys.modules
    modules.set_item("launch_xml", &launch_xml_mod)?;
    modules.set_item("launch_xml.launch_description_sources", &launch_xml_sources)?;

    Ok(())
}
