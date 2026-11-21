//! Mock Python API for ROS 2 launch system

pub mod actions;
pub mod conditions;
pub mod event_handlers;
pub(crate) mod exit_probe;
pub mod launch;
pub mod launch_description_sources;
pub mod launch_ros;
pub mod parameter_descriptions;
pub mod some_substitutions_type;
pub mod substitutions;
pub mod utilities;
pub mod utils;
pub mod visit;

use pyo3::{PyTypeInfo, prelude::*};

/// Bind `child` on `parent` under BOTH its dotted `__name__` and its bare
/// final segment.
///
/// pyo3's `add_submodule` binds the child under whatever `__name__` it was
/// created with. These modules are created as `PyModule::new(py,
/// "launch.actions")` so that `sys.modules` and `from launch.actions import X`
/// work — but that made the attribute literally `getattr(launch,
/// "launch.actions")`, leaving `launch.actions` unreachable. Stock launch
/// files written as `import launch` + `launch.actions.RegisterEventHandler(...)`
/// therefore failed with "module 'launch' has no attribute 'actions'", while
/// the `from ... import` spelling of the same thing worked. `demo_nodes_cpp
/// add_two_ints.launch.py` is the canonical example.
fn add_submodule_both_names(
    parent: &Bound<'_, PyModule>,
    child: &Bound<'_, PyModule>,
) -> PyResult<()> {
    parent.add_submodule(child)?;
    let full = child.name()?.extract::<String>()?;
    if let Some((_, bare)) = full.rsplit_once('.') {
        parent.add(bare, child)?;
    }
    Ok(())
}

/// The module `launch_extensions` keeps in `sys.modules`.
const LAUNCH_EXTENSIONS: &str = "_play_launch_launch_extensions";

/// What launch extensions expose, by name: the stand-in for `launch`'s
/// frontend registry (`launch.frontend.expose`) and for
/// `Parser.load_launch_extensions`, after `launch` Jazzy (#0099).
///
/// A module of its own, outside `launch.*`, made once per interpreter:
/// [`register_modules`] rebuilds the stand-ins for every file, while an
/// extension's module is imported once and its decorators run once, so what
/// they register has to outlive the rebuilds. `perform_substitution` is what
/// `crate::extension_impl` calls; the rest mirrors `launch`:
///
/// - a class is exposed through its `parse` class method, and a function as
///   the parse method itself; a substitution class without one is refused,
///   as `launch` refuses it. An action is recorded and never refused — one
///   may inherit `parse` from `launch.Action`, which the stand-in does not
///   define, and pyexec runs an extension's action through its own
///   `execute` (#0095);
/// - extensions load once; one that fails to import is skipped, and why is
///   kept for the error that names a substitution nothing exposes;
/// - `$(name a b)` is `parse([[a], [b]])` — `launch`'s grammar hands one
///   list of substitutions per argument — and `type(**kwargs)` of what it
///   returns, performed in the context.
pub(crate) fn launch_extensions(py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let modules = py.import("sys")?.getattr("modules")?;
    if !modules.contains(LAUNCH_EXTENSIONS)? {
        let module = PyModule::new(py, LAUNCH_EXTENSIONS)?;
        py.run(
            cr#"
import importlib.metadata
import inspect
import traceback

substitutions = {}
actions = {}
failures = []
_loaded = []


def _expose(name, registry, kind):
    def expose(exposed):
        if inspect.isclass(exposed):
            if 'parse' in dir(exposed) and inspect.ismethod(exposed.parse):
                registry[name] = exposed.parse
            elif kind == 'substitution':
                raise RuntimeError(
                    "Did not find a class method called 'parse' in the class being decorated.")
        elif inspect.isfunction(exposed):
            registry[name] = exposed
        elif kind == 'substitution':
            raise RuntimeError(
                f'Exposed {kind} parser for {name} is not a callable or a class'
                ' containing a parse method')
        return exposed
    return expose


def expose_substitution(name):
    return _expose(name, substitutions, 'substitution')


def expose_action(name):
    return _expose(name, actions, 'action')


def load_launch_extensions():
    if _loaded:
        return
    _loaded.append(True)
    for entry_point in importlib.metadata.entry_points(
            group='launch.frontend.launch_extension'):
        try:
            entry_point.load()
        except Exception:
            failures.append(f'{entry_point.name}: {traceback.format_exc()}')


def perform_substitution(name, args, context):
    load_launch_extensions()
    parse = substitutions.get(name)
    if parse is None:
        return None
    from launch.substitutions import TextSubstitution
    subst_type, kwargs = parse([[TextSubstitution(text=arg)] for arg in args])
    return context.perform_substitution(subst_type(**kwargs))
"#,
            Some(&module.dict()),
            None,
        )?;
        modules.set_item(LAUNCH_EXTENSIONS, &module)?;
    }
    py.import(LAUNCH_EXTENSIONS)
}

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
    // What `crate::executor::ensure_mocks` looks for: these stand-ins, not
    // the real package, are what `sys.modules` holds.
    launch_mod.add("__play_launch_mock__", true)?;
    launch_mod.add_class::<launch::LaunchDescription>()?;

    // Add SomeSubstitutionsType_types_tuple to launch module (for top-level imports)
    let str_type = pyo3::types::PyString::type_object(py);
    let tuple_type = pyo3::types::PyTuple::type_object(py);
    let list_type = pyo3::types::PyList::type_object(py);
    let types_tuple = pyo3::types::PyTuple::new(py, [&str_type, &tuple_type, &list_type])?;
    launch_mod.add("SomeSubstitutionsType_types_tuple", &types_tuple)?;
    // Also add SomeSubstitutionsType as None (it's a type alias used for type hints)
    launch_mod.add("SomeSubstitutionsType", py.None())?;

    // Base classes a launch file may SUBCLASS (#0073): `nav2_common.launch`'s
    // `RewrittenYaml`, `ReplaceString` and `HasNodeParams` derive from
    // `launch.Substitution` and annotate with `launch.LaunchContext` and
    // `launch.Condition`, which Python evaluates when the class body runs.
    // Pure Python so `super().__init__()` works (a #[pyclass] `#[new]` is
    // `__new__`). A subclass is performed by `pyobject_to_string`, which
    // hands it the walk's context, as `launch` does.
    let base_locals = pyo3::types::PyDict::new(py);
    py.run(
        c"
class Substitution:
    def __init__(self, *args, **kwargs):
        pass
    def describe(self):
        return repr(self)
    def perform(self, context):
        raise NotImplementedError(type(self).__name__ + '.perform')

class Condition:
    def __init__(self, *, predicate=None):
        self._predicate = predicate
    def evaluate(self, context):
        if self._predicate is None:
            raise NotImplementedError(type(self).__name__ + '.evaluate')
        return bool(self._predicate(context))

class Action:
    def __init__(self, *, condition=None, **kwargs):
        self.condition = condition
    def describe(self):
        return repr(self)
",
        None,
        Some(&base_locals),
    )?;
    for name in ["Substitution", "Condition", "Action"] {
        launch_mod.add(name, base_locals.get_item(name)?.unwrap())?;
    }
    // `launch.substitution`, the module `launch` defines that base in, and
    // where a launch extension imports it from (`from launch.substitution
    // import Substitution`, `ceilbot_profiles`, #0099).
    let launch_substitution = PyModule::new(py, "launch.substitution")?;
    launch_substitution.add(
        "Substitution",
        base_locals.get_item("Substitution")?.unwrap(),
    )?;

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
    def __init__(self, *, event=None, condition=None, **kwargs):
        # Kept so a node's on_exit=EmitEvent(event=Shutdown()) -- or a
        # subclass such as SSv2's ShutdownOnce -- reads as 'shut down'.
        self.event = event
        # An action's condition, which the walk evaluates (#0108).
        self.condition = condition
    def __repr__(self):
        return 'EmitEvent(...)'

class _ShutdownEvent:
    _play_launch_shutdown = True
    def __init__(self, *args, **kwargs):
        pass
    def __repr__(self):
        return 'Shutdown()'

class _MatchesAction:
    # `launch.events.matches_action(action)`: launch's matcher is a lambda
    # comparing an event's action with `action`. Kept as an object so the
    # walk can read which action a `ChangeState` names (#0108).
    def __init__(self, action):
        self.action = action
    def __call__(self, action):
        return action is self.action
    def __repr__(self):
        return 'matches_action(...)'

def matches_action(target_action):
    return _MatchesAction(target_action)

def matches_name(target_name):
    return lambda action: getattr(action, 'name', None) == target_name

class ChangeState:
    # `launch_ros.events.lifecycle.ChangeState`: ask a lifecycle node, picked
    # by `lifecycle_node_matcher`, to take transition `transition_id`.
    _play_launch_change_state = True
    def __init__(self, *, lifecycle_node_matcher, transition_id, **kwargs):
        self.lifecycle_node_matcher = lifecycle_node_matcher
        self.transition_id = transition_id
    def __repr__(self):
        return f'ChangeState(transition_id={self.transition_id!r})'

class StateTransition:
    def __init__(self, *args, **kwargs):
        pass
",
        // As globals, so `matches_action` finds `_MatchesAction`.
        Some(&locals),
        None,
    )?;
    let emit_event_class = locals.get_item("EmitEvent")?.unwrap();
    launch_actions.add("EmitEvent", &emit_event_class)?;

    // Create launch.events submodule (stub event classes for import compatibility)
    // Used by launch files that import:
    //   from launch.events import Shutdown as ShutdownEvent
    let launch_events = PyModule::new(py, "launch.events")?;
    let shutdown_event_class = locals.get_item("_ShutdownEvent")?.unwrap();
    launch_events.add("Shutdown", &shutdown_event_class)?;
    launch_events.add(
        "matches_action",
        locals.get_item("matches_action")?.unwrap(),
    )?;
    launch_events.add("matches_name", locals.get_item("matches_name")?.unwrap())?;
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
    add_submodule_both_names(&launch_events, &launch_events_process)?;

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
    // `launch.LaunchContext` too: an annotation a subclass's `perform` uses.
    launch_mod.add("LaunchContext", &launch_context_class)?;

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
    launch_subs.add_class::<substitutions::AnySubstitution>()?;
    launch_subs.add_class::<substitutions::AllSubstitution>()?;
    launch_subs.add_class::<substitutions::LocalSubstitution>()?;
    launch_subs.add(
        "SubstitutionFailure",
        py.get_type::<utils::SubstitutionFailure>(),
    )?;

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
    // `@expose_action("name")` / `@expose_substitution("name")`: `launch`'s
    // frontend registry. They must be decorators that return what they
    // decorate, or a module defining a launch extension cannot be imported
    // (`autoware_agnocast_wrapper`'s `AutowareNode`, #0095), and they record
    // it, so a substitution only an extension defines can be performed
    // (`$(robot-value …)`, #0099). The record is `launch_extensions`', which
    // outlives these stand-ins.
    let extensions = launch_extensions(py)?;
    for name in ["expose_action", "expose_substitution"] {
        launch_frontend.add(name, extensions.getattr(name)?)?;
    }

    // `launch.logging.get_logger`, which an action's `execute` logs through
    // (#0095). Python's own logging, under launch's logger names.
    let launch_logging = PyModule::new(py, "launch.logging")?;
    py.run(
        c"
import logging as _logging
def get_logger(name=None):
    return _logging.getLogger(name or 'launch')
",
        Some(&launch_logging.dict()),
        None,
    )?;

    // Create launch.frontend.type_utils submodule with stub functions
    let launch_frontend_type_utils = PyModule::new(py, "launch.frontend.type_utils")?;
    // Add stub functions that return None - not used in static parsing
    launch_frontend_type_utils.add("check_is_list_entity", py.None())?;
    launch_frontend_type_utils.add("get_data_type", py.None())?;
    launch_frontend_type_utils.add("is_iterable", py.None())?;
    add_submodule_both_names(&launch_frontend, &launch_frontend_type_utils)?;

    // Create launch.utilities submodule: the two helpers a user-defined
    // substitution calls from its constructor and its `perform` (#0073).
    let launch_utilities = PyModule::new(py, "launch.utilities")?;
    launch_utilities.add_function(wrap_pyfunction!(
        utilities::normalize_to_list_of_substitutions,
        &launch_utilities
    )?)?;
    launch_utilities.add_function(wrap_pyfunction!(
        utilities::perform_substitutions,
        &launch_utilities
    )?)?;
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
    add_submodule_both_names(&launch_utilities, &launch_utilities_type_utils)?;

    // Add submodules to parent module as attributes
    add_submodule_both_names(&launch_mod, &launch_actions)?;
    add_submodule_both_names(&launch_mod, &launch_events)?;
    add_submodule_both_names(&launch_mod, &launch_subs)?;
    add_submodule_both_names(&launch_mod, &launch_conditions)?;
    add_submodule_both_names(&launch_mod, &launch_event_handlers)?;
    add_submodule_both_names(&launch_mod, &launch_sources)?;
    add_submodule_both_names(&launch_mod, &launch_frontend)?;
    add_submodule_both_names(&launch_mod, &launch_utilities)?;
    add_submodule_both_names(&launch_mod, &launch_logging)?;
    add_submodule_both_names(&launch_mod, &launch_substitution)?;

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
    // Jazzy's names (Humble's stay as deprecated aliases there); nav2_bringup
    // on Jazzy imports PushROSNamespace.
    launch_ros_actions.add(
        "PushROSNamespace",
        launch_ros_actions.getattr("PushRosNamespace")?,
    )?;
    launch_ros_actions.add(
        "PopROSNamespace",
        launch_ros_actions.getattr("PopRosNamespace")?,
    )?;
    launch_ros_actions.add_class::<launch_ros::LoadComposableNodes>()?;
    launch_ros_actions.add_class::<launch_ros::RosTimer>()?;
    launch_ros_actions.add_class::<launch_ros::SetUseSimTime>()?;
    launch_ros_actions.add_class::<launch_ros::SetRemap>()?;
    launch_ros_actions.add_class::<launch_ros::SetROSLogDir>()?;

    // Create launch_ros.descriptions submodule
    let launch_ros_desc = PyModule::new(py, "launch_ros.descriptions")?;
    launch_ros_desc.add_class::<launch_ros::ComposableNode>()?;
    // launch_ros.descriptions re-exports these (`from launch_ros.descriptions
    // import ParameterFile` in nav2_bringup on Jazzy).
    launch_ros_desc.add_class::<parameter_descriptions::ParameterFile>()?;
    launch_ros_desc.add_class::<parameter_descriptions::ParameterValue>()?;

    // Create launch_ros.substitutions submodule
    let launch_ros_subs = PyModule::new(py, "launch_ros.substitutions")?;
    launch_ros_subs.add_class::<substitutions::FindPackageShare>()?;
    launch_ros_subs.add_class::<substitutions::ExecutableInPackage>()?;
    launch_ros_subs.add_class::<substitutions::FindPackage>()?;
    // `launch_ros`'s concrete name for the prefix lookup; `FindPackage` is
    // its base class.
    launch_ros_subs.add("FindPackagePrefix", launch_ros_subs.getattr("FindPackage")?)?;
    launch_ros_subs.add_class::<substitutions::Parameter>()?;

    // `launch_ros.ros_adapters.get_ros_node(context)`: the node `launch_ros`
    // runs for actions that talk to the graph, as a launch file's own
    // `OpaqueFunction` may (ceilbot's `nav2_composed.launch.py` waits for its
    // container's `list_nodes`, #0100). A walk models the launch coming up,
    // and has no graph: this node's clients find their service at once and
    // every request is answered at once, so a wait returns where the running
    // launch would have gone on. Nothing is sent anywhere.
    let launch_ros_adapters = PyModule::new(py, "launch_ros.ros_adapters")?;
    py.run(
        cr#"
import logging as _logging


class _Future:
    def done(self):
        return True

    def cancelled(self):
        return False

    def result(self):
        return None

    def exception(self):
        return None

    def add_done_callback(self, callback):
        callback(self)

    def cancel(self):
        return False


class _Client:
    def __init__(self, srv_type, srv_name):
        self.srv_type = srv_type
        self.srv_name = srv_name

    def wait_for_service(self, timeout_sec=None):
        return True

    def service_is_ready(self):
        return True

    def call_async(self, request):
        return _Future()

    def call(self, request, timeout_sec=None):
        return None

    def destroy(self):
        return None


class _Node:
    def get_name(self):
        return 'launch_ros'

    def get_logger(self):
        return _logging.getLogger('launch_ros')

    def create_client(self, srv_type, srv_name, **kwargs):
        return _Client(srv_type, srv_name)

    def destroy_client(self, client):
        return True


_node = _Node()


def get_ros_node(context):
    return _node
"#,
        Some(&launch_ros_adapters.dict()),
        None,
    )?;

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

    // launch_ros.events and launch_ros.events.lifecycle: the ChangeState
    // event a launch file emits to drive a LifecycleNode (#0108), and
    // launch_ros.event_handlers, where upstream's OnStateTransition lives.
    let launch_ros_events = PyModule::new(py, "launch_ros.events")?;
    launch_ros_events.setattr(
        "__path__",
        pyo3::types::PyList::new(py, Vec::<String>::new())?,
    )?;
    let launch_ros_events_lifecycle = PyModule::new(py, "launch_ros.events.lifecycle")?;
    launch_ros_events_lifecycle.add("ChangeState", locals.get_item("ChangeState")?.unwrap())?;
    launch_ros_events_lifecycle.add(
        "StateTransition",
        locals.get_item("StateTransition")?.unwrap(),
    )?;
    add_submodule_both_names(&launch_ros_events, &launch_ros_events_lifecycle)?;
    let launch_ros_event_handlers = PyModule::new(py, "launch_ros.event_handlers")?;
    launch_ros_event_handlers.add_class::<event_handlers::OnStateTransition>()?;

    // Add submodules to parent module as attributes
    add_submodule_both_names(&launch_ros_mod, &launch_ros_events)?;
    add_submodule_both_names(&launch_ros_mod, &launch_ros_event_handlers)?;
    add_submodule_both_names(&launch_ros_mod, &launch_ros_actions)?;
    add_submodule_both_names(&launch_ros_mod, &launch_ros_desc)?;
    add_submodule_both_names(&launch_ros_mod, &launch_ros_subs)?;
    add_submodule_both_names(&launch_ros_mod, &launch_ros_param_desc)?;
    add_submodule_both_names(&launch_ros_mod, &launch_ros_utilities)?;
    add_submodule_both_names(&launch_ros_mod, &launch_ros_adapters)?;

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
        "launch.substitution",
        "launch.conditions",
        "launch.event_handlers",
        "launch.launch_description_sources",
        "launch.frontend",
        "launch.frontend.type_utils",
        "launch.utilities",
        "launch.utilities.type_utils",
        "launch.logging",
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
        "launch_ros.ros_adapters",
        "launch_ros.events",
        "launch_ros.events.lifecycle",
        "launch_ros.event_handlers",
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
    modules.set_item("launch.substitution", &launch_substitution)?;
    modules.set_item("launch.conditions", &launch_conditions)?;
    modules.set_item("launch.event_handlers", &launch_event_handlers)?;
    modules.set_item("launch.launch_description_sources", &launch_sources)?;
    modules.set_item("launch.frontend", &launch_frontend)?;
    modules.set_item("launch.frontend.type_utils", &launch_frontend_type_utils)?;
    modules.set_item("launch.utilities", &launch_utilities)?;
    modules.set_item("launch.logging", &launch_logging)?;
    modules.set_item("launch.utilities.type_utils", &launch_utilities_type_utils)?;
    modules.set_item("launch_ros", &launch_ros_mod)?;
    modules.set_item("launch_ros.actions", &launch_ros_actions)?;
    modules.set_item("launch_ros.descriptions", &launch_ros_desc)?;
    modules.set_item("launch_ros.substitutions", &launch_ros_subs)?;
    modules.set_item("launch_ros.parameter_descriptions", &launch_ros_param_desc)?;
    modules.set_item("launch_ros.utilities", &launch_ros_utilities)?;
    modules.set_item("launch_ros.ros_adapters", &launch_ros_adapters)?;
    modules.set_item("launch_ros.events", &launch_ros_events)?;
    modules.set_item("launch_ros.events.lifecycle", &launch_ros_events_lifecycle)?;
    modules.set_item("launch_ros.event_handlers", &launch_ros_event_handlers)?;

    // Create and register launch.some_substitutions_type submodule
    let launch_some_subs_type = some_substitutions_type::register_module(py)?;
    add_submodule_both_names(&launch_mod, &launch_some_subs_type)?;
    modules.set_item("launch.some_substitutions_type", &launch_some_subs_type)?;

    // Create launch_xml module
    let launch_xml_mod = PyModule::new(py, "launch_xml")?;

    // Create launch_xml.launch_description_sources submodule
    let launch_xml_sources = PyModule::new(py, "launch_xml.launch_description_sources")?;
    launch_xml_sources.add_class::<launch_description_sources::XMLLaunchDescriptionSource>()?;
    add_submodule_both_names(&launch_xml_mod, &launch_xml_sources)?;

    // Register launch_xml in sys.modules
    modules.set_item("launch_xml", &launch_xml_mod)?;
    modules.set_item("launch_xml.launch_description_sources", &launch_xml_sources)?;

    Ok(())
}
