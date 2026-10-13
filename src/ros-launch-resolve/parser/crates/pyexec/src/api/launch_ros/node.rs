//! Mock `launch_ros.actions.Node` (and what `LifecycleNode` and
//! `ComposableNodeContainer` share with it).
//!
//! A node records its arguments when constructed and is captured when the
//! walk EXECUTES it (`api::visit`), with what `launch_ros` applies at that
//! moment: the `ros_namespace` pushed so far, the global parameters
//! (`SetParameter`) and remappings (`SetRemap`) in effect, ahead of its own,
//! and the environment `SetEnvironmentVariable` built up.

use super::helpers::is_yaml_file;
use crate::api::utils::pyobject_to_string;
use play_launch_parser::{bridge::with_launch_context, captures::NodeCapture};
use pyo3::{
    prelude::*,
    types::{PyDict, PyList},
};

/// Everything a `Node` was constructed with, performed only when it executes.
#[derive(Clone)]
pub(crate) struct NodeSpec {
    pub package: Option<Py<PyAny>>,
    pub executable: Option<Py<PyAny>>,
    pub name: Option<Py<PyAny>>,
    pub namespace: Option<Py<PyAny>>,
    pub parameters: Option<Py<PyAny>>,
    pub remappings: Option<Py<PyAny>>,
    pub arguments: Option<Py<PyAny>>,
    pub ros_arguments: Option<Py<PyAny>>,
    pub env: Option<Py<PyAny>>,
    pub additional_env: Option<Py<PyAny>>,
    /// `respawn=` / `respawn_delay=` (#0070), from the constructor's
    /// `**kwargs`: `launch_ros.actions.Node` forwards them to
    /// `ExecuteProcess`. See [`RespawnArgs`].
    pub respawn: RespawnArgs,
}

/// `respawn=` and `respawn_delay=` as an `ExecuteProcess` (and so every
/// `Node`, `LifecycleNode` and `ComposableNodeContainer`) was constructed
/// with them, performed only when the action executes (#0070).
#[derive(Clone, Default)]
pub(crate) struct RespawnArgs {
    respawn: Option<Py<PyAny>>,
    respawn_delay: Option<Py<PyAny>>,
}

impl RespawnArgs {
    /// Read both from a constructor's `**kwargs`. A Python `None` is the
    /// same as not passing the argument.
    pub(crate) fn from_kwargs(kwargs: Option<&Bound<'_, PyDict>>) -> Self {
        let get = |key: &str| {
            kwargs
                .and_then(|k| k.get_item(key).ok().flatten())
                .filter(|v| !v.is_none())
                .map(Bound::unbind)
        };
        Self {
            respawn: get("respawn"),
            respawn_delay: get("respawn_delay"),
        }
    }

    /// `(respawn, respawn_delay)` as the record carries them.
    ///
    /// `respawn` is `perform_typed_substitution(context, respawn, bool)`: a
    /// Python bool as is, anything else performed and coerced the way
    /// `launch` coerces to `bool`, and a value `launch` would refuse raises.
    /// Only `true` is kept -- false and unset are both `None`, as the Python
    /// dumper and the XML container record them, so the two parsers'
    /// models compare equal. `respawn_delay` is a float in `launch` (no
    /// substitution); it is kept as given.
    pub(crate) fn resolve(&self, py: Python, what: &str) -> PyResult<(Option<bool>, Option<f64>)> {
        let respawn = match &self.respawn {
            None => None,
            Some(obj) => {
                let value = if let Ok(b) = obj.bind(py).cast::<pyo3::types::PyBool>() {
                    b.is_true()
                } else {
                    let text = crate::api::utils::resolve_tokens(&pyobject_to_string(py, obj)?);
                    play_launch_parser::actions::node::coerce_launch_bool(text.trim()).ok_or_else(
                        || {
                            pyo3::exceptions::PyValueError::new_err(format!(
                                "{what}(respawn={text:?}): cannot convert to bool"
                            ))
                        },
                    )?
                };
                value.then_some(true)
            }
        };
        let respawn_delay = match &self.respawn_delay {
            None => None,
            Some(obj) => Some(match obj.bind(py).extract::<f64>() {
                Ok(f) => f,
                Err(_) => {
                    let text = crate::api::utils::resolve_tokens(&pyobject_to_string(py, obj)?);
                    text.trim().parse::<f64>().map_err(|_| {
                        pyo3::exceptions::PyValueError::new_err(format!(
                            "{what}(respawn_delay={text:?}): not a number"
                        ))
                    })?
                }
            }),
        };
        Ok((respawn, respawn_delay))
    }
}

/// A node's identity and command-line contents, performed.
pub(crate) struct ExpandedNode {
    pub package: String,
    pub executable: String,
    pub name: Option<String>,
    /// `None` when neither the node nor the context names one.
    pub namespace: Option<String>,
    pub parameters: Vec<(String, String)>,
    pub params_files: Vec<String>,
    pub remappings: Vec<(String, String)>,
    pub arguments: Vec<String>,
    pub ros_arguments: Vec<String>,
    pub env: Vec<(String, String)>,
    pub global_params: Vec<(String, String)>,
}

/// `launch_ros.utilities.prefix_namespace`.
pub(crate) fn prefix_namespace(base: Option<&str>, ns: Option<&str>) -> Option<String> {
    if base.is_none() && ns.is_none() {
        return None;
    }
    let combined = match ns {
        None => base.unwrap_or_default().to_string(),
        Some(ns) if base.is_none_or(str::is_empty) || ns.starts_with('/') => ns.to_string(),
        Some(ns) => {
            let base = base.unwrap_or_default();
            let base = if base == "/" { "" } else { base };
            format!("{base}/{ns}")
        }
    };
    Some(if combined == "/" {
        combined
    } else {
        combined.trim_end_matches('/').to_string()
    })
}

/// `launch_ros.utilities.make_namespace_absolute`.
pub(crate) fn make_namespace_absolute(ns: Option<String>) -> Option<String> {
    ns.map(|ns| {
        if ns.starts_with('/') {
            ns
        } else {
            format!("/{ns}")
        }
    })
}

/// `context.launch_configurations.get('ros_namespace')`.
pub(crate) fn ros_namespace() -> Option<String> {
    let ns = with_launch_context(|ctx| ctx.current_namespace());
    (ns != "/").then_some(ns)
}

/// A node's namespace as `Node._perform_substitutions` expands it.
pub(crate) fn expand_namespace(py: Python, ns: Option<&Py<PyAny>>) -> PyResult<Option<String>> {
    let own = ns.map(|n| pyobject_to_string(py, n)).transpose()?;
    let base = ros_namespace();
    Ok(make_namespace_absolute(prefix_namespace(
        base.as_deref(),
        own.as_deref(),
    )))
}

fn strings(py: Python, list: Option<&Py<PyAny>>) -> PyResult<Vec<String>> {
    let mut out = Vec::new();
    if let Some(list) = list {
        let list = list.bind(py);
        if list.is_none() {
            return Ok(out);
        }
        for item in list.try_iter()? {
            out.push(pyobject_to_string(py, &item?.unbind())?);
        }
    }
    Ok(out)
}

/// `remappings=[(from, to), ...]`, performed.
pub(crate) fn remappings(py: Python, list: Option<&Py<PyAny>>) -> PyResult<Vec<(String, String)>> {
    let mut out = Vec::new();
    if let Some(list) = list {
        let list = list.bind(py);
        if list.is_none() {
            return Ok(out);
        }
        for item in list.try_iter()? {
            let (from, to): (Py<PyAny>, Py<PyAny>) = item?.extract()?;
            out.push((pyobject_to_string(py, &from)?, pyobject_to_string(py, &to)?));
        }
    }
    Ok(out)
}

fn env_pairs(py: Python, mapping: Option<&Py<PyAny>>) -> PyResult<Vec<(String, String)>> {
    let mut out = Vec::new();
    if let Some(mapping) = mapping {
        let mapping = mapping.bind(py);
        if mapping.is_none() {
            return Ok(out);
        }
        let items = if let Ok(d) = mapping.cast::<PyDict>() {
            d.items().into_any()
        } else {
            mapping.clone()
        };
        for item in items.try_iter()? {
            let (k, v): (Py<PyAny>, Py<PyAny>) = item?.extract()?;
            out.push((pyobject_to_string(py, &k)?, pyobject_to_string(py, &v)?));
        }
    }
    Ok(out)
}

impl NodeSpec {
    /// Perform everything, in the context the walk is in now.
    pub(crate) fn expand(&self, py: Python) -> PyResult<ExpandedNode> {
        let package = self
            .package
            .as_ref()
            .map(|p| pyobject_to_string(py, p))
            .transpose()?
            .unwrap_or_default();
        let executable = self
            .executable
            .as_ref()
            .map(|e| pyobject_to_string(py, e))
            .transpose()?
            .unwrap_or_default();
        let name = self
            .name
            .as_ref()
            .map(|n| pyobject_to_string(py, n))
            .transpose()?;
        let namespace = expand_namespace(py, self.namespace.as_ref())?;

        let mut parameters = Vec::new();
        let mut params_files = Vec::new();
        if let Some(params) = &self.parameters {
            let params = params.bind(py);
            if !params.is_none() {
                for item in params.try_iter()? {
                    for (key, value) in parse_parameter_item(&item?)? {
                        if key == "__param_file" {
                            params_files.push(value);
                        } else {
                            parameters.push((key, value));
                        }
                    }
                }
            }
        }

        // `ros_remaps` first, then the node's own.
        let mut all_remaps = with_launch_context(|ctx| ctx.remappings());
        all_remaps.extend(remappings(py, self.remappings.as_ref())?);

        // `env=` replaces the environment; otherwise the process inherits
        // the launch's, with `additional_env` on top.
        let mut env = if self.env.is_some() {
            env_pairs(py, self.env.as_ref())?
        } else {
            with_launch_context(|ctx| ctx.environment().into_iter().collect())
        };
        env.extend(env_pairs(py, self.additional_env.as_ref())?);

        Ok(ExpandedNode {
            package,
            executable,
            name,
            namespace,
            parameters,
            params_files,
            remappings: all_remaps,
            arguments: strings(py, self.arguments.as_ref())?,
            ros_arguments: strings(py, self.ros_arguments.as_ref())?,
            env,
            global_params: with_launch_context(|ctx| ctx.global_parameters())
                .into_iter()
                .collect(),
        })
    }

    /// Capture this node as executed now. Every caller passes its own
    /// `on_exit` (#0064): `LifecycleNode` and containers are nodes too.
    /// `lifecycle` is `Some(autostart)` for a `LifecycleNode` (phase 86) —
    /// the marker, with its resolved `autostart` (`None` when not given) —
    /// and `None` for a plain `Node`.
    pub(crate) fn capture_with(
        &self,
        py: Python,
        on_exit_shutdown: Option<bool>,
        lifecycle: Option<Option<bool>>,
    ) -> PyResult<ExpandedNode> {
        let n = self.expand(py)?;
        let what = if lifecycle.is_some() {
            "LifecycleNode"
        } else {
            "Node"
        };
        let (respawn, respawn_delay) = self.respawn.resolve(py, what)?;
        let mark = crate::api::visit::lens();
        play_launch_parser::bridge::capture_node(NodeCapture {
            package: n.package.clone(),
            executable: n.executable.clone(),
            name: n.name.clone(),
            namespace: n.namespace.clone(),
            parameters: n.parameters.clone(),
            params_files: n.params_files.clone(),
            param_sources: Vec::new(),
            remappings: n.remappings.clone(),
            arguments: n.arguments.clone(),
            ros_arguments: n.ros_arguments.clone(),
            env_vars: n.env.clone(),
            global_params: Some(n.global_params.clone()),
            scope_id: None,
            start_delay_secs: None,
            on_exit_shutdown,
            lifecycle: lifecycle.map(|_| true),
            lifecycle_autostart: lifecycle.flatten(),
            respawn,
            respawn_delay,
        });
        crate::api::visit::stamp_delay(mark);
        Ok(n)
    }
}

/// One element of a `parameters=[...]` list, as `(name, value)` pairs, with
/// a parameter FILE as `("__param_file", path)`.
pub(crate) fn parse_parameter_item(item: &Bound<'_, PyAny>) -> PyResult<Vec<(String, String)>> {
    let py = item.py();
    let mut parsed = Vec::new();
    if let Ok(dict) = item.cast::<PyDict>() {
        Node::parse_dict_params(dict, "", &mut parsed)?;
        return Ok(parsed);
    }
    if let Ok(list) = item.cast::<PyList>()
        && list.iter().any(|sub| sub.is_instance_of::<PyDict>())
    {
        for sub in list.iter() {
            parsed.extend(parse_parameter_item(&sub)?);
        }
        return Ok(parsed);
    }
    let type_name = item
        .get_type()
        .name()
        .map(|n| n.to_string())
        .unwrap_or_default();
    if type_name.contains("ParameterFile") {
        if let Ok(path) = item.call_method0("evaluate_path")
            && let Ok(path) = path.extract::<String>()
        {
            parsed.push(("__param_file".to_string(), path));
        } else if let Ok(s) = item.call_method0("__str__")
            && let Ok(path) = s.extract::<String>()
        {
            parsed.push((
                "__param_file".to_string(),
                crate::api::utils::resolve_tokens(&path),
            ));
        }
        return Ok(parsed);
    }
    // Anything else is a path (a string or a substitution performing to one).
    let path = crate::api::utils::resolve_tokens(&pyobject_to_string(py, &item.clone().unbind())?);
    if is_yaml_file(&path) || path.contains('/') {
        parsed.push(("__param_file".to_string(), path));
    }
    Ok(parsed)
}

/// Mock Node class: `Node(package=..., executable=..., name=None,
/// namespace=None, parameters=None, remappings=None, arguments=None,
/// ros_arguments=None, condition=None, ...)`.
#[pyclass(module = "launch_ros.actions", from_py_object, subclass)]
#[derive(Clone)]
pub struct Node {
    pub(crate) spec: NodeSpec,
    /// `on_exit=` shuts the launch down (see [`on_exit_shuts_down`]).
    on_exit_shutdown: Option<bool>,
    #[pyo3(get)]
    condition: Option<Py<PyAny>>,
    /// The fully-qualified name once executed (`Node.node_name`).
    executed_fqn: Option<String>,
}

#[pymethods]
impl Node {
    #[new]
    #[pyo3(signature = (
        *,
        executable=None,
        package=None,
        name=None,
        namespace=None,
        exec_name=None,
        parameters=None,
        remappings=None,
        ros_arguments=None,
        arguments=None,
        env=None,
        additional_env=None,
        condition=None,
        **_kwargs
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        executable: Option<Py<PyAny>>,
        package: Option<Py<PyAny>>,
        name: Option<Py<PyAny>>,
        namespace: Option<Py<PyAny>>,
        exec_name: Option<Py<PyAny>>,
        parameters: Option<Py<PyAny>>,
        remappings: Option<Py<PyAny>>,
        ros_arguments: Option<Py<PyAny>>,
        arguments: Option<Py<PyAny>>,
        env: Option<Py<PyAny>>,
        additional_env: Option<Py<PyAny>>,
        condition: Option<Py<PyAny>>,
        _kwargs: Option<&Bound<'_, PyDict>>,
    ) -> Self {
        let _ = exec_name;
        let on_exit_shutdown = on_exit_kwarg_shuts_down(_kwargs);
        Self {
            on_exit_shutdown,
            spec: NodeSpec {
                package,
                executable,
                name,
                namespace,
                parameters,
                remappings,
                arguments,
                ros_arguments,
                env,
                additional_env,
                respawn: RespawnArgs::from_kwargs(_kwargs),
            },
            condition,
            executed_fqn: None,
        }
    }

    /// `Node.node_name`: the fully-qualified name, valid once executed.
    #[getter]
    fn node_name(&self) -> Option<String> {
        self.executed_fqn.clone()
    }

    fn __repr__(&self) -> String {
        "Node(...)".to_string()
    }
}

impl Node {
    pub(crate) fn execute(this: &Bound<'_, Self>, py: Python) -> PyResult<()> {
        let spec = this.borrow().spec.clone();
        let label = Self::label(py, &spec);
        if !crate::api::visit::claim_execution(this.as_any(), &label) {
            return Ok(());
        }
        let n = spec.capture_with(py, this.borrow().on_exit_shutdown, None)?;
        this.borrow_mut().executed_fqn = n.name.as_ref().map(|name| {
            let ns = n.namespace.clone().unwrap_or_default();
            if ns.is_empty() || ns == "/" {
                format!("/{name}")
            } else {
                format!("{ns}/{name}")
            }
        });
        Ok(())
    }

    /// `package/executable (name=...)`, for diagnostics.
    pub(crate) fn label(py: Python, spec: &NodeSpec) -> String {
        let s = |o: &Option<Py<PyAny>>| {
            o.as_ref()
                .and_then(|o| pyobject_to_string(py, o).ok())
                .unwrap_or_default()
        };
        let base = format!("{}/{}", s(&spec.package), s(&spec.executable));
        match &spec.name {
            Some(_) => format!("{base} (name={})", s(&spec.name)),
            None => base,
        }
    }

    pub(crate) fn parse_dict_params(
        dict: &Bound<'_, PyDict>,
        prefix: &str,
        params: &mut Vec<(String, String)>,
    ) -> PyResult<()> {
        for (key, value) in dict.iter() {
            let key_str = pyobject_to_string(dict.py(), &key.unbind())?;
            let full_key = if prefix.is_empty() {
                key_str.clone()
            } else {
                format!("{}.{}", prefix, key_str)
            };

            if let Ok(nested_dict) = value.cast::<PyDict>() {
                Self::parse_dict_params(nested_dict, &full_key, params)?;
            } else {
                let value_str = Self::extract_param_value(&value)?;
                params.push((full_key, value_str));
            }
        }

        Ok(())
    }

    pub(crate) fn extract_param_value(value: &Bound<'_, PyAny>) -> PyResult<String> {
        use pyo3::types::PyBool;

        // A Python string is read with YAML rules once performed, as
        // `evaluate_parameter_dict` reads it: `'yes'` is a boolean, `'5'` an
        // integer.
        if let Ok(s) = value.extract::<String>() {
            return Ok(play_launch_parser::param_value::yaml_value(&s).unwrap_or(s));
        }

        // A `ParameterValue` renders itself, `value_type` and all.
        if value
            .get_type()
            .name()
            .map(|n| n == "ParameterValue")
            .unwrap_or(false)
        {
            return pyobject_to_string(value.py(), &value.clone().unbind());
        }

        if value.is_instance_of::<PyBool>()
            && let Ok(b) = value.extract::<bool>()
        {
            return Ok(if b { "true" } else { "false" }.to_string());
        }

        if let Ok(i) = value.extract::<i64>() {
            return Ok(i.to_string());
        }

        if let Ok(f) = value.extract::<f64>() {
            if f.fract() == 0.0 && f.is_finite() {
                return Ok(format!("{:.1}", f));
            } else {
                return Ok(f.to_string());
            }
        }

        if let Ok(list) = value.cast::<pyo3::types::PySequence>()
            && !value.is_instance_of::<pyo3::types::PyString>()
        {
            // `_normalize_parameter_array_value`: strings and substitutions
            // together form ONE string; ints and floats together are floats.
            let items: Vec<Bound<'_, PyAny>> = list.try_iter()?.collect::<PyResult<_>>()?;
            let is_scalar = |i: &Bound<'_, PyAny>| {
                i.is_instance_of::<pyo3::types::PyString>()
                    || i.is_instance_of::<PyBool>()
                    || i.is_instance_of::<pyo3::types::PyInt>()
                    || i.is_instance_of::<pyo3::types::PyFloat>()
            };
            let has_substitution = items
                .iter()
                .any(|i| !is_scalar(i) && !i.is_instance_of::<PyList>());
            let only_text = items.iter().all(|i| {
                i.is_instance_of::<pyo3::types::PyString>()
                    || (!is_scalar(i) && !i.is_instance_of::<PyList>())
            });
            if has_substitution && only_text {
                let py = value.py();
                return pyobject_to_string(py, &PyList::new(py, &items)?.into_any().unbind());
            }
            let has_float = items
                .iter()
                .any(|i| i.is_instance_of::<pyo3::types::PyFloat>());
            let all_numeric = items.iter().all(|i| {
                (i.is_instance_of::<pyo3::types::PyInt>() && !i.is_instance_of::<PyBool>())
                    || i.is_instance_of::<pyo3::types::PyFloat>()
            });
            let mut formatted_items = Vec::new();
            for item in items.iter() {
                if has_float && all_numeric {
                    let f: f64 = item.extract()?;
                    formatted_items.push(if f.fract() == 0.0 && f.is_finite() {
                        format!("{f:.1}")
                    } else {
                        f.to_string()
                    });
                    continue;
                }
                // A string element stays a string (a list of strings is a
                // string array in `launch_ros`, whatever the strings say).
                if let Ok(text) = item.extract::<String>() {
                    formatted_items.push(format!("'{}'", text.replace('\'', "''")));
                    continue;
                }
                let val = Self::extract_param_value(item)?;
                let is_numeric_or_bool = val.parse::<f64>().is_ok()
                    || val.parse::<i64>().is_ok()
                    || matches!(val.as_str(), "true" | "false" | "True" | "False");
                if is_numeric_or_bool {
                    formatted_items.push(val);
                } else {
                    formatted_items.push(format!("'{}'", val.replace('\'', "''")));
                }
            }
            return Ok(format!("[{}]", formatted_items.join(", ")));
        }

        // A substitution: performed, then read with YAML rules.
        let py = value.py();
        let obj_py: Py<PyAny> = value.clone().unbind();
        let text = pyobject_to_string(py, &obj_py)?;
        Ok(play_launch_parser::param_value::yaml_value(&text).unwrap_or(text))
    }
}

/// A native Python scalar as parameter text.
pub(crate) fn node_value(value: &Bound<'_, PyAny>) -> PyResult<String> {
    Node::extract_param_value(value)
}

/// Does `on_exit=` shut the launch down, as `ros2 launch` would?
///
/// `on_exit` takes one action or a list of them. Shutdown is
/// `launch.actions.Shutdown`, or any `EmitEvent` whose event is
/// `launch.events.Shutdown` -- SSv2's `ShutdownOnce` is one. Other handlers
/// are not modelled (`Some(false)`), and no `on_exit` at all is `None`.
pub(crate) fn on_exit_shuts_down(on_exit: &Bound<'_, PyAny>) -> bool {
    let one = |a: &Bound<'_, PyAny>| {
        a.is_instance_of::<crate::api::actions::Shutdown>()
            || a.getattr("event")
                .ok()
                .filter(|e| !e.is_none())
                .is_some_and(|e| e.hasattr("_play_launch_shutdown").unwrap_or(false))
    };
    if on_exit.is_instance_of::<pyo3::types::PyList>()
        || on_exit.is_instance_of::<pyo3::types::PyTuple>()
    {
        on_exit
            .try_iter()
            .map(|it| it.flatten().any(|a| one(&a)))
            .unwrap_or(false)
    } else {
        one(on_exit)
    }
}

/// [`on_exit_shuts_down`] for a constructor's `**kwargs`.
pub(crate) fn on_exit_kwarg_shuts_down(kwargs: Option<&Bound<'_, PyDict>>) -> Option<bool> {
    let on_exit = kwargs?.get_item("on_exit").ok()??;
    if on_exit.is_none() {
        return None;
    }
    Some(on_exit_shuts_down(&on_exit))
}
