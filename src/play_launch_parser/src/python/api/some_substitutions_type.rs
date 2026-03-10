//! Mock launch.some_substitutions_type module
//!
//! Provides type utilities for substitutions.

use pyo3::prelude::*;

/// Register the some_substitutions_type module with its type tuples
pub fn register_module(py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let module = PyModule::new(py, "launch.some_substitutions_type")?;

    // SomeSubstitutionsType_types_tuple is a tuple of types that can be used as substitutions
    // In the real launch package, it's: (str, launch.substitution.Substitution, collections.abc.Iterable)
    // We'll create a simple tuple approximation
    let str_type = py.get_type::<pyo3::types::PyString>();
    let tuple_type = py.get_type::<pyo3::types::PyTuple>();
    let list_type = py.get_type::<pyo3::types::PyList>();

    // Create a tuple with the types
    let types_tuple = pyo3::types::PyTuple::new(
        py,
        [str_type.as_any(), tuple_type.as_any(), list_type.as_any()],
    )?;

    module.add("SomeSubstitutionsType_types_tuple", &types_tuple)?;

    Ok(module)
}
