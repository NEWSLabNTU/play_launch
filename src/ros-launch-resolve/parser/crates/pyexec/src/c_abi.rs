//! The C ABI this crate exports when built as a `cdylib`.
//!
//! This is the boundary a driver `dlopen`s (nano-ros issue 0897 W3).
//! Without it the `cdylib` is EMPTY — measured: zero defined *and* zero
//! undefined `Py_` symbols, because nothing was reachable from its
//! exported surface and the linker dropped the lot. An artifact with
//! the right SHAPE and no content is the failure this file removes.
//!
//! # Why C and not Rust
//!
//! Rust has no stable ABI, so a `dyn PythonBackend` cannot cross a
//! `dlopen` boundary — the vtable layout is not guaranteed between two
//! compilations. The export is therefore `extern "C"` with a contract
//! made of nothing but pointers and lengths.
//!
//! # Why JSON rather than a struct
//!
//! Same reason one layer up: a `#[repr(C)]` struct would pin a layout
//! both sides must agree on forever, and the resolver already
//! exchanges JSON with its own caller. Reusing that shape costs
//! nothing new and keeps the boundary describable in prose.
//!
//! # Ownership
//!
//! Every returned pointer is owned by THIS library and must be handed
//! back to [`play_launch_py_free`]. Freeing it with the caller's
//! allocator is undefined — they are frequently not the same allocator,
//! and a `cdylib` may be built by a different toolchain than its
//! loader.

use std::ffi::{CStr, CString, c_char};

/// What the caller asked for, and what came back.
///
/// Serialised rather than passed as a struct so neither side pins a
/// layout. `ok` discriminates: on failure `error` says what went wrong
/// in the same terms the in-process path would have.
#[derive(serde::Deserialize)]
struct Request {
    /// `exec_file` or `eval_expr` — the two things ROS 2 defines in
    /// terms of CPython.
    op: String,
    /// A launch file path, or an expression.
    arg: String,
}

#[derive(serde::Serialize)]
struct Response {
    ok: bool,
    /// The `$(eval …)` result. Empty for `exec_file`, whose effects land
    /// in the parser's thread-local launch context, not here.
    value: String,
    error: String,
}

fn respond(r: Response) -> *mut c_char {
    // A serialisation failure here cannot itself be reported through
    // the channel it just broke, so it degrades to a fixed, valid
    // response rather than a null the caller must special-case.
    let s = serde_json::to_string(&r).unwrap_or_else(|_| {
        r#"{"ok":false,"value":"","error":"pyexec: response serialisation failed"}"#.to_string()
    });
    match CString::new(s) {
        Ok(c) => c.into_raw(),
        // A NUL byte in a Rust String is unreachable, but returning
        // null on it would be an unannounced second failure mode.
        Err(_) => CString::new(r#"{"ok":false,"value":"","error":"pyexec: NUL in response"}"#)
            .expect("literal has no NUL")
            .into_raw(),
    }
}

/// Execute a `.launch.py` file, or evaluate a `$(eval …)` expression.
///
/// `req` is a NUL-terminated JSON object `{"op":…,"arg":…}`. Returns a
/// NUL-terminated JSON object which the caller MUST release with
/// [`play_launch_py_free`]. Never returns null.
///
/// # Safety
///
/// `req` must be a valid NUL-terminated C string for the duration of
/// the call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn play_launch_py_call(req: *const c_char) -> *mut c_char {
    if req.is_null() {
        return respond(Response {
            ok: false,
            value: String::new(),
            error: "pyexec: null request".into(),
        });
    }
    let text = match unsafe { CStr::from_ptr(req) }.to_str() {
        Ok(t) => t,
        Err(e) => {
            return respond(Response {
                ok: false,
                value: String::new(),
                error: format!("pyexec: request is not UTF-8: {e}"),
            });
        }
    };
    let req: Request = match serde_json::from_str(text) {
        Ok(r) => r,
        Err(e) => {
            return respond(Response {
                ok: false,
                value: String::new(),
                error: format!("pyexec: malformed request: {e}"),
            });
        }
    };

    use play_launch_parser::python_backend::PythonBackend;
    let backend = crate::Pyo3Backend;
    let result = match req.op.as_str() {
        "exec_file" => backend.exec_file(&req.arg).map(|()| String::new()),
        "eval_expr" => backend.eval_expr(&req.arg),
        other => Err(format!(
            "pyexec: unknown op `{other}` (expected `exec_file` or `eval_expr`)"
        )),
    };
    match result {
        Ok(value) => respond(Response {
            ok: true,
            value,
            error: String::new(),
        }),
        Err(error) => respond(Response {
            ok: false,
            value: String::new(),
            error,
        }),
    }
}

/// Release a pointer returned by [`play_launch_py_call`].
///
/// # Safety
///
/// `p` must be a pointer this library returned and has not already been
/// freed. Null is accepted and ignored.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn play_launch_py_free(p: *mut c_char) {
    if !p.is_null() {
        drop(unsafe { CString::from_raw(p) });
    }
}

/// The contract version this object speaks.
///
/// A loader checks this before trusting the two functions above. It
/// exists so a mismatched pair fails with a statement rather than a
/// segfault — the same reason `play_launch`'s own control channel
/// closes on a version it does not know.
#[unsafe(no_mangle)]
pub extern "C" fn play_launch_py_abi_version() -> u32 {
    1
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drive the real export, the way a loader would.
    fn call(json: &str) -> serde_json::Value {
        let req = CString::new(json).unwrap();
        let raw = unsafe { play_launch_py_call(req.as_ptr()) };
        assert!(!raw.is_null(), "the export must never return null");
        let out = unsafe { CStr::from_ptr(raw) }.to_str().unwrap().to_string();
        unsafe { play_launch_py_free(raw) };
        serde_json::from_str(&out).unwrap()
    }

    #[test]
    fn eval_crosses_the_boundary() {
        let v = call(r#"{"op":"eval_expr","arg":"1 + 1"}"#);
        assert_eq!(v["ok"], true, "{v}");
        assert_eq!(v["value"], "2", "{v}");
    }

    #[test]
    fn a_python_error_comes_back_as_a_message_not_a_crash() {
        let v = call(r#"{"op":"eval_expr","arg":"1 +"}"#);
        assert_eq!(v["ok"], false, "{v}");
        assert!(!v["error"].as_str().unwrap().is_empty());
    }

    #[test]
    fn an_unknown_op_is_named() {
        let v = call(r#"{"op":"nope","arg":""}"#);
        assert_eq!(v["ok"], false);
        assert!(v["error"].as_str().unwrap().contains("unknown op"));
    }

    #[test]
    fn malformed_json_does_not_panic() {
        let v = call("not json");
        assert_eq!(v["ok"], false);
        assert!(v["error"].as_str().unwrap().contains("malformed"));
    }

    /// Null is the one input a loader can pass by accident.
    #[test]
    fn null_request_is_answered() {
        let raw = unsafe { play_launch_py_call(std::ptr::null()) };
        assert!(!raw.is_null());
        let out = unsafe { CStr::from_ptr(raw) }.to_str().unwrap().to_string();
        unsafe { play_launch_py_free(raw) };
        assert!(out.contains("null request"));
    }

    #[test]
    fn free_accepts_null() {
        unsafe { play_launch_py_free(std::ptr::null_mut()) };
    }
}
