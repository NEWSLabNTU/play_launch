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
    /// The caller's launch configurations — nano-ros issue 0935.
    ///
    /// `exec_file` needs these and CANNOT read them from the caller's
    /// thread-local: both sides statically link `play_launch_parser`, so each
    /// has its own. Empty for `eval_expr`, whose argument is self-contained.
    #[serde(default)]
    configs: std::collections::BTreeMap<String, String>,
}

#[derive(serde::Serialize)]
struct Response {
    ok: bool,
    /// The `$(eval …)` result. Empty for `exec_file`, which reports through
    /// `captures`.
    value: String,
    error: String,
    /// What `exec_file` produced — nano-ros issue 0935.
    ///
    /// These used to be left in this object's thread-local context and
    /// discarded when the call returned, because the design assumed the caller
    /// shared it. `None` for `eval_expr`, and for any failure.
    #[serde(skip_serializing_if = "Option::is_none")]
    captures: Option<play_launch_parser::bridge::ExecCaptures>,
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
    // CATCH PANICS AT THE BOUNDARY — nano-ros issue 0953.
    //
    // A panic unwinding out of an `extern "C"` function is undefined
    // behaviour, so rustc emits a guard that ABORTS. Every other way out
    // of this function is already a structured error (null request,
    // non-UTF-8, malformed JSON, unknown op, even a NUL byte in our own
    // response); a panic was the one path that took the whole process
    // down instead, with no diagnostic the caller could read.
    //
    // That mattered because it defeats the promise `pyload` makes one
    // layer up — "a `Result`, never an abort" (issue 0897). 0897 removed
    // the LOADER abort; this removes the execution one. Reproduced with a
    // `.launch.py` driven outside `LaunchTraverser`, where the parser
    // panics `No LaunchContext set` and PyO3 resumes it as a
    // `PanicException`: the resolver dumped core.
    //
    // `AssertUnwindSafe` because the closure borrows `req` and calls into
    // the parser: making every captured type `UnwindSafe` buys nothing at
    // a boundary whose entire contract is a JSON string in and a JSON
    // string out. A panic here is reported and the object is not reused
    // for anything that outlives the call.
    //
    // The ABI VERSION does not change: this adds no field and no new
    // shape, only one more `ok: false` reason on a channel that already
    // carries them.
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| unsafe { call_inner(req) })) {
        Ok(response) => respond(response),
        Err(payload) => respond(Response {
            ok: false,
            value: String::new(),
            error: format!("pyexec: panicked: {}", panic_message(&payload)),
            captures: None,
        }),
    }
}

/// The panic payload's message, when it is one of the two shapes
/// `panic!` produces. Anything else has no readable text, and saying so
/// beats printing `Any { .. }`.
fn panic_message(payload: &Box<dyn std::any::Any + Send>) -> String {
    if let Some(s) = payload.downcast_ref::<String>() {
        s.clone()
    } else if let Some(s) = payload.downcast_ref::<&'static str>() {
        (*s).to_string()
    } else {
        "panic with a non-string payload".to_string()
    }
}

/// The real body. Returns a [`Response`] rather than a pointer so the
/// shim above owns BOTH the serialisation and the panic guard, and there
/// is exactly one place that can turn one into the other.
unsafe fn call_inner(req: *const c_char) -> Response {
    if req.is_null() {
        return Response {
            ok: false,
            value: String::new(),
            error: "pyexec: null request".into(),
            captures: None,
        };
    }
    let text = match unsafe { CStr::from_ptr(req) }.to_str() {
        Ok(t) => t,
        Err(e) => {
            return Response {
                ok: false,
                value: String::new(),
                error: format!("pyexec: request is not UTF-8: {e}"),
                captures: None,
            };
        }
    };
    let req: Request = match serde_json::from_str(text) {
        Ok(r) => r,
        Err(e) => {
            return Response {
                ok: false,
                value: String::new(),
                error: format!("pyexec: malformed request: {e}"),
                captures: None,
            };
        }
    };

    use play_launch_parser::python_backend::PythonBackend;
    let backend = crate::Pyo3Backend;
    let mut captures = None;
    let result = match req.op.as_str() {
        "exec_file" => {
            // issue 0935 — stand up THIS object's launch context around the
            // execution, seeded from the caller's configurations. Python calls
            // back into the copy of `play_launch_parser` linked HERE, so the
            // context it reads has to be established here; the caller's is a
            // different variable in a different copy of the crate.
            let mut ctx = play_launch_parser::substitution::context::LaunchContext::new();
            for (k, v) in &req.configs {
                ctx.set_configuration(k.clone(), v.clone());
            }
            let r = {
                let _guard = play_launch_parser::bridge::LaunchContextGuard::new(&mut ctx);
                backend.exec_file(&req.arg)
            };
            // Read them back BEFORE returning: they live in `ctx`, which dies
            // with this scope. That silent discard is what issue 0935 was.
            if r.is_ok() {
                captures = Some(play_launch_parser::bridge::ExecCaptures::drain_from(&mut ctx));
            }
            r.map(|()| String::new())
        }
        "eval_expr" => backend.eval_expr(&req.arg),
        // Test-only: the ONLY way to drive a panic through the real export
        // and assert it comes back as `ok: false` rather than aborting the
        // process. `#[cfg(test)]` keeps it out of the shipped cdylib, so the
        // ABI a loader sees is unchanged (issue 0953).
        #[cfg(test)]
        "__test_panic" => panic!("deliberate panic from the test op"),
        other => Err(format!(
            "pyexec: unknown op `{other}` (expected `exec_file` or `eval_expr`)"
        )),
    };
    match result {
        Ok(value) => Response {
            ok: true,
            value,
            error: String::new(),
            captures,
        },
        Err(error) => Response {
            ok: false,
            value: String::new(),
            error,
            captures: None,
        },
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
    // 2 since nano-ros issue 0935: `exec_file` gained `configs` in the request
    // and `captures` in the response. A v1 object linked against a v2 loader
    // would run the file and silently return nothing, which is the bug — so
    // the version is what makes that mismatch a statement instead.
    2
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

    /// A panic must not cross `extern "C"` — issue 0953.
    ///
    /// Unguarded this does not fail the test, it ABORTS the test binary:
    /// rustc's guard on an unwinding `extern "C"` calls `abort()`. So the
    /// assertion below only ever runs when the guard is present, and the
    /// mutation check for it is "delete the `catch_unwind` and watch the
    /// suite die rather than fail".
    #[test]
    fn a_panic_comes_back_as_an_error_not_an_abort() {
        let v = call(r#"{"op":"__test_panic","arg":""}"#);
        assert_eq!(v["ok"], false, "{v}");
        let e = v["error"].as_str().unwrap();
        assert!(e.contains("panicked"), "{e}");
        // The original message survives — "panicked" alone would leave the
        // reader no better off than the abort did.
        assert!(e.contains("deliberate panic from the test op"), "{e}");
    }

    #[test]
    fn panic_message_reads_both_payload_shapes() {
        let s: Box<dyn std::any::Any + Send> = Box::new("static str".to_string());
        assert_eq!(panic_message(&s), "static str");
        let s: Box<dyn std::any::Any + Send> = Box::new("borrowed");
        assert_eq!(panic_message(&s), "borrowed");
        let s: Box<dyn std::any::Any + Send> = Box::new(42u8);
        assert!(panic_message(&s).contains("non-string"));
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

    /// nano-ros issue 0935 — `exec_file` must report through the CHANNEL, not
    /// through a thread-local the caller cannot see.
    ///
    /// This is the test the old design could not have: every in-process test
    /// links one copy of `play_launch_parser`, so a thread-local looked shared
    /// and `.launch.py` passed here while aborting as shipped. Asserting on the
    /// RESPONSE is what makes the boundary the subject.
    #[test]
    fn exec_file_returns_its_captures_over_the_wire() {
        let dir = std::env::temp_dir().join("pyexec_abi_0935");
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("t.launch.py");
        std::fs::write(
            &file,
            "from launch import LaunchDescription\n\
             from launch_ros.actions import Node\n\
             def generate_launch_description():\n\
             \x20   return LaunchDescription([Node(package='p', executable='e', name='n')])\n",
        )
        .unwrap();

        let req = serde_json::json!({
            "op": "exec_file",
            "arg": file.to_str().unwrap(),
            "configs": {},
        })
        .to_string();
        let v = call(&req);

        assert_eq!(v["ok"], true, "{v}");
        let nodes = v["captures"]["nodes"]
            .as_array()
            .unwrap_or_else(|| panic!("captures.nodes must be in the RESPONSE: {v}"));
        assert_eq!(nodes.len(), 1, "{v}");
        assert_eq!(nodes[0]["package"], "p", "{v}");
        assert_eq!(nodes[0]["executable"], "e", "{v}");
    }

    /// The configurations a launch file reads come from the REQUEST, because
    /// the caller's context is a different copy of the crate (issue 0935).
    #[test]
    fn exec_file_sees_the_configurations_it_was_sent() {
        let dir = std::env::temp_dir().join("pyexec_abi_0935_cfg");
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("cfg.launch.py");
        std::fs::write(
            &file,
            "from launch import LaunchDescription\n\
             from launch.substitutions import LaunchConfiguration\n\
             from launch_ros.actions import Node\n\
             def generate_launch_description():\n\
             \x20   return LaunchDescription([\n\
             \x20       Node(package='p', executable='e',\n\
             \x20            name=LaunchConfiguration('who'))])\n",
        )
        .unwrap();

        let req = serde_json::json!({
            "op": "exec_file",
            "arg": file.to_str().unwrap(),
            "configs": { "who": "from_the_request" },
        })
        .to_string();
        let v = call(&req);
        assert_eq!(v["ok"], true, "{v}");
        let nodes = v["captures"]["nodes"].as_array().expect("nodes");
        assert_eq!(nodes.len(), 1, "{v}");
        assert_eq!(
            nodes[0]["name"], "from_the_request",
            "the configuration must reach Python through the request: {v}"
        );
    }

    /// The version is what turns a stale pairing into a sentence rather than a
    /// launch tree that silently resolves to nothing.
    #[test]
    fn the_abi_version_moved_with_the_contract() {
        assert_eq!(play_launch_py_abi_version(), 2);
    }

}
