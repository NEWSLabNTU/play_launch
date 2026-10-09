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
    /// `eval_expr` — the one operation left on this entry point. Running a
    /// `.launch.py` goes through [`play_launch_py_exec`] (ABI 7), because it
    /// needs a channel BACK to the caller at every include.
    op: String,
    /// An expression.
    arg: String,
}

#[derive(serde::Serialize)]
struct Response {
    ok: bool,
    /// The `$(eval …)` result.
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
        };
    }
    let text = match unsafe { CStr::from_ptr(req) }.to_str() {
        Ok(t) => t,
        Err(e) => {
            return Response {
                ok: false,
                value: String::new(),
                error: format!("pyexec: request is not UTF-8: {e}"),
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
            };
        }
    };

    use play_launch_parser::python_backend::PythonBackend;
    let backend = crate::Pyo3Backend;
    let result = match req.op.as_str() {
        "eval_expr" => backend.eval_expr(&req.arg),
        "exec_file" => {
            Err("pyexec: `exec_file` moved to `play_launch_py_exec` in C ABI 7".to_string())
        }
        // Test-only: the ONLY way to drive a panic through the real export
        // and assert it comes back as `ok: false` rather than aborting the
        // process. `#[cfg(test)]` keeps it out of the shipped cdylib, so the
        // ABI a loader sees is unchanged (issue 0953).
        #[cfg(test)]
        "__test_panic" => panic!("deliberate panic from the test op"),
        other => Err(format!(
            "pyexec: unknown op `{other}` (expected `eval_expr`)"
        )),
    };
    match result {
        Ok(value) => Response {
            ok: true,
            value,
            error: String::new(),
        },
        Err(error) => Response {
            ok: false,
            value: String::new(),
            error,
        },
    }
}

/// The caller's include callback: `request` is a NUL-terminated JSON
/// [`play_launch_parser::exchange::IncludeRequest`]; the answer is a
/// NUL-terminated JSON `{"ok":true,"state":ContextState}` or
/// `{"ok":false,"error":"…"}`, owned by the CALLER and handed back to its
/// `free` callback.
pub type IncludeCallback =
    unsafe extern "C" fn(data: *mut std::ffi::c_void, request: *const c_char) -> *mut c_char;
/// Releases what [`IncludeCallback`] returned, with the caller's allocator.
pub type FreeCallback = unsafe extern "C" fn(data: *mut std::ffi::c_void, p: *mut c_char);

/// The traverser on the far side of the C boundary.
struct CallbackHost {
    include: IncludeCallback,
    free: FreeCallback,
    data: *mut std::ffi::c_void,
}

impl play_launch_parser::exchange::IncludeHost for CallbackHost {
    fn include(
        &mut self,
        request: play_launch_parser::exchange::IncludeRequest,
    ) -> Result<play_launch_parser::exchange::ContextState, String> {
        let json = serde_json::to_string(&request)
            .map_err(|e| format!("pyexec: include request serialisation failed: {e}"))?;
        let json =
            CString::new(json).map_err(|e| format!("pyexec: NUL in include request: {e}"))?;
        // SAFETY: the caller promised `include`, `free` and `data` are valid for
        // the duration of the `play_launch_py_exec` call this runs inside.
        let raw = unsafe { (self.include)(self.data, json.as_ptr()) };
        if raw.is_null() {
            return Err("pyexec: the include callback returned null".to_string());
        }
        let text = unsafe { CStr::from_ptr(raw) }
            .to_string_lossy()
            .into_owned();
        unsafe { (self.free)(self.data, raw) };
        let v: serde_json::Value = serde_json::from_str(&text)
            .map_err(|e| format!("pyexec: malformed include answer: {e}"))?;
        if v["ok"].as_bool().unwrap_or(false) {
            serde_json::from_value(v["state"].clone())
                .map_err(|e| format!("pyexec: malformed state in include answer: {e}"))
        } else {
            Err(v["error"].as_str().unwrap_or("include failed").to_string())
        }
    }
}

#[derive(serde::Deserialize)]
struct ExecRequest {
    path: String,
    state: play_launch_parser::exchange::ContextState,
}

/// Run a `.launch.py` (C ABI 7).
///
/// `req` is a NUL-terminated JSON `{"path":…,"state":ContextState}`. Every
/// `IncludeLaunchDescription` the file reaches is handed to `include` WHEN it
/// is reached, and the file continues in the state that comes back — see
/// [`play_launch_parser::exchange`]. Returns a NUL-terminated JSON
/// `{"ok":true,"result":ExecResult}` or `{"ok":false,"error":"…"}`, which the
/// caller MUST release with [`play_launch_py_free`]. Never returns null.
///
/// # Safety
///
/// `req` must be a valid NUL-terminated C string, and `include`, `free` and
/// `data` valid, for the duration of the call. `include` may call back into
/// this library (an included `.launch.py`), so it must not hold a lock this
/// call could need.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn play_launch_py_exec(
    req: *const c_char,
    include: IncludeCallback,
    free: FreeCallback,
    data: *mut std::ffi::c_void,
) -> *mut c_char {
    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| unsafe {
        exec_inner(req, include, free, data)
    }));
    let value = match outcome {
        Ok(Ok(result)) => serde_json::json!({"ok": true, "result": result}),
        Ok(Err(error)) => serde_json::json!({"ok": false, "error": error}),
        Err(payload) => serde_json::json!({
            "ok": false,
            "error": format!("pyexec: panicked: {}", panic_message(&payload)),
        }),
    };
    let text = value.to_string();
    CString::new(text)
        .unwrap_or_else(|_| {
            CString::new(r#"{"ok":false,"error":"pyexec: NUL in response"}"#)
                .expect("literal has no NUL")
        })
        .into_raw()
}

unsafe fn exec_inner(
    req: *const c_char,
    include: IncludeCallback,
    free: FreeCallback,
    data: *mut std::ffi::c_void,
) -> Result<play_launch_parser::exchange::ExecResult, String> {
    use play_launch_parser::python_backend::PythonBackend;
    if req.is_null() {
        return Err("pyexec: null request".into());
    }
    let text = unsafe { CStr::from_ptr(req) }
        .to_str()
        .map_err(|e| format!("pyexec: request is not UTF-8: {e}"))?;
    let req: ExecRequest =
        serde_json::from_str(text).map_err(|e| format!("pyexec: malformed request: {e}"))?;
    let mut host = CallbackHost {
        include,
        free,
        data,
    };
    crate::Pyo3Backend.exec_file(&req.path, req.state, &mut host)
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
    //
    // 3: the request carries `namespace_stack` and the captures carry
    // `includes`. Both are serde-defaulted, so a v2 object would ACCEPT a v3
    // request and answer it wrong — every Python-declared node at `/`, every
    // Python include dropped — which is exactly the silent shape a version
    // exists to refuse.
    //
    // 4: the request carries the caller's `global_parameters`. A v3 object
    // accepts a v4 request (serde-defaulted) and answers it wrong — every
    // `OpaqueFunction` that reads `global_params` sees none — which is the
    // KeyError of issue 0028 back again, silently. Hence the bump.
    //
    // 5: the captures carry `declared_arguments` (issue 0030). A v4 object
    // reports none, and the required-argument rule an include is held to is
    // then satisfied by silence. Same shape, same answer.
    //
    // 6: the captures carry `unsupported` — what this half recognised but
    // could not model, `TimerAction`'s delay first among them. A v5 object
    // reports none, so `check` passes a `.launch.py` whose delays were
    // discarded. Serde-defaulted, hence silent, hence a version bump.
    //
    // STILL 6 now that a Python `TimerAction` attributes its delay (see
    // `api::delay`). The field that carries it, `start_delay_secs` on each
    // capture, predates this channel, and the pairing that would matter — a
    // new loader with a v6 object that cannot attribute — is not silent: such
    // an object reports every timer in `unsupported`, so `check` still
    // refuses. A bump would only break a working installed pair.
    //
    // 7: `exec_file` moved to `play_launch_py_exec`, which takes an include
    // CALLBACK: a `.launch.py`'s includes run when the file reaches them, in
    // the state it has built up, instead of being replayed from a list after
    // it finished — and the state crosses back both at each include and at
    // the end (`play_launch_parser::exchange`). Without that, a configuration
    // a `.launch.py` set reached neither the files it included nor the file
    // that included it. A different entry point and a different response,
    // so a v6 object cannot even be asked.
    7
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drive the real export, the way a loader would.
    ///
    /// Takes the interpreter guard for the whole call: these tests share one
    /// embedded interpreter and one thread-local context bridge, and the
    /// harness runs them as threads (issue #0050). One place to take it,
    /// rather than one line per test that can forget.
    fn call(json: &str) -> serde_json::Value {
        let _guard = crate::python_test_guard();
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

    /// What the test callback saw, and what it answers with.
    struct Recorder {
        requests: Vec<serde_json::Value>,
    }

    unsafe extern "C" fn record_include(
        data: *mut std::ffi::c_void,
        request: *const c_char,
    ) -> *mut c_char {
        let recorder = unsafe { &mut *(data as *mut Recorder) };
        let text = unsafe { CStr::from_ptr(request) }.to_str().unwrap();
        let v: serde_json::Value = serde_json::from_str(text).unwrap();
        // Answer with the state the file sent, plus one configuration the
        // "included file" set — what the file must continue with.
        let mut state = v["state"].clone();
        state["configurations"]["set_by_include"] = serde_json::json!("yes");
        recorder.requests.push(v);
        CString::new(serde_json::json!({"ok": true, "state": state}).to_string())
            .unwrap()
            .into_raw()
    }

    unsafe extern "C" fn free_answer(_data: *mut std::ffi::c_void, p: *mut c_char) {
        drop(unsafe { CString::from_raw(p) });
    }

    /// Run a file through the ABI 7 export, the way a loader would.
    fn exec(path: &std::path::Path, state: serde_json::Value) -> (serde_json::Value, Recorder) {
        let _guard = crate::python_test_guard();
        let req = CString::new(
            serde_json::json!({"path": path.to_str().unwrap(), "state": state}).to_string(),
        )
        .unwrap();
        let mut recorder = Recorder {
            requests: Vec::new(),
        };
        let raw = unsafe {
            play_launch_py_exec(
                req.as_ptr(),
                record_include,
                free_answer,
                &mut recorder as *mut Recorder as *mut std::ffi::c_void,
            )
        };
        assert!(!raw.is_null(), "the export must never return null");
        let out = unsafe { CStr::from_ptr(raw) }.to_str().unwrap().to_string();
        unsafe { play_launch_py_free(raw) };
        (serde_json::from_str(&out).unwrap(), recorder)
    }

    fn write(name: &str, body: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("pyexec_abi7_{name}"));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join(format!("{name}.launch.py"));
        std::fs::write(&file, body).unwrap();
        file
    }

    /// nano-ros issue 0935 — what a file produced comes back over the CHANNEL,
    /// not through a thread-local the caller cannot see.
    #[test]
    fn exec_returns_what_the_file_produced_over_the_wire() {
        let file = write(
            "produced",
            "from launch import LaunchDescription\n\
             from launch_ros.actions import Node\n\
             def generate_launch_description():\n\
             \x20   return LaunchDescription([Node(package='p', executable='e', name='n')])\n",
        );
        let (v, _) = exec(&file, serde_json::json!({}));
        assert_eq!(v["ok"], true, "{v}");
        let nodes = v["result"]["produced"]["nodes"].as_array().expect("nodes");
        assert_eq!(nodes.len(), 1, "{v}");
        assert_eq!(nodes[0]["package"], "p", "{v}");
        assert_eq!(nodes[0]["executable"], "e", "{v}");
    }

    /// The configurations a file reads come from the REQUEST's state.
    #[test]
    fn exec_sees_the_configurations_it_was_sent() {
        let file = write(
            "cfg",
            "from launch import LaunchDescription\n\
             from launch.substitutions import LaunchConfiguration\n\
             from launch_ros.actions import Node\n\
             def generate_launch_description():\n\
             \x20   return LaunchDescription([\n\
             \x20       Node(package='p', executable='e', name=LaunchConfiguration('who'))])\n",
        );
        let (v, _) = exec(
            &file,
            serde_json::json!({"configurations": {"who": "from_the_request"}}),
        );
        assert_eq!(v["ok"], true, "{v}");
        let nodes = v["result"]["produced"]["nodes"].as_array().expect("nodes");
        assert_eq!(nodes[0]["name"], "from_the_request", "{v}");
    }

    /// ABI 7: what the file sets comes back in its final state — an include
    /// scopes nothing, so it is the includer's from then on.
    #[test]
    fn exec_returns_the_configurations_the_file_set() {
        let file = write(
            "set",
            "from launch import LaunchDescription\n\
             from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration\n\
             def generate_launch_description():\n\
             \x20   return LaunchDescription([\n\
             \x20       DeclareLaunchArgument('declared', default_value='d'),\n\
             \x20       SetLaunchConfiguration('set_here', 'x')])\n",
        );
        let (v, _) = exec(&file, serde_json::json!({}));
        assert_eq!(v["ok"], true, "{v}");
        let cfg = &v["result"]["state"]["configurations"];
        assert_eq!(cfg["declared"], "d", "{v}");
        assert_eq!(cfg["set_here"], "x", "{v}");
    }

    /// ABI 7: an include is handed over WHEN it is reached — with what came
    /// before it and the state reached — and the file continues in the state
    /// that comes back.
    #[test]
    fn an_include_is_run_where_it_is_reached() {
        let file = write(
            "inc",
            "from launch import LaunchDescription\n\
             from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration\n\
             from launch.substitutions import LaunchConfiguration\n\
             from launch_ros.actions import Node\n\
             def generate_launch_description():\n\
             \x20   return LaunchDescription([\n\
             \x20       Node(package='p', executable='e', name='before'),\n\
             \x20       SetLaunchConfiguration('mine', 'before_include'),\n\
             \x20       IncludeLaunchDescription('/nonexistent/child.launch.xml',\n\
             \x20                                launch_arguments=[('k', 'v')]),\n\
             \x20       Node(package='p', executable='e',\n\
             \x20            name=LaunchConfiguration('set_by_include'))])\n",
        );
        let (v, rec) = exec(
            &file,
            serde_json::json!({"namespace_stack": ["/", "/system"]}),
        );
        assert_eq!(v["ok"], true, "{v}");
        assert_eq!(rec.requests.len(), 1);
        let req = &rec.requests[0];
        assert_eq!(req["file_path"], "/nonexistent/child.launch.xml", "{req}");
        assert_eq!(req["args"][0], serde_json::json!(["k", "v"]), "{req}");
        // The include's arguments are set in the includer's context, and so
        // is what the file set before it.
        assert_eq!(req["state"]["configurations"]["k"], "v", "{req}");
        assert_eq!(
            req["state"]["configurations"]["mine"], "before_include",
            "{req}"
        );
        assert_eq!(req["state"]["namespace_stack"][1], "/system", "{req}");
        // What came before the include travels WITH it, in order.
        assert_eq!(req["produced"]["nodes"][0]["name"], "before", "{req}");
        // And the node after it read what the include set.
        let nodes = v["result"]["produced"]["nodes"].as_array().expect("nodes");
        assert_eq!(nodes.len(), 1, "{v}");
        assert_eq!(nodes[0]["name"], "yes", "{v}");
        assert_eq!(nodes[0]["namespace"], "/system", "{v}");
    }

    /// ABI 5: every `DeclareLaunchArgument` comes back with whether it had a
    /// default, whether launch's include check can see it, and whether it was
    /// unset when it executed.
    #[test]
    fn exec_returns_its_declared_arguments_over_the_wire() {
        let file = write(
            "decl",
            "from launch import LaunchDescription\n\
             from launch.actions import DeclareLaunchArgument, OpaqueFunction\n\
             def launch_setup(context, *args, **kwargs):\n\
             \x20   return [DeclareLaunchArgument('opaque', description='inside')]\n\
             def generate_launch_description():\n\
             \x20   return LaunchDescription([\n\
             \x20       DeclareLaunchArgument('required', description='must be passed'),\n\
             \x20       DeclareLaunchArgument('optional', default_value='1'),\n\
             \x20       OpaqueFunction(function=launch_setup)])\n",
        );
        let (v, _) = exec(
            &file,
            serde_json::json!({"configurations": {"required": "x", "opaque": "y"}}),
        );
        assert_eq!(v["ok"], true, "{v}");
        let decl = v["result"]["produced"]["declared_arguments"]
            .as_array()
            .expect("declared_arguments");
        let find = |name: &str| {
            decl.iter()
                .find(|d| d["name"] == name)
                .unwrap_or_else(|| panic!("{name} missing from {v}"))
                .clone()
        };
        assert_eq!(find("required")["has_default"], false, "{v}");
        assert_eq!(find("required")["opaque"], false, "{v}");
        assert_eq!(find("required")["unset_at_execute"], false, "{v}");
        assert_eq!(find("required")["description"], "must be passed", "{v}");
        assert_eq!(find("optional")["has_default"], true, "{v}");
        assert_eq!(find("opaque")["has_default"], false, "{v}");
        assert_eq!(find("opaque")["opaque"], true, "{v}");
    }

    /// ABI 4: global parameters the caller already holds reach this file's
    /// `OpaqueFunction` as `launch_configurations['global_params']`, the way
    /// `launch_ros` stores them (play_launch issue 0028).
    #[test]
    fn exec_sees_the_global_parameters_it_was_sent() {
        let file = write(
            "gp",
            "from launch import LaunchDescription\n\
             from launch.actions import OpaqueFunction\n\
             from launch_ros.actions import Node\n\
             def launch_setup(context, *args, **kwargs):\n\
             \x20   gp = dict(context.launch_configurations.get('global_params', {}))\n\
             \x20   ro = gp['rear_overhang']\n\
             \x20   return [Node(package='p', executable='e',\n\
             \x20                name='ro_%d' % round(ro * 1000))]\n\
             def generate_launch_description():\n\
             \x20   return LaunchDescription([OpaqueFunction(function=launch_setup)])\n",
        );
        let (v, _) = exec(
            &file,
            serde_json::json!({"global_parameters": {"items": [["rear_overhang", "0.821"], ["wheel_base", "2.061"]], "same": false}}),
        );
        assert_eq!(v["ok"], true, "{v}");
        let nodes = v["result"]["produced"]["nodes"].as_array().expect("nodes");
        assert_eq!(nodes[0]["name"], "ro_821", "{v}");
    }

    /// ABI 3: the namespace the caller is in reaches what Python declares.
    #[test]
    fn exec_declares_nodes_under_the_callers_namespace() {
        let file = write(
            "ns",
            "from launch import LaunchDescription\n\
             from launch_ros.actions import ComposableNodeContainer\n\
             from launch_ros.descriptions import ComposableNode\n\
             def generate_launch_description():\n\
             \x20   c = ComposableNode(namespace='monitor', name='component', package='p', plugin='P')\n\
             \x20   return LaunchDescription([ComposableNodeContainer(\n\
             \x20       namespace='monitor', name='container', package='rclcpp_components',\n\
             \x20       executable='component_container', composable_node_descriptions=[c])])\n",
        );
        let (v, _) = exec(
            &file,
            serde_json::json!({"namespace_stack": ["/", "/system"]}),
        );
        assert_eq!(v["ok"], true, "{v}");
        let containers = v["result"]["produced"]["containers"]
            .as_array()
            .expect("containers");
        assert_eq!(containers[0]["namespace"], "/system/monitor", "{v}");
        let loads = v["result"]["produced"]["load_nodes"]
            .as_array()
            .expect("load_nodes");
        assert_eq!(loads[0]["namespace"], "/system/monitor", "{v}");
        assert_eq!(
            loads[0]["target_container_name"], "/system/monitor/container",
            "{v}"
        );
    }

    /// The version is what turns a stale pairing into a sentence rather than a
    /// launch tree that silently resolves to nothing.
    #[test]
    fn the_abi_version_moved_with_the_contract() {
        assert_eq!(play_launch_py_abi_version(), 7);
    }

    /// `exec_file` on the old entry point is refused by name, not answered
    /// wrong.
    #[test]
    fn exec_file_on_the_old_entry_point_names_the_new_one() {
        let v = call(r#"{"op":"exec_file","arg":"/x.launch.py"}"#);
        assert_eq!(v["ok"], false, "{v}");
        assert!(
            v["error"].as_str().unwrap().contains("play_launch_py_exec"),
            "{v}"
        );
    }
}
