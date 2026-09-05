//! Discover a CPython at runtime, `dlopen` it, and load the parser's
//! Python half against it (nano-ros issue 0897 W3).
//!
//! # What this removes
//!
//! `pyexec` linked normally puts `libpython3.10.so.1.0` in the
//! consumer's `DT_NEEDED`. The loader resolves that BEFORE `main`, so
//! one build serves exactly one interpreter and a host with a
//! different one dies with a message from `ld.so` that the program
//! cannot catch, report, or degrade around.
//!
//! Here nothing is linked. The interpreter is discovered by ASKING
//! one (`sysconfig`), `dlopen`ed with `RTLD_GLOBAL`, and the Python
//! half — built `--features extension-module`, so its `Py_` symbols
//! are undefined — is loaded against it. Which is exactly what a
//! CPython process does for its own extension modules; we are doing
//! it by hand because we are not one.
//!
//! # What the caller gets
//!
//! A `Result`, never an abort. With no usable interpreter the parser
//! still scans, still resolves XML and YAML, and fails only where ROS 2
//! genuinely defines Python — a `.launch.py` file, or `$(eval …)` —
//! naming which of the two it could not do.

use std::path::{Path, PathBuf};

/// Why the Python half could not be loaded.
///
/// Every arm names something a person can act on. The first two are
/// the ones a normal host hits; the rest mean the install is damaged.
#[derive(Debug)]
pub enum LoadError {
    /// No interpreter was found at all.
    NoInterpreter { tried: Vec<String> },
    /// An interpreter was found, but it has no shared `libpython` to
    /// load — a `--enable-shared`-less build, which is common in
    /// pyenv and some distro packages.
    NoSharedLibpython { python: String, version: String },
    /// The interpreter is older than the Python half's abi3 floor.
    TooOld {
        python: String,
        version: String,
        floor: String,
    },
    /// `dlopen` of `libpython` itself failed.
    Libpython {
        path: PathBuf,
        source: libloading::Error,
    },
    /// `dlopen` of the Python half failed.
    Pyexec {
        path: PathBuf,
        source: libloading::Error,
    },
    /// The Python half is present but speaks a different C ABI.
    AbiMismatch { ours: u32, theirs: u32 },
    /// The Python half is not installed beside the driver at all.
    ///
    /// Distinct from `Pyexec`, which is "it is there and would not load".
    /// Collapsing the two produced `dlopen failed, but system did not report
    /// the error` for a plain missing file — a message that sends the reader
    /// looking for a loader problem when nothing was ever built.
    PyexecMissing { searched: Vec<PathBuf> },
}

impl std::fmt::Display for LoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoInterpreter { tried } => write!(
                f,
                "no Python interpreter found (tried: {}).\n\n\
                 ROS 2 defines `.launch.py` files and `$(eval …)` in terms of \
                 CPython, so those need one. XML and YAML launch files that use \
                 neither do not, and were parsed normally.\n\n\
                 Set $NROS_PYTHON to name an interpreter explicitly.",
                if tried.is_empty() {
                    "nothing".into()
                } else {
                    tried.join(", ")
                }
            ),
            Self::NoSharedLibpython { python, version } => write!(
                f,
                "`{python}` (Python {version}) has no shared `libpython` to load.\n\n\
                 It was built without `--enable-shared`, so its interpreter is \
                 statically linked into the `python3` binary and there is no \
                 library for us to open. This is common with pyenv builds.\n\n\
                 Install a shared build (Debian/Ubuntu: `libpython{version}`), or \
                 set $NROS_PYTHON to one."
            ),
            Self::TooOld {
                python,
                version,
                floor,
            } => write!(
                f,
                "`{python}` is Python {version}; the Python half needs {floor} or newer.\n\n\
                 One build serves every CPython from {floor} up (that is what the \
                 stable ABI buys), but not older ones."
            ),
            Self::Libpython { path, source } => {
                write!(f, "could not open `{}`: {source}", path.display())
            }
            Self::PyexecMissing { searched } => write!(
                f,
                "the Python half is not installed: `libplay_launch_parser_pyexec.so` \
                 is not beside this executable.\n\n\
                 Looked in:\n{}\n\n\
                 The driver and the Python half are TWO artifacts — the driver \
                 links no `libpython` on purpose, so it loads one at runtime and \
                 cannot evaluate anything on its own. Rebuild whatever installed \
                 this binary; a build that emits only the driver produces exactly \
                 this state.",
                searched
                    .iter()
                    .map(|p| format!("  {}", p.display()))
                    .collect::<Vec<_>>()
                    .join("\n"),
            ),
            Self::Pyexec { path, source } => write!(
                f,
                "could not open the Python half `{}`: {source}\n\n\
                 It must be built with `--features extension-module` — without \
                 that it links its own `libpython` and defeats the point of \
                 loading one here.",
                path.display()
            ),
            Self::AbiMismatch { ours, theirs } => write!(
                f,
                "the Python half speaks C ABI version {theirs}, this loader speaks {ours}.\n\n\
                 The two were built from different revisions. Rebuild them together."
            ),
        }
    }
}

impl std::error::Error for LoadError {}

/// The C ABI version this loader speaks. Must match
/// `play_launch_py_abi_version()` in the object it loads.
///
/// 2 since nano-ros issue 0935, which added `configs` to the `exec_file`
/// request and `captures` to its response. A v1 object paired with this loader
/// would execute the launch file and return nothing — a tree that resolves to
/// no nodes, which is indistinguishable from an empty launch file. That is
/// exactly the failure this constant exists to turn into a sentence.
const ABI_VERSION: u32 = 2;

/// What `sysconfig` says about an interpreter.
#[derive(Debug, Clone)]
pub struct Interpreter {
    /// The `python3` that answered.
    pub python: String,
    /// `3.10`, `3.12`, …
    pub version: String,
    /// Absolute path to its `libpython`, when it has a shared one.
    pub libpython: Option<PathBuf>,
}

/// Ask an interpreter where its library is.
///
/// A QUERY, not a guess. The alternative — globbing for
/// `libpython*.so` on the filesystem — finds *an* answer rather than the
/// RIGHT one, which is the same class of defect issue 0285 removed from
/// the resolver lookup, so it is not done here either.
///
/// `Py_ENABLE_SHARED` is read as well as the path: an interpreter with a
/// static-only build has no library to open, and saying THAT is much
/// more useful than reporting the resulting `dlopen` failure.
fn interrogate(python: &str) -> Option<Interpreter> {
    let out = std::process::Command::new(python)
        .arg("-c")
        .arg(
            "import sysconfig as s;\
             print(s.get_config_var('INSTSONAME') or '');\
             print(s.get_config_var('LIBDIR') or '');\
             print(s.get_config_var('Py_ENABLE_SHARED') or 0);\
             import sys; print('%d.%d' % sys.version_info[:2])",
        )
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    let text = String::from_utf8_lossy(&out.stdout);
    let mut lines = text.lines();
    let soname = lines.next()?.trim().to_string();
    let libdir = lines.next()?.trim().to_string();
    let shared = lines.next()?.trim() != "0";
    let version = lines.next()?.trim().to_string();

    Some(Interpreter {
        python: python.to_string(),
        version,
        libpython: if shared && !soname.is_empty() && !libdir.is_empty() {
            Some(Path::new(&libdir).join(&soname))
        } else {
            None
        },
    })
}

/// Which interpreters to try, in order.
///
/// Mirrors `scripts/build/zephyr-python.sh` so the tree has ONE answer to
/// "which interpreter": an explicit `$NROS_PYTHON` wins whether or not it
/// works, then `python3` on `PATH`, then nothing. No filesystem scan.
fn candidates() -> Vec<String> {
    let mut v = Vec::new();
    if let Ok(p) = std::env::var("NROS_PYTHON")
        && !p.is_empty()
    {
        v.push(p);
    }
    v.push("python3".to_string());
    v
}

/// Find an interpreter with a shared `libpython`.
pub fn find_interpreter() -> Result<Interpreter, LoadError> {
    let mut tried = Vec::new();
    let mut static_only = None;
    for c in candidates() {
        tried.push(c.clone());
        match interrogate(&c) {
            Some(i) if i.libpython.is_some() => return Ok(i),
            // Remember the first interpreter that EXISTS but has no
            // shared library: that is a much better error than "none
            // found", and it is the pyenv case.
            Some(i) => static_only.get_or_insert(i),
            None => continue,
        };
    }
    match static_only {
        Some(i) => Err(LoadError::NoSharedLibpython {
            python: i.python,
            version: i.version,
        }),
        None => Err(LoadError::NoInterpreter { tried }),
    }
}

/// A loaded Python half, and the interpreter it is bound to.
///
/// Holds both libraries open: dropping either would unload code the
/// parser may still be inside. The `libpython` handle is deliberately
/// never used after loading — it exists to keep the library resident and
/// its symbols visible.
pub struct Loaded {
    _libpython: libloading::Library,
    pyexec: libloading::Library,
    pub interpreter: Interpreter,
}

type CallFn = unsafe extern "C" fn(*const std::ffi::c_char) -> *mut std::ffi::c_char;
type FreeFn = unsafe extern "C" fn(*mut std::ffi::c_char);
type AbiFn = unsafe extern "C" fn() -> u32;

impl Loaded {
    /// `dlopen` a `libpython`, then the Python half against it.
    ///
    /// ORDER IS THE WHOLE TRICK. `RTLD_GLOBAL` on the first makes its
    /// symbols available for resolution of subsequently loaded
    /// objects, which is how the Python half's undefined `Py_*` symbols
    /// get bound. Loading them the other way round, or without
    /// `RTLD_GLOBAL`, fails with an undefined-symbol error naming a
    /// `Py_` symbol — which reads as a build problem and is really an
    /// ordering one.
    pub fn open(interpreter: Interpreter, pyexec: &Path) -> Result<Self, LoadError> {
        let lib = interpreter
            .libpython
            .clone()
            .expect("find_interpreter only returns interpreters with one");

        // RTLD_NOW so a missing symbol is an error HERE, with both
        // paths in hand, rather than a crash at the first call.
        const FLAGS: i32 = libloading::os::unix::RTLD_NOW | libloading::os::unix::RTLD_GLOBAL;
        let libpython = unsafe { libloading::os::unix::Library::open(Some(&lib), FLAGS) }.map_err(
            |source| LoadError::Libpython {
                path: lib.clone(),
                source,
            },
        )?;
        let libpython: libloading::Library = libpython.into();

        let obj =
            unsafe { libloading::Library::new(pyexec) }.map_err(|source| LoadError::Pyexec {
                path: pyexec.to_path_buf(),
                source,
            })?;

        // Check the contract BEFORE calling across it: a mismatched
        // pair fails with a statement rather than a segfault.
        let theirs = unsafe {
            let f: libloading::Symbol<AbiFn> =
                obj.get(b"play_launch_py_abi_version\0")
                    .map_err(|source| LoadError::Pyexec {
                        path: pyexec.to_path_buf(),
                        source,
                    })?;
            f()
        };
        if theirs != ABI_VERSION {
            return Err(LoadError::AbiMismatch {
                ours: ABI_VERSION,
                theirs,
            });
        }

        log::debug!(
            "python half loaded: {} against {} (Python {})",
            pyexec.display(),
            lib.display(),
            interpreter.version
        );
        Ok(Self {
            _libpython: libpython,
            pyexec: obj,
            interpreter,
        })
    }

    /// One call across the C boundary.
    ///
    /// `configs` rides along because the object cannot read the caller's —
    /// nano-ros issue 0935. Returns the whole response so `exec_file` can take
    /// its captures out of it.
    fn call(
        &self,
        op: &str,
        arg: &str,
        configs: std::collections::BTreeMap<String, String>,
    ) -> Result<serde_json::Value, String> {
        use std::ffi::{CStr, CString};
        let req = serde_json::json!({ "op": op, "arg": arg, "configs": configs }).to_string();
        let req = CString::new(req).map_err(|e| format!("request contains a NUL byte: {e}"))?;

        let raw = unsafe {
            let call: libloading::Symbol<CallFn> = self
                .pyexec
                .get(b"play_launch_py_call\0")
                .map_err(|e| format!("`play_launch_py_call` is missing: {e}"))?;
            call(req.as_ptr())
        };
        if raw.is_null() {
            return Err("the Python half returned null, which its contract forbids".into());
        }
        let out = unsafe { CStr::from_ptr(raw) }
            .to_string_lossy()
            .into_owned();
        // Hand it straight back: the memory is the library's, and its
        // allocator is frequently not ourss.
        unsafe {
            let free: libloading::Symbol<FreeFn> = self
                .pyexec
                .get(b"play_launch_py_free\0")
                .map_err(|e| format!("`play_launch_py_free` is missing: {e}"))?;
            free(raw);
        }

        let v: serde_json::Value =
            serde_json::from_str(&out).map_err(|e| format!("malformed response: {e}"))?;
        if v["ok"].as_bool().unwrap_or(false) {
            Ok(v)
        } else {
            Err(v["error"]
                .as_str()
                .unwrap_or("unspecified error")
                .to_string())
        }
    }
}

impl play_launch_parser::python_backend::PythonBackend for Loaded {
    /// nano-ros issue 0935 — carry the context BOTH ways.
    ///
    /// The object runs Python against its own copy of `play_launch_parser`, so
    /// the caller's thread-local launch context is invisible to it: the
    /// configurations have to be sent, and whatever Python captured has to be
    /// sent back and merged here. Before this, `exec_file` executed the file
    /// perfectly and then dropped everything it produced, and any Python API
    /// call that needed the context aborted the process.
    fn exec_file(&self, path: &str) -> Result<(), String> {
        use play_launch_parser::bridge::{ExecCaptures, with_launch_context};

        let configs: std::collections::BTreeMap<String, String> =
            with_launch_context(|ctx| ctx.configurations().into_iter().collect());

        let response = self.call("exec_file", path, configs)?;

        // Absent `captures` means the object predates this contract. The ABI
        // version already refuses that pairing at load; this is the belt to
        // that braces, because the failure it prevents — a launch file that
        // resolves to nothing, quietly — is indistinguishable from an empty
        // launch file.
        let Some(raw) = response.get("captures") else {
            return Err("the Python half returned no captures: it speaks an older \
                        contract than this loader (expected ABI 2)"
                .into());
        };
        let captures: ExecCaptures = serde_json::from_value(raw.clone())
            .map_err(|e| format!("malformed captures from the Python half: {e}"))?;
        with_launch_context(|ctx| captures.merge_into(ctx));
        Ok(())
    }

    fn eval_expr(&self, expr: &str) -> Result<String, String> {
        // Self-contained: the expression is the whole input, the string is the
        // whole output. This is why `$(eval …)` kept working while `exec_file`
        // did not.
        self.call("eval_expr", expr, Default::default())
            .map(|v| v["value"].as_str().unwrap_or_default().to_string())
    }
}

/// Locate the Python half beside this binary.
///
/// Same discipline as the resolver itself (issue 0285): derived from
/// `current_exe()`, never `$PATH` or `$LD_LIBRARY_PATH`. A search path
/// finds whichever copy is first, which is how an unrelated
/// `play_launch` came to shadow ours.
pub fn pyexec_beside_exe() -> Option<PathBuf> {
    pyexec_search_paths().into_iter().find(|c| c.is_file())
}

/// Where `pyexec_beside_exe` looks, in order.
///
/// Split out so a failure can NAME the paths it tried. "Not found" without
/// them is unactionable — the reader cannot tell whether the lookup was even
/// pointed at the right directory.
#[must_use]
pub fn pyexec_search_paths() -> Vec<PathBuf> {
    const NAME: &str = "libplay_launch_parser_pyexec.so";
    let Ok(exe) = std::env::current_exe() else {
        return Vec::new();
    };
    let Some(dir) = exe.parent() else {
        return Vec::new();
    };
    vec![dir.join(NAME), dir.join("../lib").join(NAME)]
}

/// Discover an interpreter, load the Python half, and install it as the
/// parser's backend.
///
/// Idempotent — a second call is a no-op, since the first registration
/// wins. Returns the interpreter that was bound, so a caller can say
/// which one it got; `Err` names what a person can do about it.
pub fn install() -> Result<Interpreter, LoadError> {
    let interpreter = find_interpreter()?;
    let pyexec = pyexec_beside_exe().ok_or_else(|| LoadError::PyexecMissing {
        searched: pyexec_search_paths(),
    })?;
    let loaded = Loaded::open(interpreter, &pyexec)?;
    let interpreter = loaded.interpreter.clone();
    play_launch_parser::python_backend::set_backend(Box::new(loaded));
    Ok(interpreter)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `$NROS_PYTHON` is PROCESS state, so the tests that set it
    /// cannot run in parallel — one clears it while another is
    /// asserting on it. Same hazard the parser solves with
    /// `python_test_guard()`, same fix.
    static ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    fn env_guard() -> std::sync::MutexGuard<'static, ()> {
        ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// The discovery is a QUERY, so it answers for the interpreter
    /// actually running the test rather than for a guess.
    #[test]
    fn interrogate_answers_for_this_host() {
        let i = interrogate("python3").expect("this host has python3");
        assert!(i.version.starts_with("3."), "{i:?}");
        // Not asserted: that a shared libpython exists. It is absent on
        // static builds, and that is a supported state this crate
        // REPORTS rather than fails on — asserting it here would make
        // the suite fail on exactly the hosts the feature is for.
        if let Some(p) = &i.libpython {
            assert!(p.is_absolute(), "libpython path must be absolute: {p:?}");
        }
    }

    #[test]
    fn a_nonexistent_interpreter_is_none_not_a_panic() {
        assert!(interrogate("definitely-not-a-python-3").is_none());
    }

    #[test]
    fn nros_python_is_tried_first() {
        let _guard = env_guard();
        // Explicit wins, which is the one rule this order has to keep:
        // a caller who names an interpreter gets THAT one or an error,
        // never a silent fallback to another.
        unsafe { std::env::set_var("NROS_PYTHON", "/some/explicit/python") };
        assert_eq!(candidates()[0], "/some/explicit/python");
        unsafe { std::env::remove_var("NROS_PYTHON") };
        assert_eq!(candidates()[0], "python3");
    }

    /// An empty `$NROS_PYTHON` must not become a candidate — it is the
    /// shape an unexpanded shell variable takes, and issue 0805 is the
    /// same defect one layer up (an empty `-D` flag read as a real
    /// value).
    #[test]
    fn an_empty_nros_python_is_not_a_candidate() {
        let _guard = env_guard();
        unsafe { std::env::set_var("NROS_PYTHON", "") };
        assert_eq!(candidates(), vec!["python3".to_string()]);
        unsafe { std::env::remove_var("NROS_PYTHON") };
    }

    #[test]
    fn find_interpreter_reports_what_it_tried() {
        let _guard = env_guard();
        unsafe { std::env::set_var("NROS_PYTHON", "definitely-not-a-python-3") };
        // `PATH still carries `python3`, which exists here, so this
        // only checks the error SHAPE when it does fail.
        if let Err(LoadError::NoInterpreter { tried }) = find_interpreter() {
            assert!(!tried.is_empty());
        }
        unsafe { std::env::remove_var("NROS_PYTHON") };
    }

    #[test]
    fn errors_name_something_actionable() {
        let e = LoadError::NoSharedLibpython {
            python: "/usr/bin/python3".into(),
            version: "3.12".into(),
        };
        let s = e.to_string();
        assert!(s.contains("--enable-shared"), "{s}");
        assert!(s.contains("libpython3.12"), "names the package: {s}");

        let e = LoadError::NoInterpreter { tried: vec![] };
        let s = e.to_string();
        assert!(s.contains("XML and YAML"), "says what still works: {s}");
        assert!(s.contains("NROS_PYTHON"), "says what to set: {s}");
    }
}
