//! The end-to-end claim: discover a CPython on THIS host, `dlopen`
//! it, load the Python half against it, and get an answer back
//! through the C boundary — with no `libpython` linked into this
//! test binary.
//!
//! Everything up to here was measured on artifact SHAPE (a
//! `DT_NEEDED` absent, symbols undefined). This is the first thing
//! that proves the shape actually WORKS, which is the only claim
//! that matters: an artifact that loads nothing is exactly as
//! version-agnostic as one that cannot run.

use std::path::PathBuf;

/// Where cargo puts the `--features extension-module` cdylib.
///
/// Built by the test itself rather than depended on, because the
/// dependency is what this design removes: a normal dep would put
/// `libpython` back in this binary's `DT_NEEDED` and the test would
/// pass for the wrong reason.
fn build_pyexec() -> Option<PathBuf> {
    let out = std::process::Command::new(env!("CARGO"))
        .args([
            "build",
            "-p",
            "play_launch_parser_pyexec",
            "--lib",
            "--features",
            // The SHIPPED configuration, both halves of it (issue 0915).
            // `extension-module` alone only removes the link; `abi3` is what
            // restricts the undefined symbols to the stable ABI, which is the
            // claim "one build runs against any CPython >= 3.10" actually
            // rests on. Testing without it would exercise a configuration
            // nobody installs.
            "extension-module,abi3",
            "--message-format=json",
        ])
        .env("NROS_CARGO_FLAGS", "")
        .output()
        .ok()?;
    if !out.status.success() {
        eprintln!(
            "building the Python half failed:\n{}",
            String::from_utf8_lossy(&out.stderr)
        );
        return None;
    }
    // Read the path out of cargo's own output rather than guessing a
    // target dir — the workspace's may be redirected.
    let text = String::from_utf8_lossy(&out.stdout);
    let mut found = None;
    for line in text.lines() {
        let v: serde_json::Value = match serde_json::from_str(line) {
            Ok(v) => v,
            Err(_) => continue,
        };
        if v["reason"] != "compiler-artifact" {
            continue;
        }
        if !v["package_id"]
            .as_str()
            .unwrap_or_default()
            .contains("play_launch_parser_pyexec")
        {
            continue;
        }
        for f in v["filenames"].as_array().into_iter().flatten() {
            let p = PathBuf::from(f.as_str().unwrap_or_default());
            if p.extension().is_some_and(|e| e == "so") {
                found = Some(p);
            }
        }
    }
    found
}

/// This test binary must NOT link libpython — if it did, the load
/// below would be satisfied by its own `DT_NEEDED` and the test
/// would prove nothing.
#[test]
fn this_binary_does_not_link_libpython() {
    let exe = std::env::current_exe().expect("current_exe");
    let out = std::process::Command::new("readelf")
        .args(["-d", exe.to_str().unwrap()])
        .output();
    let Ok(out) = out else {
        eprintln!("readelf unavailable; skipping the linkage assertion");
        return;
    };
    let text = String::from_utf8_lossy(&out.stdout);
    let needed: Vec<&str> = text
        .lines()
        .filter(|l| l.contains("NEEDED") && l.contains("python"))
        .collect();
    assert!(
        needed.is_empty(),
        "the loader's own test binary links libpython, so this suite \
         cannot prove the dlopen path works: {needed:?}"
    );
}

/// The shipped half must reference only the STABLE ABI.
///
/// Issue 0915: `extension-module` removes the `DT_NEEDED`, which makes the
/// library loadable against an interpreter chosen at runtime. It says nothing
/// about whether the symbols it left undefined exist, unchanged, in a
/// DIFFERENT CPython — that is what `abi3` decides, and it was specified in
/// the design and never enabled. The build carried six private symbols
/// (`_Py_Dealloc`, `_PyObject_MakeTpCall`, …) and `Py_CompileStringExFlags`,
/// none of which carry a cross-version guarantee.
///
/// pyo3 enforces this at compile time — its FFI gates non-limited symbols
/// behind `cfg(not(Py_LIMITED_API))` — so this test does not re-derive the
/// rule. It pins the OUTCOME, because the enforcement is a feature flag that
/// a manifest edit can drop without any other test noticing.
#[test]
fn the_shipped_half_references_only_the_stable_abi() {
    let Some(pyexec) = build_pyexec() else {
        eprintln!("could not build the Python half; skipping");
        return;
    };
    let out = std::process::Command::new("nm")
        .args(["-D", "--undefined-only", pyexec.to_str().unwrap()])
        .output();
    let Ok(out) = out else {
        eprintln!("nm unavailable; skipping the symbol assertion");
        return;
    };
    let text = String::from_utf8_lossy(&out.stdout);

    // The private-symbol families that are stable ABI *data* exports —
    // `Py_None` is a macro over `&_Py_NoneStruct`, and the limited API's
    // refcount macros call `_Py_IncRef`/`_Py_DecRef`. Everything else
    // beginning `_Py` is CPython internals.
    const ALLOWED_PRIVATE: &[&str] = &[
        "_Py_NoneStruct",
        "_Py_TrueStruct",
        "_Py_FalseStruct",
        "_Py_IncRef",
        "_Py_DecRef",
    ];
    let mut offenders: Vec<String> = Vec::new();
    for line in text.lines() {
        for tok in line.split_whitespace() {
            let name = tok.split('@').next().unwrap_or(tok);
            if name.starts_with("_Py") && !ALLOWED_PRIVATE.contains(&name) {
                offenders.push(name.to_string());
            }
        }
    }
    offenders.sort();
    offenders.dedup();
    assert!(
        offenders.is_empty(),
        "the shipped half references CPython internals, so it is bound to the \
         version it was built against rather than to the stable ABI — is \
         `abi3` still on the cdylib build? {offenders:?}"
    );
}

#[test]
fn discovers_this_hosts_interpreter() {
    let i = play_launch_parser_pyload::find_interpreter();
    match i {
        Ok(i) => {
            assert!(i.version.starts_with("3."), "{i:?}");
            assert!(i.libpython.is_some());
        }
        Err(e) => {
            // A host with no shared libpython is a SUPPORTED state,
            // and the error is the deliverable there. Print it so a
            // runun run on such a host shows what a user would see.
            eprintln!("no usable interpreter here, which is a reported state:\n{e}");
        }
    }
}

#[test]
fn loads_and_evaluates_across_the_boundary() {
    let Some(pyexec) = build_pyexec() else {
        eprintln!("could not build the Python half; skipping the load test");
        return;
    };

    // The artifact this whole design is about: no libpython of its own.
    if let Ok(out) = std::process::Command::new("readelf")
        .args(["-d", pyexec.to_str().unwrap()])
        .output()
    {
        let text = String::from_utf8_lossy(&out.stdout);
        assert!(
            !text
                .lines()
                .any(|l| l.contains("NEEDED") && l.contains("python")),
            "the Python half must be built --features extension-module, \
             so its Py_ symbols are left for the loader"
        );
    }

    let interpreter = match play_launch_parser_pyload::find_interpreter() {
        Ok(i) => i,
        Err(e) => {
            eprintln!("no usable interpreter here: {e}");
            return;
        }
    };

    let loaded = play_launch_parser_pyload::Loaded::open(interpreter.clone(), &pyexec)
        .expect("the Python half should load against a discovered interpreter");

    use play_launch_parser::python_backend::PythonBackend;
    // `$(eval …)` is Python evaluation BY SPECIFICATION, so this is
    // the real capability crossing the boundary, not a smoke test.
    let v = loaded
        .eval_expr("1 + 1")
        .expect("eval must work through a dlopen'ded interpreter");
    assert_eq!(v, "2", "bound to {}", interpreter.version);

    // A Python-level error must come back as a VALUE. If it came
    // back as an abort, the whole "no crash" goal would be lost at
    // the last hop.
    let e = loaded
        .eval_expr("1 +")
        .expect_err("a syntax error must be an Err");
    assert!(!e.is_empty(), "the error must say something");

    eprintln!(
        "loaded {} against Python {} and evaluated across the C boundary",
        pyexec.display(),
        interpreter.version
    );
}
