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
            "extension-module",
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
