//! `migrate` -- rewrite contracts from deprecated spellings to current ones
//! (phase 90 G8).
//!
//! The rewrites are the grammar's, in `ros_launch_manifest_types::migrate`
//! (its `REWRITES` table); this verb finds the files, shows what would
//! change and, with `write`, changes it. Each rewrite is a minimal textual
//! edit, so a migrated contract differs from the original only in the tokens
//! a rewrite names: comments, quoting, key order and layout survive.
//!
//! Every deprecated-spelling diagnostic the checker prints names this verb.

use std::path::{Path, PathBuf};

use eyre::{Result, WrapErr as _};
use ros_launch_manifest_types::migrate::{self, Migration, REWRITES};

/// Everything `migrate` reads. Owned plain values -- see [`crate::verbs`].
#[derive(Debug, Clone, Default)]
pub struct MigrateInputs {
    /// Contract files, or directories searched for `*.contract.yaml`.
    pub paths: Vec<PathBuf>,
    /// Rewrite the files in place. Without it nothing is written and the
    /// diff is printed.
    pub write: bool,
    /// Exit 1 when any file would change (a CI gate for "nothing
    /// deprecated is left").
    pub check: bool,
    /// Print the rewrite table and exit.
    pub list: bool,
    /// Print the edits only, not the diff.
    pub quiet: bool,
}

/// Directory names a directory walk does not enter: build trees, installs,
/// logs and VCS metadata hold copies, not sources.
const SKIP_DIRS: &[&str] = &[
    "build",
    "install",
    "log",
    "target",
    "node_modules",
    "play_log",
];

/// Run the verb. Returns the exit status it intends: 0 done (or nothing to
/// do), 1 with `check` when a file would change, 2 when a file could not be
/// read or migrated.
pub fn run(inputs: MigrateInputs) -> Result<i32> {
    if inputs.list {
        print!("{}", rewrite_table());
        return Ok(0);
    }
    if inputs.paths.is_empty() {
        eyre::bail!("name a contract file or a directory to search for *.contract.yaml");
    }
    let mut files = Vec::new();
    for p in &inputs.paths {
        if p.is_dir() {
            walk(p, &mut files)?;
        } else if p.is_file() {
            files.push(p.clone());
        } else {
            eyre::bail!("{}: no such file or directory", p.display());
        }
    }
    files.sort();
    files.dedup();

    let (mut changed, mut edits, mut failed) = (0usize, 0usize, 0usize);
    for f in &files {
        let src =
            std::fs::read_to_string(f).wrap_err_with(|| format!("reading {}", f.display()))?;
        let m = match migrate::migrate_str(&src) {
            Ok(m) => m,
            Err(e) => {
                eprintln!("error: {}: {e}", f.display());
                failed += 1;
                continue;
            }
        };
        for s in &m.skipped {
            eprintln!("warning: {}: not rewritten, edit by hand: {s}", f.display());
        }
        if !m.changed() {
            continue;
        }
        changed += 1;
        edits += m.edits.len();
        print_edits(f, &src, &m);
        if !inputs.quiet && !inputs.write {
            print!(
                "{}",
                unified_diff(&src, &m.output, &f.display().to_string())
            );
        }
        if inputs.write {
            std::fs::write(f, &m.output).wrap_err_with(|| format!("writing {}", f.display()))?;
        }
    }
    let verb = if inputs.write {
        "rewritten"
    } else {
        "to rewrite"
    };
    println!(
        "{} contract(s) scanned, {changed} {verb} ({edits} edit(s)){}",
        files.len(),
        if changed > 0 && !inputs.write {
            "; run with --write to apply"
        } else {
            ""
        }
    );
    if failed > 0 {
        return Ok(2);
    }
    Ok(if inputs.check && changed > 0 { 1 } else { 0 })
}

fn walk(dir: &Path, out: &mut Vec<PathBuf>) -> Result<()> {
    let mut entries: Vec<_> = std::fs::read_dir(dir)
        .wrap_err_with(|| format!("reading {}", dir.display()))?
        .filter_map(|e| e.ok())
        .collect();
    entries.sort_by_key(|e| e.file_name());
    for e in entries {
        let name = e.file_name().to_string_lossy().into_owned();
        let Ok(ft) = e.file_type() else { continue };
        if ft.is_dir() {
            if name.starts_with('.')
                || SKIP_DIRS
                    .iter()
                    .any(|s| name == *s || name.starts_with("build-"))
            {
                continue;
            }
            walk(&e.path(), out)?;
        } else if ft.is_file() && name.ends_with(".contract.yaml") {
            out.push(e.path());
        }
    }
    Ok(())
}

fn print_edits(file: &Path, src: &str, m: &Migration) {
    for e in &m.edits {
        println!(
            "{}:{}: {}: {} -> {} ({})",
            file.display(),
            e.line,
            e.rewrite,
            brief(&src[e.range.clone()]),
            brief(&e.replacement),
            e.path
        );
    }
}

/// An edit's text on one line: a key rename prints whole; a moved or
/// deleted block prints its first line and how many lines it spans.
fn brief(text: &str) -> String {
    let lines: Vec<&str> = text.lines().filter(|l| !l.trim().is_empty()).collect();
    match lines.as_slice() {
        [] => "(nothing)".to_string(),
        [one] => format!("`{}`", one.trim()),
        [first, ..] => format!("`{} ...` ({} lines)", first.trim(), lines.len()),
    }
}

/// The rewrite table, as `--list` prints it.
pub fn rewrite_table() -> String {
    let mut out = String::from("rewrites, applied in this order:\n");
    for r in REWRITES {
        out.push_str(&format!(
            "  {:<26} {} (v{}): {}  ->  {}\n  {:<26} when {}\n",
            r.id, r.unit, r.since, r.from, r.to, "", r.when
        ));
    }
    out
}

/// A unified diff of two texts, three lines of context, from a line-level
/// longest common subsequence. Contracts are a few thousand lines at most.
pub fn unified_diff(a: &str, b: &str, name: &str) -> String {
    let al: Vec<&str> = a.split_inclusive('\n').collect();
    let bl: Vec<&str> = b.split_inclusive('\n').collect();
    let (n, m) = (al.len(), bl.len());
    // lcs[i][j]: LCS length of al[i..], bl[j..].
    let mut lcs = vec![vec![0u32; m + 1]; n + 1];
    for i in (0..n).rev() {
        for j in (0..m).rev() {
            lcs[i][j] = if al[i] == bl[j] {
                lcs[i + 1][j + 1] + 1
            } else {
                lcs[i + 1][j].max(lcs[i][j + 1])
            };
        }
    }
    // The edit script: (' ' | '-' | '+', line); a deletion before an insertion.
    let mut ops: Vec<(char, &str)> = Vec::new();
    let (mut i, mut j) = (0, 0);
    while i < n || j < m {
        if i < n && j < m && al[i] == bl[j] {
            ops.push((' ', al[i]));
            i += 1;
            j += 1;
        } else if i < n && (j == m || lcs[i + 1][j] >= lcs[i][j + 1]) {
            ops.push(('-', al[i]));
            i += 1;
        } else {
            ops.push(('+', bl[j]));
            j += 1;
        }
    }
    const CTX: usize = 3;
    let mut out = format!("--- a/{name}\n+++ b/{name}\n");
    let changed: Vec<usize> = (0..ops.len()).filter(|&k| ops[k].0 != ' ').collect();
    let mut k = 0;
    while k < changed.len() {
        let start = changed[k].saturating_sub(CTX);
        let mut end = changed[k];
        while k < changed.len() && changed[k] <= end + 2 * CTX {
            end = changed[k];
            k += 1;
        }
        let end = (end + CTX + 1).min(ops.len());
        // Line numbers of the hunk's first line in each text.
        let a0 = ops[..start].iter().filter(|o| o.0 != '+').count();
        let b0 = ops[..start].iter().filter(|o| o.0 != '-').count();
        let alen = ops[start..end].iter().filter(|o| o.0 != '+').count();
        let blen = ops[start..end].iter().filter(|o| o.0 != '-').count();
        out.push_str(&format!("@@ -{},{alen} +{},{blen} @@\n", a0 + 1, b0 + 1));
        for (c, line) in &ops[start..end] {
            out.push(*c);
            out.push_str(line);
            if !line.ends_with('\n') {
                out.push_str("\n\\ No newline at end of file\n");
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_one_line_change_is_one_hunk_with_context() {
        let a = "1\n2\n3\n4\n5\n6\n7\n8\n9\n";
        let b = "1\n2\n3\n4\nfive\n6\n7\n8\n9\n";
        let d = unified_diff(a, b, "x");
        assert_eq!(
            d,
            "--- a/x\n+++ b/x\n@@ -2,7 +2,7 @@\n 2\n 3\n 4\n-5\n+five\n 6\n 7\n 8\n"
        );
    }

    #[test]
    fn identical_texts_have_no_hunk() {
        assert_eq!(unified_diff("a\n", "a\n", "x"), "--- a/x\n+++ b/x\n");
    }

    #[test]
    fn the_table_lists_every_rewrite() {
        let t = rewrite_table();
        for r in REWRITES {
            assert!(t.contains(r.id), "{t}");
        }
    }

    /// A directory walk finds contracts and skips build trees.
    #[test]
    fn migrate_writes_in_place_and_reports_nothing_the_second_time() {
        let dir = tempfile::tempdir().unwrap();
        let c = dir.path().join("a.contract.yaml");
        std::fs::write(
            &c,
            "version: 1\nmodes:\n  d:\n    fallback: [s]\n  s:\n    reaction: x.stop\n\
             hazards:\n  h:\n    reaction: d   # the ladder\n",
        )
        .unwrap();
        std::fs::create_dir(dir.path().join("build")).unwrap();
        std::fs::write(
            dir.path().join("build/b.contract.yaml"),
            "hazards: {h: {reaction: d}}\nmodes: {d: {}}\n",
        )
        .unwrap();
        let run_with = |write: bool, check: bool| {
            run(MigrateInputs {
                paths: vec![dir.path().to_path_buf()],
                write,
                check,
                ..Default::default()
            })
            .unwrap()
        };
        assert_eq!(run_with(false, true), 1);
        assert_eq!(run_with(true, false), 0);
        assert_eq!(
            std::fs::read_to_string(&c).unwrap(),
            "version: 1\nmodes:\n  d:\n    fallback: [s]\n  s:\n    reaction: x.stop\n\
             hazards:\n  h:\n    enters: d   # the ladder\n"
        );
        assert_eq!(run_with(false, true), 0);
        // The build tree was not entered.
        assert!(
            std::fs::read_to_string(dir.path().join("build/b.contract.yaml"))
                .unwrap()
                .contains("reaction: d")
        );
    }
}
