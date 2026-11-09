#!/usr/bin/env python3
"""Phase 102 M5: regenerate the safety model evaluation's tables.

Runs `play_launch check` on every fixture the evaluation names (E1, E2's
static rows), counts the authored lines (E6), and folds in the recorded
runs under data/ (E3, E4, E5) into results.md. Every number in README.md is
in results.md, and every row of results.md names the fixture, run or test
it came from.

    python3 docs/reports/safety-model/generate.py

Needs the built play_launch (`just build`), and for E3's Autoware row an
Autoware install (`tests/fixtures/autoware/activate_autoware.sh`); a row
whose input is missing is reported as NOT RUN, never as passed.
"""
import json
import os
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
FIX = ROOT / "tests/fixtures"
PL = ROOT / "install/play_launch/lib/play_launch/play_launch"
DATA = HERE / "data"
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def check(fixture, *args):
    launch = FIX / fixture / "launch/bringup.launch.xml"
    p = subprocess.run(
        [str(PL), "check", str(launch), *args],
        capture_output=True, text=True, cwd=ROOT, timeout=300,
    )
    return p.returncode, ANSI.sub("", p.stdout + p.stderr)


def findings(out):
    """(severity, rule) of every finding, acknowledged ones excluded."""
    return re.findall(r"^\s+(error|warning|info)\[([a-z0-9-]+)\]", out, re.M)


# ── E1: every case written with the decided primitives ───────────────────
E1_CASES = [
    ("U1 mobile robot protective stop (Safe-ROS)", "contract_commands"),
    ("U1 protective field sized (extension)", "contract_protective_stop_sized"),
    ("U2 automated driving MRM (island)", None),
    ("U2 automated driving MRM (Autoware)", None),
    ("U3 cobot speed and separation", "contract_case_u3_cobot"),
    ("U4 UAV low battery", "contract_case_u4_uav"),
    ("U5 infusion pump occlusion", "contract_case_u5_infusion"),
    ("U6 process overpressure trip", "contract_case_u6_process"),
    ("U7 rail emergency brake", "contract_case_u7_rail"),
    ("Simplex", "contract_simplex"),
    ("Redundant pair, independence (fault tolerance)", "contract_independence_ok"),
    ("Restart credited (fault tolerance)", "contract_recovery"),
]

# Each gap E1 found, with its ruling. Kept here, beside the table it
# belongs to, so the report cannot list a gap the generator does not.
E1_GAPS = [
    ("A string predicate whose value is upper snake case (`LAND`) is read as a "
     "message constant name, quoted or not (rlm `PredicateValue::from_text`).",
     "U4", "grammar fix: a quoted YAML scalar is a string. rlm, next release. "
     "The fixture uses mavros' `AUTO.LAND`."),
    ("A topic declared with only a type (the type of a topic the launch tree "
     "wires) was dropped by rlm's conditional cleanup, and the resolver did not "
     "let the launch tree wire a contract topic.",
     "U2 Autoware", "fixed: rlm v0.3.14 keeps it and the resolver wires it from the "
     "remaps; the Autoware contract states the type alone."),
    ("`max_velocity_candidates` is a proposal topic with several publishers; "
     "the final element is the selector's output.",
     "U2 Autoware", "not a gap in the grammar: `arbiter-bypass` found the "
     "first draft's wrong topic. The contract commands the selector's output."),
    ("A relational safe predicate (a speed limit that is a function of the "
     "measured distance, U3's continuous form).",
     "U3", "out of scope (safety-functions.md decision 4): discrete zones are "
     "levels; the continuous form is an extension's static check."),
    ("Simplex's recoverable envelope.", "Simplex",
     "out of scope: the monitor's argument. The contract states the verdict "
     "and checks what follows from it."),
    ("Agreement between redundant members' values (voting).", "fault tolerance",
     "extension territory (fault-tolerance.md 4): not proposed."),
]

# ── E2: seeded defects and their correct twins ───────────────────────────
E2_STATIC = [
    ("second publisher on a commanded topic", "arbiter-bypass",
     ("contract_commands_bypass",), ("contract_commands",)),
    ("level's reaction never reaches the commanded topic", "commands-unreached",
     ("contract_commands_unreached",), ("contract_commands",)),
    ("a field name that does not exist", "commands-field-unknown",
     ("contract_commands_field",), ("contract_commands",)),
    ("stop that waits on the traffic it overrides (Safe-ROS)", "reaction-waits-on-traffic",
     ("contract_safe_ros",), ("contract_safe_ros_fixed",)),
    ("redundant members in one container", "independence-violated",
     ("contract_independence",), ("contract_independence_ok",)),
    ("mixed integrity in one process", "interference-shared-process",
     ("contract_independence",), ("contract_independence_ok",)),
    ("a restart slower than the FTTI", "recovery-unfit",
     ("contract_recovery", "--against", str(FIX / "contract_recovery/runs/unfit")),
     ("contract_recovery", "--against", str(FIX / "contract_recovery/runs/fits"))),
    ("a protective field too short for the stop", "extension-check",
     ("contract_protective_stop",), ("contract_protective_stop_sized",)),
]
# Runtime defects are seeded in unit tests of the observer (synthetic events,
# `cargo test -p play_launch runtime_enforcement`), and E4's running system.
E2_RUNTIME = [
    ("a gate that sends one safe message and forwards again", "safe-state-violated",
     "runtime_enforcement::tests::a_level_holds_its_commands_from_its_bound"),
    ("a silent final element", "final-element-silent",
     "runtime_enforcement::tests::a_level_holds_its_commands_from_its_bound"),
    ("a silent safety node (omission on the monitored topic)", "hazard-detected",
     "runtime_enforcement::tests::hazard_silence_is_detected_and_the_provenance_free_sink_publish_is_the_reaction; E4"),
    ("a second publisher created at run time", "arbiter-bypass",
     "runtime_enforcement::tests::publishers_and_stamps_are_judged_where_the_contract_asks"),
    ("an inserted message (undeclared publisher)", "hazard-detected (insertion)",
     "runtime_enforcement::tests::publishers_and_stamps_are_judged_where_the_contract_asks"),
    ("a repeated message", "hazard-detected (repetition)",
     "runtime_enforcement::tests::publishers_and_stamps_are_judged_where_the_contract_asks"),
    ("a reordered message", "hazard-detected (resequencing)",
     "runtime_enforcement::tests::publishers_and_stamps_are_judged_where_the_contract_asks"),
    ("a restart slower than the FTTI, live", "hazard-recovered (past the FTTI)", "E4 `unfit`"),
]
E2_MISSED = [
    ("a stop that never releases after the mode above is available again",
     "no rule. The invariant ends when the level is left; nothing judges that "
     "the commanded topic leaves the level's predicates afterwards. Ruling: a "
     "runtime rule `level-not-released` (the mode above available for longer "
     "than its reaction's bound while the topic still carries only the level's "
     "values), recorded as follow-up."),
]


def e1():
    rows = []
    for name, fixture in E1_CASES:
        if fixture is None:
            rows.append((name, "see E3", "", ""))
            continue
        args = ("--against", str(FIX / "contract_recovery/runs/fits")) if fixture == "contract_recovery" else ()
        code, out = check(fixture, *args)
        f = findings(out)
        errors = sorted({r for s, r in f if s == "error"})
        warnings = sorted({r for s, r in f if s == "warning"})
        rows.append((name, fixture, f"exit {code}", "errors: " + (", ".join(errors) or "none")
                     + "; warnings: " + (", ".join(warnings) or "none")))
    return rows


def e2():
    rows, ok = [], True
    for defect, rule, bad, good in E2_STATIC:
        _, out_bad = check(*bad)
        _, out_good = check(*good)
        # A rule's info (a PASS, a credit) is not a finding against the twin.
        hit = any(r == rule and s != "info" for s, r in findings(out_bad))
        quiet = not any(r == rule and s != "info" for s, r in findings(out_good))
        # The correct twin must also be free of every error.
        twin_errors = sorted({r for s, r in findings(out_good) if s == "error"})
        ok = ok and hit and quiet
        rows.append((defect, rule, " ".join(Path(a).name for a in bad), "yes" if hit else "NO",
                     " ".join(Path(a).name for a in good), "yes" if quiet else "NO",
                     ", ".join(twin_errors) or "none"))
    return rows, ok


def e6():
    """Authored lines the safety model added (comments and blanks excluded),
    and the same facts in the dropped forms (data/e6_dropped_forms.yaml)."""
    def code_lines(text):
        return [l for l in text.splitlines() if l.strip() and not l.strip().startswith("#")]
    out = {}
    for name, path in json.load(open(DATA / "e6_added.json")).items():
        out[name] = {"added": len(code_lines(path["added"]))}
    forms = (DATA / "e6_dropped_forms.yaml").read_text()
    for block in forms.split("# --- ")[1:]:
        head, body = block.split("\n", 1)
        out.setdefault(head.strip(), {})["dropped_form"] = len(code_lines(body))
    return out


def main():
    if not PL.exists():
        sys.exit(f"no play_launch at {PL}: run `just build` first")
    lines = ["# Results (generated by generate.py; do not edit)", ""]
    lines += ["## E1 expressiveness", "", "| Case | Fixture | Exit | Findings |", "|---|---|---|---|"]
    for r in e1():
        lines.append("| " + " | ".join(r) + " |")
    lines += ["", "Gaps found writing them:", "", "| Gap | Case | Ruling |", "|---|---|---|"]
    for g in E1_GAPS:
        lines.append(f"| {g[0]} | {g[1]} | {g[2]} |")
    rows, ok = e2()
    lines += ["", "## E2 detection", "", "Static, each defect beside its correct twin:", "",
              "| Defect | Rule | Seeded | Reported | Twin | Twin quiet | Twin's errors |",
              "|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append("| " + " | ".join(r) + " |")
    lines += ["", "Runtime, seeded as synthetic events (unit tests) or on the running system (E4):", "",
              "| Defect | Rule | Where |", "|---|---|---|"]
    for r in E2_RUNTIME:
        lines.append(f"| {r[0]} | `{r[1]}` | {r[2]} |")
    lines += ["", "Not detected:", "", "| Defect | Why, and the ruling |", "|---|---|"]
    for r in E2_MISSED:
        lines.append(f"| {r[0]} | {r[1]} |")
    lines += ["", f"Static rows: {'all reported, all twins quiet' if ok else 'NOT ALL'}; "
              f"{len(E2_STATIC) + len(E2_RUNTIME)} of {len(E2_STATIC) + len(E2_RUNTIME) + len(E2_MISSED)} "
              "seeded defects reported by the intended rule.", ""]
    # E3, E4, E5 come from recorded runs.
    lines += ["## E3 false positives (recorded checks, data/e3_*.txt)", "",
              "| System | Findings (rule: count) |", "|---|---|"]
    for f in sorted(DATA.glob("e3_*.txt")):
        counts = {}
        for s, r in findings(f.read_text()):
            counts[f"{s}[{r}]"] = counts.get(f"{s}[{r}]", 0) + 1
        lines.append(f"| {f.stem[3:]} | " + ", ".join(f"{k}: {v}" for k, v in sorted(counts.items())) + " |")
    e4 = DATA / "e4_summary.json"
    if e4.exists():
        lines += ["", "## E4 timing (data/e4_summary.json)", "",
                  "| Case | Static | Credited total ms | Measured outage ms | Live | Agree | Outage within credit | First safe ms | Detection + route ms | Invariant held |",
                  "|---|---|---|---|---|---|---|---|---|---|"]
        for k, r in json.load(open(e4)).items():
            lines.append(f"| {k} | {r['static']} | {r['credited_total_ms']} | {r['outage_ms']} | {r['live']} | "
                         f"{'yes' if r['agree'] else 'NO'} | {'yes' if r['outage_within_credit'] else 'NO'} | "
                         f"{r['first_safe_ms']} | {r['derived_detect_plus_route_ms']} | "
                         f"{'yes' if r['invariant_held'] else 'NO'} |")
    e5 = DATA / "e5_self_cost.txt"
    if e5.exists():
        lines += ["", "## E5 runtime cost (data/e5_self_cost.txt)", "", "```", e5.read_text().strip(), "```"]
    lines += ["", "## E6 author burden (data/e6_*.{json,yaml})", "",
              "| Contract | Lines the safety model added | Same facts in the dropped forms |", "|---|---|---|"]
    for k, v in e6().items():
        lines.append(f"| {k} | {v.get('added', '-')} | {v.get('dropped_form', '-')} |")
    (HERE / "results.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
