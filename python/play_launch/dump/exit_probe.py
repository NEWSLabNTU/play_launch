"""How a process ends: its shutdown timeouts (#0084) and an `on_exit` function (#0090).

The Rust parser's `pyexec` does the same (`api/exit_probe.rs`,
`actions/process.rs`); the records must agree, since the parity gate compares
the models.

`launch` calls an `on_exit` function with the exit event when the process
exits and runs what it returns. play_launch's runtime has no Python, so the
function is called here, once for every code a process can exit with (a status
0 to 255, or `-N` for a death by signal N), and the codes it returns a shutdown
for are recorded. A function that raises, or answers differently when asked
twice (it keeps state), is refused with a warning.
"""

import contextlib
import io
import types

import launch.logging

ALL_EXIT_CODES = list(range(0, 256)) + [-s for s in range(1, 65)]
DEFAULT_STOP_TIMEOUT = 5.0


def _logger():
    return launch.logging.get_logger("dump_launch")


def stop_timeouts(process, context) -> tuple[float | None, float | None]:
    """`sigterm_timeout` and `sigkill_timeout` as launch performs them, None for 5.

    launch's default for each is `LaunchConfiguration('<name>', default=5)`, so
    performing the attribute already falls back to the launch configuration.
    """

    def one(attr, name):
        subs = getattr(process, attr, None)
        if subs is None:
            return None
        text = "".join(context.perform_substitution(s) for s in subs)
        try:
            secs = float(text)
        except ValueError as e:
            raise ValueError(f"{name} '{text}' is not a number of seconds") from e
        return None if secs == DEFAULT_STOP_TIMEOUT else secs

    return (
        one("_ExecuteLocal__sigterm_timeout", "sigterm_timeout"),
        one("_ExecuteLocal__sigkill_timeout", "sigkill_timeout"),
    )


def _shuts_down(entity) -> bool:
    from launch.actions import EmitEvent
    from launch.events import Shutdown as ShutdownEvent

    event = getattr(entity, "_EmitEvent__event", None)
    return isinstance(entity, EmitEvent) and isinstance(event, ShutdownEvent)


def _answer(result) -> tuple[bool, list[str]]:
    from launch.actions import LogInfo

    if result is None:
        return False, []
    items = list(result) if isinstance(result, (list, tuple)) else [result]
    shuts_down = False
    others = []
    for item in items:
        if _shuts_down(item):
            shuts_down = True
        elif item is not None and not isinstance(item, LogInfo):
            others.append(type(item).__name__)
    return shuts_down, others


def _call_all(fn, action, context, name):
    answers = []
    for code in ALL_EXIT_CODES:
        event = types.SimpleNamespace(
            action=action,
            execute_process_action=action,
            name=name,
            process_name=name,
            cmd=[],
            cwd=None,
            env=None,
            pid=0,
            returncode=code,
        )
        answers.append(_answer(fn(event, context)))
    return answers


def probe(fn, action, context, what: str):
    """Evaluate `fn` for every exit code.

    Returns "always" (every exit shuts down), a set of codes, or None (none
    does, or the function cannot be evaluated: warned about).
    """
    sink = io.StringIO()
    try:
        with contextlib.redirect_stdout(sink), contextlib.redirect_stderr(sink):
            first = _call_all(fn, action, context, what)
            second = _call_all(fn, action, context, what)
    except Exception as e:  # noqa: BLE001 -- any failure refuses the function
        _refuse(what, f"it raised {type(e).__name__}: {e}")
        return None
    if any(a[0] != b[0] for a, b in zip(first, second, strict=True)):
        _refuse(
            what,
            "it answered differently when called twice with the same exit code, so it "
            "depends on more than the exit",
        )
        return None
    others = sorted({o for _, os_ in first for o in os_})
    if others:
        _logger().warning(
            f"The on_exit function of {what} returns {', '.join(others)} on some exits, "
            "which play_launch does not run; only a shutdown it returns is honoured."
        )
    codes = {code for code, (down, _) in zip(ALL_EXIT_CODES, first, strict=True) if down}
    if not codes:
        return None
    if len(codes) == len(ALL_EXIT_CODES):
        return "always"
    return codes


def _refuse(what, why):
    _logger().warning(
        f"The on_exit function of {what} cannot be evaluated at parse time ({why}). "
        "play_launch evaluates such a function for every exit code when it parses the "
        "launch file, since it runs no Python when the process exits; this one's exits "
        "will not end the launch."
    )


def codes_record(codes: set[int]) -> dict:
    """The record's shorter spelling: `{"only": [...]}` or `{"except": [...]}`."""
    only = sorted(codes)
    rest = sorted(c for c in ALL_EXIT_CODES if c not in codes)
    return {"only": only} if len(only) <= len(rest) else {"except": rest}


def record_codes(spelled: dict | None) -> set[int]:
    """The codes a record's spelling holds."""
    if not spelled:
        return set()
    if "only" in spelled:
        return set(spelled["only"])
    return {c for c in ALL_EXIT_CODES if c not in set(spelled["except"])}


def mark(record, on) -> None:
    """The exits `on` ("always" or a set of codes) end the launch for `record`."""
    if record.on_exit_shutdown:
        return
    if on == "always":
        record.on_exit_shutdown = True
        record.shutdown_on_exit_codes = None
        return
    codes = record_codes(record.shutdown_on_exit_codes) | set(on)
    if len(codes) == len(ALL_EXIT_CODES):
        record.on_exit_shutdown = True
        record.shutdown_on_exit_codes = None
    else:
        record.shutdown_on_exit_codes = codes_record(codes)


def merge(a, b):
    """Two handlers' exits, as one."""
    if a is None:
        return b
    if a == "always" or b == "always":
        return "always"
    return set(a) | set(b)
