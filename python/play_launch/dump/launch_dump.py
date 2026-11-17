import os
import re
import shlex
from dataclasses import dataclass, field


@dataclass
class ScopeOrigin:
    """Origin of a scope — identifies the launch file."""

    pkg: str | None
    file: str
    # Resolved absolute path of the launch file (Phase 40.1; needed for
    # provider-sidecar contract lookup). Matches the Rust parser's
    # `ScopeOrigin.path` field; both sides emit `null` (not omitted) when
    # unknown, mirroring how `pkg: None` is already serialized here via
    # `dataclasses.asdict`.
    path: str | None = None


@dataclass
class ScopeEntry:
    """A scope in the launch tree."""

    id: int
    origin: ScopeOrigin | None
    ns: str
    args: dict[str, str] = field(default_factory=dict)
    parent: int | None = None
    # Phase 94: the entries of `LaunchDump.config_log` made while this file
    # ran, [start, end) -- its include's arguments, its own declarations and
    # sets, and the files it included.
    config_span: tuple[int, int] | None = None

    def pkg(self) -> str | None:
        """Package name (if file scope with known package)."""
        return self.origin.pkg if self.origin else None

    def file(self) -> str | None:
        """File name (if file scope)."""
        return self.origin.file if self.origin else None

    def path(self) -> str | None:
        """Canonicalized absolute path of the launch file (if file scope)."""
        return self.origin.path if self.origin else None


def extract_package_from_path(path: str) -> str | None:
    """Extract ROS package name from an ament install path (/share/<pkg>/)."""
    idx = path.find("/share/")
    if idx < 0:
        return None
    after = path[idx + 7 :]  # skip "/share/"
    slash = after.find("/")
    if slash > 0:
        return after[:slash]
    return None


def process_prefix_and_cwd(process, context) -> tuple[str | None, list[str], str | None]:
    """A prepared process's prefix as text and as words, and its cwd (#0085, #0087).

    Mirrors `launch.descriptions.Executable.prepare`: the action's `prefix`,
    else the `launch-prefix` configuration when `launch-prefix-filter`
    `re.match`es the basename of the command's first word.
    """
    description = getattr(process, "_ExecuteLocal__process_description", None)
    if description is None:
        return None, [], None
    from launch.utilities import perform_substitutions

    # Performed here rather than read from `final_cwd`: a container is
    # recorded without its description being prepared.
    cwd = None
    if description.cwd is not None:
        cwd = "".join(context.perform_substitution(x) for x in description.cwd)
    prefix = perform_substitutions(context, description.prefix) if description.prefix else ""
    prefix_filter = getattr(description, "_Executable__prefix_filter", None)
    if prefix_filter is not None and prefix.strip():
        cmd0 = perform_substitutions(context, description.cmd[0]) if description.cmd else ""
        pattern = perform_substitutions(context, prefix_filter)
        if not re.match(pattern, os.path.basename(cmd0)):
            prefix = ""
    words = shlex.split(prefix)
    if not words:
        return None, [], cwd
    return prefix, words, cwd


@dataclass
class NodeRecord:
    executable: str
    package: str | None
    name: str | None
    namespace: str | None
    exec_name: str | None
    params: list[tuple[str, str]]
    params_files: list[str]
    remaps: list[tuple[str, str]]
    ros_args: list[str] | None
    args: list[str] | None
    cmd: list[str]
    env: list[tuple[str, str]] | None = None
    respawn: bool | None = None
    respawn_delay: float | None = None
    # on_exit=Shutdown(): when this node exits, the whole launch goes with it.
    on_exit_shutdown: bool | None = None
    # LifecycleNode (phase 86): the marker, and its resolved Jazzy `autostart`.
    lifecycle: bool | None = None
    lifecycle_autostart: bool | None = None
    global_params: list[tuple[str, str]] | None = None  # From SetParameter action
    scope: int | None = None
    # Phase 94: how many `config_log` entries had happened where this member
    # was declared -- the point a contract's `$(var x @<node>)` reads at.
    config_seq: int | None = None
    # What the process starts in and under (#0085, #0087): the variables
    # launch unset (inherited ones included), its prefix, its cwd.
    env_unset: list[str] = field(default_factory=list)
    launch_prefix: str | None = None
    cwd: str | None = None
    # How it ends (#0084, #0090): seconds from SIGINT to SIGTERM and from
    # SIGTERM to SIGKILL (None is launch's 5), and the exit codes an `on_exit`
    # function shuts the launch down on (`{"only": [...]}`/`{"except": [...]}`).
    sigterm_timeout: float | None = None
    sigkill_timeout: float | None = None
    shutdown_on_exit_codes: dict | None = None


@dataclass
class LoadNodeRecord:
    package: str
    plugin: str
    target_container_name: str
    node_name: str
    namespace: str
    log_level: str | None
    remaps: list[tuple[str, str]]
    params: list[tuple[str, str]]
    extra_args: dict[str, str]
    env: list[tuple[str, str]] | None = None
    scope: int | None = None
    # Phase 94: how many `config_log` entries had happened where this member
    # was declared -- the point a contract's `$(var x @<node>)` reads at.
    config_seq: int | None = None


@dataclass
class ComposableNodeContainerRecord:
    # All fields from NodeRecord to avoid duplication
    executable: str
    package: str
    name: str
    namespace: str
    exec_name: str | None
    params: list[tuple[str, str]]
    params_files: list[str]
    remaps: list[tuple[str, str]]
    ros_args: list[str] | None
    args: list[str] | None
    cmd: list[str]
    env: list[tuple[str, str]] | None = None
    respawn: bool | None = None
    respawn_delay: float | None = None
    # A container is a Node: on_exit=Shutdown(), or an OnProcessExit shutdown
    # handler targeting it (#0081), makes it required.
    on_exit_shutdown: bool | None = None
    global_params: list[tuple[str, str]] | None = None
    scope: int | None = None
    # Phase 94: how many `config_log` entries had happened where this member
    # was declared -- the point a contract's `$(var x @<node>)` reads at.
    config_seq: int | None = None
    # What the process starts in and under (#0085, #0087): the variables
    # launch unset (inherited ones included), its prefix, its cwd.
    env_unset: list[str] = field(default_factory=list)
    launch_prefix: str | None = None
    cwd: str | None = None
    # How it ends (#0084, #0090): seconds from SIGINT to SIGTERM and from
    # SIGTERM to SIGKILL (None is launch's 5), and the exit codes an `on_exit`
    # function shuts the launch down on (`{"only": [...]}`/`{"except": [...]}`).
    sigterm_timeout: float | None = None
    sigkill_timeout: float | None = None
    shutdown_on_exit_codes: dict | None = None


@dataclass
class LaunchDump:
    node: list[NodeRecord]
    load_node: list[LoadNodeRecord]
    container: list[ComposableNodeContainerRecord]
    lifecycle_node: list[str]
    file_data: dict[str, str]
    scopes: list[ScopeEntry] = field(default_factory=list)
    # Phase 94: every change to the launch configurations, in order.
    config_log: list[dict] = field(default_factory=list)
    _scope_stack: list[int] = field(default_factory=list, repr=False)
    # The dumper's own environment, before any `<set_env>` ran (#0077).
    _base_env: dict[str, str] = field(default_factory=lambda: dict(os.environ), repr=False)

    # #0081: `RegisterEventHandler(OnProcessExit(target_action=X, on_exit=
    # Shutdown()))` makes X required, as `X(on_exit=Shutdown())` does. The
    # handler may run before or after X, so both sides are remembered, keyed
    # by object identity. Plain attributes, not fields: `dataclasses.asdict`
    # would deep-copy the launch actions they hold.
    # #0090: an `on_exit` function shuts down on some exits only; `on` is
    # "always" or the set of exit codes (`exit_probe`).
    def note_process(self, action, records) -> None:
        """`action` (a process) produced `records`; mark them if a handler wants it."""
        from .exit_probe import mark

        processes = self.__dict__.setdefault("_process_records", {})
        targets = self.__dict__.setdefault("_exit_shutdown_targets", {})
        processes[id(action)] = (action, records)
        pending = targets.pop(id(action), None)
        if pending is not None:
            for record in records:
                mark(record, pending[1])

    def shutdown_on_exit_of(self, action, on="always") -> None:
        """An OnProcessExit handler shuts the launch down when `action` exits."""
        from .exit_probe import mark, merge

        processes = self.__dict__.setdefault("_process_records", {})
        targets = self.__dict__.setdefault("_exit_shutdown_targets", {})
        entry = processes.get(id(action))
        if entry is None:
            had = targets.get(id(action))
            targets[id(action)] = (action, merge(had[1] if had else None, on))
            return
        for record in entry[1]:
            mark(record, on)

    def set_env_in_scope(self, context) -> dict[str, str]:
        """The variables `<set_env>` (`SetEnvironmentVariable`) set in scope (#0077).

        launch starts a process with `context.environment` (on Jazzy
        `os.environ` itself, which `<set_env>` writes and a scoped group
        restores) and lays its `additional_env` over it. A process inherits the
        dumper's own environment at replay anyway, so only what differs from
        it is recorded, as the Rust parser records its `<set_env>`s. An
        `<unset_env>` of an inherited variable cannot be represented.
        """
        env = getattr(context, "environment", os.environ)
        return {k: v for k, v in env.items() if self._base_env.get(k) != v}

    def unset_env_in_scope(self, context, own_env) -> list[str]:
        """The variables `<unset_env>` (`UnsetEnvironmentVariable`) removed (#0085).

        launch deletes them from `context.environment`, which began as a copy
        of the dumper's own environment, so an inherited variable absent from
        it now was unset. A name the process's own environment sets again is
        not removed from that process.
        """
        env = getattr(context, "environment", os.environ)
        own = set(own_env)
        return sorted(k for k in self._base_env if k not in env and k not in own)

    def log_config(
        self,
        kind: str,
        name: str | None = None,
        value: str | None = None,
        origin: str | None = None,
        file: str | None = None,
    ) -> None:
        """Append one configuration event (`set`, `unset`, `push`, `pop`, `reset`)."""
        event = {"kind": kind}
        for key, v in (("name", name), ("value", value), ("origin", origin), ("file", file)):
            if v is not None:
                event[key] = v
        self.config_log.append(event)

    @property
    def current_file(self) -> str | None:
        """The launch file being run: the current scope's file."""
        sid = self.current_scope_id
        return self.scopes[sid].file() if sid is not None else None

    def push_scope(
        self,
        pkg: str | None,
        filename: str,
        ns: str,
        args: dict[str, str] | None = None,
        path: str | None = None,
    ) -> int:
        """Push a new scope entry. Returns the scope ID."""
        scope_id = len(self.scopes)
        parent = self._scope_stack[-1] if self._scope_stack else None
        origin = ScopeOrigin(pkg=pkg, file=filename, path=path) if filename else None
        self.scopes.append(
            ScopeEntry(
                id=scope_id,
                origin=origin,
                ns=ns,
                args=args or {},
                parent=parent,
                config_span=(len(self.config_log), len(self.config_log)),
            )
        )
        self._scope_stack.append(scope_id)
        return scope_id

    def pop_scope(self) -> None:
        """Pop the current scope."""
        if self._scope_stack:
            sid = self._scope_stack.pop()
            span = self.scopes[sid].config_span
            if span is not None:
                self.scopes[sid].config_span = (span[0], len(self.config_log))

    @property
    def current_scope_id(self) -> int | None:
        """Return the current scope ID, or None if no scope is active."""
        return self._scope_stack[-1] if self._scope_stack else None
