from dataclasses import dataclass, field


@dataclass
class ScopeEntry:
    """A scope in the launch include tree."""

    id: int
    pkg: str | None
    file: str
    ns: str
    args: dict[str, str] = field(default_factory=dict)
    parent: int | None = None


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
    global_params: list[tuple[str, str]] | None = None  # From SetParameter action
    scope: int | None = None


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
    global_params: list[tuple[str, str]] | None = None
    scope: int | None = None


@dataclass
class LaunchDump:
    node: list[NodeRecord]
    load_node: list[LoadNodeRecord]
    container: list[ComposableNodeContainerRecord]
    lifecycle_node: list[str]
    file_data: dict[str, str]
    scopes: list[ScopeEntry] = field(default_factory=list)
    _scope_stack: list[int] = field(default_factory=list, repr=False)

    def push_scope(
        self, pkg: str | None, filename: str, ns: str, args: dict[str, str] | None = None
    ) -> int:
        """Push a new scope entry. Returns the scope ID."""
        scope_id = len(self.scopes)
        parent = self._scope_stack[-1] if self._scope_stack else None
        self.scopes.append(
            ScopeEntry(
                id=scope_id,
                pkg=pkg,
                file=filename,
                ns=ns,
                args=args or {},
                parent=parent,
            )
        )
        self._scope_stack.append(scope_id)
        return scope_id

    def pop_scope(self) -> None:
        """Pop the current scope."""
        if self._scope_stack:
            self._scope_stack.pop()

    @property
    def current_scope_id(self) -> int | None:
        """Return the current scope ID, or None if no scope is active."""
        return self._scope_stack[-1] if self._scope_stack else None
