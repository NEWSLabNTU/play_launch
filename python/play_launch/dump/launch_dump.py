from dataclasses import dataclass


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


@dataclass
class ComposableNodeContainerRecord:
    name: str
    namespace: str


@dataclass
class LaunchDump:
    node: list[NodeRecord]
    load_node: list[LoadNodeRecord]
    container: list[ComposableNodeContainerRecord]
    lifecycle_node: list[str]
    file_data: dict[str, str]
