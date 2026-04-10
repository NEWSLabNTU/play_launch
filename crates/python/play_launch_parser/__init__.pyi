from typing import Optional, TypedDict

__version__: str

class ScopeOrigin(TypedDict, total=False):
    pkg: Optional[str]
    file: str

class ScopeEntry(TypedDict, total=False):
    id: int
    origin: Optional[ScopeOrigin]
    ns: str
    args: dict[str, str]
    parent: Optional[int]

class NodeRecord(TypedDict, total=False):
    name: Optional[str]
    namespace: Optional[str]
    package: Optional[str]
    executable: str
    exec_name: Optional[str]
    cmd: list[str]
    params: list[list[str]]
    params_files: list[str]
    remaps: list[list[str]]
    env: Optional[list[list[str]]]
    args: Optional[list[str]]
    ros_args: Optional[list[str]]
    global_params: Optional[list[list[str]]]
    respawn: Optional[bool]
    respawn_delay: Optional[float]
    scope: Optional[int]

class ContainerRecord(TypedDict, total=False):
    name: str
    namespace: str
    package: str
    executable: str
    exec_name: Optional[str]
    cmd: list[str]
    params: list[list[str]]
    params_files: list[str]
    remaps: list[list[str]]
    env: Optional[list[list[str]]]
    args: Optional[list[str]]
    ros_args: Optional[list[str]]
    global_params: Optional[list[list[str]]]
    respawn: Optional[bool]
    respawn_delay: Optional[float]
    scope: Optional[int]

class LoadNodeRecord(TypedDict, total=False):
    node_name: str
    namespace: str
    package: str
    plugin: str
    target_container_name: str
    params: list[list[str]]
    remaps: list[list[str]]
    extra_args: dict[str, str]
    env: Optional[list[list[str]]]
    log_level: Optional[str]
    scope: Optional[int]

class ParseResult(TypedDict):
    node: list[NodeRecord]
    container: list[ContainerRecord]
    load_node: list[LoadNodeRecord]
    scopes: list[ScopeEntry]
    variables: dict[str, str]
    file_data: dict[str, str]
    lifecycle_node: list[str]

def parse_file(
    path: str,
    args: Optional[dict[str, str]] = None,
) -> ParseResult:
    """Parse a ROS 2 launch file by path.

    Args:
        path: Path to the launch file (.launch.xml, .launch.py, .launch.yaml)
        args: Optional dict of launch arguments (e.g. {"vehicle_model": "sample"})

    Returns:
        dict with keys: node, container, load_node, scopes, variables,
        file_data, lifecycle_node

    Raises:
        FileNotFoundError: If the launch file does not exist.
        RuntimeError: If parsing fails.
    """
    ...

def parse_package(
    package: str,
    file: str,
    args: Optional[dict[str, str]] = None,
) -> ParseResult:
    """Parse a ROS 2 launch file by package name.

    Searches AMENT_PREFIX_PATH for the launch file.

    Args:
        package: ROS package name (e.g. "autoware_launch")
        file: Launch file name (e.g. "planning_simulator.launch.xml")
        args: Optional dict of launch arguments

    Returns:
        dict with keys: node, container, load_node, scopes, variables,
        file_data, lifecycle_node

    Raises:
        FileNotFoundError: If the package or file is not found,
            or AMENT_PREFIX_PATH is not set.
        RuntimeError: If parsing fails.
    """
    ...
