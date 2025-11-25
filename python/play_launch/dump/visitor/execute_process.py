from typing import List, Optional

from launch.actions import ExecuteProcess
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity

from ..launch_dump import LaunchDump
from .execute_local import visit_execute_local


def visit_execute_process(
    process: ExecuteProcess, context: LaunchContext, dump: LaunchDump
) -> Optional[List[LaunchDescriptionEntity]]:
    visit_execute_local(process, context, dump)
