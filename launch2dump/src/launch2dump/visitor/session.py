from typing import List
from dataclasses import dataclass

from ..dump import LaunchDescriptionDump


@dataclass
class Session:
    launch_desc_list: List[LaunchDescriptionDump]
