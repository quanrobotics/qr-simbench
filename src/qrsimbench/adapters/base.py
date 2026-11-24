"""Adapter contracts (spec §21). The OS core talks only to these; each simulator, navigation stack,
recorder and robot family plugs in behind them. v0.1 implements Gazebo Harmonic + Nav2 + rosbag2/MCAP
for differential-drive AMRs; the other robot families are declared so data and scenarios can already
name them.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Protocol, runtime_checkable


class RobotType(str, Enum):
    DIFF_DRIVE_AMR = "diff_drive_amr"  # implemented (Nav2)
    OMNI_AMR = "omni_amr"              # declared
    LEGGED = "legged"                  # declared (quadruped / humanoid locomotion)
    MANIPULATOR = "manipulator"        # declared (arm, mobile manipulator)
    HUMANOID = "humanoid"              # declared


IMPLEMENTED_ROBOT_TYPES = {RobotType.DIFF_DRIVE_AMR}


@dataclass
class RobotSpec:
    """What the OS knows about one robot instance in a run."""

    robot_id: str                      # unique within a run, e.g. "amr_1"
    robot_type: RobotType
    profile: str                       # profiles/robots/<profile>.yaml
    namespace: str = ""                # ROS namespace ("" for single-robot runs)
    sim_model_name: str = ""           # entity name in the simulator
    capabilities: list[str] = field(default_factory=list)  # e.g. ["navigate", "lidar_2d", "camera"]


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@runtime_checkable
class SimulatorAdapter(Protocol):
    name: str

    def start(self, config: dict[str, Any], run_dir: Path) -> None: ...
    def wait_ready(self, timeout: float) -> bool: ...
    def spawn_entity(self, asset: str, pose: dict[str, float], name: str) -> tuple[bool, str]: ...
    def delete_entity(self, name: str) -> tuple[bool, str]: ...
    def reset(self) -> None: ...
    def stop(self) -> None: ...


@runtime_checkable
class NavigationAdapter(Protocol):
    def wait_ready(self) -> None: ...
    def set_initial_pose(self, pose: Pose2D) -> None: ...
    def go_to_pose(self, goal: Pose2D, timeout: float) -> None: ...
    def cancel(self) -> None: ...
    def result(self) -> str: ...


@runtime_checkable
class RunRecorder(Protocol):
    def start(self, topics: list[str], path: Path) -> None: ...
    def stop(self) -> None: ...
