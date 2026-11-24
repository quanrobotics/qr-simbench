"""QR-RobotData episode schema v0.1.

An *episode* is one robot's data for one run: time-aligned state, action and sensor-summary signals
plus metadata (scenario, provenance, outcome). Raw data stays in the run's MCAP; episodes are the
analysis- and learning-ready view of it, indexed in the local catalogue.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

DATA_SCHEMA_VERSION = "qr-robotdata/0.1"
EPISODE_HZ = 10.0

# Column layout of an episode table. Units: m, rad, s, m/s, rad/s.
SIGNALS: dict[str, str] = {
    "timestamp": "seconds since mission start",
    "x": "true x (simulator ground truth, world/map frame)",
    "y": "true y",
    "yaw": "true heading",
    "est_x": "localization estimate x (AMCL)",
    "est_y": "localization estimate y",
    "v": "measured linear velocity (odometry)",
    "w": "measured angular velocity (odometry)",
    "cmd_v": "commanded linear velocity (/cmd_vel)",
    "cmd_w": "commanded angular velocity",
    "scan_min": "minimum valid LiDAR range (clearance proxy)",
    "goal_dist": "distance from true position to goal",
}

# LeRobot-style feature mapping (state / action vectors), used by the exporter.
STATE_KEYS = ["x", "y", "yaw", "v", "w"]
ACTION_KEYS = ["cmd_v", "cmd_w"]


@dataclass
class EpisodeMeta:
    episode_id: str                 # "<run_id>/<robot_id>"
    run_id: str
    robot_id: str
    robot_type: str
    robot_profile: str
    scenario: str
    world: str
    seed: int
    started_at: str
    duration_s: float
    n_samples: int
    hz: float
    success: bool
    verdict: str
    codes: list[str]
    metrics: dict[str, Any]
    events: list[dict[str, Any]]
    git_commit: str | None
    scenario_sha256: str
    resolved_sha256: str
    files: dict[str, str]           # relative paths: table, bag, manifest, report
    tags: list[str] = field(default_factory=list)
    schema_version: str = DATA_SCHEMA_VERSION

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
