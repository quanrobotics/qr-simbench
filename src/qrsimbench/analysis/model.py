"""Canonical run model (spec §11 layer 5). Every metric consumes this, never raw bags."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class NavigationRun:
    # Pose samples (odom or map frame), seconds since mission start.
    pose_t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    # Velocity samples (from /odom twist; falls back to /cmd_vel).
    vel_t: np.ndarray
    v: np.ndarray
    w: np.ndarray
    # LiDAR: per-scan minimum valid range (NaN when a scan has no valid return).
    scan_t: np.ndarray
    scan_min: np.ndarray
    goal: tuple[float, float]
    start: tuple[float, float]
    result: str  # Nav2 TaskResult name: SUCCEEDED / FAILED / CANCELED / TIMEOUT
    mission_start: float = 0.0
    mission_end: float | None = None
    events: list[dict] = field(default_factory=list)
    # Raw odom-frame positions (same samples as x/y). Smooth, so preferred for path length;
    # map-frame x/y contain small AMCL correction jumps.
    odom_x: np.ndarray | None = None
    odom_y: np.ndarray | None = None
    # Localization estimate (AMCL) for the same samples; x/y above are the ground-truth proxy.
    est_x: np.ndarray | None = None
    est_y: np.ndarray | None = None
    # Intermediate goals for multi-goal (task chain) missions, in order, ending at `goal`.
    waypoints: list[tuple[float, float]] | None = None
    robot_id: str = "robot_0"

    @staticmethod
    def arr(values) -> np.ndarray:
        return np.asarray(values, dtype=float)
