"""P0 metrics (spec §17). Pure functions over NavigationRun; return None rather than nonsense."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from qrsimbench.analysis.model import NavigationRun

STOP_V = 0.02  # m/s
STOP_W = 0.03  # rad/s
MOVING_V = 0.05  # m/s, clearance only evaluated above this
MAX_STEP_M = 0.5  # pose jumps larger than this between samples are discarded


def path_length(x: np.ndarray, y: np.ndarray, max_step: float = MAX_STEP_M) -> float | None:
    if len(x) < 2:
        return None
    steps = np.hypot(np.diff(x), np.diff(y))
    steps = steps[np.isfinite(steps) & (steps <= max_step)]
    return float(steps.sum())


def straight_line(start: tuple[float, float], goal: tuple[float, float]) -> float:
    return math.hypot(goal[0] - start[0], goal[1] - start[1])


def path_efficiency(start, goal, executed: float | None) -> float | None:
    if not executed or executed <= 1e-6:
        return None
    return min(1.0, straight_line(start, goal) / executed)


def goal_error(x: np.ndarray, y: np.ndarray, goal: tuple[float, float]) -> float | None:
    if len(x) == 0:
        return None
    return float(math.hypot(x[-1] - goal[0], y[-1] - goal[1]))


def valid_scan_min(ranges, range_min: float, range_max: float) -> float:
    """Min finite range inside sensor limits; NaN when none."""
    r = np.asarray(ranges, dtype=float)
    r = r[np.isfinite(r) & (r >= range_min) & (r <= range_max)]
    return float(r.min()) if r.size else float("nan")


def _durations(t: np.ndarray) -> np.ndarray:
    """Time each sample holds until the next one (last sample gets 0)."""
    return np.append(np.diff(t), 0.0) if len(t) else t


def stop_stats(run: NavigationRun, warmup_s: float = 2.0, min_episode_s: float = 1.0) -> dict[str, Any]:
    t, v, w = run.vel_t, run.v, run.w
    end = run.mission_end if run.mission_end is not None else (t[-1] if len(t) else 0.0)
    m = (t >= run.mission_start + warmup_s) & (t <= end)
    t, v, w = t[m], v[m], w[m]
    if len(t) < 2:
        return {"stopped_time": None, "stop_episodes": None}
    stopped = (np.abs(v) < STOP_V) & (np.abs(w) < STOP_W)
    dt = _durations(t)
    episodes, run_len = 0, 0.0
    for s, d in zip(stopped, dt, strict=True):
        if s:
            run_len += d
        else:
            episodes += run_len >= min_episode_s
            run_len = 0.0
    episodes += run_len >= min_episode_s
    return {"stopped_time": float(dt[stopped].sum()), "stop_episodes": int(episodes)}


def clearance_stats(run: NavigationRun, threshold: float) -> dict[str, Any]:
    if len(run.scan_t) == 0 or len(run.vel_t) == 0:
        return {"min_clearance": None, "clearance_p5": None, "time_below_threshold": None}
    speed = np.interp(run.scan_t, run.vel_t, np.abs(run.v))
    moving = (speed > MOVING_V) & np.isfinite(run.scan_min)
    if not moving.any():
        return {"min_clearance": None, "clearance_p5": None, "time_below_threshold": None}
    vals = run.scan_min[moving]
    dt = _durations(run.scan_t)[moving]
    return {
        "min_clearance": float(vals.min()),
        "clearance_p5": float(np.percentile(vals, 5)),
        "time_below_threshold": float(dt[vals < threshold].sum()),
    }


def localization_error(run: NavigationRun) -> dict[str, Any]:
    if run.est_x is None or run.est_y is None or len(run.x) == 0:
        return {"localization_error_mean": None, "localization_error_max": None}
    e = np.hypot(run.est_x - run.x, run.est_y - run.y)
    return {"localization_error_mean": float(e.mean()), "localization_error_max": float(e.max())}


def _reference_length(run: NavigationRun) -> float:
    """Straight-line length through the goal chain (start -> waypoints... -> goal)."""
    pts = [run.start, *(run.waypoints or [run.goal])]
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:], strict=False)))


def compute_metrics(run: NavigationRun, clearance_threshold: float = 0.3) -> dict[str, Any]:
    if run.odom_x is not None and run.odom_y is not None:
        pl = path_length(run.odom_x, run.odom_y)
    else:
        pl = path_length(run.x, run.y)
    t_goal = None
    if run.result == "SUCCEEDED" and run.mission_end is not None:
        t_goal = run.mission_end - run.mission_start
    end = run.mission_end if run.mission_end is not None else np.inf
    in_mission = (run.vel_t >= run.mission_start) & (run.vel_t <= end)
    speeds = np.abs(run.v[in_mission])
    return {
        "success": run.result == "SUCCEEDED",
        "nav2_result": run.result,
        "time_to_goal": t_goal,
        "path_length": pl,
        "straight_line_distance": _reference_length(run),
        "path_efficiency": (min(1.0, _reference_length(run) / pl) if pl and pl > 1e-6 else None),
        "goal_error": goal_error(run.x, run.y, run.goal),
        "mean_speed": float(speeds.mean()) if speeds.size else None,
        "max_speed": float(speeds.max()) if speeds.size else None,
        **stop_stats(run),
        **clearance_stats(run, clearance_threshold),
        **localization_error(run),
    }
