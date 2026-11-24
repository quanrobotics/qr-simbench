"""Run bundle → episode table (resampled, time-aligned signals) + EpisodeMeta."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import yaml

from qrsimbench.analysis.bag_reader import GT_TOPIC, find_mcap, read_run
from qrsimbench.robotdata.schema import EPISODE_HZ, SIGNALS, EpisodeMeta


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _read_extra(bag: Path, ns: str = "") -> tuple[np.ndarray, np.ndarray]:
    """(cmd_vel [t, v, w], truth yaw [t, yaw]) — not part of NavigationRun."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    cmd, yaw = [], []
    with open(bag, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _s, ch, m, msg in reader.iter_decoded_messages(topics=[f"{ns}/cmd_vel", f"{ns}{GT_TOPIC}"]):
            t = m.log_time / 1e9
            if ch.topic == f"{ns}/cmd_vel":
                tw = getattr(msg, "twist", msg)  # Twist or TwistStamped
                cmd.append((t, tw.linear.x, tw.angular.z))
            else:
                yaw.append((t, _yaw(msg.pose.orientation)))
    return (np.array(cmd) if cmd else np.empty((0, 3))), (np.array(yaw) if yaw else np.empty((0, 2)))


def _interp(t: np.ndarray, ts: np.ndarray, vs: np.ndarray, hold: bool = False) -> np.ndarray:
    if len(ts) == 0:
        return np.full_like(t, np.nan)
    if hold:  # zero-order hold (commands)
        idx = np.clip(np.searchsorted(ts, t, side="right") - 1, 0, len(ts) - 1)
        out = vs[idx]
        out[t < ts[0]] = 0.0
        return out
    return np.interp(t, ts, vs)


def extract_episode(run_dir: Path, robot_id: str = "robot_0") -> tuple[dict[str, np.ndarray], EpisodeMeta]:
    resolved = yaml.safe_load((run_dir / "scenario.resolved.yaml").read_text())
    man = json.loads((run_dir / "manifest.json").read_text())
    met = json.loads((run_dir / "metrics.json").read_text())
    fleet = bool(resolved.get("fleet"))
    mission = json.loads((run_dir / (f"mission_{robot_id}.json" if fleet else "mission.json")).read_text())
    run = read_run(run_dir, resolved, robot_id if fleet else None)
    cmd, tyaw = _read_extra(find_mcap(run_dir), f"/{robot_id}" if fleet else "")
    if fleet:  # per-robot outcome instead of the fleet summary
        met = {**met, **met.get("robots", {}).get(robot_id, {})}
    t0 = mission["mission_start"]

    end = run.mission_end if run.mission_end is not None else (run.pose_t[-1] if len(run.pose_t) else 0.0)
    t = np.arange(0.0, max(end, 0.0) + 1e-9, 1.0 / EPISODE_HZ)
    yaw_t, yaw_v = (tyaw[:, 0] - t0, np.unwrap(tyaw[:, 1])) if len(tyaw) else (np.array([]), np.array([]))
    table = {
        "timestamp": t,
        "x": _interp(t, run.pose_t, run.x),
        "y": _interp(t, run.pose_t, run.y),
        "yaw": (np.mod(_interp(t, yaw_t, yaw_v) + np.pi, 2 * np.pi) - np.pi),
        "est_x": _interp(t, run.pose_t, run.est_x) if run.est_x is not None else np.full_like(t, np.nan),
        "est_y": _interp(t, run.pose_t, run.est_y) if run.est_y is not None else np.full_like(t, np.nan),
        "v": _interp(t, run.vel_t, run.v),
        "w": _interp(t, run.vel_t, run.w),
        "cmd_v": _interp(t, cmd[:, 0] - t0, cmd[:, 1], hold=True) if len(cmd) else np.zeros_like(t),
        "cmd_w": _interp(t, cmd[:, 0] - t0, cmd[:, 2], hold=True) if len(cmd) else np.zeros_like(t),
        "scan_min": _interp(t, run.scan_t[np.isfinite(run.scan_min)],
                            run.scan_min[np.isfinite(run.scan_min)]),
    }
    table["goal_dist"] = np.hypot(table["x"] - run.goal[0], table["y"] - run.goal[1])
    assert set(table) == set(SIGNALS), "episode columns must match the schema"

    robot = resolved.get("resolved_profiles", {}).get("robot", {})
    m = met.get("metrics", {})
    meta = EpisodeMeta(
        episode_id=f"{man['run_id']}/{robot_id}",
        run_id=man["run_id"],
        robot_id=robot_id,
        robot_type=robot.get("robot_type", "diff_drive_amr"),
        robot_profile=resolved["robot"]["profile"],
        scenario=man["scenario_name"],
        world=resolved["world"]["id"],
        seed=man["seed"],
        started_at=man["started_at"],
        duration_s=float(t[-1]) if len(t) else 0.0,
        n_samples=len(t),
        hz=EPISODE_HZ,
        success=bool(m.get("success")),
        verdict=met.get("verdict", "FAIL"),
        codes=met.get("codes", []),
        metrics={k: v for k, v in m.items() if isinstance(v, (int, float, bool, str)) or v is None},
        events=[{"id": e["id"], "t": e["t"] - t0, "type": e.get("type")} for e in run.events
                if not e["id"].startswith("_")],
        git_commit=man.get("git_commit"),
        scenario_sha256=man["scenario_sha256"],
        resolved_sha256=man["resolved_sha256"],
        files={},
        tags=[man["scenario_name"], met.get("verdict", "FAIL").lower()],
    )
    return table, meta
