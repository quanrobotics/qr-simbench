"""MCAP → NavigationRun. Uses the pure-Python mcap readers, so analysis runs without ROS installed."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from qrsimbench.analysis.metrics import valid_scan_min
from qrsimbench.analysis.model import NavigationRun

GT_TOPIC = "/sim/ground_truth"


def find_mcap(run_dir: Path) -> Path | None:
    hits = sorted(run_dir.glob("bag/*.mcap")) + sorted(run_dir.glob("*.mcap"))
    return hits[0] if hits else None


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def to_map_frame(od: np.ndarray, map_odom: np.ndarray | None, start: dict) -> tuple[np.ndarray, np.ndarray]:
    """Odom poses -> map frame using AMCL's map->odom TF (latest one at or before each sample).

    Before the first map->odom transform (or if none was recorded) fall back to odom origin = start pose.
    """
    if map_odom is None or len(map_odom) == 0:
        map_odom = np.array([[-np.inf, start["x"], start["y"], start.get("yaw", 0.0)]])
    idx = np.searchsorted(map_odom[:, 0], od[:, 0], side="right") - 1
    fallback = np.array([start["x"], start["y"], start.get("yaw", 0.0)])
    T = np.where(idx[:, None] >= 0, map_odom[np.clip(idx, 0, None), 1:4], fallback)
    c, s = np.cos(T[:, 2]), np.sin(T[:, 2])
    return T[:, 0] + c * od[:, 1] - s * od[:, 2], T[:, 1] + s * od[:, 1] + c * od[:, 2]


def fleet_ids(resolved: dict) -> list[str]:
    return [m["id"] for m in resolved.get("fleet") or []]


def read_run(run_dir: Path, resolved: dict, robot_id: str | None = None,
             until: float | None = None) -> NavigationRun:
    """Read one robot's run. robot_id=None is a single-robot run (un-namespaced topics).

    Poses are kept from mission start to mission end, or to `until` (wall clock) when given: fleet
    analysis reads every robot up to the end of the whole fleet's shift, since a robot that has
    finished can still move (e.g. making room for another robot)."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    ns = f"/{robot_id}" if robot_id else ""
    mission = json.loads((run_dir / (f"mission_{robot_id}.json" if robot_id else "mission.json")).read_text())
    events = json.loads((run_dir / "events.json").read_text()) if (run_dir / "events.json").exists() else []
    bag = find_mcap(run_dir)
    if bag is None:
        raise FileNotFoundError(f"no .mcap under {run_dir}")

    lidar = resolved.get("resolved_profiles", {}).get("robot", {}).get("lidar", {})
    rmin, rmax = lidar.get("range_min_m", 0.0), lidar.get("range_max_m", float("inf"))
    odom, vel, scans, map_odom, truth = [], [], [], [], []
    with open(bag, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _schema, channel, message, msg in reader.iter_decoded_messages(
            topics=[f"{ns}/odom", f"{ns}/scan", f"{ns}/tf", f"{ns}{GT_TOPIC}"]
        ):
            t = message.log_time / 1e9
            topic = channel.topic[len(ns):]
            if topic == GT_TOPIC:
                p = msg.pose
                truth.append((t, p.position.x, p.position.y))
            elif topic == "/tf":
                for tf in msg.transforms:
                    if tf.header.frame_id == "map" and tf.child_frame_id == "odom":
                        tr = tf.transform
                        map_odom.append((t, tr.translation.x, tr.translation.y, _yaw(tr.rotation)))
            elif topic == "/odom":
                p = msg.pose.pose
                odom.append((t, p.position.x, p.position.y, _yaw(p.orientation)))
                vel.append((t, msg.twist.twist.linear.x, msg.twist.twist.angular.z))
            else:
                lo, hi = max(rmin, msg.range_min), min(rmax, msg.range_max)
                scans.append((t, valid_scan_min(msg.ranges, lo, hi)))

    t0 = mission["mission_start"]
    sp, gp = resolved["mission"]["start_pose"], resolved["mission"]["goal_pose"]
    waypoints = None
    if robot_id:
        member = next(m for m in resolved["fleet"] if m["id"] == robot_id)
        sp, gp = member["start_pose"], member["goal_poses"][-1]
        waypoints = [(g["x"], g["y"]) for g in member["goal_poses"]]
    od = np.array(odom) if odom else np.empty((0, 4))
    # Pose used for path/goal metrics: simulator odometry anchored at the spawn pose. In Gazebo the
    # diff-drive odometry has no wheel slip, so this is a ground-truth proxy. The AMCL estimate
    # (odom composed with map->odom) is kept separately to measure localization error.
    x, y = to_map_frame(od, None, sp)
    if len(truth) > 10:  # simulator ground truth (PosePublisher), resampled onto odom timestamps
        tr = np.array(truth)
        x, y = np.interp(od[:, 0], tr[:, 0], tr[:, 1]), np.interp(od[:, 0], tr[:, 0], tr[:, 2])
    ax_, ay_ = to_map_frame(od, np.array(map_odom) if map_odom else None, sp)
    m = od[:, 0] >= t0
    end = until if until is not None else mission.get("mission_end")
    if end:
        m &= od[:, 0] <= end
    ve = np.array(vel) if vel else np.empty((0, 3))
    sc = np.array(scans) if scans else np.empty((0, 2))
    return NavigationRun(
        pose_t=od[m, 0] - t0, x=x[m], y=y[m],
        vel_t=ve[:, 0] - t0, v=ve[:, 1], w=ve[:, 2],
        scan_t=sc[:, 0] - t0, scan_min=sc[:, 1],
        goal=(gp["x"], gp["y"]), start=(sp["x"], sp["y"]),
        result=mission["result"] or "FAILED",
        mission_start=0.0,
        mission_end=(mission["mission_end"] - t0) if mission.get("mission_end") else None,
        events=[{**e, "t_rel": e["t"] - t0} for e in events],
        odom_x=od[m, 1], odom_y=od[m, 2],
        est_x=ax_[m], est_y=ay_[m],
        waypoints=waypoints, robot_id=robot_id or "robot_0",
    )
