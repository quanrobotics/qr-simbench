"""Mission + scenario-event executor (QRB-007, QRB-010). Runs inside the ROS env as a subprocess:

    python -m qrsimbench.navigation.mission RUN_DIR [ROBOT_ID]

Reads RUN_DIR/scenario.resolved.yaml, sets the initial pose, waits for Nav2, sends the goal(s),
fires timed events, enforces the timeout, and writes RUN_DIR/mission.json + RUN_DIR/events.json.
Times are wall-clock UNIX seconds so they line up with rosbag2 receive timestamps.

Fleet coordination layers are optional plugins, registered under the `qrsimbench.fleet_layers`
entry point group (see `load_fleet_layer`). Without one, every fleet robot drives its own goals
with stock Nav2.
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import yaml

MAX_RETRIES = 2
START_STAGGER_S = 4.0    # s between fleet robots' departures

_BUILTIN_LAYERS = ("qrsimbench.navigation.nav2_commander",)  # present only in builds that ship it


def load_fleet_layer():
    """Return the installed fleet layer module, or None (stock Nav2 per robot).

    A fleet layer module provides `make_traffic(nav, sc, robot_id, run_dir, events_log, policy)` and may
    provide `nav2_overrides(resolved) -> dict` (extra Nav2 parameters merged into every robot's params).
    """
    import importlib
    from importlib.metadata import entry_points

    for ep in entry_points(group="qrsimbench.fleet_layers"):
        return importlib.import_module(ep.value.split(":")[0])
    for mod in _BUILTIN_LAYERS:
        try:
            return importlib.import_module(mod)
        except ImportError:
            continue
    return None


def _pose_stamped(nav, p):
    from geometry_msgs.msg import PoseStamped

    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = nav.get_clock().now().to_msg()
    ps.pose.position.x = float(p["x"])
    ps.pose.position.y = float(p["y"])
    ps.pose.orientation.z = math.sin(p.get("yaw", 0.0) / 2)
    ps.pose.orientation.w = math.cos(p.get("yaw", 0.0) / 2)
    return ps


def _due(trigger: dict, elapsed: float, robot_xy) -> bool:
    if trigger["type"] == "elapsed_time":
        return elapsed >= trigger["seconds"]
    if trigger["type"] == "robot_near" and robot_xy is not None:
        return math.hypot(robot_xy[0] - trigger["x"], robot_xy[1] - trigger["y"]) <= trigger["radius_m"]
    return False


def _wait_localized(nav, start_pose, timeout: float) -> bool:
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile

    got = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = nav.create_subscription(PoseWithCovarianceStamped, "amcl_pose", got.append, qos)
    deadline = time.time() + timeout
    try:
        while time.time() < deadline:
            got.clear()
            nav.setInitialPose(_pose_stamped(nav, start_pose))
            end = time.time() + 3.0
            while time.time() < end and not got:
                rclpy.spin_once(nav, timeout_sec=0.1)
            if got:
                p = got[-1].pose.pose.position
                if math.hypot(p.x - start_pose["x"], p.y - start_pose["y"]) < 0.5:
                    time.sleep(1.0)  # let costmaps pick up the map->odom transform
                    return True
        return False
    finally:
        nav.destroy_subscription(sub)


def main(run_dir: str, robot_id: str | None = None) -> int:
    """robot_id=None: single-robot run. Otherwise drive fleet member `robot_id` in its namespace."""
    import rclpy
    from nav2_simple_commander.robot_navigator import BasicNavigator

    from qrsimbench.adapters.gazebo.spawn import GazeboSpawner

    rd = Path(run_dir)
    sc = yaml.safe_load((rd / "scenario.resolved.yaml").read_text())
    mission = sc["mission"]
    timeout = float(mission["timeout_seconds"])
    if robot_id:
        member = next(m for m in sc["fleet"] if m["id"] == robot_id)
        start_pose, goals = member["start_pose"], member["goal_poses"]
        first_robot = sc["fleet"][0]["id"] == robot_id
        suffix = f"_{robot_id}"
    else:
        start_pose, goals, first_robot, suffix = mission["start_pose"], [mission["goal_pose"]], True, ""
    # Events with an explicit trigger.robot go to that robot's commander; the rest to the first robot.
    def mine(e: dict) -> bool:
        owner = e["trigger"].get("robot")
        return owner == robot_id if owner else first_robot

    # Fleet runs: events are fired by the manager-independent executor (qrsimbench.runner.events).
    my_events = [] if robot_id else [e for e in sc.get("events", []) if mine(e)]
    out = {"robot_id": robot_id, "result": None, "mission_start": None, "mission_end": None,
           "legs": [], "error": None}
    events_log: list[dict] = []

    def dump():
        (rd / f"mission{suffix}.json").write_text(json.dumps(out, indent=2))
        (rd / f"events{suffix}.json").write_text(json.dumps(events_log, indent=2))

    rclpy.init()
    nav = BasicNavigator(node_name=f"qr_commander{suffix}", namespace=robot_id or "")
    try:
        t0 = time.time()
        nav.setInitialPose(_pose_stamped(nav, start_pose))
        nav.waitUntilNav2Active()
        # AMCL can drop an initial pose sent before TF is up; re-send until AMCL publishes a pose.
        localized = _wait_localized(nav, start_pose, timeout=45.0)
        events_log.append({"id": "_nav2_active", "type": "lifecycle", "t": time.time(),
                           "detail": f"ready after {time.time() - t0:.1f}s, localized={localized}"})
        if not localized:
            raise RuntimeError("AMCL never published a pose after initial pose")
        spawner = GazeboSpawner(world=sc.get("resolved_profiles", {}).get("world", {}).get("gz_world"))
        pending = list(my_events)

        traffic = None
        policy = sc.get("fleet_policy") or {}
        max_retries = int(policy.get("retries", MAX_RETRIES))
        if robot_id:
            ids = [m["id"] for m in sc["fleet"]]
            if policy.get("reservations", True):
                layer = load_fleet_layer()
                if layer is not None:
                    traffic = layer.make_traffic(nav, sc, robot_id, rd, events_log, policy)
            time.sleep(float(policy.get("stagger_s", START_STAGGER_S)) * ids.index(robot_id))
        out["mission_start"] = time.time()
        deadline = out["mission_start"] + timeout
        for gi, goal in enumerate(goals):
            if traffic is not None:  # fleet layer: reserve what this goal needs (may wait)
                traffic.acquire(goal, gi, deadline)
            attempts = 0
            while True:  # retry policy: a failed leg is re-sent up to MAX_RETRIES times
                nav.goToPose(_pose_stamped(nav, goal))
                dump()
                leg = _drive(nav, out, pending, spawner, events_log, robot_id, timeout, dump, traffic)
                if leg in ("SUCCEEDED", "TIMEOUT") or attempts >= max_retries:
                    break
                attempts += 1
                events_log.append({"id": "_retry", "type": "recovery", "t": time.time(), "robot": robot_id,
                                   "detail": f"goal {gi} {leg}, retry {attempts}/{max_retries}"})
                time.sleep(2.0)
            if traffic is not None:
                traffic.end_leg()
            out["legs"].append({"goal_index": gi, "result": leg, "t": time.time(), "retries": attempts})
            if leg != "SUCCEEDED":
                out["result"] = leg
                break
        out["mission_end"] = time.time()
        out["result"] = out["result"] or "SUCCEEDED"
        dump()
        if traffic is not None and traffic.yielding:  # stay available to make room for the others
            traffic.linger("done" if out["result"] == "SUCCEEDED" else "failed", deadline)
    except KeyboardInterrupt:  # stopped by the runner's overall deadline
        out["result"] = out["result"] or ("TIMEOUT" if out["mission_start"] else "NAV_STACK_NOT_READY")
    except Exception as e:  # recorded, then reported as a nonzero exit
        out["error"] = repr(e)
        out["result"] = out["result"] or "NAV_STACK_NOT_READY"
    finally:
        dump()
        nav.destroy_node()
        rclpy.try_shutdown()
    return 0 if out["error"] is None else 1


def _drive(nav, out, pending, spawner, events_log, robot_id, timeout, dump, traffic=None) -> str:
    """Wait for the current goal, firing due events; returns the leg result."""
    from nav2_simple_commander.robot_navigator import TaskResult

    robot_xy, last_progress_xy = None, None
    while not nav.isTaskComplete():
        elapsed = time.time() - out["mission_start"]
        fb = nav.getFeedback()
        if fb is not None:
            robot_xy = (fb.current_pose.pose.position.x, fb.current_pose.pose.position.y)
            if traffic is not None and (last_progress_xy is None or math.hypot(
                    robot_xy[0] - last_progress_xy[0], robot_xy[1] - last_progress_xy[1]) > 0.2):
                traffic.progress(robot_xy)  # alive and moving: tell the fleet layer
                last_progress_xy = robot_xy
        for ev in [e for e in pending if _due(e["trigger"], elapsed, robot_xy)]:
            pending.remove(ev)
            ok, detail = spawner.apply(ev)
            events_log.append({"id": ev["id"], "type": ev["action"]["type"], "t": time.time(),
                               "elapsed": elapsed, "robot_xy": robot_xy, "robot": robot_id,
                               "ok": ok, "detail": detail})
            dump()
        if elapsed > timeout:
            nav.cancelTask()
            out["result"] = "TIMEOUT"
            return "TIMEOUT"
        time.sleep(0.1)
    r = nav.getResult()
    return {TaskResult.SUCCEEDED: "SUCCEEDED", TaskResult.CANCELED: "CANCELED",
            TaskResult.FAILED: "FAILED"}.get(r, str(r))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None))
