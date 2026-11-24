"""Scenario event executor for fleet runs, independent of the fleet manager.

    python -m qrsimbench.runner.events RUN_DIR

Triggers use simulator ground truth (elapsed time since the first mission start, or a robot's true
position) so every stack is perturbed identically. Actions: spawn/delete entities, disable a robot's
drive (breakdown) and re-initialise a robot's localization with an offset (localization kick).
Writes RUN_DIR/events_scenario.json.
"""

from __future__ import annotations

import json
import math
import signal
import sys
import threading
import time
from pathlib import Path

import yaml


def main(run_dir: str) -> int:
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_pb2 import Pose as GzPose
    from gz.transport13 import Node, SubscribeOptions
    from rclpy.qos import DurabilityPolicy, QoSProfile

    from qrsimbench.adapters.gazebo.spawn import GazeboSpawner

    rd = Path(run_dir)
    sc = yaml.safe_load((rd / "scenario.resolved.yaml").read_text())
    pending = list(sc.get("events") or [])
    ids = [m["id"] for m in sc.get("fleet") or []]
    if not pending:
        return 0
    gz = Node()
    truth: dict[str, tuple[float, float]] = {}
    opts = SubscribeOptions()
    opts.msgs_per_sec = 20
    for rid in ids:
        gz.subscribe(GzPose, f"/model/{rid}/pose",
                     lambda m, r=rid: truth.__setitem__(r, (m.position.x, m.position.y)), opts)
    rclpy.init()
    node = rclpy.create_node("qr_scenario_events")
    est: dict[str, tuple[float, float, float]] = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_est(r):
        def cb(m):
            q = m.pose.pose.orientation
            est[r] = (m.pose.pose.position.x, m.pose.pose.position.y,
                      math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        return cb

    init_pubs = {}
    for rid in ids:
        node.create_subscription(PoseWithCovarianceStamped, f"/{rid}/amcl_pose", on_est(rid), qos)
        init_pubs[rid] = node.create_publisher(PoseWithCovarianceStamped, f"/{rid}/initialpose", 10)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    spawner = GazeboSpawner(world=sc["resolved_profiles"]["world"].get("gz_world"))
    log: list[dict] = []
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())

    def mission_start() -> float | None:
        starts = []
        for rid in ids:
            f = rd / f"mission_{rid}.json"
            if f.exists():
                try:
                    s = json.loads(f.read_text()).get("mission_start")
                except json.JSONDecodeError:
                    s = None
                if s:
                    starts.append(s)
        return min(starts) if starts else None

    def due(trig: dict, elapsed: float) -> bool:
        if trig["type"] == "elapsed_time":
            return elapsed >= trig["seconds"]
        rid = trig.get("robot") or (ids[0] if ids else None)
        cands = list(truth.values()) if rid == "any" else [truth[rid]] if rid in truth else []
        return any(math.hypot(p[0] - trig["x"], p[1] - trig["y"]) <= trig["radius_m"] for p in cands)

    def apply(ev: dict) -> tuple[bool, str]:
        a = ev["action"]
        if a["type"] in ("spawn_entity", "delete_entity"):
            return spawner.apply(ev)
        if a["type"] == "disable_robot":
            # DiffDrive listens on /model/<name>/enable; false makes it ignore all velocity commands.
            pub = gz.advertise(f"/model/{a['robot']}/enable", Boolean)
            for _ in range(5):
                pub.publish(Boolean(data=False))
                time.sleep(0.05)
            return True, f"drive of {a['robot']} disabled"
        if a["type"] == "relocalize_robot":
            p = est.get(a["robot"])
            if p is None:
                return False, "no localization estimate yet"
            m = PoseWithCovarianceStamped()
            m.header.frame_id = "map"
            m.header.stamp = node.get_clock().now().to_msg()
            yaw = p[2] + a.get("dyaw", 0.0)
            m.pose.pose.position.x, m.pose.pose.position.y = p[0] + a.get("dx", 1.0), p[1] + a.get("dy", 0.0)
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.pose.covariance[0] = m.pose.covariance[7] = 0.05
            m.pose.covariance[35] = 0.05
            init_pubs[a["robot"]].publish(m)
            return True, f"localization of {a['robot']} re-initialised with offset ({a.get('dx')}, {a.get('dy')})"
        return False, f"unsupported action {a['type']}"

    while not stop.is_set() and pending:
        t0 = mission_start()
        if t0 is not None:
            elapsed = time.time() - t0
            for ev in [e for e in pending if due(e["trigger"], elapsed)]:
                pending.remove(ev)
                ok, detail = apply(ev)
                log.append({"id": ev["id"], "type": ev["action"]["type"], "t": time.time(), "elapsed": elapsed,
                            "ok": ok, "detail": detail, "robot": ev["action"].get("robot")})
                (rd / "events_scenario.json").write_text(json.dumps(log, indent=2))
        stop.wait(0.1)
    (rd / "events_scenario.json").write_text(json.dumps(log, indent=2))
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
