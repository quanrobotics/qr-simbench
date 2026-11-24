"""Drive moving actors (workers, forklifts) along looped paths in a running Gazebo world.

    python -m qrsimbench.adapters.gazebo.actors RUN_DIR

Reads the actors from the run's frozen scenario (world layout), moves them with the world's
set_pose_vector service at 20 Hz, and logs their true positions to RUN_DIR/actors.jsonl at 10 Hz
(wall-clock time, aligned with the bag) so analysis can measure robot-to-person separation.
"""

from __future__ import annotations

import json
import math
import signal
import sys
import threading
import time
import zlib
from pathlib import Path

import numpy as np
import yaml

TICK_HZ = 20.0
# Actors pause while any robot is within this distance (people make way, drivers stop for AMRs).
YIELD_M = {"worker": 1.3, "forklift": 2.4}
# With actor yielding on (Studio product setting), centre distances between actors:
# - a forklift stops for a worker that is within FORKLIFT_SEES_WORKER_M and getting closer (pedestrians first);
# - nobody steps closer than ACTOR_CLEAR_M to another actor (forklift body + forks ~0.85 m from its centre,
#   worker ~0.3 m), so a worker never walks into a standing forklift and workers do not walk through each other.
FORKLIFT_SEES_WORKER_M = 2.2
ACTOR_CLEAR_M = {frozenset(["forklift", "worker"]): 1.25, frozenset(["worker"]): 0.7,
                 frozenset(["forklift"]): 2.2}
YIELD_TURN_S = 4.0  # after yielding this long, the actor turns around
LOG_HZ = 10.0


class PathFollower:
    """Constant-speed motion along a closed polyline (2-point paths go back and forth)."""

    def __init__(self, path: list[list[float]], speed: float):
        pts = np.asarray(path, dtype=float)
        if len(pts) == 2:
            pts = np.vstack([pts, pts[:1]])
        elif not np.allclose(pts[0], pts[-1]):
            pts = np.vstack([pts, pts[:1]])
        self.pts = pts
        seg = np.hypot(*np.diff(pts, axis=0).T)
        self.cum = np.concatenate([[0.0], np.cumsum(seg)])
        self.length = float(self.cum[-1])
        self.speed = speed

    def pose(self, t: float) -> tuple[float, float, float]:
        return self.at((t * self.speed) % self.length)

    def at(self, s: float) -> tuple[float, float, float]:
        s = s % self.length
        i = int(np.clip(np.searchsorted(self.cum, s, side="right") - 1, 0, len(self.pts) - 2))
        a, b = self.pts[i], self.pts[i + 1]
        f = (s - self.cum[i]) / max(self.cum[i + 1] - self.cum[i], 1e-9)
        x, y = a + f * (b - a)
        return float(x), float(y), math.atan2(b[1] - a[1], b[0] - a[0])


class ActorSim:
    """Scripted actor motion with giving way, independent of Gazebo (unit-tested).

    Each step an actor moves along its path unless the step would bring it closer to something it gives way
    to: any robot within YIELD_M (unless it is stubborn or, in a crowd, chose to ignore this encounter) and,
    with actors_yield, other actors (see FORKLIFT_SEES_WORKER_M and ACTOR_CLEAR_M). After waiting
    YIELD_TURN_S the actor turns around, so two actors (or an actor and a robot) never wait for each other forever.
    """

    def __init__(self, followers: dict[str, PathFollower], kinds: dict[str, str], stubborn=(), p_yield: float = 1.0,
                 actors_yield: bool = False, seed: int = 0):
        self.f, self.kinds = followers, kinds
        self.stubborn, self.p_yield, self.actors_yield = set(stubborn), p_yield, actors_yield
        self.rng = np.random.default_rng(seed)
        self.progress = {n: 0.0 for n in followers}
        self.direction = {n: 1.0 for n in followers}
        self.held_since: dict[str, float | None] = {n: None for n in followers}
        self.encounter: dict[str, tuple[bool, float] | None] = {}
        self.at = {n: f.at(0.0)[:2] for n, f in followers.items()}
        if actors_yield:  # scripted start points may overlap (e.g. forklift and a worker at a crossing): move
            for n, f in followers.items():  # a forklift's start along its route until it is clear of everyone
                if kinds[n] != "forklift":
                    continue
                for _ in range(int(f.length / 0.25)):
                    x, y, _ = f.at(self.progress[n])
                    if all(math.hypot(x - ox, y - oy) >= FORKLIFT_SEES_WORKER_M
                           for o, (ox, oy) in self.at.items() if o != n):
                        break
                    self.progress[n] += 0.25
                self.at[n] = f.at(self.progress[n])[:2]

    @staticmethod
    def closer(x0, y0, x1, y1, others, limit: float) -> bool:
        """True if stepping (x0,y0)->(x1,y1) brings the actor closer to any of `others` within `limit`."""
        for rx, ry in others:
            d0, d1 = math.hypot(rx - x0, ry - y0), math.hypot(rx - x1, ry - y1)
            if d1 < limit and d1 < d0:
                return True
        return False

    def blocked_by_actor(self, name: str, x0, y0, x1, y1) -> bool:
        if not self.actors_yield:
            return False
        k = self.kinds[name]
        for n, (ox, oy) in self.at.items():
            if n == name:
                continue
            pair = frozenset([k, self.kinds[n]])
            limit = ACTOR_CLEAR_M.get(pair, 0.7)
            if k == "forklift" and self.kinds[n] == "worker":
                limit = FORKLIFT_SEES_WORKER_M
            if self.closer(x0, y0, x1, y1, [(ox, oy)], limit):
                return True
        return False

    def step(self, dt: float, robots: list[tuple[float, float]], now: float) -> dict[str, tuple[float, float, float]]:
        poses = {}
        for name, f in self.f.items():
            x, y, yaw = f.at(self.progress[name])
            if self.direction[name] < 0:
                yaw += math.pi
            step = self.direction[name] * f.speed * dt
            nx, ny, _ = f.at(self.progress[name] + step)
            near = self.closer(x, y, nx, ny, robots, YIELD_M.get(self.kinds[name], 1.5))
            # Crowd: workers may ignore a robot; with the Studio setting a forklift driver always gives way.
            if near and self.p_yield < 1.0 and not (self.actors_yield and self.kinds[name] == "forklift"):
                enc = self.encounter.get(name)
                if enc is None or now - enc[1] > 2.0:  # new encounter: decide once
                    enc = (bool(self.rng.random() < self.p_yield), now)
                self.encounter[name] = (enc[0], now)
                near = enc[0]
            if (name not in self.stubborn and near) or self.blocked_by_actor(name, x, y, nx, ny):
                # Yield; after a few seconds turn around so nobody is deadlocked.
                self.held_since[name] = self.held_since[name] or now
                if now - self.held_since[name] > YIELD_TURN_S:
                    self.direction[name], self.held_since[name] = -self.direction[name], None
            else:
                self.held_since[name] = None
                self.progress[name] += step
                x, y, yaw = f.at(self.progress[name])
                if self.direction[name] < 0:
                    yaw += math.pi
            poses[name] = (x, y, yaw)
            self.at[name] = (x, y)
        return poses

    def held_names(self) -> list[str]:
        return [n for n, h in self.held_since.items() if h]

    def ignoring_names(self) -> list[str]:
        return [n for n, e in self.encounter.items() if e and not e[0]]


def main(run_dir: str) -> int:
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    rd = Path(run_dir)
    sc = yaml.safe_load((rd / "scenario.resolved.yaml").read_text())
    world = sc["resolved_profiles"]["world"]
    actors = world.get("layout_data", {}).get("actors", [])
    if not actors:
        return 0
    scale = float(sc.get("world", {}).get("actor_speed_scale", 1.0))
    followers = {a["name"]: PathFollower(a["path"], a.get("speed", 0.5) * scale) for a in actors}
    robots: dict[str, tuple[float, float]] = {}
    from gz.msgs10.pose_pb2 import Pose as GzPose
    kinds = {a["name"]: a["kind"] for a in actors}
    service = f"/world/{world.get('gz_world', 'default')}/set_pose_vector"
    node = Node()
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    from gz.transport13 import SubscribeOptions

    opts = SubscribeOptions()
    opts.msgs_per_sec = 20  # throttle: we only need fresh positions, never a backlog
    for rid in [m["id"] for m in sc.get("fleet") or []] or [
            sc["resolved_profiles"]["robot"].get("gz_model_name", "turtlebot3_waffle")]:
        node.subscribe(GzPose, f"/model/{rid}/pose",
                       lambda msg, r=rid: robots.__setitem__(r, (msg.position.x, msg.position.y)), opts)

    stubborn = set(sc.get("world", {}).get("non_yielding_actors") or [])
    # Crowd behaviour: each time a robot comes near, an actor yields with this probability (decided
    # once per encounter, seeded from the run so runs differ but are reproducible from the log).
    p_yield = float(sc.get("world", {}).get("actor_yield_probability", 1.0))
    # Optional: forklifts and workers also give way to each other. Off by default so recorded benchmark runs
    # reproduce (their scripted forklift and workers do not avoid each other where routes cross).
    actors_yield = bool(sc.get("world", {}).get("vehicles_yield_to_workers", False))
    sim = ActorSim(followers, kinds, stubborn=stubborn, p_yield=p_yield, actors_yield=actors_yield,
                   seed=zlib.crc32(rd.name.encode()))

    t0, next_log, last = time.time(), 0.0, time.time()
    with open(rd / "actors.jsonl", "w") as log:
        while not stop.is_set():
            t = time.time() - t0
            req = Pose_V()
            dt, last = time.time() - last, time.time()
            poses = sim.step(dt, list(robots.values()), now=time.time())
            for name, (x, y, yaw) in poses.items():
                p = req.pose.add()
                p.name = name
                p.position.x, p.position.y, p.position.z = x, y, 0.0
                p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            # Short timeout: a blocking request starves the pose callbacks (seen as multi-second lag).
            node.request(service, req, Pose_V, Boolean, 30)
            if t >= next_log:
                rec = {n: {"x": round(x, 3), "y": round(y, 3), "kind": kinds[n]}
                       for n, (x, y, _) in poses.items()}
                log.write(json.dumps({"t": time.time(), "actors": rec,
                                      "robots_seen": {k: [round(v[0], 2), round(v[1], 2)] for k, v in robots.items()},
                                      "held": sim.held_names(),
                                      "ignoring": sim.ignoring_names()}) + "\n")
                log.flush()
                next_log = t + 1.0 / LOG_HZ
            stop.wait(max(0.0, 1.0 / TICK_HZ - (time.time() - t0 - t)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
