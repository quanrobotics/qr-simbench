"""Browser 3D replay of a recorded run (open viewer).

    qrsimbench replay latest:blocked_aisle          # builds RUN/replay/ and opens it in the browser

Robot positions come from the run's own files: `actors.jsonl` (10 Hz, written when the world has walking
workers or a forklift) or, for every other run, the ground-truth poses in the MCAP recording. Site layout
comes from the world's layout file; objects spawned by scenario events appear at the time they were
spawned. Nothing is simulated again: the replay shows what was recorded, only the rendering differs.
"""

from __future__ import annotations

import json
import math
import shutil
from pathlib import Path
from typing import Any

import numpy as np
import yaml

VIEWER = Path(__file__).parent / "viewer"
LAYOUT_KEYS = ("bounds", "wall", "racks", "blocks", "clutter", "markings", "stations")
ASSET_SIZE = {"pallet": (1.2, 0.8, 0.5), "box": (0.4, 0.4, 0.5)}  # simulation/models/*/model.sdf
LABELS = {"rmf": "Open-RMF", "free_fleet": "free_fleet", "mapf": "Route planning (MAPF)"}
HZ = 5.0  # replay frames per second of simulated time (the browser interpolates between frames)


def stack_label(sc: dict[str, Any]) -> str:
    if sc.get("fleet_manager") in LABELS:
        return LABELS[sc["fleet_manager"]]
    if not sc.get("fleet"):
        return "Nav2"
    return "Nav2 fleet"


def _bounds(lay: dict, pts: list[tuple[float, float]]) -> dict:
    if lay.get("bounds"):
        return lay["bounds"]
    xs = [p[0] for p in pts] or [0.0]
    ys = [p[1] for p in pts] or [0.0]
    return {"x_min": min(xs) - 2.0, "x_max": max(xs) + 2.0, "y_min": min(ys) - 2.0, "y_max": max(ys) + 2.0}


def _from_actors(rd: Path, ids: list[str]) -> tuple[float, list[float], dict, dict, dict] | None:
    f = rd / "actors.jsonl"
    if not f.is_file():
        return None
    recs = [json.loads(line) for line in f.read_text().splitlines() if line.strip()]
    recs = [r for r in recs if r.get("robots_seen")]
    if not recs:
        return None
    t0 = recs[0]["t"]
    kinds = {n: a["kind"] for n, a in recs[0]["actors"].items()}
    ts, robots, actors = [], {i: [] for i in ids}, {n: [] for n in kinds}
    last: dict[str, Any] = {i: None for i in ids}
    for r in recs[:: max(1, round(10 / HZ))]:
        ts.append(round(r["t"] - t0, 2))
        for i in ids:
            p = r["robots_seen"].get(i) or last[i]
            last[i] = p
            robots[i].append(p)
        for n in kinds:
            a = r["actors"].get(n)
            actors[n].append([a["x"], a["y"]] if a else None)
    return t0, ts, robots, actors, kinds


def _from_bag(rd: Path, sc: dict, ids: list[str]) -> tuple[float, list[float], dict, dict, dict]:
    from qrsimbench.analysis.bag_reader import read_run

    runs = {i: read_run(rd, sc, i or None) for i in ids}
    start = {}  # wall-clock mission start per robot (pose_t is relative to it)
    for i, r in runs.items():
        f = rd / (f"mission_{i}.json" if i else "mission.json")
        ms = json.loads(f.read_text()).get("mission_start") if f.is_file() else None
        start[i] = r.mission_start or ms or 0.0
    t0 = min(start.values())
    end = max(start[i] + (r.pose_t[-1] if len(r.pose_t) else 0.0) for i, r in runs.items())
    grid = np.arange(0.0, max(end - t0, 0.0) + 1e-9, 1.0 / HZ)
    robots = {}
    for i, r in runs.items():
        if not len(r.pose_t):
            robots[i or "robot"] = [None] * len(grid)
            continue
        tt = r.pose_t + (start[i] - t0)
        x = np.interp(grid, tt, r.x)
        y = np.interp(grid, tt, r.y)
        robots[i or "robot"] = [[round(float(a), 3), round(float(b), 3)] for a, b in zip(x, y, strict=True)]
    return t0, [round(float(t), 2) for t in grid], robots, {}, {}


def build_replay(rd: Path) -> dict[str, Any]:
    """Replay data for one run folder (same format the viewer reads)."""
    rd = Path(rd)
    sc = yaml.safe_load((rd / "scenario.resolved.yaml").read_text())
    world = sc["resolved_profiles"]["world"]
    lay = world.get("layout_data") or {}
    ids = [m["id"] for m in sc.get("fleet") or []]
    got = _from_actors(rd, ids) if ids else None
    t0, ts, robots, actors, kinds = got or _from_bag(rd, sc, ids or [""])
    if not ts:
        raise ValueError(f"no robot positions in {rd}")

    tasks, total = [], 0
    for i in ids or [""]:
        f = rd / (f"mission_{i}.json" if i else "mission.json")
        if not f.is_file():
            continue
        m = json.loads(f.read_text())
        legs = m.get("legs") or ([{"goal_index": 0, "result": m.get("result"), "t": m.get("mission_end")}]
                                 if m.get("mission_end") else [])
        for leg in legs:
            if leg.get("t"):
                tasks.append({"t": round(leg["t"] - t0, 2), "robot": i or "robot", "goal": leg["goal_index"],
                              "ok": leg["result"] == "SUCCEEDED"})
    tasks.sort(key=lambda e: e["t"])
    total = sum(len(f.get("goals", [])) for f in sc.get("fleet") or []) or 1

    by_id = {e["id"]: e for e in sc.get("events") or []}
    events, spawns = [], []
    for name in ("events_scenario.json", "events.json"):
        f = rd / name
        if not f.is_file():
            continue
        for e in json.loads(f.read_text()):
            if not e.get("t") or not e.get("ok", True) or str(e.get("id", "")).startswith("_"):
                continue
            t = round(e["t"] - t0, 2)
            events.append({"t": t, "type": e.get("type"), "robot": e.get("robot"),
                           "detail": e.get("detail") or e.get("id")})
            src = by_id.get(e.get("id"), {}).get("action", {})
            if src.get("type") == "spawn_entity":
                sx, sy, h = ASSET_SIZE.get(src.get("asset"), (0.6, 0.6, 0.6))
                p = src.get("pose", {})
                spawns.append({"t": t, "asset": src.get("asset"), "x": p.get("x", 0.0), "y": p.get("y", 0.0),
                               "yaw": p.get("yaw", 0.0), "sx": sx, "sy": sy, "h": h})
        break

    met: dict[str, Any] = {}
    if (rd / "metrics.json").is_file():
        mj = json.loads((rd / "metrics.json").read_text())
        m = mj.get("metrics", {})
        keys = ("success", "time_to_goal", "min_clearance", "tasks_completed", "tasks_assigned", "makespan",
                "human_contact_robot_moving_s", "human_near_miss_events", "min_human_separation",
                "min_robot_separation")
        met = {k: m.get(k) for k in keys if m.get(k) is not None}
        met = {k: (round(v, 2) if isinstance(v, float) and math.isfinite(v) else v) for k, v in met.items()}
        met["verdict"] = mj.get("verdict")

    pts = [tuple(p) for seq in robots.values() for p in seq if p]
    layout = {k: lay.get(k) for k in LAYOUT_KEYS}
    layout["bounds"] = _bounds(lay, pts)
    return {"run_id": rd.name, "stack": stack_label(sc), "world": world.get("id", "world"), "layout": layout,
            "t": ts, "robots": robots, "actors": actors, "kinds": kinds, "tasks": tasks, "tasks_total": total,
            "events": events, "spawns": spawns, "metrics": met}


def export_replay(run_dirs: list[Path], out: Path) -> Path:
    """Write a self-contained replay site (index.html + data + three.js) to `out`; returns index.html."""
    out = Path(out)
    if out.exists():
        shutil.rmtree(out)
    (out / "data/replay").mkdir(parents=True)
    listed = []
    for rd in run_dirs:
        rep = build_replay(rd)
        (out / f"data/replay/{rd.name}.json").write_text(json.dumps(rep, separators=(",", ":")))
        listed.append({"run_id": rd.name, "stack": rep["stack"], "robots": len(rep["robots"]),
                       "world": rep["world"]})
    (out / "data/replay/runs.json").write_text(json.dumps(listed))
    flag = "<head><script>window.QR_STATIC = true;</script>"
    html = (VIEWER / "replay.html").read_text().replace("<head>", flag, 1)
    (out / "index.html").write_text(html)
    shutil.copytree(VIEWER / "static", out / "static")
    return out / "index.html"


def serve(directory: Path, port: int = 8766, open_browser: bool = True) -> None:
    """Serve the replay site on localhost (browsers do not load ES modules from file://)."""
    import functools
    import http.server
    import webbrowser

    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(directory))
    with http.server.ThreadingHTTPServer(("127.0.0.1", port), handler) as httpd:
        url = f"http://127.0.0.1:{port}/index.html"
        print(f"replay: {url}  (Ctrl+C to stop)")
        if open_browser:
            webbrowser.open(url)
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass
