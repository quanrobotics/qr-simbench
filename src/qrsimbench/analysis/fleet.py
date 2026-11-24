"""Fleet analysis: per-robot metrics plus fleet-level metrics and one Engineering Validation Verdict."""

from __future__ import annotations

import dataclasses
import json
from itertools import combinations
from pathlib import Path
from typing import Any

import numpy as np

from qrsimbench.analysis.bag_reader import fleet_ids, read_run
from qrsimbench.analysis.metrics import compute_metrics
from qrsimbench.analysis.model import NavigationRun
from qrsimbench.analysis.verdict import evaluate

HUMAN_NEAR_M = 1.0  # robot centre to person centre
NEAR_MISS_M = 0.8   # centre-to-centre distance; two TB3 Waffle footprints are ~0.6 m wide together
CONTACT_M = 0.45    # robot centre to person centre: body contact
MOVING_MPS = 0.05   # robot counts as moving above this ground-truth speed
GRID_HZ = 10.0


def separation(runs: list[NavigationRun], t_offsets: list[float]) -> dict[str, Any]:
    """Pairwise robot separation on a common time grid (seconds since fleet start)."""
    if len(runs) < 2:
        return {"min_robot_separation": None, "near_miss_events": 0, "near_miss_time": 0.0, "pairs": {}}
    spans = [(r.pose_t[0] + o, r.pose_t[-1] + o)
             for r, o in zip(runs, t_offsets, strict=True) if len(r.pose_t)]
    t = np.arange(max(0.0, min(a for a, _ in spans)), max(b for _, b in spans), 1.0 / GRID_HZ)
    pos = []
    for r, o in zip(runs, t_offsets, strict=True):
        tt = r.pose_t + o
        # Before its mission starts / after it ends a robot is parked at its first/last pose.
        pos.append(np.stack([np.interp(t, tt, r.x), np.interp(t, tt, r.y)], axis=1) if len(tt) else None)
    sep_min, events, near_time, pairs = np.inf, 0, 0.0, {}
    for (i, a), (j, b) in combinations(enumerate(pos), 2):
        if a is None or b is None:
            continue
        d = np.hypot(*(a - b).T)
        pairs[f"{runs[i].robot_id}-{runs[j].robot_id}"] = float(d.min())
        sep_min = min(sep_min, float(d.min()))
        close = d < NEAR_MISS_M
        events += int(np.sum(close[1:] & ~close[:-1]) + close[0])
        near_time += float(close.sum() / GRID_HZ)
    return {"min_robot_separation": None if sep_min == np.inf else sep_min, "near_miss_events": events,
            "near_miss_time": near_time, "pairs": pairs}


def actor_separation(run_dir: Path, runs: list[NavigationRun], starts: list[float]) -> dict[str, Any]:
    """Min distance from any robot to moving workers / vehicles (actors.jsonl, true positions)."""
    f = run_dir / "actors.jsonl"
    if not f.exists():
        return {}
    recs = [json.loads(line) for line in f.read_text().splitlines() if line.strip()]
    if not recs:
        return {}
    ta = np.array([r["t"] for r in recs])
    out: dict[str, Any] = {"min_human_separation": None, "human_near_miss_events": 0,
                           "min_vehicle_separation": None}
    best = {"worker": np.inf, "forklift": np.inf}
    events = 0
    contact_s = contact_moving_s = 0.0  # person within CONTACT_M; and with the robot moving
    onsets: list[dict[str, Any]] = []
    for name, kind in recs[0]["actors"].items():
        ax = np.array([r["actors"][name]["x"] for r in recs])
        ay = np.array([r["actors"][name]["y"] for r in recs])
        k = "worker" if kind["kind"] == "worker" else "forklift"
        for r, t0 in zip(runs, starts, strict=True):
            if not len(r.pose_t):
                continue
            tt = r.pose_t + t0  # wall clock
            d = np.hypot(np.interp(tt, ta, ax) - r.x, np.interp(tt, ta, ay) - r.y)
            best[k] = min(best[k], float(d.min()))
            if k == "worker":
                dtt = np.diff(tt, prepend=tt[0])
                speed = (np.hypot(np.gradient(r.x, tt), np.gradient(r.y, tt)) if len(tt) > 2
                         else np.zeros_like(d))
                touch = d < CONTACT_M
                contact_s += float(dtt[touch].sum())
                contact_moving_s += float(dtt[touch & (speed > MOVING_MPS)].sum())
                close = d < HUMAN_NEAR_M
                starts_idx = np.flatnonzero(np.r_[close[0], close[1:] & ~close[:-1]])
                events += len(starts_idx)
                for j in starts_idx:
                    seg_end = j + int(np.argmax(~close[j:])) if (~close[j:]).any() else len(close)
                    onsets.append({"t": float(tt[j]), "robot": r.robot_id, "actor": name,
                                   "min_distance": float(d[j:seg_end].min())})
    out["min_human_separation"] = None if best["worker"] == np.inf else best["worker"]
    out["min_vehicle_separation"] = None if best["forklift"] == np.inf else best["forklift"]
    out["human_near_miss_events"] = events
    # Responsibility split used by speed-and-separation monitoring: contact while the robot moves is
    # robot-caused; a person walking into a stopped robot is not.
    out["human_contact_s"] = contact_s
    out["human_contact_robot_moving_s"] = contact_moving_s
    out["human_near_misses"] = sorted(onsets, key=lambda e: e["t"])
    return out


def _separation_gate(name, value, gate, code_warn, code_fail, what):
    """Returns (verdict_effect, code, gate_result) for a min-distance gate."""
    if value is None or not gate:
        return None, None, None
    if gate.get("greater_than_m") is not None and value <= gate["greater_than_m"]:
        return "FAIL", code_fail, {"gate": name, "passed": False, "severity": "FAIL",
                                   "detail": f"{what} within {value:.2f} m"}
    if gate.get("warn_below_m") is not None and value < gate["warn_below_m"]:
        return "WARN", code_warn, {"gate": name, "passed": False, "severity": "WARN",
                                   "detail": f"{what} within {value:.2f} m"}
    return None, None, {"gate": name, "passed": True, "severity": "FAIL", "detail": f"min {value:.2f} m"}


def _trim(r: NavigationRun) -> NavigationRun:
    """The robot's own mission window (for per-robot metrics) from a fleet-window run."""
    if r.mission_end is None:
        return r
    m = r.pose_t <= r.mission_end
    cut = {k: getattr(r, k)[m] for k in ("pose_t", "x", "y", "odom_x", "odom_y", "est_x", "est_y")
           if getattr(r, k) is not None}
    return dataclasses.replace(r, **cut)


def analyze_fleet(run_dir: Path, resolved: dict) -> dict[str, Any]:
    gates = resolved.get("gates", {})
    warn = gates.get("min_clearance", {}).get("warn_below_m") or 0.3
    ids = fleet_ids(resolved)
    missions = {i: json.loads((run_dir / f"mission_{i}.json").read_text()) for i in ids}
    runs, per_robot = [], {}
    # Separation (robot-robot, robot-person) is measured over the whole fleet window: until the last
    # robot finishes. Per-robot metrics use each robot's own mission window.
    fleet_end = max((mm["mission_end"] for mm in missions.values() if mm.get("mission_end")), default=None)
    for i in ids:
        r = read_run(run_dir, resolved, i, until=fleet_end)
        runs.append(r)
        m = compute_metrics(_trim(r), clearance_threshold=warn)
        m["tasks_completed"] = sum(1 for leg in missions[i].get("legs", []) if leg["result"] == "SUCCEEDED")
        m["tasks_assigned"] = len(next(f for f in resolved["fleet"] if f["id"] == i)["goals"])
        v, codes, _ = evaluate(m, {k: g for k, g in gates.items()
                                   if k not in ("robot_separation", "human_separation")})
        per_robot[i] = {"verdict": v, "codes": codes, "metrics": m}

    # Robots broken down by a scenario event are excluded from shift success (their tasks still count).
    ev_file = run_dir / "events_scenario.json"
    disabled = {e.get("robot") for e in (json.loads(ev_file.read_text()) if ev_file.exists() else [])
                if e.get("type") == "disable_robot" and e.get("ok")}
    starts = [missions[i]["mission_start"] for i in ids if missions[i].get("mission_start")]
    ends = [missions[i]["mission_end"] for i in ids if missions[i].get("mission_end") and i not in disabled]
    t_fleet0 = min(starts) if starts else 0.0
    offsets = [(missions[i].get("mission_start") or t_fleet0) - t_fleet0 for i in ids]
    sep = separation(runs, offsets)
    makespan = (max(ends) - t_fleet0) if ends else None
    tasks = sum(p["metrics"]["tasks_completed"] for p in per_robot.values())
    pm = [p["metrics"] for p in per_robot.values()]
    active = {i: p for i, p in per_robot.items() if i not in disabled}
    all_ok = all(p["metrics"]["success"] for p in active.values()) if active else False
    clear = [m["min_clearance"] for m in pm if m.get("min_clearance") is not None]
    fleet = {
        "success": all_ok,
        "nav2_result": "SUCCEEDED" if all_ok else next(
            (p["metrics"]["nav2_result"] for p in active.values() if not p["metrics"]["success"]), "FAILED"),
        "robots_disabled": len(disabled),
        "n_robots": len(ids),
        "robots_succeeded": sum(1 for m in pm if m["success"]),
        "tasks_completed": tasks,
        "tasks_assigned": sum(m["tasks_assigned"] for m in pm),
        "makespan": makespan,
        "time_to_goal": makespan if all_ok else None,
        "throughput_tasks_per_hour": (tasks / makespan * 3600) if makespan else None,
        "path_length": float(sum(m["path_length"] or 0 for m in pm)),
        "path_efficiency": float(np.mean([m["path_efficiency"] for m in pm if m["path_efficiency"]]))
        if any(m["path_efficiency"] for m in pm) else None,
        "goal_error": max((m["goal_error"] for m in pm if m["goal_error"] is not None), default=None),
        "min_clearance": min(clear) if clear else None,
        "stopped_time": float(sum(p["metrics"]["stopped_time"] or 0 for i, p in per_robot.items()
                                  if i not in disabled)),
        "localization_error_max": max((m.get("localization_error_max") or 0 for m in pm), default=None),
        **{k: v for k, v in sep.items() if k != "pairs"},
        **actor_separation(run_dir, runs, [missions[i].get("mission_start") or t_fleet0 for i in ids]),
    }
    near_list = fleet.pop("human_near_misses", [])
    robot_gates = {k: g for k, g in gates.items() if k not in ("robot_separation", "human_separation")}
    verdict, codes, gate_results = evaluate(fleet, robot_gates, prolonged_stop_s=20.0 * len(ids))
    for name, value, wc, fc, what in (
        ("robot_separation", fleet["min_robot_separation"], "NEAR_MISS", "ROBOT_PROXIMITY", "robots came"),
        ("human_separation", fleet.get("min_human_separation"), "HUMAN_NEAR_MISS", "HUMAN_PROXIMITY",
         "robot came to a person"),
    ):
        effect, code, gr = _separation_gate(name, value, gates.get(name, {}), wc, fc, what)
        if gr:
            gate_results.append(gr)
        if effect == "FAIL":
            verdict = "FAIL"
        elif effect == "WARN" and verdict == "PASS":
            verdict = "WARN"
        if code:
            codes = [*codes, code]
    if any(p["verdict"] == "FAIL" for p in active.values()):
        verdict = "FAIL"
    return {"verdict": verdict, "verdict_label": "Engineering Validation Verdict", "codes": codes,
            "gates": gate_results, "metrics": fleet, "robots": per_robot, "separation_pairs": sep["pairs"],
            "human_near_misses": near_list}
