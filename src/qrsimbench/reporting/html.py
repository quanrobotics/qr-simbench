"""Standalone HTML evidence report (QRB-011). Everything is inlined; opens offline."""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml
from jinja2 import Environment, FileSystemLoader, select_autoescape

from qrsimbench.analysis.bag_reader import fleet_ids, read_run
from qrsimbench.reporting import plots

TEMPLATES = Path(__file__).parent / "templates"


def jinja() -> Environment:
    return Environment(loader=FileSystemLoader(TEMPLATES), autoescape=select_autoescape(["html", "j2"]))


def fmt(v: Any, digits: int = 2) -> str:
    if v is None:
        return "—"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, float):
        return f"{v:.{digits}f}"
    return str(v)


def cards(m: dict[str, Any]) -> list[dict[str, str]]:
    eff = m.get("path_efficiency")
    return [
        {"label": "Result", "value": m.get("nav2_result") or "—", "unit": ""},
        {"label": "Time to goal", "value": fmt(m.get("time_to_goal"), 1), "unit": "s"},
        {"label": "Path length", "value": fmt(m.get("path_length")), "unit": "m"},
        {"label": "Efficiency", "value": fmt(eff * 100 if eff is not None else None, 0), "unit": "%"},
        {"label": "Min clearance", "value": fmt(m.get("min_clearance")), "unit": "m"},
        {"label": "Goal error", "value": fmt(m.get("goal_error")), "unit": "m"},
    ]


def mission_labels(mission: dict, ids: list[str]) -> dict[str, str]:
    if ids:
        return {"start": f"{len(ids)} robots", "goal": "task chains"}
    return {k: mission[k] if isinstance(mission[k], str) else "pose" for k in ("start", "goal")}


def fleet_cards(m: dict[str, Any]) -> list[dict[str, str]]:
    return [
        {"label": "Robots OK", "value": f"{m.get('robots_succeeded')}/{m.get('n_robots')}", "unit": ""},
        {"label": "Tasks done", "value": f"{m.get('tasks_completed')}/{m.get('tasks_assigned')}", "unit": ""},
        {"label": "Makespan", "value": fmt(m.get("makespan"), 1), "unit": "s"},
        {"label": "Throughput", "value": fmt(m.get("throughput_tasks_per_hour"), 0), "unit": "tasks/h"},
        {"label": "Min robot gap", "value": fmt(m.get("min_robot_separation")), "unit": "m"},
        {"label": "Min clearance", "value": fmt(m.get("min_clearance")), "unit": "m"},
    ]


def load_bundle(run_dir: Path) -> dict[str, Any]:
    return {
        "dir": run_dir,
        "manifest": json.loads((run_dir / "manifest.json").read_text()),
        "metrics": json.loads((run_dir / "metrics.json").read_text()),
        "resolved": yaml.safe_load((run_dir / "scenario.resolved.yaml").read_text()),
        "env": json.loads((run_dir / "environment.json").read_text()),
    }


def render_report(run_dir: Path) -> Path:
    b = load_bundle(run_dir)
    resolved, man, met = b["resolved"], b["manifest"], b["metrics"]
    m = met.get("metrics", {})
    ids = fleet_ids(resolved)
    gates = resolved.get("gates", {}).get("min_clearance", {})
    pd = run_dir / "plots"
    pd.mkdir(exist_ok=True)
    others = None
    if ids:
        fleet_runs = [read_run(run_dir, resolved, i) for i in ids]
        others = [(r, i, plots.FLEET_COLORS[k % len(plots.FLEET_COLORS)])
                  for k, (r, i) in enumerate(zip(fleet_runs, ids, strict=True))]
        run = fleet_runs[0]
        t_fleet0 = json.loads((run_dir / "mission.json").read_text())["mission_start"]
        run.events = [{**e, "t_rel": e["t"] - t_fleet0}
                      for e in json.loads((run_dir / "events.json").read_text())]
    else:
        run = read_run(run_dir, resolved)
    figs = {
        "trajectory": plots.trajectory(run, resolved, pd / "trajectory.png", others=others),
        "velocity": plots.velocity(run, pd / "velocity.png", others=others),
        "clearance": plots.clearance(run, gates.get("warn_below_m"), gates.get("greater_than_m"),
                                     pd / "clearance.png", others=others),
    }
    robots = [{"id": i, "verdict": p["verdict"], "codes": p["codes"],
               **{k: fmt(p["metrics"].get(k), 2) for k in
                  ("tasks_completed", "time_to_goal", "path_length", "path_efficiency", "min_clearance",
                   "goal_error", "stopped_time", "localization_error_max")}}
              for i, p in met.get("robots", {}).items()]
    exp = resolved["experiment"]
    mission = resolved["mission"]
    prov_keys = ["run_id", "started_at", "finished_at", "scenario_source", "scenario_sha256",
                 "resolved_sha256", "git_commit", "git_dirty", "nav2_profile_sha256", "robot_profile_sha256",
                 "world_profile_sha256", "recorded_topics", "command"]
    provenance = {k: man.get(k) for k in prov_keys}
    provenance["artifacts"] = ", ".join(man.get("artifacts", {}).values())
    html = jinja().get_template("report.html.j2").render(
        css=(TEMPLATES / "base.css").read_text(),
        title=exp["name"].replace("_", " "),
        description=exp.get("description", ""),
        generated=datetime.now().strftime("%Y-%m-%d %H:%M"),
        manifest=man,
        verdict=met.get("verdict", "FAIL"),
        cards=fleet_cards(m) if ids else cards(m),
        robots=robots,
        plots=figs,
        mission=mission_labels(mission, ids),
        events=[e for e in run.events if not e["id"].startswith("_")],
        end_t=fmt(run.mission_end, 1),
        codes=met.get("codes", []),
        gates=met.get("gates", []),
        metrics={k: fmt(v, 3) for k, v in m.items()},
        reproduce=f"qrsimbench reproduce {man['run_id']}\n# or\nbash {run_dir}/reproduce.sh",
        env={k: (", ".join(v) if isinstance(v, list) else v) for k, v in b["env"].items()},
        provenance={k: (", ".join(v) if isinstance(v, list) else v) for k, v in provenance.items()},
        scenario_yaml=yaml.safe_dump(resolved, sort_keys=False),
    )
    out = run_dir / "report.html"
    out.write_text(html)
    return out
