"""Run comparison (QRB-012): delta table + overlaid trajectories, standalone HTML."""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any

from qrsimbench.analysis.bag_reader import read_run
from qrsimbench.recording.manifest import RUNS_DIR
from qrsimbench.reporting import plots
from qrsimbench.reporting.html import TEMPLATES, fmt, jinja, load_bundle

# (key, label, unit, lower_is_better)
ROWS = [
    ("time_to_goal", "Time to goal", "s", True),
    ("path_length", "Path length", "m", True),
    ("path_efficiency", "Path efficiency", "", False),
    ("min_clearance", "Min clearance", "m", False),
    ("clearance_p5", "Clearance p5", "m", False),
    ("goal_error", "Goal error", "m", True),
    ("stopped_time", "Stopped time", "s", True),
    ("mean_speed", "Mean speed", "m/s", False),
]


def resolve_run(ref: str, runs_dir: Path = RUNS_DIR) -> Path:
    """Accept a path, a run id, or latest:<scenario_name>."""
    p = Path(ref)
    if p.is_dir() and (p / "manifest.json").exists():
        return p
    if (runs_dir / ref / "manifest.json").exists():
        return runs_dir / ref
    if ref.startswith("latest:"):
        name = ref.split(":", 1)[1]
        hits = sorted(d for d in runs_dir.glob(f"*_{name}*")
                      if (d / "metrics.json").exists()
                      and json.loads((d / "manifest.json").read_text()).get("scenario_name") == name)
        if hits:
            return hits[-1]
    raise FileNotFoundError(f"run not found: {ref}")


def delta_rows(ma: dict[str, Any], mb: dict[str, Any]) -> list[dict[str, str]]:
    rows = [{"label": "Success", "a": fmt(ma.get("success")), "b": fmt(mb.get("success")),
             "delta": "—" if ma.get("success") == mb.get("success") else "changed", "cls": ""}]
    for key, label, unit, lower_better in ROWS:
        a, b = ma.get(key), mb.get(key)
        delta, cls = "—", ""
        if isinstance(a, (int, float)) and isinstance(b, (int, float)):
            if abs(a) >= 0.1:
                pct = (b - a) / abs(a) * 100
                delta = f"{pct:+.0f}%" if round(pct) != 0 else "0%"
            else:
                delta = f"{b - a:+.2f}"
            if abs(b - a) > 1e-9:
                cls = "better" if (b < a) == lower_better else "worse"
        rows.append({"label": f"{label}" + (f" [{unit}]" if unit else ""), "a": fmt(a), "b": fmt(b),
                     "delta": delta, "cls": cls})
    return rows


def _overlay(ra, rb, A, B) -> str:
    # Obstacles drawn from run B's scenario (the perturbed one in the usual A=baseline order).
    return plots.trajectory(rb, B["resolved"], others=[
        (ra, f"A · {A['manifest']['scenario_name']}", plots.MUTED),
        (rb, f"B · {B['manifest']['scenario_name']}", plots.ACCENT),
    ])


def render_compare(ref_a: str, ref_b: str, out: Path | None = None) -> Path:
    da, db = resolve_run(ref_a), resolve_run(ref_b)
    A, B = load_bundle(da), load_bundle(db)
    ra, rb = read_run(da, A["resolved"]), read_run(db, B["resolved"])
    out = out or (db / f"compare_vs_{da.name}.html")
    html = jinja().get_template("compare.html.j2").render(
        css=(TEMPLATES / "base.css").read_text(),
        generated=datetime.now().strftime("%Y-%m-%d %H:%M"),
        A=A, B=B, rows=delta_rows(A["metrics"].get("metrics", {}), B["metrics"].get("metrics", {})),
        overlay=_overlay(ra, rb, A, B),
        same_config=A["manifest"]["resolved_sha256"] == B["manifest"]["resolved_sha256"],
    )
    out.write_text(html)
    return out
