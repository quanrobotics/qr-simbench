"""Multi-run aggregate statistics (P1): distribution of each metric over repeated runs."""

from __future__ import annotations

import json
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np

KEYS = ["time_to_goal", "path_length", "path_efficiency", "goal_error", "min_clearance",
        "stopped_time", "localization_error_max"]


def aggregate(run_dirs: list[Path]) -> dict[str, Any]:
    ms = [json.loads((d / "metrics.json").read_text()) for d in run_dirs if (d / "metrics.json").exists()]
    out: dict[str, Any] = {
        "n_runs": len(ms),
        "runs": [d.name for d in run_dirs],
        "verdicts": dict(Counter(m.get("verdict", "?") for m in ms)),
        "success_rate": (float(np.mean([bool(m.get("metrics", {}).get("success")) for m in ms]))
                         if ms else None),
        "metrics": {},
    }
    for k in KEYS:
        v = np.array([m["metrics"][k] for m in ms if isinstance(m.get("metrics", {}).get(k), (int, float))])
        out["metrics"][k] = None if v.size == 0 else {
            "n": int(v.size), "mean": float(v.mean()), "std": float(v.std(ddof=1)) if v.size > 1 else 0.0,
            "min": float(v.min()), "max": float(v.max()),
        }
    return out


def format_table(agg: dict[str, Any]) -> str:
    lines = [f"runs: {agg['n_runs']}  verdicts: {agg['verdicts']}  success rate: {agg['success_rate']:.0%}",
             f"{'metric':24s} {'n':>3s} {'mean':>9s} {'std':>8s} {'min':>9s} {'max':>9s}"]
    for k, s in agg["metrics"].items():
        if s:
            lines.append(f"{k:24s} {s['n']:3d} {s['mean']:9.3f} {s['std']:8.3f} "
                         f"{s['min']:9.3f} {s['max']:9.3f}")
    return "\n".join(lines)
