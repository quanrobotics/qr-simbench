"""Analysis + report + compare on two stored real runs (no ROS needed)."""

import json
import shutil
from pathlib import Path

import pytest
import yaml

from qrsimbench.analysis.bag_reader import read_run
from qrsimbench.analysis.metrics import compute_metrics
from qrsimbench.reporting.compare import render_compare
from qrsimbench.reporting.html import render_report

FIX = Path(__file__).resolve().parents[1] / "fixtures" / "runs"


@pytest.fixture()
def runs(tmp_path):
    for d in FIX.iterdir():
        shutil.copytree(d, tmp_path / d.name)
    return sorted(tmp_path.iterdir())


def test_recomputed_metrics_match_stored(runs):
    for d in runs:
        stored = json.loads((d / "metrics.json").read_text())["metrics"]
        m = compute_metrics(read_run(d, yaml.safe_load((d / "scenario.resolved.yaml").read_text())))
        for k in ("success", "path_length", "goal_error"):
            assert m[k] == pytest.approx(stored[k], abs=1e-6), k


def test_report_renders_standalone(runs):
    out = render_report(runs[0])
    html = out.read_text()
    assert "Engineering Validation Verdict" in html and "<svg" in html
    assert "Safety Certified" not in html
    assert "http://" not in html.split("<svg")[0]  # no external assets before the inline plots
    assert (runs[0] / "plots" / "trajectory.png").exists()


def test_compare_renders(runs):
    a, b = runs  # baseline, blocked_aisle (sorted by timestamp)
    html = render_compare(str(a), str(b)).read_text()
    assert "baseline" in html and "blocked_aisle" in html and "Change" in html


def test_aggregate(runs):
    from qrsimbench.reporting.aggregate import aggregate, format_table

    agg = aggregate(runs)
    assert agg["n_runs"] == 2 and agg["success_rate"] == 1.0
    assert agg["metrics"]["path_length"]["min"] <= agg["metrics"]["path_length"]["max"]
    assert "path_length" in format_table(agg)
