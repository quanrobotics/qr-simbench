import json
import math

import numpy as np
import pytest

from qrsimbench.analysis.metrics import compute_metrics, path_length, stop_stats, valid_scan_min
from qrsimbench.analysis.model import NavigationRun
from qrsimbench.analysis.verdict import evaluate
from qrsimbench.config.loader import load_scenario, resolve
from qrsimbench.recording import manifest as mf

A = NavigationRun.arr


def straight_run(result="SUCCEEDED", scan_min=1.0, pause=False):
    # 4 m straight line at 0.2 m/s over 20 s, 10 Hz; optional 5 s pause in the middle.
    t = np.arange(0, 20.01, 0.1)
    v = np.full_like(t, 0.2)
    if pause:
        v[(t >= 8) & (t < 13)] = 0.0
    x = np.concatenate([[0.0], np.cumsum(v[:-1] * 0.1)])
    return NavigationRun(
        pose_t=t, x=x, y=np.zeros_like(t), vel_t=t, v=v, w=np.zeros_like(t),
        scan_t=t, scan_min=np.full_like(t, scan_min), goal=(float(x[-1]), 0.0), start=(0.0, 0.0),
        result=result, mission_start=0.0, mission_end=20.0,
    )


def test_path_length_and_efficiency_straight():
    m = compute_metrics(straight_run())
    assert m["path_length"] == pytest.approx(4.0, abs=1e-6)
    assert m["path_efficiency"] == pytest.approx(1.0)
    assert m["goal_error"] == pytest.approx(0.0)
    assert m["time_to_goal"] == pytest.approx(20.0)


def test_path_length_filters_jumps():
    assert path_length(A([0, 1, 50, 51]) * 0.1, A([0, 0, 0, 0])) == pytest.approx(0.2)


def test_path_length_too_few_samples_is_none():
    assert path_length(A([0]), A([0])) is None


def test_efficiency_detour():
    # L-shaped path 3 + 4 = 7 m, straight line 5 m
    x = A(np.r_[np.linspace(0, 3, 31), np.full(40, 3.0)])
    y = A(np.r_[np.zeros(31), np.linspace(0.1, 4, 40)])
    t = np.arange(len(x)) * 0.1
    run = NavigationRun(t, x, y, t, np.full_like(t, 0.3), np.zeros_like(t), t, np.ones_like(t),
                        goal=(3, 4), start=(0, 0), result="SUCCEEDED", mission_end=t[-1])
    m = compute_metrics(run)
    assert m["path_length"] == pytest.approx(7.0, abs=1e-6)
    assert m["path_efficiency"] == pytest.approx(5 / 7)


def test_scan_filtering():
    assert valid_scan_min([math.nan, math.inf, 0.05, 0.8, 0.5, 9.0], 0.12, 3.5) == 0.5
    assert math.isnan(valid_scan_min([math.inf, math.nan], 0.12, 3.5))


def test_stop_detection():
    s = stop_stats(straight_run(pause=True))
    assert s["stopped_time"] == pytest.approx(5.0, abs=0.11)
    assert s["stop_episodes"] == 1


def test_clearance_only_while_moving():
    run = straight_run(pause=True)
    run.scan_min[(run.scan_t >= 9) & (run.scan_t < 12)] = 0.05  # close only while stopped
    m = compute_metrics(run)
    assert m["min_clearance"] == pytest.approx(1.0)


def test_failed_run_has_no_time_to_goal():
    m = compute_metrics(straight_run(result="FAILED"))
    assert m["success"] is False and m["time_to_goal"] is None


GATES = {"success": {"equals": True}, "min_clearance": {"greater_than_m": 0.2, "warn_below_m": 0.3},
         "timeout": {"less_than_seconds": 180}}


@pytest.mark.parametrize(
    "run, expected",
    [
        (straight_run(), "PASS"),
        (straight_run(scan_min=0.25), "WARN"),
        (straight_run(scan_min=0.15), "FAIL"),
        (straight_run(result="FAILED"), "FAIL"),
        (straight_run(result="TIMEOUT"), "FAIL"),
    ],
)
def test_verdicts(run, expected):
    verdict, codes, _ = evaluate(compute_metrics(run), GATES)
    assert verdict == expected


def test_manifest_bundle(tmp_path):
    from pathlib import Path
    src = Path(__file__).resolve().parents[2] / "examples/warehouse_amr/scenarios/baseline.yaml"
    resolved = resolve(load_scenario(src))
    d = mf.create_run_dir("baseline", runs_dir=tmp_path)
    m = mf.write_bundle_start(d, src, resolved, ["qrsimbench", "run", str(src)])
    mf.finish_manifest(d, m, "SUCCEEDED")
    mf.write_reproduce_script(d)
    data = json.loads((d / "manifest.json").read_text())
    assert data["terminal_result"] == "SUCCEEDED" and len(data["scenario_sha256"]) == 64
    assert (d / "scenario.resolved.yaml").exists() and (d / "environment.json").exists()
    assert (d / "reproduce.sh").stat().st_mode & 0o100
    assert mf.create_run_dir("baseline", runs_dir=tmp_path) != d  # unique even within same second


def test_fleet_separation_and_near_miss():
    from qrsimbench.analysis.fleet import separation

    a = straight_run()
    b = straight_run()
    b.y = b.y + 0.5  # parallel, 0.5 m apart the whole time
    a.robot_id, b.robot_id = "amr_1", "amr_2"
    s = separation([a, b], [0.0, 0.0])
    assert s["min_robot_separation"] == pytest.approx(0.5)
    assert s["near_miss_events"] == 1 and s["near_miss_time"] > 15


def test_fleet_awareness_disc():
    fa = pytest.importorskip("qrsimbench.navigation.fleet_awareness")  # closed fleet layer
    RADIUS, disc_points = fa.RADIUS, fa.disc_points

    pts = disc_points(1.0, 2.0)
    assert max(math.hypot(x - 1.0, y - 2.0) for x, y, _ in pts) == pytest.approx(RADIUS)
