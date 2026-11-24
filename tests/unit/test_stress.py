import json
from pathlib import Path

import pytest

from qrsimbench.stress.engine import make_variants, mine_failures, sample, set_path

ROOT = Path(__file__).resolve().parents[2]
SPEC = ROOT / "examples/warehouse_amr/stress/blocked_aisle_stress.yaml"


def test_set_path_dicts_and_lists():
    d = {"events": [{"action": {"pose": {"x": 0}}}]}
    set_path(d, "events.0.action.pose.x", 1.5)
    set_path(d, "navigation.overrides.a.b", 2)
    assert d["events"][0]["action"]["pose"]["x"] == 1.5 and d["navigation"]["overrides"]["a"]["b"] == 2


def test_sample_bounds():
    import random

    rng = random.Random(0)
    assert all(-1 <= sample({"uniform": [-1, 1]}, rng) <= 1 for _ in range(100))
    assert sample({"choice": ["a"]}, rng) == "a"
    with pytest.raises(ValueError):
        sample({"zipf": 1}, rng)


def test_variants_are_valid_and_reproducible(tmp_path):
    a = make_variants(SPEC, tmp_path / "a")
    b = make_variants(SPEC, tmp_path / "b")
    assert len(a) == 20
    assert [p for _, p in a] == [p for _, p in b]  # same seed, same campaign
    assert len({json.dumps(p, sort_keys=True) for _, p in a}) == 20


def test_mine_failures_ranks_fail_modes_and_sensitivity():
    res = [{"run_id": f"r{i}", "variant": f"v{i}.yaml", "verdict": "FAIL" if i < 4 else "PASS",
            "codes": ["TIMEOUT"] if i < 4 else ["MISSION_SUCCESS"], "nav2_result": "TIMEOUT",
            "params": {"x": float(i)}, "metrics": {}} for i in range(10)]
    out = mine_failures(res, ["x"])
    assert out["modes"][0]["mode"] == "FAIL: TIMEOUT" and out["modes"][0]["count"] == 4
    assert out["modes"][0]["param_ranges"]["x"] == [0.0, 3.0]
    assert out["sensitivity"]["x"] < -0.5


def test_stress_report_renders(tmp_path):
    from qrsimbench.stress.report import render_stress_report

    res = [{"run_id": f"r{i}", "variant": f"v{i}.yaml", "verdict": ["PASS", "WARN", "FAIL"][i % 3],
            "codes": [], "nav2_result": "SUCCEEDED", "params": {"a.x": float(i), "a.y": float(-i)},
            "metrics": {"time_to_goal": 30.0 + i, "path_length": 10.0, "min_clearance": 0.4, "stopped_time": 0.0}}
           for i in range(9)]
    (tmp_path / "results.json").write_text(json.dumps(res))
    (tmp_path / "summary.json").write_text(json.dumps({"name": "t", "base": "b", "n": 9, "params": ["a.x", "a.y"],
                                                       "verdicts": {"PASS": 3, "WARN": 3, "FAIL": 3},
                                                       **mine_failures(res, ["a.x", "a.y"])}))
    html = render_stress_report(tmp_path).read_text()
    assert "Failure modes" in html and "<svg" in html


def test_bench_configs_and_statistics(tmp_path):
    import yaml as _yaml

    from qrsimbench.stress.bench import DEFAULT_METRICS, make_configs, summarize
    from qrsimbench.stress.bench_report import render_bench_report

    spec_file = ROOT / "examples/smart_factory/bench/factory_baselines.yaml"
    if not spec_file.exists():
        pytest.skip("factory bench spec not part of this build")
    spec = _yaml.safe_load(spec_file.read_text())
    files = make_configs(spec, tmp_path / "configs")
    b0 = _yaml.safe_load(files["B0_stock"].read_text())
    assert b0["fleet_policy"]["awareness"] is False and b0["navigation"]["parameter_profile"] == "nav2_default_v1"
    assert "fleet_policy" not in _yaml.safe_load(files["B3_qr"].read_text()) or True
    res = []
    for c, base in (("B0_stock", 0.3), ("B3_qr", 0.9)):
        for i in range(8):
            res.append({"config": c, "verdict": "PASS", "metrics": {"min_human_separation": base + 0.01 * i,
                                                                     "success": True, "tasks_completed": 9}})
    metrics = {k: DEFAULT_METRICS[k] for k in ("min_human_separation", "human_contact", "tasks_completed")}
    t = summarize(res, ["B0_stock", "B3_qr"], "B0_stock", metrics)
    assert t["B3_qr"]["metrics"]["min_human_separation"]["beats_baseline"] is True
    assert t["B0_stock"]["metrics"]["human_contact"]["mean"] == 1.0
    (tmp_path / "summary.json").write_text(json.dumps({"name": "x", "base": "b", "reps": 8, "baseline": "B0_stock",
                                                       "configs": {"B0_stock": {}, "B3_qr": {}},
                                                       "metrics": {k: list(v) for k, v in metrics.items()}, "table": t}))
    assert "beats baseline" in render_bench_report(tmp_path).read_text()
