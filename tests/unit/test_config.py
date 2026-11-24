from pathlib import Path

import pytest
import yaml
from typer.testing import CliRunner

from qrsimbench.cli import app
from qrsimbench.config.hashing import sha256_obj
from qrsimbench.config.loader import ScenarioError, load_scenario, resolve

ROOT = Path(__file__).resolve().parents[2]
SCENARIOS = sorted((ROOT / "examples/warehouse_amr/scenarios").glob("*.yaml"))


@pytest.mark.parametrize("path", SCENARIOS, ids=lambda p: p.stem)
def test_examples_validate_and_resolve(path):
    sc = load_scenario(path)
    data = resolve(sc)
    stations = data["resolved_profiles"]["world"]["stations"]
    goal = sc.fleet[0].goals[-1] if sc.fleet else sc.mission.goal
    g, st = data["mission"]["goal_pose"], stations[goal]
    assert (g["x"], g["y"]) == pytest.approx((st["x"], st["y"]))
    assert "robot" in data["resolved_profiles"]


def test_four_demo_scenarios_exist():
    assert {p.stem for p in SCENARIOS} >= {
        "baseline", "blocked_aisle", "narrow_aisle", "impossible_route", "fleet_shift"
    }


def test_fleet_resolution():
    data = resolve(load_scenario(ROOT / "examples/warehouse_amr/scenarios/fleet_shift.yaml"))
    ids = [m["id"] for m in data["fleet"]]
    assert ids == ["amr_1", "amr_2", "amr_3"]
    assert all(len(m["goal_poses"]) == len(m["goals"]) for m in data["fleet"])
    assert data["events"][0]["trigger"]["robot"] == "amr_1"


def test_fleet_requires_unique_ids(tmp_path):
    d = yaml.safe_load((ROOT / "examples/warehouse_amr/scenarios/fleet_shift.yaml").read_text())
    d["fleet"][1]["id"] = "amr_1"
    with pytest.raises(ScenarioError, match="duplicate fleet"):
        load_scenario(_write(tmp_path, d))


def _write(tmp_path, data):
    f = tmp_path / "s.yaml"
    f.write_text(yaml.safe_dump(data))
    return f


def _base():
    return yaml.safe_load((ROOT / "examples/warehouse_amr/scenarios/baseline.yaml").read_text())


@pytest.mark.parametrize(
    "mutate, needle",
    [
        (lambda d: d.pop("robot"), "robot"),
        (lambda d: d.__setitem__("schema_version", "9.9"), "schema_version"),
        (lambda d: d["metrics"].append("vibes"), "unknown metrics"),
        (lambda d: d["mission"].__setitem__("timeout_seconds", -1), "timeout_seconds"),
        (lambda d: d.__setitem__("typo_key", 1), "typo_key"),
        (lambda d: d["experiment"].__setitem__("name", "Bad Name"), "experiment.name"),
    ],
)
def test_invalid_scenarios_give_actionable_errors(tmp_path, mutate, needle):
    d = _base()
    mutate(d)
    with pytest.raises(ScenarioError) as ei:
        load_scenario(_write(tmp_path, d))
    assert needle in str(ei.value)


def test_unknown_station_is_reported(tmp_path):
    d = _base()
    d["mission"]["goal"] = "station_z"
    with pytest.raises(ScenarioError, match="station_z"):
        resolve(load_scenario(_write(tmp_path, d)))


def test_explicit_pose_goal(tmp_path):
    d = _base()
    d["mission"]["goal"] = {"x": 1.0, "y": 2.0}
    data = resolve(load_scenario(_write(tmp_path, d)))
    assert data["mission"]["goal_pose"]["y"] == 2.0


def test_hash_is_key_order_independent():
    assert sha256_obj({"a": 1, "b": 2}) == sha256_obj({"b": 2, "a": 1})


def test_cli_validate():
    r = CliRunner().invoke(app, ["validate", str(SCENARIOS[0])])
    assert r.exit_code == 0, r.output
    assert "[PASS]" in r.output


def test_cli_validate_missing_file():
    r = CliRunner().invoke(app, ["validate", "nope.yaml"])
    assert r.exit_code == 1
