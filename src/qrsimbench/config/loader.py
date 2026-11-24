"""Load, validate and resolve scenario files."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from pydantic import ValidationError

from qrsimbench.config.hashing import sha256_file
from qrsimbench.config.models import Pose, Scenario

REPO_ROOT = Path(__file__).resolve().parents[3]
PROFILES_DIR = REPO_ROOT / "profiles"


class ScenarioError(Exception):
    """Raised with a human-readable, actionable message."""


def _format_errors(err: ValidationError) -> str:
    lines = []
    for e in err.errors():
        loc = ".".join(str(p) for p in e["loc"]) or "<root>"
        lines.append(f"  - {loc}: {e['msg']}")
    return "\n".join(lines)


def load_scenario(path: str | Path) -> Scenario:
    p = Path(path)
    if not p.is_file():
        raise ScenarioError(f"scenario file not found: {p}")
    try:
        raw = yaml.safe_load(p.read_text())
    except yaml.YAMLError as e:
        raise ScenarioError(f"{p}: invalid YAML: {e}") from e
    if not isinstance(raw, dict):
        raise ScenarioError(f"{p}: top level must be a mapping")
    try:
        return Scenario.model_validate(raw)
    except ValidationError as e:
        raise ScenarioError(f"{p}: {e.error_count()} validation error(s):\n{_format_errors(e)}") from e


def load_profile(kind: str, name: str, profiles_dir: Path = PROFILES_DIR) -> dict[str, Any]:
    f = profiles_dir / kind / f"{name}.yaml"
    if not f.is_file():
        raise ScenarioError(f"{kind} profile '{name}' not found at {f}")
    return yaml.safe_load(f.read_text()) or {}


def resolve(scenario: Scenario, profiles_dir: Path = PROFILES_DIR) -> dict[str, Any]:
    """Return the scenario with referenced profiles inlined and named stations turned into poses.

    The result is what gets frozen into the run folder, so reruns don't change when
    profile files change later.
    """
    data = scenario.model_dump(mode="json")
    robot = load_profile("robots", scenario.robot.profile, profiles_dir)
    world = load_profile("worlds", scenario.world.id, profiles_dir)
    nav = load_profile("nav2", scenario.navigation.parameter_profile, profiles_dir)
    if "layout" in world:  # stations live in the generated-world layout file
        layout = yaml.safe_load((REPO_ROOT / world["layout"]).read_text())
        world["stations"] = {k: {kk: v[kk] for kk in ("x", "y", "yaw") if kk in v}
                             for k, v in layout.get("stations", {}).items()}
        world["layout_data"] = layout
    for key in ("layout", "sim_world", "map"):  # pin generated assets by content hash
        if key in world and (REPO_ROOT / world[key]).is_file():
            world[f"{key}_sha256"] = sha256_file(REPO_ROOT / world[key])
    stations = world.get("stations", {})

    def to_pose(v: Any, field: str) -> dict[str, float]:
        if isinstance(v, dict):
            return v
        if v not in stations:
            raise ScenarioError(
                f"mission.{field}: station '{v}' not in world '{scenario.world.id}' "
                f"(known: {sorted(stations)})"
            )
        return Pose.model_validate(stations[v]).model_dump()

    if scenario.fleet:
        for m in data["fleet"]:
            m["start_pose"] = to_pose(m["start"], f"fleet.{m['id']}.start")
            m["goal_poses"] = [to_pose(g, f"fleet.{m['id']}.goals") for g in m["goals"]]
            m["profile"] = m.get("profile") or scenario.robot.profile
        first = data["fleet"][0]
        data["mission"]["start"], data["mission"]["goal"] = first["start"], first["goals"][-1]
        data["mission"]["start_pose"] = first["start_pose"]
        data["mission"]["goal_pose"] = first["goal_poses"][-1]
    else:
        data["mission"]["start_pose"] = to_pose(data["mission"]["start"], "start")
        data["mission"]["goal_pose"] = to_pose(data["mission"]["goal"], "goal")
    data["resolved_profiles"] = {"robot": robot, "world": world, "nav2": nav}
    return data
