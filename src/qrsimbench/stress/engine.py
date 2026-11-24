"""Stress engine: randomized scenario variants, unattended execution, failure mining.

A stress spec (YAML) names a base scenario and the parameters to vary:

    base: examples/warehouse_amr/scenarios/blocked_aisle.yaml
    name: blocked_aisle_stress
    runs: 20
    seed: 0
    vary:
      - {path: events.0.action.pose.x, uniform: [-3.0, -1.8]}
      - {path: events.0.trigger.radius_m, uniform: [0.3, 1.5]}
      - {path: navigation.overrides.controller_server.ros__parameters.FollowPath.vx_max, uniform: [0.3, 0.6]}
      - {path: world.actor_speed_scale, choice: [0.5, 1.0, 1.5]}

Each variant is a complete, valid scenario frozen to disk with its own seed, so every failure found
is replayable with `qrsimbench run <variant.yaml>`.
"""

from __future__ import annotations

import copy
import json
import random
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from qrsimbench.config.loader import REPO_ROOT, ScenarioError, load_scenario
from qrsimbench.recording.manifest import RUNS_DIR


def set_path(d: dict, path: str, value: Any) -> None:
    keys = path.split(".")
    cur: Any = d
    for k in keys[:-1]:
        if isinstance(cur, list):
            cur = cur[int(k)]
        else:
            cur = cur.setdefault(k, {})
    last = keys[-1]
    if isinstance(cur, list):
        cur[int(last)] = value
    else:
        cur[last] = value


def sample(spec: dict, rng: random.Random) -> Any:
    if "uniform" in spec:
        a, b = spec["uniform"]
        return round(rng.uniform(a, b), 3)
    if "normal" in spec:
        mu, sigma = spec["normal"]
        return round(rng.gauss(mu, sigma), 3)
    if "choice" in spec:
        return rng.choice(spec["choice"])
    if "int" in spec:
        a, b = spec["int"]
        return rng.randint(a, b)
    raise ValueError(f"unknown distribution in {spec}")


def make_variants(spec_path: Path, out_dir: Path) -> list[tuple[Path, dict[str, Any]]]:
    spec = yaml.safe_load(spec_path.read_text())
    base_path = REPO_ROOT / spec["base"]
    base = yaml.safe_load(base_path.read_text())
    rng = random.Random(spec.get("seed", 0))
    out_dir.mkdir(parents=True, exist_ok=True)
    variants = []
    for i in range(int(spec.get("runs", 10))):
        v = copy.deepcopy(base)
        params = {}
        for item in spec.get("vary", []):
            params[item["path"]] = sample(item, rng)
            set_path(v, item["path"], params[item["path"]])
        v["experiment"]["seed"] = rng.randint(0, 2**31 - 1)
        v["experiment"]["description"] = (f"{spec.get('name', 'stress')} variant {i:03d} of "
                                          f"{base['experiment']['name']}: {params}")
        f = out_dir / f"v{i:03d}.yaml"
        f.write_text(yaml.safe_dump(v, sort_keys=False))
        try:
            load_scenario(f)  # every variant must still satisfy the scenario contract
        except ScenarioError as e:
            raise ScenarioError(f"variant {i} is invalid: {e}") from e
        variants.append((f, params))
    return variants


def mine_failures(results: list[dict[str, Any]], params: list[str]) -> dict[str, Any]:
    """Group non-PASS runs into failure modes and rank parameters by association with failure."""
    clusters: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for r in results:
        if r["verdict"] == "PASS":
            continue
        if r.get("infrastructure_failure"):
            clusters["INFRA: " + " + ".join(r["codes"])].append(r)
            continue
        codes = [c for c in r["codes"] if c != "MISSION_SUCCESS"] or [r["nav2_result"] or "UNKNOWN"]
        key = f"{r['verdict']}: " + " + ".join(sorted(set(codes)))
        clusters[key].append(r)
    ranked = sorted(clusters.items(), key=lambda kv: (kv[0].startswith("INFRA"), -kv[0].startswith("FAIL"), -len(kv[1])))
    modes = [{"mode": k, "count": len(v), "runs": [x["run_id"] for x in v],
              "example": v[0]["run_id"], "example_variant": v[0]["variant"],
              "param_ranges": {p: [min(x["params"][p] for x in v), max(x["params"][p] for x in v)]
                               for p in params if all(isinstance(x["params"][p], (int, float)) for x in v)}}
             for k, v in ranked]
    # Sensitivity: point-biserial correlation between each numeric parameter and "not PASS".
    sens = {}
    valid = [r for r in results if not r.get("infrastructure_failure")]
    y = np.array([r["verdict"] != "PASS" for r in valid], dtype=float)
    for p in params:
        xs = [r["params"][p] for r in valid]
        if all(isinstance(x, (int, float)) for x in xs) and y.std() > 0 and np.std(xs) > 0:
            sens[p] = float(np.corrcoef(np.array(xs, dtype=float), y)[0, 1])
    return {"modes": modes, "sensitivity": dict(sorted(sens.items(), key=lambda kv: -abs(kv[1])))}


def run_stress(spec_path: Path, headless: bool = True, limit: int | None = None) -> Path:
    from qrsimbench.runner.experiment import log, run_experiment

    spec = yaml.safe_load(spec_path.read_text())
    stamp = datetime.now().strftime("%Y-%m-%dT%H%M%S")
    sdir = RUNS_DIR / f"stress_{stamp}_{spec.get('name', spec_path.stem)}"
    variants = make_variants(spec_path, sdir / "variants")[: limit or None]
    params = [v["path"] for v in spec.get("vary", [])]
    results = []
    for i, (vf, p) in enumerate(variants):
        log(f"stress {i + 1}/{len(variants)}: {p}")
        infra_retries = 0
        while True:
            try:
                run_dir, verdict = run_experiment(vf, ["qrsimbench", "stress", str(spec_path)], headless=headless)
                met = json.loads((run_dir / "metrics.json").read_text())
            except Exception as e:  # a crashed variant is itself a finding
                run_dir, verdict, met = None, "FAIL", {"codes": ["RUNNER_ERROR"], "error": repr(e)}
            # Infrastructure flakes (stack never came up) are re-run once and reported separately;
            # they say nothing about the system under test.
            if met.get("infrastructure_failure") and infra_retries < 1:
                infra_retries += 1
                log(f"stress {i + 1}: infrastructure failure {met.get('codes')}, re-running once")
                continue
            break
        m = met.get("metrics", {})
        results.append({"variant": str(vf.relative_to(REPO_ROOT)), "run_id": run_dir.name if run_dir else None,
                        "params": p, "verdict": verdict, "codes": met.get("codes", []),
                        "nav2_result": m.get("nav2_result"), "metrics": m, "infra_retries": infra_retries,
                        "infrastructure_failure": bool(met.get("infrastructure_failure"))})
        (sdir / "results.json").write_text(json.dumps(results, indent=2, default=float))
    summary = {"spec": str(spec_path), "name": spec.get("name"), "base": spec["base"], "n": len(results),
               "infra_retries": sum(r["infra_retries"] for r in results),
               "verdicts": {k: sum(r["verdict"] == k for r in results) for k in ("PASS", "WARN", "FAIL")},
               "params": params, **mine_failures(results, params)}
    (sdir / "summary.json").write_text(json.dumps(summary, indent=2, default=float))
    from qrsimbench.stress.report import render_stress_report

    render_stress_report(sdir)
    return sdir
