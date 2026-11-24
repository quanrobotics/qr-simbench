"""Benchmark: repeated runs of configurations of one scenario (baselines vs QR stack), with statistics.

    base: examples/smart_factory/scenarios/factory_shift.yaml
    name: factory_baselines
    reps: 10
    baseline: B0_stock
    configs:
      B0_stock: {navigation.parameter_profile: nav2_default_v1, fleet_policy.awareness: false, ...}
      B3_qr: {}                        # the base scenario as-is
    metrics: [success, tasks_completed, makespan, ...]

Statistics: mean with a 95 % bootstrap confidence interval per configuration, and a two-sided
permutation test of each configuration against the baseline (difference in means). Infrastructure
failures (stack never ready) are re-run once and excluded from statistics, but counted.
"""

from __future__ import annotations

import copy
import json
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from qrsimbench.config.loader import REPO_ROOT, load_scenario
from qrsimbench.recording.manifest import RUNS_DIR
from qrsimbench.stress.engine import set_path

# metric -> (label, higher_is_better)
DEFAULT_METRICS = {
    "success": ("Shift success (all robots)", True),
    "tasks_completed": ("Tasks completed", True),
    "throughput_tasks_per_hour": ("Throughput [tasks/h]", True),
    "makespan": ("Makespan [s]", False),
    "min_human_separation": ("Min person gap [m]", True),
    "human_near_miss_events": ("Human near-miss events", False),
    "human_contact": ("Runs with person contact (<0.45 m)", False),
    "human_contact_s": ("Person contact time [s]", False),
    "human_contact_robot_moving_s": ("Robot-caused person contact [s] (robot moving)", False),
    "min_robot_separation": ("Min robot gap [m]", True),
    "robot_contact": ("Runs with robot contact (<0.45 m)", False),
    "min_clearance": ("Min LiDAR clearance [m]", True),
    "localization_error_max": ("Max localization error [m]", False),
    "stopped_time": ("Stopped time, all robots [s]", False),
}
CONTACT_M = 0.45


def make_configs(spec: dict, out_dir: Path) -> dict[str, Path]:
    base = yaml.safe_load((REPO_ROOT / spec["base"]).read_text())
    out_dir.mkdir(parents=True, exist_ok=True)
    files = {}
    for name, changes in spec["configs"].items():
        sc = copy.deepcopy(base)
        for path, value in {**(spec.get("common") or {}), **(changes or {})}.items():
            set_path(sc, path, value)
        sc["experiment"]["description"] = f"{spec.get('name', 'bench')} config {name}: {changes or 'base'}"
        f = out_dir / f"{name}.yaml"
        f.write_text(yaml.safe_dump(sc, sort_keys=False))
        load_scenario(f)
        files[name] = f
    return files


def derived(m: dict[str, Any]) -> dict[str, Any]:
    out = dict(m)
    out["success"] = 1.0 if m.get("success") else 0.0
    hs, rs = m.get("min_human_separation"), m.get("min_robot_separation")
    out["human_contact"] = None if hs is None else float(hs < CONTACT_M)
    out["robot_contact"] = None if rs is None else float(rs < CONTACT_M)
    return out


def bootstrap_ci(x: np.ndarray, rng: np.random.Generator, n: int = 4000) -> tuple[float, float]:
    if len(x) < 2:
        return (float("nan"), float("nan"))
    means = rng.choice(x, size=(n, len(x)), replace=True).mean(axis=1)
    return float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5))


def permutation_p(a: np.ndarray, b: np.ndarray, rng: np.random.Generator, n: int = 10000) -> float:
    if len(a) < 2 or len(b) < 2:
        return float("nan")
    obs = abs(a.mean() - b.mean())
    pool = np.concatenate([a, b])
    cnt = 0
    for _ in range(n):
        rng.shuffle(pool)
        cnt += abs(pool[: len(a)].mean() - pool[len(a):].mean()) >= obs - 1e-12
    return (cnt + 1) / (n + 1)


def summarize(results: list[dict[str, Any]], configs: list[str], baseline: str,
              metrics: dict[str, tuple[str, bool]]) -> dict[str, Any]:
    rng = np.random.default_rng(0)
    table: dict[str, dict[str, Any]] = {}
    valid = [r for r in results if not r.get("infrastructure_failure")]
    for c in configs:
        rows = [derived(r["metrics"]) for r in valid if r["config"] == c]
        table[c] = {"n": len(rows), "infra_failures": sum(1 for r in results if r["config"] == c
                                                          and r.get("infrastructure_failure")),
                    "verdicts": {k: sum(1 for r in valid if r["config"] == c and r["verdict"] == k)
                                 for k in ("PASS", "WARN", "FAIL")}, "metrics": {}}
        for k in metrics:
            x = np.array([r[k] for r in rows if isinstance(r.get(k), (int, float))], dtype=float)
            lo, hi = bootstrap_ci(x, rng)
            table[c]["metrics"][k] = {"n": int(len(x)), "mean": float(x.mean()) if len(x) else None,
                                      "ci": [lo, hi], "values": x.tolist()}
    for c in configs:
        for k, (_, higher) in metrics.items():
            a = np.array(table[baseline]["metrics"][k]["values"])
            b = np.array(table[c]["metrics"][k]["values"])
            cell = table[c]["metrics"][k]
            cell["p_vs_baseline"] = None if c == baseline else permutation_p(a, b, rng)
            if c != baseline and len(a) and len(b):
                better = (b.mean() > a.mean()) if higher else (b.mean() < a.mean())
                cell["beats_baseline"] = bool(better and cell["p_vs_baseline"] < 0.05)
                cell["delta"] = float(b.mean() - a.mean())
    return table


def run_bench(spec_path: Path, headless: bool = True) -> Path:
    from qrsimbench.runner.experiment import log, run_experiment

    spec = yaml.safe_load(spec_path.read_text())
    stamp = datetime.now().strftime("%Y-%m-%dT%H%M%S")
    bdir = RUNS_DIR / f"bench_{stamp}_{spec.get('name', spec_path.stem)}"
    files = make_configs(spec, bdir / "configs")
    reps = int(spec.get("reps", 5))
    results: list[dict[str, Any]] = []
    # Interleave configurations (rep-major) so slow drifts in machine load affect all equally.
    for rep in range(reps):
        for name, f in files.items():
            log(f"bench rep {rep + 1}/{reps} config {name}")
            for attempt in range(2):
                try:
                    run_dir, verdict = run_experiment(f, ["qrsimbench", "bench", str(spec_path)], headless=headless)
                    met = json.loads((run_dir / "metrics.json").read_text())
                except Exception as e:
                    run_dir, verdict, met = None, "FAIL", {"codes": ["RUNNER_ERROR"], "error": repr(e),
                                                           "infrastructure_failure": True}
                if not met.get("infrastructure_failure") or attempt == 1:
                    break
                log("infrastructure failure, re-running once")
            results.append({"config": name, "rep": rep, "run_id": run_dir.name if run_dir else None,
                            "verdict": verdict, "codes": met.get("codes", []), "metrics": met.get("metrics", {}),
                            "infrastructure_failure": bool(met.get("infrastructure_failure")),
                            "human_near_misses": met.get("human_near_misses", [])})
            (bdir / "results.json").write_text(json.dumps(results, indent=2, default=float))
    metrics = {k: DEFAULT_METRICS[k] for k in spec.get("metrics", DEFAULT_METRICS)}
    table = summarize(results, list(files), spec.get("baseline", list(files)[0]), metrics)
    (bdir / "summary.json").write_text(json.dumps({"spec": str(spec_path), "name": spec.get("name"),
                                                   "base": spec["base"], "reps": reps, "configs": spec["configs"],
                                                   "baseline": spec.get("baseline", list(files)[0]),
                                                   "metrics": {k: list(v) for k, v in metrics.items()},
                                                   "table": table}, indent=2, default=float))
    from qrsimbench.stress.bench_report import render_bench_report

    render_bench_report(bdir)
    return bdir


def _write_summary(bdir: Path, spec: dict, results: list[dict[str, Any]], configs: list[str],
                   baseline: str, spec_path: str) -> None:
    metrics = {k: DEFAULT_METRICS[k] for k in spec.get("metrics", DEFAULT_METRICS)}
    table = summarize(results, configs, baseline, metrics)
    (bdir / "summary.json").write_text(json.dumps({"spec": spec_path, "name": spec.get("name"),
                                                   "base": spec["base"], "reps": spec.get("reps"),
                                                   "configs": spec["configs"], "baseline": baseline,
                                                   "metrics": {k: list(v) for k, v in metrics.items()},
                                                   "table": table}, indent=2, default=float))
    from qrsimbench.stress.bench_report import render_bench_report

    render_bench_report(bdir)


def rescore_bench(bdir: Path) -> Path:
    """Re-run the analysis of every run in a bench (e.g. after an analysis fix) and rebuild its
    results, summary and report. Each run's first metrics.json is kept as metrics.orig.json."""
    from qrsimbench.runner.experiment import analyze

    results = json.loads((bdir / "results.json").read_text())
    for r in results:
        if not r.get("run_id"):
            continue
        rd = RUNS_DIR / r["run_id"]
        orig = rd / "metrics.orig.json"
        if (rd / "metrics.json").exists() and not orig.exists():
            orig.write_text((rd / "metrics.json").read_text())
        met = analyze(rd, yaml.safe_load((rd / "scenario.resolved.yaml").read_text()))
        r.update({"verdict": met["verdict"], "codes": met.get("codes", []), "metrics": met.get("metrics", {}),
                  "infrastructure_failure": bool(met.get("infrastructure_failure")),
                  "human_near_misses": met.get("human_near_misses", [])})
    (bdir / "results.json").write_text(json.dumps(results, indent=2, default=float))
    s = json.loads((bdir / "summary.json").read_text())
    spec = {"name": s["name"], "base": s["base"], "reps": s["reps"], "configs": s["configs"],
            "metrics": list(dict.fromkeys([*s["metrics"], "human_contact_s", "human_contact_robot_moving_s"]))}
    _write_summary(bdir, spec, results, list(s["configs"]), s["baseline"], s["spec"])
    return bdir


def merge_benches(out: Path, bdirs: list[Path], baseline: str) -> Path:
    """One table from several benches of the same scenario (e.g. baselines from a frozen campaign plus
    a later QR version). Configs keep their names; runs are not copied, only referenced."""
    out.mkdir(parents=True, exist_ok=True)
    results, configs, merged = [], {}, None
    for b in bdirs:  # a config in a later bench replaces that config's rows from earlier benches
        s = json.loads((b / "summary.json").read_text())
        merged = merged or s
        rows = json.loads((b / "results.json").read_text())
        later = {r["config"] for r in rows}
        results = [r for r in results if r["config"] not in later]
        results += [{**r, "source_bench": b.name} for r in rows]
        configs.update(s["configs"])
    (out / "results.json").write_text(json.dumps(results, indent=2, default=float))
    spec = {"name": merged["name"], "base": merged["base"], "reps": merged["reps"], "configs": configs,
            "metrics": list(merged["metrics"])}
    _write_summary(out, spec, results, list(configs), baseline, " + ".join(b.name for b in bdirs))
    return out
