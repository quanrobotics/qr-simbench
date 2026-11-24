"""Run directory, frozen config and provenance manifest (QRB-005, spec §19)."""

from __future__ import annotations

import json
import os
import platform
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from qrsimbench import SCHEMA_VERSION, __version__
from qrsimbench.config.hashing import sha256_file, sha256_obj
from qrsimbench.config.loader import REPO_ROOT

RUNS_DIR = REPO_ROOT / "runs"


def _cmd(args: list[str], cwd: Path | None = None) -> str | None:
    try:
        out = subprocess.run(args, cwd=cwd, capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return out.stdout.strip() if out.returncode == 0 else None


def git_info(repo: Path = REPO_ROOT) -> dict[str, Any]:
    sha = _cmd(["git", "rev-parse", "HEAD"], repo)
    status = _cmd(["git", "status", "--porcelain"], repo)
    return {"git_commit": sha, "git_dirty": bool(status) if status is not None else None}


def gpu_info() -> list[str]:
    out = _cmd(["nvidia-smi", "--query-gpu=name,driver_version,memory.total", "--format=csv,noheader"])
    return [line.strip() for line in out.splitlines()] if out else []


def cpu_model() -> str:
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor()


def capture_environment() -> dict[str, Any]:
    os_release = {}
    try:
        for line in Path("/etc/os-release").read_text().splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                os_release[k] = v.strip('"')
    except OSError:
        pass
    gz = _cmd(["gz", "sim", "--version"])
    return {
        "os": os_release.get("PRETTY_NAME", platform.system()),
        "kernel": platform.release(),
        "python": sys.version.split()[0],
        "qrsimbench_version": __version__,
        "schema_version": SCHEMA_VERSION,
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION") or "default (rmw_fastrtps_cpp)",
        "gazebo_version": gz.splitlines()[-1] if gz else None,
        "conda_prefix": os.environ.get("CONDA_PREFIX"),
        "cpu": cpu_model(),
        "cpu_count": os.cpu_count(),
        "gpu": gpu_info(),
        "hostname": platform.node(),
    }


def new_run_id(name: str, now: datetime | None = None) -> str:
    now = now or datetime.now(timezone.utc)
    return f"{now.strftime('%Y-%m-%dT%H%M%S')}_{name}"


def create_run_dir(name: str, runs_dir: Path = RUNS_DIR) -> Path:
    base = runs_dir / new_run_id(name)
    d, n = base, 1
    while d.exists():
        n += 1
        d = base.with_name(f"{base.name}_{n}")
    (d / "plots").mkdir(parents=True)
    return d


def write_bundle_start(
    run_dir: Path, scenario_path: Path, resolved: dict[str, Any], command: list[str]
) -> dict[str, Any]:
    """Freeze config + environment into the run dir and write the initial manifest."""
    (run_dir / "scenario.resolved.yaml").write_text(yaml.safe_dump(resolved, sort_keys=False))
    shutil.copy2(scenario_path, run_dir / "scenario.source.yaml")
    env = capture_environment()
    (run_dir / "environment.json").write_text(json.dumps(env, indent=2))
    profiles = resolved.get("resolved_profiles", {})
    manifest = {
        "run_id": run_dir.name,
        "started_at": datetime.now(timezone.utc).isoformat(),
        "finished_at": None,
        "scenario_name": resolved["experiment"]["name"],
        "scenario_source": str(scenario_path),
        "scenario_sha256": sha256_file(scenario_path),
        "resolved_sha256": sha256_obj({k: v for k, v in resolved.items()}),
        "seed": resolved["experiment"]["seed"],
        **git_info(),
        "ros_distro": env["ros_distro"],
        "gazebo_version": env["gazebo_version"],
        "nav2_profile_sha256": sha256_obj(profiles.get("nav2", {})),
        "robot_profile_sha256": sha256_obj(profiles.get("robot", {})),
        "world_profile_sha256": sha256_obj(profiles.get("world", {})),
        "recorded_topics": resolved["recording"]["topics"],
        "command": command,
        "terminal_result": None,
        "artifacts": {
            "scenario": "scenario.resolved.yaml",
            "environment": "environment.json",
            "bag": "run.mcap",
            "events": "events.json",
            "metrics": "metrics.json",
            "report": "report.html",
        },
    }
    write_manifest(run_dir, manifest)
    return manifest


def write_manifest(run_dir: Path, manifest: dict[str, Any]) -> None:
    tmp = run_dir / "manifest.json.tmp"
    tmp.write_text(json.dumps(manifest, indent=2))
    tmp.replace(run_dir / "manifest.json")


def finish_manifest(run_dir: Path, manifest: dict[str, Any], terminal_result: str) -> dict[str, Any]:
    manifest["finished_at"] = datetime.now(timezone.utc).isoformat()
    manifest["terminal_result"] = terminal_result
    write_manifest(run_dir, manifest)
    return manifest


def write_reproduce_script(run_dir: Path) -> None:
    f = run_dir / "reproduce.sh"
    f.write_text(
        "#!/usr/bin/env bash\n"
        "# Re-run this experiment from its frozen source scenario.\n"
        "set -euo pipefail\n"
        f'cd "{REPO_ROOT}"\n'
        f'git checkout "$(jq -r .git_commit "{run_dir}/manifest.json")" 2>/dev/null || true\n'
        f'qrsimbench run "{run_dir}/scenario.source.yaml"\n'
    )
    f.chmod(0o755)
