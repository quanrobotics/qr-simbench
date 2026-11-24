"""Environment doctor (QRB-004, spec §11 layer 2). Nonzero exit when a critical check fails."""

from __future__ import annotations

import os
import platform
import shutil
import subprocess
from pathlib import Path

from qrsimbench.config.loader import PROFILES_DIR, REPO_ROOT, ScenarioError, load_scenario, resolve


def _ok(cmd: list[str], timeout: float = 20) -> tuple[bool, str]:
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as e:
        return False, str(e)
    return p.returncode == 0, (p.stdout or p.stderr).strip()


def checks() -> list[tuple[str, bool, bool, str]]:
    """(name, passed, critical, detail)"""
    out = []
    osr = {}
    try:
        osr = dict(
            line.split("=", 1) for line in Path("/etc/os-release").read_text().splitlines() if "=" in line
        )
    except OSError:
        pass
    out.append(("OS", platform.system() == "Linux", True, osr.get("PRETTY_NAME", "").strip('"')))

    distro = os.environ.get("ROS_DISTRO")
    has_ros2 = shutil.which("ros2") is not None
    out.append(("ROS 2 Jazzy", has_ros2 and distro == "jazzy", True,
                f"ROS_DISTRO={distro}" if has_ros2 else "ros2 not on PATH (activate ./env)"))

    ok, ver = _ok(["gz", "sim", "--versions"]) if shutil.which("gz") else (False, "gz not on PATH")
    out.append(("Gazebo Harmonic", ok and ver.startswith("8."), True, ver.splitlines()[-1] if ver else ""))

    if has_ros2:
        ok, pkgs = _ok(["ros2", "pkg", "list"], timeout=60)
        pk = set(pkgs.split()) if ok else set()
        for name in ("nav2_bringup", "nav2_simple_commander", "nav2_minimal_tb3_sim", "ros_gz_sim"):
            out.append((name, name in pk, True, ""))
        out.append(("rosbag2", "rosbag2" in pk or "ros2bag" in pk, True, ""))
        out.append(("MCAP plugin", "rosbag2_storage_mcap" in pk, True, ""))
    else:
        out.append(("Nav2 / rosbag2 / MCAP", False, True, "skipped: no ROS"))

    try:
        resolve(load_scenario(REPO_ROOT / "examples/warehouse_amr/scenarios/baseline.yaml"))
        out.append(("example scenario + profiles", True, True, str(PROFILES_DIR)))
    except ScenarioError as e:
        out.append(("example scenario + profiles", False, True, str(e)))
    models = REPO_ROOT / "simulation/models"
    assets = sorted(p.parent.name for p in models.glob("*/model.sdf"))
    out.append(("obstacle assets", bool(assets), True, ", ".join(assets)))

    runs = REPO_ROOT / "runs"
    runs.mkdir(exist_ok=True)
    out.append(("output path writable", os.access(runs, os.W_OK), True, str(runs)))
    free_gb = shutil.disk_usage(runs).free / 1e9
    out.append(("disk space", free_gb > 10, False, f"{free_gb:.0f} GB free"))
    ok, gpu = _ok(["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"])
    out.append(("GPU", ok, False, gpu if ok else "nvidia-smi unavailable"))
    out.append(("DISPLAY (GUI runs)", bool(os.environ.get("DISPLAY")), False,
                os.environ.get("DISPLAY") or "unset; use runtime.headless: true"))
    return out


def run_doctor() -> int:
    ready = True
    for name, passed, critical, detail in checks():
        tag = "PASS" if passed else ("FAIL" if critical else "WARN")
        ready &= passed or not critical
        print(f"[{tag}] {name}" + (f"  ({detail})" if detail else ""))
    print("READY" if ready else "NOT READY")
    return 0 if ready else 1
