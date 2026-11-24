"""Spawn/delete entities in a running Gazebo Harmonic world via the ros_gz_sim CLI tools."""

from __future__ import annotations

import subprocess

from qrsimbench.config.loader import REPO_ROOT

MODELS_DIR = REPO_ROOT / "simulation" / "models"


class GazeboSpawner:
    def __init__(self, world: str | None = None):
        self.world = world
        self.count = 0

    def apply(self, event: dict) -> tuple[bool, str]:
        a = event["action"]
        if a["type"] == "spawn_entity":
            return self.spawn(a["asset"], a["pose"], a.get("name") or event["id"])
        if a["type"] == "delete_entity":
            return self.delete(a["name"])
        return False, f"unsupported action {a['type']}"

    def spawn(self, asset: str, pose: dict, name: str) -> tuple[bool, str]:
        sdf = MODELS_DIR / asset / "model.sdf"
        if not sdf.is_file():
            return False, f"asset not found: {sdf}"
        cmd = ["ros2", "run", "ros_gz_sim", "create", "-file", str(sdf), "-name", name,
               "-x", str(pose.get("x", 0)), "-y", str(pose.get("y", 0)), "-z", str(pose.get("z", 0)),
               "-Y", str(pose.get("yaw", 0))]
        if self.world:
            cmd += ["-world", self.world]
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        except subprocess.TimeoutExpired:
            return False, "spawn timed out"
        self.count += p.returncode == 0
        return p.returncode == 0, (p.stdout + p.stderr).strip()[-300:]

    def delete(self, name: str) -> tuple[bool, str]:
        if not self.world:
            return False, "world name required for delete"
        req = f'name: "{name}" type: MODEL'
        p = subprocess.run(
            ["gz", "service", "-s", f"/world/{self.world}/remove", "--reqtype", "gz.msgs.Entity",
             "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", req],
            capture_output=True, text=True, timeout=30,
        )
        return p.returncode == 0, (p.stdout + p.stderr).strip()[-300:]
