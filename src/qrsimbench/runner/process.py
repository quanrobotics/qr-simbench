"""Child-process supervision: every child gets its own process group so shutdown is total."""

from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path


class Child:
    def __init__(self, name: str, cmd: list[str], log_dir: Path, env: dict | None = None,
                 cwd: Path | None = None):
        self.name = name
        self.cmd = cmd
        self.log = open(log_dir / f"{name}.log", "w")  # noqa: SIM115 - closed in stop()
        self.proc = subprocess.Popen(
            cmd, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True,
            env={**os.environ, **(env or {})}, cwd=cwd,
        )

    @property
    def alive(self) -> bool:
        return self.proc.poll() is None

    def wait(self, timeout: float | None = None) -> int | None:
        try:
            return self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return None

    def stop(self, grace: float = 10.0) -> int | None:
        """SIGINT the whole group (lets rosbag2 close the MCAP), then TERM, then KILL."""
        for sig, wait_s in ((signal.SIGINT, grace), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                break
            deadline = time.time() + wait_s
            while time.time() < deadline:
                if self.proc.poll() is not None and not _group_alive(self.proc.pid):
                    break
                time.sleep(0.2)
            else:
                continue
            break
        self.log.close()
        return self.proc.poll()


def _group_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def group_rss_mb(pgid: int) -> float:
    """Resident memory of every process in a process group (MB), from /proc."""
    total = 0
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            if os.getpgid(int(d)) != pgid:
                continue
            for line in open(f"/proc/{d}/status"):
                if line.startswith("VmRSS:"):
                    total += int(line.split()[1])
                    break
        except (ProcessLookupError, FileNotFoundError, PermissionError, ValueError):
            continue
    return total / 1024


def mem_available_mb() -> float:
    for line in open("/proc/meminfo"):
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) / 1024
    return float("inf")
