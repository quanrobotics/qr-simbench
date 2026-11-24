"""Report plots as inline SVG strings (no external files needed by report.html)."""

from __future__ import annotations

import io
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

from qrsimbench.analysis.model import NavigationRun  # noqa: E402
from qrsimbench.config.loader import REPO_ROOT  # noqa: E402

INK = "#1f2933"
MUTED = "#7b8794"
ACCENT = "#2563eb"
EVENT = "#d97706"
FLEET_COLORS = ["#2563eb", "#16a34a", "#9333ea", "#dc2626", "#0891b2", "#ca8a04"]
plt.rcParams.update({
    "font.size": 10, "axes.edgecolor": "#cbd2d9", "axes.labelcolor": INK, "xtick.color": MUTED,
    "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False,
    "svg.fonttype": "none",
})


def _svg(fig) -> str:
    buf = io.StringIO()
    fig.savefig(buf, format="svg", bbox_inches="tight", transparent=True)
    plt.close(fig)
    s = buf.getvalue()
    return s[s.find("<svg"):]


def _events(ax, run: NavigationRun):
    for e in run.events:
        if e["id"].startswith("_"):
            continue
        t = e.get("t_rel", 0.0)
        ax.axvline(t, color=EVENT, lw=1, ls="--")
        ax.text(t, ax.get_ylim()[1], f" {e['id']}", color=EVENT, va="top", fontsize=8)


def _asset_size(asset: str) -> tuple[float, float]:
    import re

    f = REPO_ROOT / "simulation" / "models" / asset / "model.sdf"
    m = re.search(r"<size>([\d.]+) ([\d.]+) [\d.]+</size>", f.read_text()) if f.is_file() else None
    return (float(m.group(1)), float(m.group(2))) if m else (0.4, 0.4)


def _draw_obstacles(ax, resolved: dict) -> None:
    from matplotlib.patches import Rectangle
    from matplotlib.transforms import Affine2D

    for e in resolved.get("events", []):
        a = e["action"]
        if a["type"] != "spawn_entity":
            continue
        p = a["pose"]
        sx, sy = _asset_size(a["asset"])
        rect = Rectangle((-sx / 2, -sy / 2), sx, sy, color=EVENT, alpha=0.75)
        rect.set_transform(Affine2D().rotate(p.get("yaw", 0.0)).translate(p["x"], p["y"]) + ax.transData)
        ax.add_patch(rect)
        ax.annotate(e["id"], (p["x"], p["y"]), color=EVENT, fontsize=8, xytext=(8, 8),
                    textcoords="offset points")


def trajectory(run: NavigationRun, resolved: dict, path_out: Path | None = None,
               others: list[tuple[NavigationRun, str, str]] | None = None) -> str:
    """Top-down plot on the map. `others` adds extra (run, label, color) paths for comparisons."""
    fig, ax = plt.subplots(figsize=(7, 5))
    map_rel = resolved.get("resolved_profiles", {}).get("world", {}).get("map")
    mp = REPO_ROOT / map_rel if map_rel else None
    if mp and mp.is_file():
        meta = yaml.safe_load(mp.read_text())
        img = plt.imread(mp.parent / meta["image"])
        h, w = img.shape[:2]
        r, (ox, oy) = meta["resolution"], meta["origin"][:2]
        ax.imshow(img, cmap="gray", extent=[ox, ox + w * r, oy, oy + h * r], alpha=0.55)
    _draw_obstacles(ax, resolved)
    paths = [(run, "executed path", ACCENT)] if not others else others
    for rr, label, color in paths:
        ax.plot(rr.x, rr.y, color=color, lw=2, label=label)
    ax.plot(*run.start, "o", color=INK, ms=7)
    ax.annotate("start", run.start, xytext=(6, -12), textcoords="offset points", fontsize=8)
    ax.plot(*run.goal, "*", color="#16a34a", ms=14)
    ax.annotate("goal", run.goal, xytext=(6, 6), textcoords="offset points", fontsize=8)
    xs = np.concatenate([rr.x for rr, _, _ in paths] + [[run.start[0], run.goal[0]]])
    ys = np.concatenate([rr.y for rr, _, _ in paths] + [[run.start[1], run.goal[1]]])
    pad = 1.0
    ax.set_xlim(xs.min() - pad, xs.max() + pad)
    ax.set_ylim(ys.min() - pad, ys.max() + pad)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    if others:
        ax.legend(loc="best", frameon=False, fontsize=8)
    if path_out:
        fig.savefig(path_out, dpi=120, bbox_inches="tight")
    return _svg(fig)


def velocity(run: NavigationRun, path_out: Path | None = None,
             others: list[tuple[NavigationRun, str, str]] | None = None) -> str:
    fig, ax = plt.subplots(figsize=(7, 2.4))
    if others:
        for rr, label, color in others:
            ax.plot(rr.vel_t, rr.v, color=color, lw=1.2, label=f"{label} [m/s]")
    else:
        ax.plot(run.vel_t, run.v, color=ACCENT, lw=1.4, label="linear [m/s]")
        ax.plot(run.vel_t, run.w, color=MUTED, lw=1, label="angular [rad/s]")
    if run.mission_end is not None:
        ax.set_xlim(-2, run.mission_end + 2)
    ax.set_xlabel("time since mission start [s]")
    ax.legend(loc="upper right", frameon=False, fontsize=8)
    _events(ax, run)
    if path_out:
        fig.savefig(path_out, dpi=120, bbox_inches="tight")
    return _svg(fig)


def clearance(
    run: NavigationRun, warn: float | None, fail: float | None, path_out: Path | None = None,
    others: list[tuple[NavigationRun, str, str]] | None = None,
) -> str:
    fig, ax = plt.subplots(figsize=(7, 2.4))
    for rr, label, color in others or [(run, None, ACCENT)]:
        ax.plot(rr.scan_t, rr.scan_min, color=color, lw=1.1, label=label)
    if warn:
        ax.axhline(warn, color=EVENT, lw=1, ls=":", label=f"warn {warn} m")
    if fail:
        ax.axhline(fail, color="#dc2626", lw=1, ls=":", label=f"fail {fail} m")
    if run.mission_end is not None:
        ax.set_xlim(-2, run.mission_end + 2)
    ax.set_ylim(0, max(1.5, float(np.nanmax(run.scan_min)) if len(run.scan_min) else 1.5))
    ax.set_xlabel("time since mission start [s]")
    ax.set_ylabel("min LiDAR range [m]")
    ax.legend(loc="upper right", frameon=False, fontsize=8)
    _events(ax, run)
    if path_out:
        fig.savefig(path_out, dpi=120, bbox_inches="tight")
    return _svg(fig)
