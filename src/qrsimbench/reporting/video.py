"""Compose the per-camera demo videos of a run into one labelled grid video (ffmpeg)."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

SINGLE_LAYOUT = [("overview", "Overview"), ("isometric", "Factory view"), ("chase", "Chase camera"),
                 ("follow_top", "Top-down follow")]


def camera_specs(resolved: dict) -> list[tuple[str, str]]:
    """(camera spec for the recorder, panel label) for this run."""
    ids = [m["id"] for m in resolved.get("fleet") or []]
    if not ids:
        return SINGLE_LAYOUT
    specs = [("overview", "Overview"), ("isometric", "Factory view")]
    specs += [(f"{i}/chase", f"{i} chase camera") for i in ids[:3]]
    specs.append((f"{ids[0]}/follow_top", f"{ids[0]} top-down"))
    return specs


def compose_run_video(run_dir: Path, resolved: dict) -> Path:
    vdir = run_dir / "video"
    specs = camera_specs(resolved)
    inputs = [vdir / f"video_{spec.replace('/', '_')}.mp4" for spec, _ in specs]
    missing = [p.name for p in inputs if not p.is_file() or p.stat().st_size == 0]
    if missing:
        raise FileNotFoundError(f"missing camera videos: {missing}")
    cols = 2 if len(specs) <= 4 else 3
    w, h = (960, 540) if cols == 2 else (640, 360)
    title = resolved["experiment"]["name"].replace("_", " ").upper()
    fontfile = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    font = f"fontfile={fontfile}:" if Path(fontfile).exists() else ""
    fs = 26 if cols == 2 else 20
    parts = [f"[{i}:v]setpts=PTS-STARTPTS,scale={w}:{h},drawtext={font}text='{label}':x=14:y=12:"
             f"fontsize={fs}:fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=7[v{i}]"
             for i, (_, label) in enumerate(specs)]
    layout = "|".join(f"{(i % cols) * w}_{(i // cols) * h}" for i in range(len(specs)))
    parts.append("".join(f"[v{i}]" for i in range(len(specs)))
                 + f"xstack=inputs={len(specs)}:layout={layout}:shortest=1:fill=black,"
                 f"drawtext={font}text='QR-SimBench  ·  {title}':x=(w-tw)/2:y=h-46:fontsize=28:"
                 "fontcolor=white:box=1:boxcolor=black@0.6:boxborderw=10[out]")
    out = vdir / ("demo_2x2.mp4" if cols == 2 else "demo_grid.mp4")
    cmd = [os.environ.get("FFMPEG", "/usr/bin/ffmpeg"), "-hide_banner", "-loglevel", "error", "-y"]
    for p in inputs:
        cmd += ["-i", str(p)]
    cmd += ["-filter_complex", ";".join(parts), "-map", "[out]", "-c:v", "libx264", "-preset", "veryfast",
            "-crf", "22", "-pix_fmt", "yuv420p", str(out)]
    subprocess.run(cmd, check=True, timeout=900)
    return out
