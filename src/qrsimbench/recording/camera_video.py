"""Record Gazebo demo cameras straight to MP4 (no screen capture, no PNG frames).

    python -m qrsimbench.recording.camera_video OUT_DIR overview isometric chase amr_1/chase ...

Each camera spec is [<robot namespace>/]<name>; it subscribes to gz topic /[<ns>/]qr_demo/<name> and
pipes raw RGB frames into one ffmpeg per camera, writing OUT_DIR/video_<ns>_<name>.mp4 (or
video_<name>.mp4). Stops cleanly on SIGINT/SIGTERM. Needs the ROS env (gz Python
bindings). Frames are written at the sensor rate (15 Hz); with RTF ~1 that is real time.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

FPS = 15


class CameraWriter:
    def __init__(self, out: Path):
        self.out = out
        self.proc: subprocess.Popen | None = None
        self.lock = threading.Lock()
        self.frames = 0
        self.first_frame_epoch: float | None = None

    def write(self, msg) -> None:
        with self.lock:
            if self.proc is None:
                self.first_frame_epoch = time.time()
                self.proc = subprocess.Popen(
                    [os.environ.get("FFMPEG", "/usr/bin/ffmpeg"), "-hide_banner", "-loglevel", "error", "-y",
                     "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{msg.width}x{msg.height}",
                     "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "veryfast", "-crf", "22",
                     "-pix_fmt", "yuv420p", str(self.out)],
                    stdin=subprocess.PIPE,
                )
            try:
                self.proc.stdin.write(msg.data)
                self.frames += 1
            except (BrokenPipeError, ValueError):
                pass

    def close(self) -> None:
        with self.lock:
            if self.proc is not None:
                self.proc.stdin.close()
                self.proc.wait(timeout=60)


def main(out_dir: str, names: list[str]) -> int:
    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    node = Node()
    writers = {}
    for n in names:
        ns, _, cam = n.rpartition("/")
        w = writers[n] = CameraWriter(out / f"video_{n.replace('/', '_')}.mp4")
        node.subscribe(Image, f"/{ns}/qr_demo/{cam}" if ns else f"/qr_demo/{cam}", w.write)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    while not stop.is_set():
        stop.wait(0.5)
    for n, w in writers.items():
        w.close()
        print(f"{n}: {w.frames} frames, first at {w.first_frame_epoch}", flush=True)
    (out / "video_timing.txt").write_text(
        "".join(f"{n} {w.frames} {w.first_frame_epoch}\n" for n, w in writers.items()))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2:]))
