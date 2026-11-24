"""Import the AWS RoboMaker small warehouse (MIT-0) as a QR layout file.

    python scripts/importers/aws_small_warehouse.py [SRC_DIR]   # default third_party/maps/aws_small_warehouse

Reads the world file's model poses and each model's collision-mesh footprint (DAE vertex bounds, rotated by
the model yaw). The shelves (ShelfD/E, whose mesh carries an internal transform) use the footprint measured
from the published occupancy map (maps/005). Writes simulation/worlds/aws_small_warehouse.layout.yaml;
stations, chargers, workers and the forklift are QR additions for the showcase (marked below).
"""
from __future__ import annotations

import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
SRC = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "third_party/maps/aws_small_warehouse"
SKIP = ("Ground", "Roof", "Wall", "Lamp", "ShelfF")  # floor/roof/outer wall; ShelfF = wall racking (stubs below)
SHELF_DE = (4.1, 0.9, -0.12)  # x size, y size, x offset of ShelfD/E as measured from the published map


def mesh_bounds(model: str) -> tuple[np.ndarray, np.ndarray]:
    f = SRC / "models" / model / "meshes" / f"{model}_collision.DAE"
    s = f.read_text(errors="ignore")
    unit = float((re.search(r'<unit[^>]*meter="([\d.eE-]+)"', s) or [None, 1])[1])
    pts = [np.array(m.group(1).split(), float).reshape(-1, 3)
           for m in re.finditer(r'<float_array[^>]*id="[^"]*position[^"]*"[^>]*>([^<]*)<', s, re.I)]
    a = np.vstack(pts) * unit
    return a.min(0), a.max(0)


def main() -> int:
    world = SRC / "worlds/no_roof_small_warehouse/no_roof_small_warehouse.world"
    root = ET.parse(world).getroot()
    blocks, racks, clutter, n = [], [], [], {}
    items = []  # (include element, pose text): pose on the include or on its wrapping <model>
    for parent in root.iter():
        for inc in parent.findall("include"):
            items.append((inc, inc.findtext("pose") or parent.findtext("pose")))
    for inc, pose in items:
        uri = inc.findtext("uri").replace("model://", "")
        short = uri.replace("aws_robomaker_warehouse_", "").replace("_01", "")
        if any(short.startswith(k) for k in SKIP):
            continue
        x, y, _, _, _, yaw = map(float, (pose or "0 0 0 0 0 0").split())
        n[short] = n.get(short, 0) + 1
        name = f"aws_{short.lower()}_{n[short]}"
        if short in ("ShelfD", "ShelfE"):
            racks.append({"name": name, "x": round(x + SHELF_DE[2], 3), "y": round(y, 3), "sx": SHELF_DE[0], "sy": SHELF_DE[1],
                          "h": 2.0, "color": [0.2, 0.42, 0.7], "uprights": {"at": [0.02, 0.5, 0.98]}})
            continue
        lo, hi = (v.astype(float).tolist() for v in mesh_bounds(uri))
        cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
        if short == "ClutteringD":  # its mesh offset is not where the map shows it; the map agrees with the pose
            cx = cy = 0.0
        sx, sy = hi[0] - lo[0], hi[1] - lo[1]
        c, s_ = math.cos(yaw), math.sin(yaw)
        quarter = abs(s_) > 0.7  # yaws in this world are multiples of ~90 degrees
        item = {"name": name, "x": round(x + c * cx - s_ * cy, 3), "y": round(y + s_ * cx + c * cy, 3),
                "sx": round(sy if quarter else sx, 2), "sy": round(sx if quarter else sy, 2),
                "h": round(float(min(hi[2], 1.8)), 2)}
        (clutter if short.startswith(("Bucket", "Trash", "PalletJack")) else blocks).append(item)
    # West wall racking (ShelfF): its frames show as four stubs in the published map
    for i, y in enumerate((7.9, 3.25, -1.2, -5.5)):
        blocks.append({"name": f"aws_wall_rack_{i + 1}", "x": -5.9, "y": y, "sx": 2.2, "sy": 0.15, "h": 2.0,
                       "color": [0.45, 0.47, 0.5]})
    layout = {
        "id": "aws_small_warehouse",
        "source": {"name": "AWS RoboMaker small warehouse world",
                   "url": "https://github.com/aws-robotics/aws-robomaker-small-warehouse-world", "license": "MIT-0",
                   "note": "geometry imported from the world file and published map; stations, chargers, workers "
                           "and forklift added by QR for the showcase"},
        "bounds": {"x_min": -6.9, "x_max": 6.9, "y_min": -10.4, "y_max": 10.4},
        "wall": {"thickness": 0.2, "height": 2.5, "color": [0.8, 0.8, 0.82]},
        "map_resolution": 0.05,
        "racks": racks, "blocks": blocks, "clutter": clutter,
        # --- QR additions (showcase task layout) ---
        "stations": {
            "station_receiving": {"x": -3.6, "y": 8.6, "yaw": 0.0, "label": "Receiving", "color": [0.15, 0.45, 0.9]},
            "station_pick_1": {"x": 3.6, "y": -0.33, "yaw": 0.0, "label": "Aisle 1", "color": [0.95, 0.65, 0.1]},
            "station_pick_3": {"x": 3.6, "y": -3.94, "yaw": 0.0, "label": "Aisle 3", "color": [0.95, 0.65, 0.1]},
            "station_pick_5": {"x": 3.6, "y": -7.71, "yaw": 0.0, "label": "Aisle 5", "color": [0.95, 0.65, 0.1]},
            "station_dispatch": {"x": -3.6, "y": -8.8, "yaw": 0.0, "label": "Dispatch", "color": [0.1, 0.65, 0.3]},
            "station_charge_1": {"x": -6.0, "y": 5.6, "yaw": 0.0, "label": "Charge 1", "color": [0.3, 0.75, 0.4]},
            "station_charge_2": {"x": -6.0, "y": 1.0, "yaw": 0.0, "label": "Charge 2", "color": [0.3, 0.75, 0.4]},
            "station_charge_3": {"x": -6.0, "y": -3.4, "yaw": 0.0, "label": "Charge 3", "color": [0.3, 0.75, 0.4]},
        },
        "actors": [
            {"name": "worker_1", "kind": "worker", "speed": 0.5, "path": [[1.6, 7.0], [1.6, -8.5]]},
            {"name": "worker_2", "kind": "worker", "speed": 0.5, "path": [[-4.6, -2.0], [1.0, -2.0]]},
            {"name": "forklift_1", "kind": "forklift", "speed": 0.7,
             "path": [[-3.0, -6.5], [-3.0, 7.0], [-3.6, 7.0], [-3.6, -6.5]]},
        ],
        "demo_cameras": [{"name": "overview", "pose": "0 0 24 0 1.5707 1.5707", "hfov": 1.1}],
    }
    out = ROOT / "simulation/worlds/aws_small_warehouse.layout.yaml"
    out.write_text("# Generated by scripts/importers/aws_small_warehouse.py; source MIT-0, see 'source'.\n"
                   + yaml.safe_dump(layout, sort_keys=False, width=120))
    print(out, len(racks), "racks", len(blocks), "blocks", len(clutter), "clutter")
    return 0


if __name__ == "__main__":
    sys.exit(main())
