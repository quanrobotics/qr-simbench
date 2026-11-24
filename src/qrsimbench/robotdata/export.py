"""Export catalogue episodes in a LeRobot-style dataset layout.

Layout follows LeRobotDataset v2.1 (one parquet per episode + meta/*.json[l]), which LeRobot's own
tooling can convert to v3. Columns: observation.state (x, y, yaw, v, w), action (cmd_v, cmd_w),
timestamp, frame_index, episode_index, index, task_index, plus the raw QR signals.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from qrsimbench.robotdata.schema import ACTION_KEYS, DATA_SCHEMA_VERSION, EPISODE_HZ, STATE_KEYS
from qrsimbench.robotdata.store import Catalogue, have_parquet, read_table


def export_lerobot(cat: Catalogue, out: Path, scenario: str | None = None) -> Path:
    if not have_parquet():
        raise RuntimeError("LeRobot export needs pyarrow (pip install pyarrow)")
    import pyarrow as pa
    import pyarrow.parquet as pq

    rows = cat.query(scenario)
    if not rows:
        raise RuntimeError("no episodes in the catalogue (run `qrsimbench data ingest`)")
    (out / "meta").mkdir(parents=True, exist_ok=True)
    (out / "data" / "chunk-000").mkdir(parents=True, exist_ok=True)
    tasks: dict[str, int] = {}
    episodes, all_state, all_action, index = [], [], [], 0
    for ei, r in enumerate(rows):
        tab = read_table(cat.data_dir / r["table_path"])
        n = len(tab["timestamp"])
        task = f"navigate: {r['scenario']}"
        ti = tasks.setdefault(task, len(tasks))
        state = np.stack([tab[k] for k in STATE_KEYS], axis=1)
        action = np.stack([tab[k] for k in ACTION_KEYS], axis=1)
        all_state.append(state)
        all_action.append(action)
        cols = {
            "observation.state": pa.array(state.tolist(), type=pa.list_(pa.float32(), len(STATE_KEYS))),
            "action": pa.array(action.tolist(), type=pa.list_(pa.float32(), len(ACTION_KEYS))),
            "timestamp": pa.array(tab["timestamp"].astype(np.float32)),
            "frame_index": pa.array(np.arange(n, dtype=np.int64)),
            "episode_index": pa.array(np.full(n, ei, dtype=np.int64)),
            "index": pa.array(np.arange(index, index + n, dtype=np.int64)),
            "task_index": pa.array(np.full(n, ti, dtype=np.int64)),
        }
        cols.update({f"qr.{k}": pa.array(v.astype(np.float32)) for k, v in tab.items() if k != "timestamp"})
        pq.write_table(pa.table(cols), out / "data" / "chunk-000" / f"episode_{ei:06d}.parquet")
        episodes.append({"episode_index": ei, "tasks": [task], "length": n,
                         "qr_episode_id": r["episode_id"], "qr_verdict": r["verdict"],
                         "qr_success": bool(r["success"]), "qr_run_dir": r["run_dir"]})
        index += n

    def stats(a: np.ndarray) -> dict:
        return {"mean": a.mean(0).tolist(), "std": a.std(0).tolist(), "min": a.min(0).tolist(),
                "max": a.max(0).tolist(), "count": [len(a)]}

    S, A = np.concatenate(all_state), np.concatenate(all_action)
    info = {
        "codebase_version": "v2.1", "robot_type": rows[0]["robot_type"], "fps": EPISODE_HZ,
        "total_episodes": len(rows), "total_frames": index, "total_tasks": len(tasks), "total_chunks": 1,
        "chunks_size": 1000, "splits": {"train": f"0:{len(rows)}"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "features": {
            "observation.state": {"dtype": "float32", "shape": [len(STATE_KEYS)], "names": STATE_KEYS},
            "action": {"dtype": "float32", "shape": [len(ACTION_KEYS)], "names": ACTION_KEYS},
            "timestamp": {"dtype": "float32", "shape": [1]},
            "frame_index": {"dtype": "int64", "shape": [1]},
            "episode_index": {"dtype": "int64", "shape": [1]},
            "index": {"dtype": "int64", "shape": [1]},
            "task_index": {"dtype": "int64", "shape": [1]},
        },
        "qr_source": DATA_SCHEMA_VERSION,
    }
    (out / "meta" / "info.json").write_text(json.dumps(info, indent=2))
    (out / "meta" / "episodes.jsonl").write_text("".join(json.dumps(e) + "\n" for e in episodes))
    (out / "meta" / "tasks.jsonl").write_text(
        "".join(json.dumps({"task_index": i, "task": t}) + "\n" for t, i in tasks.items()))
    (out / "meta" / "stats.json").write_text(json.dumps({"observation.state": stats(S), "action": stats(A)},
                                                        indent=2))
    return out
