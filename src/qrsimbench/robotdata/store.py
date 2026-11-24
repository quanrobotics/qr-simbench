"""QR-RobotData local store: episode tables on disk + SQLite catalogue (stdlib only for the index).

Layout (under data/ at the repo root):
    data/catalogue.sqlite                     index of runs and episodes
    data/episodes/<run_id>/<robot_id>.parquet episode table (.npz if pyarrow is not installed)
    data/episodes/<run_id>/<robot_id>.json    EpisodeMeta
"""

from __future__ import annotations

import json
import sqlite3
from pathlib import Path
from typing import Any

import numpy as np

from qrsimbench.config.loader import REPO_ROOT
from qrsimbench.robotdata.extract import extract_episode
from qrsimbench.robotdata.schema import EpisodeMeta

DATA_DIR = REPO_ROOT / "data"

SCHEMA_SQL = """
CREATE TABLE IF NOT EXISTS episodes (
  episode_id TEXT PRIMARY KEY, run_id TEXT, robot_id TEXT, robot_type TEXT, robot_profile TEXT,
  scenario TEXT, world TEXT, seed INTEGER, started_at TEXT, duration_s REAL, n_samples INTEGER,
  success INTEGER, verdict TEXT, codes TEXT, metrics TEXT, git_commit TEXT, resolved_sha256 TEXT,
  table_path TEXT, meta_path TEXT, run_dir TEXT, schema_version TEXT
);
CREATE INDEX IF NOT EXISTS idx_ep_scenario ON episodes(scenario);
CREATE INDEX IF NOT EXISTS idx_ep_verdict ON episodes(verdict);
"""


def have_parquet() -> bool:
    try:
        import pyarrow  # noqa: F401
    except ImportError:
        return False
    return True


def write_table(table: dict[str, np.ndarray], path_no_ext: Path) -> Path:
    if have_parquet():
        import pyarrow as pa
        import pyarrow.parquet as pq

        out = path_no_ext.with_suffix(".parquet")
        pq.write_table(pa.table({k: np.asarray(v, dtype=np.float64) for k, v in table.items()}), out)
        return out
    out = path_no_ext.with_suffix(".npz")
    np.savez_compressed(out, **table)
    return out


def read_table(path: Path) -> dict[str, np.ndarray]:
    if path.suffix == ".parquet":
        import pyarrow.parquet as pq

        t = pq.read_table(path)
        return {c: t.column(c).to_numpy() for c in t.column_names}
    with np.load(path) as z:
        return {k: z[k] for k in z.files}


class Catalogue:
    def __init__(self, data_dir: Path = DATA_DIR):
        self.data_dir = data_dir
        data_dir.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(data_dir / "catalogue.sqlite")
        self.db.row_factory = sqlite3.Row
        self.db.executescript(SCHEMA_SQL)

    def ingest(self, run_dir: Path, robot_id: str = "robot_0", force: bool = False) -> EpisodeMeta | None:
        run_dir = run_dir.resolve()
        eid = f"{run_dir.name}/{robot_id}"
        if not force and self.db.execute("SELECT 1 FROM episodes WHERE episode_id=?", (eid,)).fetchone():
            return None
        table, meta = extract_episode(run_dir, robot_id)
        ep_dir = self.data_dir / "episodes" / meta.run_id
        ep_dir.mkdir(parents=True, exist_ok=True)
        tpath = write_table(table, ep_dir / robot_id)
        meta.files = {"table": str(tpath.relative_to(self.data_dir)), "run_dir": str(run_dir),
                      "bag": "bag/", "manifest": "manifest.json", "report": "report.html"}
        mpath = ep_dir / f"{robot_id}.json"
        mpath.write_text(json.dumps(meta.to_dict(), indent=2))
        self.db.execute(
            "INSERT OR REPLACE INTO episodes VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
            (meta.episode_id, meta.run_id, meta.robot_id, meta.robot_type, meta.robot_profile,
             meta.scenario, meta.world, meta.seed, meta.started_at, meta.duration_s, meta.n_samples,
             int(meta.success), meta.verdict, json.dumps(meta.codes), json.dumps(meta.metrics),
             meta.git_commit, meta.resolved_sha256, str(tpath.relative_to(self.data_dir)),
             str(mpath.relative_to(self.data_dir)), str(run_dir), meta.schema_version))
        self.db.commit()
        return meta

    def ingest_all(self, runs_dir: Path, force: bool = False) -> tuple[int, list[str]]:
        added, errors = 0, []
        for d in sorted(p for p in runs_dir.iterdir() if (p / "metrics.json").exists()):
            try:
                import yaml

                res = yaml.safe_load((d / "scenario.resolved.yaml").read_text())
                for rid in [m["id"] for m in res.get("fleet") or []] or ["robot_0"]:
                    if self.ingest(d, robot_id=rid, force=force):
                        added += 1
            except Exception as e:  # keep going; report which runs could not be converted
                errors.append(f"{d.name}: {e!r}")
        return added, errors

    def query(self, scenario: str | None = None, verdict: str | None = None) -> list[dict[str, Any]]:
        sql, args = "SELECT * FROM episodes WHERE 1=1", []
        if scenario:
            sql += " AND scenario=?"
            args.append(scenario)
        if verdict:
            sql += " AND verdict=?"
            args.append(verdict.upper())
        rows = self.db.execute(sql + " ORDER BY started_at", args).fetchall()
        return [dict(r) for r in rows]

    def summary(self) -> list[dict[str, Any]]:
        return [dict(r) for r in self.db.execute(
            "SELECT scenario, COUNT(*) n, SUM(success) n_success, ROUND(SUM(duration_s)/60.0, 1) minutes, "
            "SUM(n_samples) samples FROM episodes GROUP BY scenario ORDER BY scenario").fetchall()]
