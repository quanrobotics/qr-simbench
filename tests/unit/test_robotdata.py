"""QR-RobotData: episode extraction and catalogue on stored real runs (no ROS needed)."""

import shutil
from pathlib import Path

import numpy as np
import pytest

from qrsimbench.adapters.base import IMPLEMENTED_ROBOT_TYPES, RobotType
from qrsimbench.robotdata.schema import SIGNALS
from qrsimbench.robotdata.store import Catalogue, read_table

FIX = Path(__file__).resolve().parents[1] / "fixtures" / "runs"


@pytest.fixture()
def cat(tmp_path):
    runs = tmp_path / "runs"
    for d in FIX.iterdir():
        shutil.copytree(d, runs / d.name)
    c = Catalogue(tmp_path / "data")
    added, errors = c.ingest_all(runs)
    assert added == 2 and not errors
    return c


def test_episodes_indexed_and_tables_match_schema(cat):
    rows = cat.query()
    assert {r["scenario"] for r in rows} == {"baseline", "blocked_aisle"}
    for r in rows:
        tab = read_table(cat.data_dir / r["table_path"])
        assert set(tab) == set(SIGNALS)
        n = len(tab["timestamp"])
        assert n == r["n_samples"] and n > 100
        assert np.all(np.diff(tab["timestamp"]) > 0)
        assert tab["goal_dist"][-1] < 0.3  # both fixture runs reached the goal


def test_reingest_is_idempotent(cat):
    first = cat.query()[0]["run_dir"]
    assert cat.ingest(Path(first)) is None
    assert len(cat.query()) == 2


def test_summary_and_filters(cat):
    assert cat.query(verdict="pass") and not cat.query(verdict="fail")
    assert {r["scenario"] for r in cat.summary()} == {"baseline", "blocked_aisle"}


def test_robot_types_declared():
    assert RobotType.DIFF_DRIVE_AMR in IMPLEMENTED_ROBOT_TYPES
    assert {"legged", "manipulator", "humanoid"} <= {t.value for t in RobotType}


def test_lerobot_export(cat, tmp_path):
    import json

    pq = pytest.importorskip("pyarrow.parquet")
    from qrsimbench.robotdata.export import export_lerobot

    out = export_lerobot(cat, tmp_path / "ds")
    info = json.loads((out / "meta" / "info.json").read_text())
    assert info["total_episodes"] == 2 and info["features"]["observation.state"]["shape"] == [5]
    t = pq.read_table(out / "data" / "chunk-000" / "episode_000000.parquet")
    assert t.num_rows == info["total_frames"] - pq.read_table(
        out / "data" / "chunk-000" / "episode_000001.parquet").num_rows
    assert len((out / "meta" / "episodes.jsonl").read_text().splitlines()) == 2
