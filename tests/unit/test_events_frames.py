import math

import numpy as np
import pytest

from qrsimbench.analysis.bag_reader import to_map_frame
from qrsimbench.navigation.mission import _due


def test_elapsed_trigger():
    assert not _due({"type": "elapsed_time", "seconds": 8}, 7.9, None)
    assert _due({"type": "elapsed_time", "seconds": 8}, 8.0, None)


def test_robot_near_trigger():
    trig = {"type": "robot_near", "x": 1.0, "y": 1.0, "radius_m": 0.5}
    assert not _due(trig, 100, None)
    assert not _due(trig, 0, (0.0, 0.0))
    assert _due(trig, 0, (1.3, 1.2))


def test_map_frame_uses_latest_transform_and_falls_back_to_start():
    od = np.array([[0.0, 1.0, 0.0, 0.0], [2.0, 1.0, 0.0, 0.0]])
    mo = np.array([[1.0, 10.0, 0.0, math.pi / 2]])
    x, y = to_map_frame(od, mo, {"x": -2.0, "y": 0.5, "yaw": 0.0})
    assert (x[0], y[0]) == pytest.approx((-1.0, 0.5))  # before first TF: start-pose fallback
    assert (x[1], y[1]) == pytest.approx((10.0, 1.0))  # rotated by 90 deg, shifted by (10, 0)
