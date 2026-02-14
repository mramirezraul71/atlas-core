"""Tests depth estimation: estimate_depth, get_distance_in_bbox."""
import numpy as np
import pytest


def test_estimate_depth_returns_array():
    try:
        from modules.humanoid.vision.depth_estimation import estimate_depth
    except ImportError as e:
        pytest.skip(f"depth deps: {e}")
    frame = np.random.randint(0, 255, (120, 160, 3), dtype=np.uint8)
    depth = estimate_depth(frame)
    assert depth.shape == (120, 160)
    assert depth.dtype == np.float32
    assert depth.min() >= 0 and depth.max() <= 1.01


def test_get_distance_in_bbox():
    try:
        from modules.humanoid.vision.depth_estimation import get_distance_in_bbox
    except ImportError as e:
        pytest.skip(f"depth deps: {e}")
    depth_map = np.linspace(0, 1, 100).reshape(10, 10).astype(np.float32)
    stats = get_distance_in_bbox(depth_map, [1, 1, 5, 5])
    assert "min" in stats and "max" in stats and "mean" in stats and "median" in stats
    assert stats["min"] <= stats["max"]


def test_depth_map_to_colored():
    try:
        from modules.humanoid.vision.depth_estimation import estimate_depth, depth_map_to_colored
    except ImportError as e:
        pytest.skip(f"depth deps: {e}")
    frame = np.zeros((64, 64, 3), dtype=np.uint8)
    depth = estimate_depth(frame)
    colored = depth_map_to_colored(depth)
    assert colored.shape == (*depth.shape, 3)
