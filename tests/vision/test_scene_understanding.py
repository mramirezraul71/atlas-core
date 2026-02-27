"""Tests scene understanding: describe_scene."""
import numpy as np
import pytest


def test_describe_scene_returns_dict():
    try:
        from modules.humanoid.vision.scene_understanding import describe_scene
    except ImportError as e:
        pytest.skip(f"scene deps: {e}")
    frame = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
    data = describe_scene(frame, detail_level="brief")
    assert "description" in data
    assert "objects" in data
    assert "confidence" in data
    assert isinstance(data["description"], str)
    assert len(data["description"]) > 0
