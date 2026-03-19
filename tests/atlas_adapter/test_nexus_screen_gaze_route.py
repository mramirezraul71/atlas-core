from __future__ import annotations

from atlas_adapter.atlas_http_api import app


def test_nexus_screen_gaze_route_is_registered():
    paths = {route.path for route in app.routes}

    assert "/api/primitives/nexus/look-at-screen" in paths
