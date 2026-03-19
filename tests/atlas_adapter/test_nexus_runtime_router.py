from pathlib import Path

from atlas_adapter.routes.nexus_runtime import build_router


def test_nexus_runtime_router_exposes_expected_paths():
    router = build_router(Path("C:/repo"), Path("C:/repo/config/atlas.env"))
    paths = {route.path for route in router.routes}

    assert "/api/nexus/connection" in paths
    assert "/api/nexus/reconnect" in paths
    assert "/api/robot/status" in paths
    assert "/api/robot/reconnect" in paths
    assert "/api/cuerpo/reconnect" in paths
    assert "/api/robot/start-commands" in paths
