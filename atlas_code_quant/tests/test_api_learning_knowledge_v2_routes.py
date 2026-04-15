from __future__ import annotations

def _route_paths(app) -> set[str]:
    return {getattr(route, "path", "") for route in app.routes}


def test_knowledge_v2_routes_present(api_main_module) -> None:
    paths = _route_paths(api_main_module.app)
    assert "/api/v2/quant/knowledge/search" in paths
    assert "/api/v2/quant/knowledge/ingest" in paths
    assert "/api/v2/quant/knowledge/ingest/status" in paths


def test_learning_orchestrator_v2_routes_present(api_main_module) -> None:
    paths = _route_paths(api_main_module.app)
    assert "/api/v2/quant/learning/orchestrator/status" in paths
    assert "/api/v2/quant/learning/orchestrator/reconcile" in paths
    assert "/api/v2/quant/learning/orchestrator/daily-analysis" in paths
    assert "/api/v2/quant/learning/ic/summary" in paths


def test_operation_status_lite_v2_route_present(api_main_module) -> None:
    paths = _route_paths(api_main_module.app)
    assert "/api/v2/quant/operation/status/lite" in paths
