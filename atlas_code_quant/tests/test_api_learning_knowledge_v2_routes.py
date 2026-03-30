from __future__ import annotations

from atlas_code_quant.api.main import app


def _route_paths() -> set[str]:
    return {getattr(route, "path", "") for route in app.routes}


def test_knowledge_v2_routes_present() -> None:
    paths = _route_paths()
    assert "/api/v2/quant/knowledge/search" in paths
    assert "/api/v2/quant/knowledge/ingest" in paths
    assert "/api/v2/quant/knowledge/ingest/status" in paths


def test_learning_orchestrator_v2_routes_present() -> None:
    paths = _route_paths()
    assert "/api/v2/quant/learning/orchestrator/status" in paths
    assert "/api/v2/quant/learning/orchestrator/reconcile" in paths
    assert "/api/v2/quant/learning/orchestrator/daily-analysis" in paths
    assert "/api/v2/quant/learning/ic/summary" in paths
