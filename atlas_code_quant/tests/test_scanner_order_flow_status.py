from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from scanner.opportunity_scanner import OpportunityScannerService  # noqa: E402


def test_scanner_status_exposes_hybrid_order_flow_system() -> None:
    svc = OpportunityScannerService()

    payload = svc.status()

    assert "order_flow_system" in payload
    assert "telemetry" in payload["order_flow_system"]
    assert payload["order_flow_system"]["runtime_mode"] in {"hybrid_options_intradia", "proxy_intradia"}
    assert payload["config"]["options_flow_cache_ttl_sec"] >= 15


def test_scanner_criteria_catalog_reflects_runtime_order_flow_mode() -> None:
    svc = OpportunityScannerService()

    criteria = svc.criteria_catalog()
    order_flow = next(item for item in criteria if item["key"] == "order_flow_proxy")

    assert order_flow["runtime_mode"] in {"hybrid_options_intradia", "proxy_intradia"}
    assert "telemetry_backend" in order_flow
