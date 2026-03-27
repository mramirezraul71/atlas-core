from __future__ import annotations

from atlas_code_quant.production.grafana_dashboard import GrafanaDashboard
from atlas_code_quant.production.grafana_pro import generate_pro_dashboard


def test_main_dashboard_queries_broker_metrics():
    dashboard = GrafanaDashboard().generate_dashboard_json()
    expressions = []
    for panel in dashboard["panels"]:
        for target in panel.get("targets", []):
            expressions.append(target.get("expr", ""))

    assert any("atlas_broker_equity_usd" in expr for expr in expressions)
    assert any("atlas_sync_status" in expr for expr in expressions)
    variable_names = {item["name"] for item in dashboard["templating"]["list"]}
    assert {"scope", "source", "account_id"}.issubset(variable_names)


def test_pro_dashboard_uses_canonical_variables_and_metrics():
    dashboard = generate_pro_dashboard()
    expressions = []
    for panel in dashboard["panels"]:
        for target in panel.get("targets", []):
            expressions.append(target.get("expr", ""))

    assert any("atlas_broker_open_pnl_usd" in expr for expr in expressions)
    assert any("atlas_reconcile_gap_usd" in expr for expr in expressions)
    variable_names = {item["name"] for item in dashboard["templating"]["list"]}
    assert {"scope", "source", "account_id"}.issubset(variable_names)
