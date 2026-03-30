from atlas_code_quant.production.grafana_dashboard import (
    GrafanaDashboard,
    _grafana_alert_rule_yaml,
    _grafana_contact_points_yaml,
    _grafana_notification_policies_yaml,
)
from atlas_code_quant.production.grafana_pro import generate_pro_dashboard


def test_contact_points_yaml_disables_telegram_when_env_missing():
    payload = _grafana_contact_points_yaml(telegram_enabled=False)
    assert "contactPoints: []" in payload
    assert "atlas-telegram" not in payload


def test_contact_points_yaml_enables_telegram_when_env_ready():
    payload = _grafana_contact_points_yaml(telegram_enabled=True)
    assert "name: atlas-telegram" in payload
    assert "secure_settings:" in payload
    assert "bottoken: $TELEGRAM_BOT_TOKEN" in payload


def test_notification_policies_yaml_omits_routes_when_telegram_disabled():
    payload = _grafana_notification_policies_yaml(telegram_enabled=False)
    assert "receiver: grafana-default-email" in payload
    assert "routes:" not in payload


def test_notification_policies_yaml_adds_routes_when_telegram_enabled():
    payload = _grafana_notification_policies_yaml(telegram_enabled=True)
    assert "receiver: grafana-default-email" in payload
    assert "receiver: atlas-telegram" in payload
    assert "atlas_scorecard" in payload


def test_alerting_yaml_includes_recent_unattributed_entries_rule():
    payload = _grafana_alert_rule_yaml()
    assert "atlas_recent_unattributed_entries_count" in payload
    assert "ATLAS Recent Unattributed Entries Present" in payload


def test_alerting_yaml_includes_visual_benchmark_rule():
    payload = _grafana_alert_rule_yaml()
    assert "atlas_visual_benchmark_feedback_score" in payload
    assert "ATLAS Visual Benchmark Feedback Low" in payload


def test_alerting_yaml_includes_options_governance_rule():
    payload = _grafana_alert_rule_yaml()
    assert "atlas_options_strategy_governance_feedback_score" in payload
    assert "ATLAS Options Governance Feedback Low" in payload


def test_main_dashboard_includes_visual_gate_and_benchmark_panels():
    dashboard = GrafanaDashboard().generate_dashboard_json()
    expressions = {
        target.get("expr")
        for panel in dashboard.get("panels", [])
        for target in panel.get("targets", [])
        if isinstance(target, dict)
    }
    assert "atlas_visual_benchmark_feedback_score" in expressions
    assert "atlas_visual_gate_readiness_score_pct" in expressions
    assert "atlas_visual_gate_blocked_total" in expressions
    assert "atlas_options_strategy_governance_feedback_score" in expressions


def test_pro_dashboard_includes_visual_discipline_panels():
    dashboard = generate_pro_dashboard()
    expressions = {
        target.get("expr")
        for panel in dashboard.get("panels", [])
        for target in panel.get("targets", [])
        if isinstance(target, dict)
    }
    assert "atlas_visual_benchmark_feedback_score" in expressions
    assert "atlas_visual_gate_alignment_score_pct" in expressions
    assert "atlas_visual_gate_evaluated_total" in expressions
    assert "atlas_options_strategy_governance_feedback_score" in expressions
