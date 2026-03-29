from atlas_code_quant.production.grafana_dashboard import (
    _grafana_alert_rule_yaml,
    _grafana_contact_points_yaml,
    _grafana_notification_policies_yaml,
)


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
