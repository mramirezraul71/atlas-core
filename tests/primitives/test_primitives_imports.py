from __future__ import annotations


def test_nexus_core_primitives_import():
    from modules.nexus_core import navigate_to, reach_pose, grasp, release, pulse_check

    assert callable(navigate_to)
    assert callable(reach_pose)
    assert callable(grasp)
    assert callable(release)
    assert callable(pulse_check)


def test_global_vision_primitives_import():
    from modules.global_vision import scan_network, stream_proxy, perimeter_check

    assert callable(scan_network)
    assert callable(stream_proxy)
    assert callable(perimeter_check)


def test_finanzas_disabled_by_default():
    from modules.finanzas import grasp_market_data, execute_trade

    r = grasp_market_data("SPY", "1m")
    assert r.get("ok") is False
    assert r.get("error") in ("trading_disabled", "market_data_provider_not_configured")
    r2 = execute_trade("SPY", "buy", 1, "market")
    assert r2.get("ok") is False


def test_productividad_schedule_event_offline(monkeypatch, tmp_path):
    monkeypatch.setenv("PRODUCTIVITY_DB_PATH", str(tmp_path / "prod.sqlite"))
    from modules.productividad import schedule_event

    r = schedule_event("Test", "mañana 9:00", "desc")
    assert r.get("ok") is True
    assert r.get("event_id")

