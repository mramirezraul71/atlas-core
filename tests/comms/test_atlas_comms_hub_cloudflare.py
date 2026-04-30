from __future__ import annotations

import time


def _build_hub(monkeypatch, tmp_path):
    monkeypatch.setenv("ATLAS_COMMS_DB_PATH", str(tmp_path / "atlas_comms_hub_test.sqlite"))
    monkeypatch.setenv("ATLAS_COMMS_NETWORK_CACHE_S", "0")
    monkeypatch.setenv("ATLAS_COMMS_TUNNEL_GRACE_S", "120")
    from modules.humanoid.comms.atlas_comms_hub import AtlasCommsHub

    return AtlasCommsHub()


def test_offline_mode_uses_grace_for_transient_tunnel_probe_errors(monkeypatch, tmp_path):
    hub = _build_hub(monkeypatch, tmp_path)
    with hub._net_cache_lock:
        hub._last_tunnel_ok_ts = time.time() - 5.0

    monkeypatch.setattr(
        hub,
        "_cloudflare_tunnel_up",
        lambda: (False, "process_check_error:Command timed out after 6 seconds"),
    )
    monkeypatch.setattr(hub, "_internet_up", lambda: (True, "ok:http://1.1.1.1/cdn-cgi/trace"))

    offline, diag = hub._is_offline_mode()
    assert offline is False
    assert diag["tunnel_up"] is True
    assert str(diag["tunnel_reason"]).startswith("transient_probe_grace:")


def test_offline_mode_does_not_grace_after_timeout_window(monkeypatch, tmp_path):
    hub = _build_hub(monkeypatch, tmp_path)
    with hub._net_cache_lock:
        hub._last_tunnel_ok_ts = time.time() - 600.0

    monkeypatch.setattr(
        hub,
        "_cloudflare_tunnel_up",
        lambda: (False, "process_check_error:Command timed out after 6 seconds"),
    )
    monkeypatch.setattr(hub, "_internet_up", lambda: (True, "ok:http://1.1.1.1/cdn-cgi/trace"))

    offline, diag = hub._is_offline_mode()
    assert offline is True
    assert diag["tunnel_up"] is False


def test_missing_cloudflare_alert_url_is_rate_limited(monkeypatch, tmp_path):
    monkeypatch.delenv("ATLAS_CLOUDFLARE_ALERT_URL", raising=False)
    monkeypatch.setenv("ATLAS_COMMS_CLOUDFLARE_ALERT_COOLDOWN_S", "3600")
    hub = _build_hub(monkeypatch, tmp_path)

    emitted = []

    def _fake_emit(text, data=None):
        emitted.append((text, data))

    monkeypatch.setattr(hub, "_emit_ops_critical", _fake_emit)

    out1 = hub._send_urgent_alert("m1", "u1", "mensaje urgente 1", {"internet_up": True})
    out2 = hub._send_urgent_alert("m2", "u1", "mensaje urgente 2", {"internet_up": True})

    assert out1["ok"] is True and out1["channel"] == "ops_fallback"
    assert out2["ok"] is True and out2["channel"] == "ops_fallback"
    assert len(emitted) == 1
