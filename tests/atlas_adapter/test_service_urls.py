from atlas_adapter.bootstrap.service_urls import (
    get_camera_proxy_timeout,
    get_nexus_base,
    get_nexus_enabled,
    get_nexus_timeout,
    get_nexus_ws_url,
    get_robot_api_base,
    get_robot_ui_base,
)


def test_nexus_enabled_infers_from_base_url(monkeypatch):
    monkeypatch.delenv("NEXUS_ENABLED", raising=False)
    monkeypatch.setenv("NEXUS_BASE_URL", "http://nexus:8000/")

    assert get_nexus_base() == "http://nexus:8000"
    assert get_nexus_enabled() is True
    assert get_nexus_ws_url() == "ws://nexus:8000/ws"


def test_nexus_enabled_explicit_flag_overrides_inference(monkeypatch):
    monkeypatch.setenv("NEXUS_ENABLED", "false")
    monkeypatch.setenv("NEXUS_BASE_URL", "http://nexus:8000")

    assert get_nexus_enabled() is False


def test_robot_aliases_resolve_consistently(monkeypatch):
    monkeypatch.delenv("NEXUS_ROBOT_API_URL", raising=False)
    monkeypatch.delenv("NEXUS_ROBOT_URL", raising=False)
    monkeypatch.setenv("ROBOT_BASE_URL", "http://robot:8002/")

    assert get_robot_api_base() == "http://robot:8002"
    assert get_robot_ui_base() == "http://robot:8002"
    assert get_robot_ui_base(default_to_api=True) == "http://robot:8002"


def test_robot_defaults_preserve_local_expectations(monkeypatch):
    monkeypatch.delenv("NEXUS_ROBOT_API_URL", raising=False)
    monkeypatch.delenv("NEXUS_ROBOT_URL", raising=False)
    monkeypatch.delenv("ROBOT_BASE_URL", raising=False)

    assert get_robot_api_base() == "http://127.0.0.1:8002"
    assert get_robot_ui_base() == "http://127.0.0.1:5174"
    assert get_robot_ui_base(default_to_api=True) == "http://127.0.0.1:8002"


def test_proxy_timeouts_are_configurable(monkeypatch):
    monkeypatch.setenv("NEXUS_TIMEOUT", "11")
    monkeypatch.setenv("NEXUS_CAMERA_PROXY_TIMEOUT", "22")

    assert get_nexus_timeout() == 11.0
    assert get_camera_proxy_timeout() == 22.0
