"""Integración FastAPI del :mod:`modules.atlas_radar_kalshi.dashboard`.

Verifica que los endpoints REST del router responden con el contrato
acordado, sin necesitar el orquestador real corriendo.
"""
from __future__ import annotations

from pathlib import Path

import pytest
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from fastapi import FastAPI
from fastapi.testclient import TestClient

from modules.atlas_radar_kalshi.config import get_settings
from modules.atlas_radar_kalshi.dashboard import RadarState, build_router
from modules.atlas_radar_kalshi import preflight as preflight_mod


@pytest.fixture()
def client(tmp_path, monkeypatch) -> TestClient:
    monkeypatch.setenv("ATLAS_LOG_DIR", str(tmp_path / "logs"))
    # forzar nuevas settings
    get_settings.cache_clear()  # type: ignore[attr-defined]
    state = RadarState()
    state.balance_cents = 50_000
    app = FastAPI()
    app.include_router(build_router(state))
    return TestClient(app)


# ---------------------------------------------------------------------------
class TestRoutes:
    def test_status_ok(self, client: TestClient) -> None:
        r = client.get("/api/radar/status")
        assert r.status_code == 200
        data = r.json()
        assert data["ok"] is True
        assert data["module"] == "atlas_radar_kalshi"
        assert "balance_usd" in data

    def test_markets_empty(self, client: TestClient) -> None:
        r = client.get("/api/radar/markets")
        assert r.status_code == 200
        data = r.json()
        assert data["ok"] is True
        assert data["rows"] == []

    def test_decisions_empty(self, client: TestClient) -> None:
        r = client.get("/api/radar/decisions")
        assert r.status_code == 200
        assert r.json()["items"] == []

    def test_orders_empty(self, client: TestClient) -> None:
        r = client.get("/api/radar/orders")
        assert r.status_code == 200
        assert r.json()["items"] == []

    def test_arbitrage_empty(self, client: TestClient) -> None:
        r = client.get("/api/radar/arbitrage")
        assert r.status_code == 200
        assert r.json()["rows"] == []
        assert r.json()["activity"] == []
        assert r.json()["stats"] == {}
        assert "window_1h" in r.json()
        assert "window_24h" in r.json()

    def test_metrics_no_journal(self, client: TestClient) -> None:
        r = client.get("/api/radar/metrics")
        assert r.status_code == 200
        data = r.json()
        assert "performance" in data
        assert data["performance"]["trades"] == 0
        assert "execution" in data
        assert "risk" in data

    def test_risk_endpoint(self, client: TestClient) -> None:
        r = client.get("/api/radar/risk")
        assert r.status_code == 200
        assert r.json()["ok"] is True

    def test_config_get(self, client: TestClient) -> None:
        r = client.get("/api/radar/config")
        assert r.status_code == 200
        data = r.json()
        assert data["ok"] is True
        assert data["environment"] in {"demo", "prod"}
        assert data["execution_mode"] in {"paper", "live"}
        assert "paper_balance_usd" in data

    def test_config_set_paper_balance(self, client: TestClient) -> None:
        r = client.post(
            "/api/radar/config",
            json={
                "environment": "demo",
                "execution_mode": "paper",
                "paper_balance_usd": 250.0,
                "persist_env": False,
            },
        )
        assert r.status_code == 200
        data = r.json()
        assert data["ok"] is True
        assert data["applied"]["execution_mode"] == "paper"
        assert data["applied"]["paper_balance_cents"] == 25_000

    def test_config_set_live_requires_credentials(self, client: TestClient) -> None:
        r = client.post(
            "/api/radar/config",
            json={
                "environment": "prod",
                "execution_mode": "live",
                "paper_balance_usd": 100.0,
                "api_key_id": "",
                "private_key_path": "C:/no/such/kalshi_private.pem",
                "persist_env": False,
            },
        )
        assert r.status_code == 400

    def test_health_endpoint(self, client: TestClient) -> None:
        r = client.get("/api/radar/health")
        assert r.status_code == 200
        data = r.json()
        assert data["ok"] is True
        assert "degraded" in data

    def test_preflight_missing_kalshi_credentials(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.setenv("KALSHI_API_KEY_ID", "")
        monkeypatch.setenv("KALSHI_PRIVATE_KEY_PATH", str(tmp_path / "missing.pem"))
        monkeypatch.setenv("POLYMARKET_ENABLED", "0")
        monkeypatch.setenv("RADAR_PREFLIGHT_REQUIRE_OLLAMA", "0")
        get_settings.cache_clear()  # type: ignore[attr-defined]

        state = RadarState()
        app = FastAPI()
        app.include_router(build_router(state))
        cli = TestClient(app)

        r = cli.get("/api/radar/preflight")
        assert r.status_code == 200
        data = r.json()
        assert "checks" in data
        assert "readiness" in data
        assert data["checks"]["kalshi_rest"]["ok"] is False
        assert data["readiness"]["paper_operational"] is False

    def test_preflight_ok_with_mocked_http(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        pem = tmp_path / "kalshi_test.pem"
        key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
        pem.write_bytes(
            key.private_bytes(
                encoding=serialization.Encoding.PEM,
                format=serialization.PrivateFormat.TraditionalOpenSSL,
                encryption_algorithm=serialization.NoEncryption(),
            )
        )
        monkeypatch.setenv("KALSHI_API_KEY_ID", "test-key")
        monkeypatch.setenv("KALSHI_PRIVATE_KEY_PATH", str(pem))
        monkeypatch.setenv("POLYMARKET_ENABLED", "0")
        monkeypatch.setenv("RADAR_PREFLIGHT_REQUIRE_OLLAMA", "0")
        get_settings.cache_clear()  # type: ignore[attr-defined]

        async def fake_http_get(url: str, headers=None):  # type: ignore[no-untyped-def]
            u = url.lower()
            if "kalshi" in u and "/markets" in u:
                return {"ok": True, "status_code": 200, "latency_ms": 1, "snippet": "{}"}
            if "gamma-api.polymarket.com" in u and "/markets" in u:
                return {"ok": True, "status_code": 200, "latency_ms": 1, "snippet": "{}"}
            if "/api/tags" in u:
                return {"ok": True, "status_code": 200, "latency_ms": 1, "snippet": "{}"}
            return {"ok": False, "status_code": 0, "latency_ms": 1, "error": f"unexpected url {url}"}

        monkeypatch.setattr(preflight_mod, "_http_get", fake_http_get)

        state = RadarState()
        app = FastAPI()
        app.include_router(build_router(state))
        cli = TestClient(app)
        r = cli.get("/api/radar/preflight")
        assert r.status_code == 200
        data = r.json()
        assert data["ok"] is True
        assert data["checks"]["kalshi_rest"]["ok"] is True
        assert data["readiness"]["paper_operational"] is True

    def test_learning_endpoint(self, client: TestClient) -> None:
        r = client.get("/api/radar/learning")
        assert r.status_code == 200
        data = r.json()
        assert data["ok"] is True
        assert "learning" in data

    def test_prometheus_text(self, client: TestClient) -> None:
        r = client.get("/api/radar/prometheus")
        assert r.status_code == 200
        body = r.text
        # Métricas mínimas presentes
        assert "radar_pnl_cents" in body
        assert "radar_trades_total" in body
        assert "radar_kill_switch" in body

    def test_kill_resume_toggle(self, client: TestClient) -> None:
        r = client.post("/api/radar/kill")
        assert r.status_code == 200
        assert r.json()["kill_switch"] is True
        r = client.post("/api/radar/resume")
        assert r.status_code == 200
        assert r.json()["kill_switch"] is False

    def test_ui_html_renders(self, client: TestClient) -> None:
        r = client.get("/ui/radar")
        assert r.status_code == 200
        assert "Atlas Radar" in r.text or "radar" in r.text.lower()


# ---------------------------------------------------------------------------
class TestStateUpdate:
    def test_update_market_then_query(self, tmp_path, monkeypatch) -> None:
        monkeypatch.setenv("ATLAS_LOG_DIR", str(tmp_path / "logs"))
        get_settings.cache_clear()  # type: ignore[attr-defined]
        state = RadarState()
        state.update_market("ABC", {"p_market": 0.6, "p_model": 0.7,
                                    "edge": 0.10, "side": "YES"})
        app = FastAPI()
        app.include_router(build_router(state))
        cli = TestClient(app)
        rows = cli.get("/api/radar/markets").json()["rows"]
        assert len(rows) == 1
        assert rows[0]["ticker"] == "ABC"
        assert rows[0]["edge"] == pytest.approx(0.10)
