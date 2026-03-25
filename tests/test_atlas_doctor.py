"""
ATLAS DOCTOR — Test Suite
Simula 5 anomalías reales y verifica detección + reparación.

Ejecutar:
    pytest tests/test_atlas_doctor.py -v
"""
from __future__ import annotations

import asyncio
import json
import os
import sys
import time
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

# Asegurar BASE_DIR en path
BASE_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BASE_DIR))

os.environ["ATLAS_DOCTOR_DRY_RUN"] = "true"  # No lanzar procesos reales en tests


# ── Imports tras configurar env ────────────────────────────────────────────────
from atlas_adapter.services.doctor_nervous_system import (
    TIER_CRASH,
    TIER_CRITICAL,
    TIER_DEGRADED,
    TIER_WARNING,
    Anomaly,
    AtlasDoctor,
    HealAction,
    _heal,
    _http,
    _tcp,
    _detect_api_layer,
    _detect_vision_layer,
    _detect_hardware_layer,
    _detect_quant_layer,
    _detect_cognitive_layer,
    _init_db,
    _record_event,
    _recent_events,
)


# ── Fixtures ───────────────────────────────────────────────────────────────────
@pytest.fixture(autouse=True)
def _dry_run():
    """Todos los tests corren en DRY_RUN."""
    os.environ["ATLAS_DOCTOR_DRY_RUN"] = "true"
    yield
    os.environ.pop("ATLAS_DOCTOR_DRY_RUN", None)


@pytest.fixture
def doctor():
    return AtlasDoctor()


# ═══════════════════════════════════════════════════════════════════════════════
# ANOMALÍA 1 — RAULI-VISION espejo.exe caído (:8080 DOWN)
# Simula: Cloudflare devuelve 502, móvil sin acceso
# ═══════════════════════════════════════════════════════════════════════════════
class TestAnomaly1_VisionEspejoDown:
    """TIER CRASH — espejo.exe :8080 caído."""

    def test_detects_espejo_down(self):
        with patch("atlas_adapter.services.doctor_nervous_system._tcp", return_value=False):
            anomalies = _detect_vision_layer()
        espejo = [a for a in anomalies if a.component == "vision_espejo"]
        assert len(espejo) >= 1
        assert espejo[0].tier == TIER_CRASH
        assert ":8080" in espejo[0].description

    def test_heal_espejo_dry_run(self):
        anomaly = Anomaly("vision_espejo", "vision", TIER_CRASH,
                          "RAULI-VISION espejo.exe :8080 caído")
        action = _heal(anomaly)
        assert action.action_type == "restart"
        assert "espejo.exe" in action.action_detail.lower()
        # En DRY_RUN: PID=-1, _wait_port no puede verificar → healed puede ser False
        # pero el intent de restart debe estar presente
        print(f"\n  ✓ ANOMALY1: espejo.exe → action={action.action_type}, detail={action.action_detail}")

    def test_anomaly_logged_to_db(self, tmp_path):
        import atlas_adapter.services.doctor_nervous_system as dns
        original_db = dns.DB_PATH
        dns.DB_PATH = tmp_path / "test_doctor.sqlite"
        dns._init_db()
        anomaly = Anomaly("vision_espejo", "vision", TIER_CRASH, "test espejo down")
        action = HealAction(anomaly=anomaly, action_type="restart",
                            action_detail="dry_run", healed=False, outcome="test")
        dns._record_event(action)
        events = dns._recent_events(10)
        assert len(events) >= 1
        assert events[0]["component"] == "vision_espejo"
        dns.DB_PATH = original_db


# ═══════════════════════════════════════════════════════════════════════════════
# ANOMALÍA 2 — ATLAS API :8791 HTTP degradado (TCP up, /health no responde)
# Simula: proceso cargando, HTTP timeout
# ═══════════════════════════════════════════════════════════════════════════════
class TestAnomaly2_AtlasApiHttpDegraded:
    """TIER CRITICAL — TCP up pero /health no responde."""

    def test_detects_http_failure_when_tcp_open(self):
        def _mock_tcp(host, port, timeout=1.0):
            return port == 8791  # TCP open solo en 8791

        def _mock_http(url, timeout=2.0):
            if "8791" in url and "/health" in url:
                return False, 0, "ConnectTimeout"
            return True, 200, '{"ok":true}'

        with patch("atlas_adapter.services.doctor_nervous_system._tcp", side_effect=_mock_tcp), \
             patch("atlas_adapter.services.doctor_nervous_system._http", side_effect=_mock_http):
            anomalies = _detect_api_layer()

        api = [a for a in anomalies if a.component == "atlas_api"]
        assert len(api) >= 1
        assert api[0].tier == TIER_CRITICAL
        print(f"\n  ✓ ANOMALY2: atlas_api HTTP degradado detectado: {api[0].description}")

    def test_alert_action_on_max_retries(self):
        import atlas_adapter.services.doctor_nervous_system as dns
        # Forzar fail_count alto para activar alert en vez de restart
        dns._fail_counts["atlas_api"] = dns.MAX_RETRIES + 1
        anomaly = Anomaly("atlas_api", "api", TIER_CRITICAL, "HTTP timeout")
        action = _heal(anomaly)
        assert action.action_type == "alert"
        assert "MAX_RETRIES" in action.action_detail or "escalado" in action.action_detail.lower()
        dns._fail_counts["atlas_api"] = 0  # reset
        print(f"\n  ✓ ANOMALY2: escalado a alert tras MAX_RETRIES")


# ═══════════════════════════════════════════════════════════════════════════════
# ANOMALÍA 3 — NEXUS robot :8000 caído (hardware)
# Simula: robot RAULI apagado o desconectado
# ═══════════════════════════════════════════════════════════════════════════════
class TestAnomaly3_NexusDown:
    """TIER CRITICAL — NEXUS :8000 hardware down."""

    def test_detects_nexus_down(self):
        with patch("atlas_adapter.services.doctor_nervous_system._tcp", return_value=False):
            anomalies = _detect_hardware_layer()
        nexus = [a for a in anomalies if a.component == "nexus_api"]
        assert len(nexus) >= 1
        assert nexus[0].tier == TIER_CRITICAL
        assert nexus[0].layer == "hardware"

    def test_nexus_heal_is_alert_not_restart(self):
        """NEXUS no puede ser reiniciado automáticamente — requiere operador."""
        anomaly = Anomaly("nexus_api", "hardware", TIER_CRITICAL,
                          "NEXUS :8000 TCP down")
        action = _heal(anomaly)
        assert action.action_type == "alert"
        assert "manual" in action.action_detail.lower() or "nexus" in action.action_detail.lower()
        print(f"\n  ✓ ANOMALY3: NEXUS down → {action.action_type}: {action.action_detail}")


# ═══════════════════════════════════════════════════════════════════════════════
# ANOMALÍA 4 — Quant LiveLoop cycle_time > 6s
# Simula: scanner bloqueado o latencia alta en Tradier API
# ═══════════════════════════════════════════════════════════════════════════════
class TestAnomaly4_QuantLoopSlow:
    """TIER WARNING — LiveLoop cycle_time > 6s."""

    def test_detects_slow_loop(self):
        def _mock_tcp(host, port, timeout=1.0):
            return True  # Quant API up

        def _mock_http(url, timeout=2.0):
            if "8795" in url and "loop/status" in url:
                body = json.dumps({
                    "data": {"last_cycle_time_sec": 8.5, "running": True}
                })
                return True, 200, body
            return True, 200, '{"ok":true}'

        with patch("atlas_adapter.services.doctor_nervous_system._tcp", side_effect=_mock_tcp), \
             patch("atlas_adapter.services.doctor_nervous_system._http", side_effect=_mock_http):
            anomalies = _detect_api_layer()

        loop_anomaly = [a for a in anomalies if a.component == "quant_loop"]
        assert len(loop_anomaly) >= 1
        assert loop_anomaly[0].tier == TIER_WARNING
        assert "8.5" in loop_anomaly[0].description or "cycle_time" in loop_anomaly[0].description
        print(f"\n  ✓ ANOMALY4: Quant LiveLoop lento: {loop_anomaly[0].description}")

    def test_slow_loop_triggers_alert(self):
        anomaly = Anomaly("quant_loop", "quant", TIER_WARNING,
                          "LiveLoop cycle_time 8.5s > 6s")
        action = _heal(anomaly)
        assert action.action_type == "alert"
        print(f"\n  ✓ ANOMALY4: LiveLoop lento → alert enviado")


# ═══════════════════════════════════════════════════════════════════════════════
# ANOMALÍA 5 — Triada IA: 2 proveedores caídos
# Simula: DeepSeek + Grok down, solo Claude disponible
# ═══════════════════════════════════════════════════════════════════════════════
class TestAnomaly5_TriadaDown:
    """TIER CRITICAL — 2 de 3 proveedores IA caídos."""

    def test_detects_two_providers_down(self):
        battery_response = json.dumps({
            "ok": True,
            "data": {
                "battery": {
                    "claude": {"ok": True, "status": "ok"},
                    "deepseek": {"ok": False, "status": "error", "error": "timeout"},
                    "grok": {"ok": False, "status": "error", "error": "rate_limit"},
                    "gemini": {"ok": True, "status": "ok"},
                }
            }
        })

        def _mock_tcp(host, port, timeout=1.0):
            return True

        def _mock_http(url, timeout=2.0):
            if "battery/status" in url:
                return True, 200, battery_response
            return True, 200, '{"ok":true}'

        with patch("atlas_adapter.services.doctor_nervous_system._tcp", side_effect=_mock_tcp), \
             patch("atlas_adapter.services.doctor_nervous_system._http", side_effect=_mock_http):
            anomalies = _detect_cognitive_layer()

        triada = [a for a in anomalies if a.component == "triada_ia"]
        assert len(triada) >= 1
        assert triada[0].tier == TIER_CRITICAL
        assert len(triada[0].context.get("down", [])) >= 2
        print(f"\n  ✓ ANOMALY5: Triada down={triada[0].context['down']} tier={triada[0].tier}")

    def test_single_provider_down_is_warning(self):
        battery_response = json.dumps({
            "ok": True,
            "data": {
                "battery": {
                    "claude": {"ok": True, "status": "ok"},
                    "deepseek": {"ok": False, "status": "error"},
                    "grok": {"ok": True, "status": "ok"},
                }
            }
        })

        def _mock_http(url, timeout=2.0):
            if "battery/status" in url:
                return True, 200, battery_response
            return True, 200, '{"ok":true}'

        with patch("atlas_adapter.services.doctor_nervous_system._tcp", return_value=True), \
             patch("atlas_adapter.services.doctor_nervous_system._http", side_effect=_mock_http):
            anomalies = _detect_cognitive_layer()

        triada = [a for a in anomalies if a.component == "triada_ia"]
        assert len(triada) >= 1
        assert triada[0].tier == TIER_WARNING
        print(f"\n  ✓ ANOMALY5b: 1 proveedor down → TIER WARNING (no crítico)")


# ═══════════════════════════════════════════════════════════════════════════════
# Tests de integración — AtlasDoctor.run_once()
# ═══════════════════════════════════════════════════════════════════════════════
class TestAtlasDoctorIntegration:

    def test_run_once_returns_valid_report(self):
        """run_once() completa sin excepción y devuelve estructura válida."""
        doctor = AtlasDoctor()

        async def _run():
            return await doctor.run_once()

        result = asyncio.run(_run())
        assert "ok" in result
        assert "cycle" in result
        assert "anomalies_count" in result
        assert "healed_count" in result
        assert isinstance(result["anomalies"], list)
        assert isinstance(result["actions"], list)
        print(f"\n  ✓ INTEGRATION: run_once() OK — {result['anomalies_count']} anomalías, "
              f"{result['healed_count']} sanadas")

    def test_status_report_structure(self):
        doctor = AtlasDoctor()
        # Inyectar anomalía sintética
        doctor.last_anomalies = [
            Anomaly("test_comp", "api", TIER_WARNING, "test anomaly")
        ]
        doctor.last_actions = [
            HealAction(anomaly=doctor.last_anomalies[0],
                       action_type="alert", action_detail="test",
                       healed=False)
        ]
        report = doctor.status_report()
        assert report["anomalies_count"] == 1
        assert report["ok"] is False
        assert "WARNING" in report["tiers"]

    def test_emergency_stop_dry_run(self):
        """emergency_stop() no explota en DRY_RUN."""
        doctor = AtlasDoctor()
        # En DRY_RUN sin Quant API corriendo, debe manejar el error
        result = doctor.emergency_stop()
        assert "doctor" in result
        assert result["doctor"] == "emergency_stop_called"
        print(f"\n  ✓ INTEGRATION: emergency_stop DRY_RUN → {result}")

    def test_db_init_and_record(self, tmp_path):
        import atlas_adapter.services.doctor_nervous_system as dns
        original = dns.DB_PATH
        dns.DB_PATH = tmp_path / "doctor_test.sqlite"
        dns._init_db()
        anomaly = Anomaly("test", "api", TIER_WARNING, "integration test")
        action = HealAction(anomaly=anomaly, action_type="alert",
                            action_detail="test detail", healed=False)
        dns._record_event(action)
        events = dns._recent_events(5)
        assert len(events) == 1
        assert events[0]["component"] == "test"
        assert events[0]["tier_label"] == "WARNING"
        dns.DB_PATH = original


# ═══════════════════════════════════════════════════════════════════════════════
# Test del daemon
# ═══════════════════════════════════════════════════════════════════════════════
class TestAtlasDoctorDaemon:

    def test_daemon_status(self):
        from modules.humanoid.healing.atlas_doctor_daemon import (
            get_daemon_status, MONITORED_PORTS, SPA_MODULE_CHECKS
        )
        status = get_daemon_status()
        assert status["monitored_ports"] == len(MONITORED_PORTS)
        assert status["spa_endpoints"] == len(SPA_MODULE_CHECKS)
        assert status["interval_sec"] > 0

    def test_monitored_ports_includes_all_layers(self):
        from modules.humanoid.healing.atlas_doctor_daemon import MONITORED_PORTS
        layers = {v[1] for v in MONITORED_PORTS.values()}
        assert "api" in layers
        assert "hardware" in layers
        assert "vision" in layers
        assert "quant" in layers
        assert 8791 in MONITORED_PORTS
        assert 8795 in MONITORED_PORTS
        assert 8080 in MONITORED_PORTS
        assert 3000 in MONITORED_PORTS
        print(f"\n  ✓ DAEMON: {len(MONITORED_PORTS)} puertos, capas={layers}")
