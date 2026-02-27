"""
Tests E2E API de Learning (PARTE 3).
Requieren servidor PUSH en http://127.0.0.1:8791.
Endpoints: consolidator/stats, daily-routine/start, daily-routine/end-report, process-situation.
"""
from __future__ import annotations

import pytest
import requests

BASE_URL = "http://127.0.0.1:8791"


def _server_available():
    try:
        r = requests.get(f"{BASE_URL}/health", timeout=2)
        return r.status_code == 200
    except Exception:
        return False


@pytest.fixture(scope="module")
def auth_headers():
    try:
        r = requests.post(
            f"{BASE_URL}/api/auth/login",
            json={"username": "admin", "password": "changeme123"},
            timeout=3,
        )
        if r.status_code == 200 and "access_token" in r.json():
            return {"Authorization": f"Bearer {r.json()['access_token']}"}
    except Exception:
        pass
    return {}


@pytest.mark.skipif(not _server_available(), reason="PUSH server not available at 8791")
class TestLearningConsolidatorStats:
    """GET /api/learning/consolidator/stats"""

    def test_consolidator_stats_returns_200(self, auth_headers):
        r = requests.get(
            f"{BASE_URL}/api/learning/consolidator/stats",
            headers=auth_headers,
            timeout=10,
        )
        assert r.status_code == 200, r.text
        data = r.json()
        assert "ok" in data
        if data.get("ok"):
            assert "data" in data
            # Con componentes cargados, data puede tener claves del consolidador
            if data["data"]:
                for key in ("total_consolidations", "last_consolidation", "hours_since_last"):
                    assert key in data["data"], f"expected key {key} in data"


@pytest.mark.skipif(not _server_available(), reason="PUSH server not available at 8791")
class TestLearningDailyRoutine:
    """POST /api/learning/daily-routine/start y end-report"""

    def test_daily_routine_start_returns_200(self, auth_headers):
        r = requests.post(
            f"{BASE_URL}/api/learning/daily-routine/start",
            headers=auth_headers,
            timeout=15,
        )
        assert r.status_code == 200, r.text
        data = r.json()
        assert data.get("ok") is True
        assert data.get("status") == "started"

    def test_daily_routine_end_report_returns_200(self, auth_headers):
        r = requests.post(
            f"{BASE_URL}/api/learning/daily-routine/end-report",
            headers=auth_headers,
            timeout=15,
        )
        assert r.status_code == 200, r.text
        data = r.json()
        assert data.get("ok") is True
        assert data.get("status") in ("no_lesson", "evaluated")
        if data.get("status") == "no_lesson":
            assert "message" in data


@pytest.mark.skipif(not _server_available(), reason="PUSH server not available at 8791")
class TestLearningProcessSituation:
    """POST /api/learning/process-situation"""

    def test_process_situation_returns_200_and_structure(self, auth_headers):
        r = requests.post(
            f"{BASE_URL}/api/learning/process-situation",
            headers={**auth_headers, "Content-Type": "application/json"},
            json={
                "description": "Objeto en mesa - agarrar con gripper",
                "type": "grab",
                "goal": "agarrar objeto",
                "risk_level": "normal",
            },
            timeout=45,
        )
        assert r.status_code == 200, r.text
        data = r.json()
        assert data.get("ok") is True
        assert "data" in data
        d = data["data"]
        assert "action_taken" in d
        assert "result" in d
        assert "learned" in d
        assert "asked_for_help" in d
        assert "new_knowledge" in d
        assert "uncertainty_score" in d
        assert "success" in d["result"]

    def test_process_situation_risk_level_validated(self, auth_headers):
        """risk_level inválido se normaliza a 'normal' (validación Pydantic)."""
        r = requests.post(
            f"{BASE_URL}/api/learning/process-situation",
            headers={**auth_headers, "Content-Type": "application/json"},
            json={
                "description": "Tarea de prueba",
                "risk_level": "HIGH",
            },
            timeout=30,
        )
        assert r.status_code == 200
        assert r.json().get("ok") is True


@pytest.mark.skipif(not _server_available(), reason="PUSH server not available at 8791")
class TestLearningKnowledgeBase:
    """GET /api/learning/knowledge-base"""

    def test_knowledge_base_returns_200(self, auth_headers):
        r = requests.get(
            f"{BASE_URL}/api/learning/knowledge-base",
            headers=auth_headers,
            timeout=10,
        )
        assert r.status_code == 200
        data = r.json()
        assert data.get("ok") is True
        assert "data" in data
        assert "concepts" in data["data"]
        assert "skills" in data["data"]
        assert "rules" in data["data"]


@pytest.mark.skipif(not _server_available(), reason="PUSH server not available at 8791")
class TestLearningConsolidate:
    """POST /api/learning/consolidate"""

    def test_consolidate_returns_200(self, auth_headers):
        r = requests.post(
            f"{BASE_URL}/api/learning/consolidate",
            headers=auth_headers,
            timeout=60,
        )
        assert r.status_code == 200
        data = r.json()
        assert data.get("ok") is True
        assert data.get("status") == "consolidated"
        assert "report" in data
        report = data["report"]
        assert "consolidation_id" in report
        assert "timestamp" in report
