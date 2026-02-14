"""
Tests E2E ciclo autónomo. Requieren PUSH en http://127.0.0.1:8791.
Auth: si no existe /api/auth/login, los tests que usan auth_token hacen skip.
"""
import time

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
def auth_token():
    """Token opcional; si no hay login, retorna None y tests pueden skip."""
    try:
        r = requests.post(
            f"{BASE_URL}/api/auth/login",
            json={"username": "admin", "password": "changeme123"},
            timeout=3,
        )
        if r.status_code == 200 and "access_token" in r.json():
            return r.json()["access_token"]
    except Exception:
        pass
    return None


def _headers(auth_token):
    if auth_token:
        return {"Authorization": f"Bearer {auth_token}"}
    return {}


@pytest.mark.skipif(not _server_available(), reason="PUSH server not available at 8791")
class TestAutonomousCycle:
    def test_health_available(self):
        r = requests.get(f"{BASE_URL}/health", timeout=5)
        assert r.status_code == 200
        assert "score" in r.json() or "ok" in r.json()

    def test_health_monitoring_loop(self, auth_token):
        headers = _headers(auth_token)
        r1 = requests.get(f"{BASE_URL}/health", headers=headers, timeout=5)
        assert r1.status_code == 200
        time.sleep(2)
        r2 = requests.get(f"{BASE_URL}/health", headers=headers, timeout=5)
        assert r2.status_code == 200

    def test_memory_stats(self, auth_token):
        headers = _headers(auth_token)
        r = requests.get(f"{BASE_URL}/api/memory/stats", headers=headers, timeout=5)
        if r.status_code == 404:
            pytest.skip("Endpoint /api/memory/stats no disponible (404)")
        assert r.status_code == 200


@pytest.mark.skipif(not _server_available(), reason="PUSH server not available at 8791")
class TestPerformance:
    def test_api_response_time(self, auth_token):
        headers = _headers(auth_token)
        start = time.time()
        r = requests.get(f"{BASE_URL}/health", headers=headers, timeout=5)
        elapsed = time.time() - start
        assert r.status_code == 200
        assert elapsed < 5.0
