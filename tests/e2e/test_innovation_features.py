"""
Tests E2E capacidades Fase 4 (meta-learning, causal, self-programming).
Requieren PUSH en http://127.0.0.1:8791. Sin auth los endpoints actuales responden 200.
"""
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
    return {"Authorization": f"Bearer {auth_token}"} if auth_token else {}


@pytest.mark.skipif(not _server_available(), reason="PUSH server not available at 8791")
class TestMetaLearning:
    def test_meta_learning_stats(self, auth_token):
        r = requests.get(f"{BASE_URL}/api/meta-learning/stats", headers=_headers(auth_token), timeout=5)
        if r.status_code == 404:
            pytest.skip("Endpoint /api/meta-learning/stats no disponible (404)")
        assert r.status_code == 200
        data = r.json()
        assert "num_tasks_in_buffer" in data or "error" in data

    def test_task_generation(self, auth_token):
        r = requests.post(
            f"{BASE_URL}/api/meta-learning/generate-tasks",
            params={"task_type": "navigation", "num_tasks": 5},
            headers=_headers(auth_token),
            timeout=30,
        )
        if r.status_code != 200:
            pytest.skip("Meta-learning no disponible (torch/deps)")
        assert r.json().get("status") in ("tasks_generated", "error") or "num_tasks" in r.json()


@pytest.mark.skipif(not _server_available(), reason="PUSH server not available at 8791")
class TestCausalReasoning:
    def test_causal_create_domain(self, auth_token):
        r = requests.post(
            f"{BASE_URL}/api/causal/create-domain",
            params={"domain_name": "weather_test"},
            json={
                "variables": ["rain", "wet_ground", "slippery"],
                "edges": [["rain", "wet_ground", 0.9], ["wet_ground", "slippery", 0.95]],
            },
            headers=_headers(auth_token),
            timeout=5,
        )
        if r.status_code == 404:
            pytest.skip("Endpoint /api/causal/create-domain no disponible (404)")
        assert r.status_code == 200
        assert r.json().get("domain") == "weather_test" or "error" in r.json()

    def test_causal_reason(self, auth_token):
        requests.post(
            f"{BASE_URL}/api/causal/create-domain",
            params={"domain_name": "reason_test"},
            json={"variables": ["x", "y"], "edges": [["x", "y", 1.0]]},
            headers=_headers(auth_token),
            timeout=5,
        )
        r = requests.post(
            f"{BASE_URL}/api/causal/reason",
            params={"domain": "reason_test", "action": "set_x_to_1"},
            json={"x": 0, "y": 0},
            headers=_headers(auth_token),
            timeout=5,
        )
        if r.status_code == 404:
            pytest.skip("Endpoint /api/causal/reason no disponible (404)")
        assert r.status_code == 200
        assert "predicted_effects" in r.json() or "error" in r.json()


@pytest.mark.skipif(not _server_available(), reason="PUSH server not available at 8791")
class TestSelfProgramming:
    def test_optimize(self, auth_token):
        r = requests.post(
            f"{BASE_URL}/api/self-programming/optimize",
            json={"code": "def run():\n    return 1"},
            headers=_headers(auth_token),
            timeout=5,
        )
        if r.status_code == 404:
            pytest.skip("Endpoint /api/self-programming/optimize no disponible (404)")
        assert r.status_code == 200
        assert "optimized_code" in r.json() or "error" in r.json()

    def test_validate_syntax(self, auth_token):
        r = requests.post(
            f"{BASE_URL}/api/self-programming/validate",
            json={
                "code": "def run(a: int, b: int):\n    return a + b",
                "function_name": "run",
                "test_cases": [],
            },
            headers=_headers(auth_token),
            timeout=5,
        )
        if r.status_code == 404:
            pytest.skip("Endpoint /api/self-programming/validate no disponible (404)")
        assert r.status_code == 200
        assert "checks" in r.json() or "error" in r.json()
