from __future__ import annotations

import os

import pytest


def _enabled() -> bool:
    return os.getenv("RUN_PYTHON_MASTERY", "").strip().lower() in ("1", "true", "yes", "on")


pytestmark = pytest.mark.skipif(not _enabled(), reason="Python Mastery tests deshabilitados (set RUN_PYTHON_MASTERY=1).")


class _Resp:
    def __init__(self, status_code: int, payload: dict, headers: dict | None = None):
        self.status_code = status_code
        self._payload = payload
        self.headers = headers or {}

    def json(self):
        return self._payload


def test_get_json_with_retries_recovers_after_server_error(monkeypatch):
    from training.python_mastery.http_client import get_json_with_retries

    calls = {"n": 0}

    class _Sess:
        def get(self, url, timeout):
            calls["n"] += 1
            if calls["n"] < 2:
                return _Resp(500, {"ok": False})
            return _Resp(200, {"ok": True, "url": url})

    # evitar sleeps reales
    monkeypatch.setattr("training.python_mastery.http_client.time.sleep", lambda s: None)

    out = get_json_with_retries("http://example.test", retries=2, session=_Sess(), backoff_s=0.0, jitter_s=0.0)
    assert out.status_code == 200
    assert out.json["ok"] is True
    assert calls["n"] == 2

