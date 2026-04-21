from __future__ import annotations

import pytest

from atlas_code_quant.local_models.ollama_client import OllamaClient, OllamaUnavailable


class _Response:
    def __init__(self, payload, ok: bool = True) -> None:
        self._payload = payload
        self._ok = ok

    def raise_for_status(self) -> None:
        if not self._ok:
            raise RuntimeError("http_error")

    def json(self):
        return self._payload


def test_ollama_client_embeddings_success(monkeypatch):
    def _fake_post(*args, **kwargs):
        return _Response({"embedding": [0.1, 0.2, 0.3]})

    monkeypatch.setattr("atlas_code_quant.local_models.ollama_client.requests.post", _fake_post)
    client = OllamaClient(base_url="http://localhost:11434", timeout_seconds=2.0, enabled=True)
    out = client.embeddings(model="nomic-embed-text:latest", input_texts=["abc"], role="embeddings")
    assert out == [[0.1, 0.2, 0.3]]


def test_ollama_client_unavailable_raises(monkeypatch):
    def _fake_post(*args, **kwargs):
        raise RuntimeError("connection refused")

    monkeypatch.setattr("atlas_code_quant.local_models.ollama_client.requests.post", _fake_post)
    client = OllamaClient(base_url="http://localhost:11434", timeout_seconds=2.0, enabled=True)
    with pytest.raises(OllamaUnavailable):
        client.generate(model="llama3.1:8b", prompt="hi", role="classifier")
