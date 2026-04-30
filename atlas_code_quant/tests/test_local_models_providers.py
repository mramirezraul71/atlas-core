from __future__ import annotations

import json
from pathlib import Path

from atlas_code_quant.local_models.classifier_provider import LocalClassifierProvider
from atlas_code_quant.local_models.embedding_provider import LocalEmbeddingProvider
from atlas_code_quant.local_models.vision_provider import LocalVisionProvider


class _FailClient:
    def embeddings(self, **kwargs):
        raise RuntimeError("down")

    def generate(self, **kwargs):
        raise RuntimeError("down")

    def chat(self, **kwargs):
        raise RuntimeError("down")


class _OkClient:
    def embeddings(self, **kwargs):
        return [[0.2, 0.4]]

    def generate(self, **kwargs):
        return json.dumps({"label": "ok", "summary": "ready", "tags": ["ok"], "confidence": 0.9})

    def chat(self, **kwargs):
        return json.dumps({"status": "ok", "observations": ["healthy"]})


def _cfg(enabled: bool = True):
    return {
        "enabled": enabled,
        "roles": {
            "embeddings": "nomic-embed-text:latest",
            "classifier": "llama3.1:8b",
            "vision": "llama3.2-vision:11b",
            "vision_premium": "qwen3-vl:30b",
        },
    }


def test_embedding_provider_fallback_on_client_failure():
    provider = LocalEmbeddingProvider(_FailClient(), _cfg(enabled=True))
    out = provider.embed_text(["abc", "xyz"])
    assert len(out) == 2
    assert all(isinstance(vec, list) and vec for vec in out)


def test_classifier_provider_success_and_fallback():
    ok = LocalClassifierProvider(_OkClient(), _cfg(enabled=True))
    got = ok.classify({"event_type": "entry_execution"})
    assert got["ok"] is True
    assert got["label"] == "ok"
    assert got["used_fallback"] is False
    assert got["provider_role"] == "classifier"
    assert got["requested_model"] == "llama3.1:8b"

    fb = LocalClassifierProvider(_FailClient(), _cfg(enabled=True))
    got_fb = fb.classify({"event_type": "error", "detail": "blocked"})
    assert got_fb["ok"] is True
    assert got_fb["model"] == "heuristic-fallback"
    assert got_fb["used_fallback"] is True
    assert got_fb["provider_role"] == "classifier"
    assert "ollama_unavailable:" in str(got_fb["fallback_reason"])


def test_vision_provider_missing_file_and_success(tmp_path: Path):
    provider = LocalVisionProvider(_FailClient(), _cfg(enabled=True))
    missing = provider.analyze_screenshot(str(tmp_path / "nope.png"))
    assert missing["ok"] is False
    assert missing["error"] == "file_not_found"
    assert missing["used_fallback"] is True
    assert missing["provider_role"] == "vision"
    assert missing["requested_model"] == "llama3.2-vision:11b"

    image = tmp_path / "a.png"
    image.write_bytes(b"fakepng")
    ok_provider = LocalVisionProvider(_OkClient(), _cfg(enabled=True))
    result = ok_provider.analyze_screenshot(str(image))
    assert result["ok"] is True
    assert result["used_fallback"] is False
    assert result["provider_role"] == "vision"
