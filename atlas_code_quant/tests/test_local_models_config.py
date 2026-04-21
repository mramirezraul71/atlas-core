from __future__ import annotations

from atlas_code_quant.local_models.config import get_role_model, load_local_model_config


def test_local_model_config_defaults():
    cfg = load_local_model_config()
    assert cfg["enabled"] is True
    assert cfg["roles"]["embeddings"] == "nomic-embed-text:latest"
    assert get_role_model("classifier", cfg) == "llama3.1:8b"


def test_local_model_config_env_overrides(monkeypatch):
    monkeypatch.setenv("ATLAS_LOCAL_MODELS_ENABLED", "false")
    monkeypatch.setenv("ATLAS_OLLAMA_BASE_URL", "http://127.0.0.1:21434")
    monkeypatch.setenv("ATLAS_LOCAL_MODEL_CLASSIFIER", "custom-classifier:1")
    cfg = load_local_model_config()
    assert cfg["enabled"] is False
    assert cfg["ollama_base_url"] == "http://127.0.0.1:21434"
    assert cfg["roles"]["classifier"] == "custom-classifier:1"
