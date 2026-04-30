from __future__ import annotations

from atlas_code_quant.local_models.router import (
    get_classifier_provider,
    get_coder_profile,
    get_embedding_provider,
    get_ollama_client,
    get_vision_provider,
    reset_router_cache,
)


def test_router_singletons_and_profile():
    reset_router_cache()
    c1 = get_ollama_client()
    c2 = get_ollama_client()
    assert c1 is c2
    assert get_embedding_provider() is get_embedding_provider()
    assert get_classifier_provider() is get_classifier_provider()
    assert get_vision_provider() is get_vision_provider()
    profile = get_coder_profile()
    assert "coder_primary" in profile
    assert "coder_fallback" in profile
