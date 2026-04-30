from __future__ import annotations

from atlas_code_quant.local_models import integrations


class _EmbeddingProviderStub:
    def embed_text(self, texts):
        # deterministic tiny vectors: query close to second entry
        return [
            [1.0, 0.0],  # query
            [0.0, 1.0],  # entry 1
            [0.9, 0.1],  # entry 2
        ][: len(texts)]


def test_semantic_journal_search(monkeypatch):
    monkeypatch.setattr(integrations, "get_embedding_provider", lambda: _EmbeddingProviderStub())
    entries = [
        {"journal_key": "a", "symbol": "AAA", "strategy_type": "x"},
        {"journal_key": "b", "symbol": "BBB", "strategy_type": "y"},
    ]
    out = integrations.semantic_journal_search("find b", entries, top_k=1)
    assert out["ok"] is True
    assert len(out["matches"]) == 1
    assert out["matches"][0]["entry"]["journal_key"] == "b"


def test_classify_and_vision_wrappers(monkeypatch):
    class _C:
        def classify(self, payload):
            return {"ok": True, "label": "test"}

    class _V:
        def analyze_screenshot(self, path, prompt=None, premium=False):
            return {"ok": True, "path": path, "premium": premium}

    monkeypatch.setattr(integrations, "get_classifier_provider", lambda: _C())
    monkeypatch.setattr(integrations, "get_vision_provider", lambda: _V())
    assert integrations.classify_journal_event({"event_type": "x"})["label"] == "test"
    assert integrations.analyze_dashboard_screenshot("a.png", premium=True)["premium"] is True
