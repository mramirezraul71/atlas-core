from __future__ import annotations

from atlas_code_quant.operations import journal_pro as jp


class _JournalStub:
    def entries(self, *, limit=24, status=None):
        return {"items": [{"journal_key": "jk1", "symbol": "SPY"}]}


def test_journal_pro_local_model_methods(monkeypatch):
    svc = jp.JournalProService(journal=_JournalStub())
    monkeypatch.setattr(jp, "semantic_journal_search", lambda query, items, top_k=5: {"ok": True, "matches": items[:top_k]})
    monkeypatch.setattr(jp, "classify_journal_event", lambda payload: {"ok": True, "label": "incident"})
    monkeypatch.setattr(jp, "analyze_dashboard_screenshot", lambda p, prompt=None, premium=False: {"ok": True, "path": p})

    sem = svc.semantic_retrieval("spy", limit=1)
    rev = svc.lightweight_event_review({"event_type": "entry_blocked"})
    vis = svc.vision_screenshot_review("x.png")
    assert sem["ok"] is True
    assert rev["label"] == "incident"
    assert vis["path"] == "x.png"
