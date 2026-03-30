from __future__ import annotations

from learning.adaptive_policy import AdaptiveLearningService


def test_adaptive_policy_fast_status_uses_cached_snapshot(tmp_path, monkeypatch):
    service = AdaptiveLearningService(snapshot_path=tmp_path / "adaptive_policy_snapshot.json")

    cached = service._empty_snapshot()
    cached["generated_at"] = "2026-03-30T00:00:00+00:00"
    cached["scopes"]["all"]["sample_count"] = 7
    cached["scopes"]["all"]["risk_multiplier"] = 1.25
    service._snapshot = cached

    def _boom(*args, **kwargs):
        raise AssertionError("fast status should not refresh from the journal")

    monkeypatch.setattr(service, "refresh", _boom)

    payload = service.status(fast=True)

    assert payload["status_mode"] == "fast_cached"
    assert payload["sample_count"] == 7
    assert payload["risk_multiplier"] == 1.25
