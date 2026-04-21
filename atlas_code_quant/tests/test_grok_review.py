from __future__ import annotations

import json

from atlas_code_quant.grok_decision_pack import build_decision_pack
from atlas_code_quant.grok_expert_adapter import GrokExpertReviewProvider, GrokReviewConfig, TransientGrokError, parse_env_flag
from atlas_code_quant.grok_policy import apply_grok_review_policy
from atlas_code_quant.options.options_scoring import GlobalRegime, Leg, OptionStructure


def test_parse_env_flag_enabled_true(monkeypatch):
    monkeypatch.setenv("QUANT_GROK_REVIEW_ENABLED", "true")
    assert parse_env_flag("QUANT_GROK_REVIEW_ENABLED", False) is True


def test_parse_env_flag_disabled_false(monkeypatch):
    monkeypatch.setenv("QUANT_GROK_REVIEW_ENABLED", "off")
    assert parse_env_flag("QUANT_GROK_REVIEW_ENABLED", True) is False


def test_parse_env_flag_default_missing(monkeypatch):
    monkeypatch.delenv("QUANT_GROK_REVIEW_ENABLED", raising=False)
    assert parse_env_flag("QUANT_GROK_REVIEW_ENABLED", False) is False


def test_grok_review_trade_parses_json_successfully(monkeypatch):
    provider = GrokExpertReviewProvider(GrokReviewConfig(enabled=True))
    monkeypatch.setenv("XAI_API_KEY", "test-key")
    monkeypatch.setattr(
        provider,
        "_request_review_text",
        lambda decision_pack: (
            '{"verdict":"reject","score_adjustment":-4.5,"contracts_multiplier":0.5,'
            '"prefer_strategy":"IRON_CONDOR","rationale":"too much event risk","confidence":0.81}'
        ),
    )
    review = provider.review_trade({"symbol": "SPY"})
    assert review["status"] == "ok"
    assert review["verdict"] == "reject"
    assert review["score_adjustment"] == -4.5
    assert review["contracts_multiplier"] == 0.5
    assert review["prefer_strategy"] == "IRON_CONDOR"


def test_grok_review_trade_falls_back_on_parse_error(monkeypatch):
    provider = GrokExpertReviewProvider(GrokReviewConfig(enabled=True))
    monkeypatch.setenv("XAI_API_KEY", "test-key")
    monkeypatch.setattr(provider, "_request_review_text", lambda decision_pack: "not-json-at-all")
    review = provider.review_trade({"symbol": "SPY"})
    assert review["status"] == "fallback"
    assert review["verdict"] == "neutral"
    assert review["error"] == "parse_error"


def test_grok_review_disabled_returns_neutral_without_api_call(monkeypatch):
    provider = GrokExpertReviewProvider(GrokReviewConfig(enabled=False))
    monkeypatch.setattr(provider, "_request_review_text", lambda decision_pack: (_ for _ in ()).throw(AssertionError("api_should_not_be_called")))
    review = provider.review_trade({"symbol": "SPY"})
    assert review["status"] == "fallback"
    assert review["error"] == "grok_review_disabled"


def test_grok_review_retries_on_transient_error(monkeypatch):
    provider = GrokExpertReviewProvider(GrokReviewConfig(enabled=True, max_retries=2, backoff_base_seconds=0.01))
    monkeypatch.setenv("XAI_API_KEY", "test-key")
    calls = {"count": 0}
    sleeps: list[float] = []

    def fake_once(decision_pack):
        calls["count"] += 1
        if calls["count"] < 3:
            raise TransientGrokError("timeout")
        return '{"verdict":"approve","rationale":"ok"}'

    monkeypatch.setattr(provider, "_request_review_text_once", fake_once)
    monkeypatch.setattr(provider, "_sleep", lambda seconds: sleeps.append(seconds))
    review = provider.review_trade({"symbol": "SPY"})
    assert review["status"] == "ok"
    assert calls["count"] == 3
    assert len(sleeps) == 2


def test_grok_review_falls_back_after_retry_exhaustion(monkeypatch):
    provider = GrokExpertReviewProvider(GrokReviewConfig(enabled=True, max_retries=2, backoff_base_seconds=0.01))
    monkeypatch.setenv("XAI_API_KEY", "test-key")
    calls = {"count": 0}
    monkeypatch.setattr(provider, "_request_review_text_once", lambda decision_pack: (_ for _ in ()).throw(TransientGrokError("timeout")))
    monkeypatch.setattr(provider, "_sleep", lambda seconds: None)
    review = provider.review_trade({"symbol": "SPY"})
    assert review["status"] == "fallback"
    assert review["verdict"] == "neutral"
    assert review["error"].startswith("api_error:transient_retries_exhausted")


def test_grok_review_does_not_retry_parse_error(monkeypatch):
    provider = GrokExpertReviewProvider(GrokReviewConfig(enabled=True))
    monkeypatch.setenv("XAI_API_KEY", "test-key")
    calls = {"count": 0}

    def fake_request(decision_pack):
        calls["count"] += 1
        return "not-json"

    monkeypatch.setattr(provider, "_request_review_text", fake_request)
    review = provider.review_trade({"symbol": "SPY"})
    assert review["status"] == "fallback"
    assert review["error"] == "parse_error"
    assert calls["count"] == 1


def test_build_decision_pack_is_json_safe_smoke():
    structure = OptionStructure(
        symbol="SPY",
        strategy="IRON_CONDOR",
        expiry="2026-06-19",
        legs=[
            Leg(side="sell", type="put", strike=510.0, delta=-0.18),
            Leg(side="buy", type="put", strike=505.0, delta=-0.09),
        ],
        contracts=2,
    )
    opportunity = {
        "symbol": "SPY",
        "strategy": "IRON_CONDOR",
        "asset_family": "ETF",
        "sector": "broad_market",
        "price": 530.12,
        "score": 84.7,
        "direction": "NEUTRAL",
        "expiry": "2026-06-19",
        "structure": structure,
        "entry_reason": "score=84.7 family=ETF",
        "earnings_days": 999,
        "feature_snapshot": {"vol": {"iv_rank": 64.0}},
        "session_id": "sess-123",
        "run_id": "run-456",
    }
    pack = build_decision_pack(
        opportunity,
        GlobalRegime(event_risk=False, regime_id="range_risk_on"),
        {"open_positions": [], "net_delta": 0.0},
        {"recent_closed": 2, "win_rate_pct": 50.0},
    )
    assert pack["symbol"] == "SPY"
    assert pack["strategy"] == "IRON_CONDOR"
    assert isinstance(pack["legs"], list)
    assert pack["session_id"] == "sess-123"
    assert pack["run_id"] == "run-456"
    assert pack["paper_only"] is True
    json.dumps(pack)


def test_provider_handles_mock_realistic_response_and_policy_stable(monkeypatch):
    provider = GrokExpertReviewProvider(GrokReviewConfig(enabled=True))
    monkeypatch.setenv("XAI_API_KEY", "test-key")
    monkeypatch.setattr(
        provider,
        "_request_review_text",
        lambda decision_pack: json.dumps(
            {
                "verdict": "approve",
                "score_adjustment": -1.25,
                "contracts_multiplier": 0.5,
                "prefer_strategy": "IRON_CONDOR",
                "rationale": "range regime but reduce size",
                "confidence": 0.74,
            }
        ),
    )
    review = provider.review_trade({"symbol": "SPY", "strategy": "IRON_CONDOR", "score": 80.0})
    result = apply_grok_review_policy({"symbol": "SPY", "strategy": "IRON_CONDOR", "score": 80.0}, review)
    assert review["status"] == "ok"
    assert result["blocked"] is False
    assert result["opportunity"]["score"] == 78.75
    assert result["opportunity"]["grok_contracts_multiplier"] == 0.5


def test_apply_grok_review_policy_reject_blocks():
    opportunity = {"symbol": "SPY", "strategy": "IRON_CONDOR", "score": 82.0}
    review = {"verdict": "reject"}
    result = apply_grok_review_policy(opportunity, review)
    assert result["blocked"] is True
    assert result["block_reason"] == "grok_reject"


def test_apply_grok_review_policy_score_adjustment_modifies_score():
    opportunity = {"symbol": "SPY", "strategy": "IRON_CONDOR", "score": 82.0}
    review = {"verdict": "neutral", "score_adjustment": -3.25}
    result = apply_grok_review_policy(opportunity, review)
    assert result["blocked"] is False
    assert result["opportunity"]["score"] == 78.75


def test_apply_grok_review_policy_contracts_multiplier_reduces_contracts():
    opportunity = {"symbol": "SPY", "strategy": "IRON_CONDOR", "score": 82.0}
    review = {"verdict": "approve", "contracts_multiplier": 0.4}
    result = apply_grok_review_policy(opportunity, review)
    assert result["blocked"] is False
    assert result["opportunity"]["grok_contracts_multiplier"] == 0.4
