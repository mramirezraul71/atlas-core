from __future__ import annotations

from typing import Any, TypedDict


def _jsonable(value: Any) -> Any:
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_jsonable(v) for v in value]
    if hasattr(value, "model_dump"):
        return _jsonable(value.model_dump())
    if hasattr(value, "__dict__"):
        return _jsonable(vars(value))
    return str(value)


class GrokDecisionPack(TypedDict):
    symbol: str | None
    strategy: str | None
    asset_family: str | None
    sector: str | None
    price: Any
    score: Any
    direction: str | None
    expiry: Any
    legs: list[Any]
    entry_reason: Any
    earnings_days: Any
    feature_snapshot: dict[str, Any]
    regime: Any
    portfolio_state: Any
    recent_journal_summary: Any
    session_id: str | None
    run_id: str | None
    paper_only: bool


def build_decision_pack(
    opportunity: dict[str, Any],
    global_regime: Any,
    portfolio_state: dict[str, Any] | None,
    recent_journal_summary: dict[str, Any] | None,
) -> GrokDecisionPack:
    """Build a JSON-safe advisory payload for Grok.

    The payload is intentionally read-only and self-contained: it contains the
    minimum context needed for advisory review without performing I/O or
    persisting anything outside the options flow.
    """
    structure = opportunity.get("structure")
    legs = []
    expiry = opportunity.get("expiry")
    if structure is not None:
        expiry = getattr(structure, "expiry", expiry)
        raw_legs = getattr(structure, "legs", []) or []
        legs = [_jsonable(getattr(leg, "model_dump", lambda: leg)()) if hasattr(leg, "model_dump") else _jsonable(leg) for leg in raw_legs]

    regime_payload = getattr(global_regime, "__dict__", global_regime)
    portfolio_payload = portfolio_state or {}
    journal_payload = recent_journal_summary or {}

    return {
        "symbol": opportunity.get("symbol"),
        "strategy": opportunity.get("strategy"),
        "asset_family": opportunity.get("asset_family"),
        "sector": opportunity.get("sector"),
        "price": opportunity.get("price"),
        "score": opportunity.get("score"),
        "direction": opportunity.get("direction"),
        "expiry": expiry,
        "legs": legs,
        "entry_reason": opportunity.get("entry_reason"),
        "earnings_days": opportunity.get("earnings_days"),
        "feature_snapshot": _jsonable(opportunity.get("feature_snapshot") or {}),
        "regime": _jsonable(regime_payload),
        "portfolio_state": _jsonable(portfolio_payload),
        "recent_journal_summary": _jsonable(journal_payload),
        "session_id": str(opportunity.get("session_id") or "") or None,
        "run_id": str(opportunity.get("run_id") or "") or None,
        "paper_only": True,
    }
