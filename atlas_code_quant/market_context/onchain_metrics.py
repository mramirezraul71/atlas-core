"""
Metricas on-chain (Fase 3) — solo cripto; API opcional (Glassnode u otra).

Sin API key: snapshot neutro (gate_pass True) para no bloquear CI.
"""
from __future__ import annotations

import logging
import os
from typing import Any

import numpy as np

logger = logging.getLogger("quant.onchain_metrics")


def _crypto_base(symbol: str) -> str:
    s = symbol.upper().replace(" ", "")
    for sep in ("/", "-"):
        if sep in s:
            return s.split(sep)[0]
    return s


def fetch_sopr(symbol: str, days: int = 30) -> dict[str, Any]:
    """SOPR aproximado; con API real sustituir por HTTP a proveedor."""
    key = (os.getenv("GLASSNODE_API_KEY") or "").strip()
    if not key:
        return {
            "sopr_current": 1.0,
            "sopr_ma7": 1.0,
            "signal": "neutral",
            "source": "stub",
        }
    logger.info("On-chain SOPR: API configurada pero fetch no implementado para %s", symbol)
    return {"sopr_current": 1.0, "sopr_ma7": 1.0, "signal": "neutral", "source": "stub"}


def fetch_mvrv_zscore(symbol: str, days: int = 30) -> dict[str, Any]:
    key = (os.getenv("GLASSNODE_API_KEY") or "").strip()
    if not key:
        return {"mvrv_zscore": 0.0, "signal": "neutral", "source": "stub"}
    return {"mvrv_zscore": 0.0, "signal": "neutral", "source": "stub"}


def fetch_age_consumed(symbol: str, days: int = 7) -> dict[str, Any]:
    key = (os.getenv("GLASSNODE_API_KEY") or "").strip()
    if not key:
        return {"age_consumed": 0.0, "spike": False, "signal": "normal", "source": "stub"}
    return {"age_consumed": 0.0, "spike": False, "signal": "normal", "source": "stub"}


def onchain_gate(symbol: str, sopr: dict[str, Any], mvrv: dict[str, Any], age: dict[str, Any]) -> dict[str, Any]:
    reasons: list[str] = []
    if float(sopr.get("sopr_current") or 1.0) > 1.10:
        reasons.append("SOPR too high")
    if mvrv.get("signal") == "bearish_extreme":
        reasons.append("MVRV extreme")
    if bool(age.get("spike")) and mvrv.get("signal") == "bearish":
        reasons.append("Age spike + MVRV bearish")
    return {"gate_pass": len(reasons) == 0, "reasons": reasons}


def onchain_score_bonus(sopr: dict[str, Any], mvrv: dict[str, Any]) -> float:
    """Hasta +10 pts si hay confluencia alcista extrema (heuristica)."""
    if sopr.get("signal") == "bullish" and mvrv.get("signal") == "bullish_extreme":
        return 10.0
    return 0.0


def bundle_for_symbol(symbol: str) -> dict[str, Any]:
    base = _crypto_base(symbol)
    sopr = fetch_sopr(base)
    mvrv = fetch_mvrv_zscore(base)
    age = fetch_age_consumed(base)
    gate = onchain_gate(symbol, sopr, mvrv, age)
    bonus = onchain_score_bonus(sopr, mvrv) if gate["gate_pass"] else 0.0
    return {"symbol": symbol, "sopr": sopr, "mvrv": mvrv, "age": age, "gate": gate, "bonus": bonus}


def apply_test_fixture(
    sopr_current: float,
    mvrv_signal: str,
    age_spike: bool,
) -> dict[str, Any]:
    """Helper para tests: construye bundle sintetico."""
    sopr = {"sopr_current": sopr_current, "sopr_ma7": np.mean([sopr_current] * 7), "signal": "bullish" if sopr_current < 1.0 else "bearish"}
    mvrv = {"mvrv_zscore": 0.0, "signal": mvrv_signal}
    age = {"age_consumed": 1.0, "spike": age_spike, "signal": "cycle_change_potential" if age_spike else "normal"}
    gate = onchain_gate("BTC", sopr, mvrv, age)
    bonus = onchain_score_bonus(sopr, mvrv) if gate["gate_pass"] else 0.0
    return {"sopr": sopr, "mvrv": mvrv, "age": age, "gate": gate, "bonus": bonus}
