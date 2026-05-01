"""Feature engineering para XGBoost Signal (pre-trade e in-trade)."""
from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from typing import Any

import numpy as np
import pandas as pd


def _g(obj: Any, *path: str, default: Any = None) -> Any:
    cur: Any = obj
    for p in path:
        if not isinstance(cur, dict):
            return default
        cur = cur.get(p, default)
        if cur is None:
            return default
    return cur


def _clip(x: float, lo: float, hi: float) -> float:
    return float(max(lo, min(hi, x)))


def _norm_pct(x: Any) -> float:
    try:
        v = float(x)
    except (TypeError, ValueError):
        return 0.0
    return _clip(v / 100.0, 0.0, 1.0)


class FeatureBuilder:
    REGIME_CLASSES = ["trending", "sideways", "high_volatility"]
    SESSION_CLASSES = [
        "intraday_opening",
        "intraday_core",
        "intraday_closing",
        "pre_market",
        "after_hours",
    ]
    MACRO_ORDINAL = {"caution": 0, "neutral": 1, "supportive": 2}

    def build_pretrade_features(self, candidate: dict) -> pd.Series:
        """Construye vector de features para un candidato pre-trade."""
        c = candidate or {}
        sel = _g(c, "selection_score_pct")
        if sel is None:
            sel = _g(c, "candidate", "selection_score_pct")
        if sel is None:
            sel = _g(c, "selected", "score")
        selection_score_pct = _norm_pct(sel) if sel is not None else 0.0

        pm = _g(c, "predicted_move_pct")
        if pm is None:
            pm = _g(c, "market_context", "predicted_move_pct")
        if pm is None:
            pm = _g(c, "candidate", "predicted_move_pct")
        predicted_move_pct = _clip(float(pm or 0.0), -0.20, 0.20)

        lwr = _g(c, "local_win_rate_pct")
        if lwr is None:
            lwr = _g(c, "candidate", "local_win_rate_pct")
        if lwr is None:
            lwr = _g(c, "asset_profile", "local_win_rate_pct")
        local_win_rate_pct = _norm_pct(lwr)

        iv = _g(c, "iv_rank_pct")
        if iv is None:
            iv = _g(c, "volatility_state", "iv_rank_pct")
        if iv is None:
            iv = _g(c, "candidate", "iv_rank_pct")
        iv_rank_pct = _norm_pct(iv)

        sp = _g(c, "spread_pct")
        if sp is None:
            sp = _g(c, "entry_validation", "spread_pct")
        try:
            spread_pct = math.log1p(max(0.0, float(sp or 0.0)))
        except (TypeError, ValueError):
            spread_pct = 0.0

        conf = _g(c, "confidence_pct")
        if conf is None:
            conf = _g(c, "market_context", "confidence_pct")
        if conf is None:
            conf = _g(c, "confirmation", "confidence_pct")
        confidence_pct = _norm_pct(conf)

        rs = _g(c, "relative_strength_pct")
        if rs is None:
            rs = _g(c, "global_market", "relative_strength_pct")
        if rs is None:
            rs = _g(c, "candidate", "relative_strength_pct")
        relative_strength_pct = _norm_pct(rs)

        adv = _g(c, "adverse_drift_pct")
        if adv is None:
            adv = _g(c, "entry_validation", "adverse_drift_pct")
        adverse_drift_pct = _clip(float(adv or 0.0), -0.10, 0.10)

        regime_raw = str(
            _g(c, "regime")
            or _g(c, "market_regime")
            or _g(c, "candidate", "regime")
            or "sideways"
        ).lower()
        if regime_raw not in self.REGIME_CLASSES:
            regime_raw = "sideways"

        sess = str(
            _g(c, "session_phase")
            or _g(c, "session")
            or _g(c, "scanner", "session_phase")
            or "intraday_core"
        ).lower()
        if sess not in self.SESSION_CLASSES:
            sess = "intraday_core"

        macro = str(_g(c, "macro_bias") or _g(c, "market_context", "macro_bias") or "neutral").lower()
        macro_bias = float(self.MACRO_ORDINAL.get(macro, 1))

        ts = _g(c, "timestamp") or _g(c, "as_of") or _g(c, "generated_at")
        if isinstance(ts, str):
            try:
                dt = datetime.fromisoformat(ts.replace("Z", "+00:00"))
            except ValueError:
                dt = datetime.now(timezone.utc)
        elif isinstance(ts, datetime):
            dt = ts if ts.tzinfo else ts.replace(tzinfo=timezone.utc)
        else:
            dt = datetime.now(timezone.utc)
        h = dt.hour + dt.minute / 60.0
        hour_sin = math.sin(2 * math.pi * h / 24.0)
        hour_cos = math.cos(2 * math.pi * h / 24.0)
        dow = dt.weekday()
        dow = dow if dow <= 4 else 4

        data: dict[str, float] = {
            "selection_score_pct": selection_score_pct,
            "predicted_move_pct": predicted_move_pct,
            "local_win_rate_pct": local_win_rate_pct,
            "iv_rank_pct": iv_rank_pct,
            "spread_pct": spread_pct,
            "confidence_pct": confidence_pct,
            "relative_strength_pct": relative_strength_pct,
            "adverse_drift_pct": adverse_drift_pct,
            "macro_bias": macro_bias,
            "hour_sin": hour_sin,
            "hour_cos": hour_cos,
        }
        for r in self.REGIME_CLASSES:
            data[f"regime_{r}"] = 1.0 if regime_raw == r else 0.0
        for s in self.SESSION_CLASSES:
            data[f"session_{s}"] = 1.0 if sess == s else 0.0
        for i in range(5):
            data[f"dow_{i}"] = 1.0 if dow == i else 0.0

        return pd.Series(data)

    PHASE1_STABLE_FEATURES = (
        "selection_score_pct",
        "local_win_rate_pct",
        "predicted_move_pct",
        "iv_rank_pct",
        "confidence_pct",
        "regime_trending",
        "regime_sideways",
        "regime_high_volatility",
        "holding_hours",
    )

    def build_intrade_features(self, position: dict, market_snapshot: dict) -> pd.Series:
        """Features para posición abierta (exit_advisor)."""
        pos = position or {}
        snap = market_snapshot or {}
        entry_ts = pos.get("entry_time") or pos.get("entry_timestamp")
        if isinstance(entry_ts, str):
            try:
                et = datetime.fromisoformat(entry_ts.replace("Z", "+00:00"))
            except ValueError:
                et = datetime.now(timezone.utc)
        elif isinstance(entry_ts, datetime):
            et = entry_ts
        else:
            et = datetime.now(timezone.utc)
        now = datetime.now(timezone.utc)
        if et.tzinfo is None:
            et = et.replace(tzinfo=timezone.utc)
        holding_hours = max(0.0, (now - et).total_seconds() / 3600.0)

        unreal = float(pos.get("unrealized_pnl") or 0.0)
        notional = float(pos.get("entry_notional") or pos.get("notional") or 1.0)
        unrealized_pnl_pct = unreal / notional if notional else 0.0

        var_95 = _g(snap, "var_95_pct_of_book")
        if var_95 is None:
            var_95 = _g(snap, "position_management", "var_95_pct_of_book")
        try:
            var_95_pct_of_book = float(var_95 or 0.0)
        except (TypeError, ValueError):
            var_95_pct_of_book = 0.0

        adv = _g(snap, "adverse_drift_pct")
        if adv is None:
            adv = _g(snap, "entry_validation", "adverse_drift_pct")
        try:
            adverse_drift_pct = _clip(float(adv or 0.0), -0.10, 0.10)
        except (TypeError, ValueError):
            adverse_drift_pct = 0.0

        entry_cand = pos.get("entry_candidate") or pos.get("candidate_snapshot") or {}
        base = self.build_pretrade_features(entry_cand if isinstance(entry_cand, dict) else {})
        base["holding_hours"] = holding_hours
        base["unrealized_pnl_pct"] = unrealized_pnl_pct
        base["var_95_pct_of_book"] = var_95_pct_of_book
        base["adverse_drift_pct"] = adverse_drift_pct
        return base

    def build_training_dataset(self, journal_df: pd.DataFrame) -> pd.DataFrame:
        """Dataset de entrenamiento desde filas del journal (cerradas)."""
        if journal_df is None or journal_df.empty:
            return pd.DataFrame()

        rows: list[dict[str, Any]] = []
        for _, row in journal_df.iterrows():
            cand: dict[str, Any] = {
                "selection_score_pct": row.get("win_rate_at_entry") or row.get("current_win_rate_pct"),
                "local_win_rate_pct": row.get("current_win_rate_pct") or row.get("win_rate_at_entry"),
                "predicted_move_pct": 0.0,
                "iv_rank_pct": row.get("iv_rank"),
                "confidence_pct": row.get("win_rate_at_entry"),
                "timestamp": row.get("entry_time"),
            }
            try:
                att = row.get("attribution_json") or "{}"
                if isinstance(att, str):
                    attd = json.loads(att)
                else:
                    attd = att or {}
                if isinstance(attd, dict):
                    cand.update(attd)
            except json.JSONDecodeError:
                pass

            s = self.build_pretrade_features(cand)
            exit_t = row.get("exit_time")
            entry_t = row.get("entry_time")
            try:
                if pd.notna(entry_t) and pd.notna(exit_t):
                    he = pd.Timestamp(entry_t).to_pydatetime()
                    hx = pd.Timestamp(exit_t).to_pydatetime()
                    s["holding_hours"] = max(0.0, (hx - he).total_seconds() / 3600.0)
                else:
                    s["holding_hours"] = 0.0
            except Exception:
                s["holding_hours"] = 0.0

            rp = float(row.get("realized_pnl") or 0.0)
            s["target_win"] = 1.0 if rp > 0 else 0.0
            en = float(row.get("entry_notional") or 0.0) or 1.0
            s["max_favorable_excursion_pct"] = min(2.0, abs(rp) / en)
            post = str(row.get("post_mortem_json") or "")
            early_stop = "stop" in post.lower() and s["holding_hours"] < 96
            s["target_exit_early"] = 1.0 if early_stop else 0.0
            out = s.to_dict()
            out["entry_time"] = row.get("entry_time")
            out["journal_id"] = row.get("id")
            rows.append(out)

        return pd.DataFrame(rows)


class OptionsFeatureBuilder(FeatureBuilder):
    """PLACEHOLDER — opciones reales."""

    OPTIONS_FEATURES = ["delta", "gamma", "theta", "vega", "dte", "spread_width_pct"]

    def build_options_features(self, position: dict) -> pd.Series:
        raise NotImplementedError("OptionsFeatureBuilder activado cuando ATLAS opere opciones reales.")
