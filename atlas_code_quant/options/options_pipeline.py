"""Pipeline multi-activo para Options Trading Engine en paper."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Iterable, Optional

from atlas_code_quant.grok_decision_pack import build_decision_pack
from atlas_code_quant.grok_expert_adapter import GrokExpertReviewProvider
from atlas_code_quant.grok_policy import apply_grok_review_policy
from atlas_code_quant.options.options_scoring import (
    GexData,
    GlobalRegime,
    Leg,
    OiData,
    OptionStructure,
    PriceRegime,
    VolData,
    calculate_gamma_score,
    calculate_oi_score,
    calculate_price_score,
    calculate_vol_score,
    combine_scores,
    get_min_total_score_for_asset_family,
    matches_context_strategy_map,
)


STRATEGY_SET = {
    "IRON_CONDOR",
    "CREDIT_SPREAD",
    "BUTTERFLY",
    "DEBIT_SPREAD",
    "LONG_CALL",
    "LONG_PUT",
    "CALENDAR",
    "PUT_CREDIT_SPREAD",
    "CALL_CREDIT_SPREAD",
    "CALL_DEBIT_SPREAD",
    "PUT_DEBIT_SPREAD",
}

SHORT_PREMIUM_SET = {"IRON_CONDOR", "CREDIT_SPREAD", "PUT_CREDIT_SPREAD", "CALL_CREDIT_SPREAD", "BUTTERFLY"}
LONG_PREMIUM_SET = {"LONG_CALL", "LONG_PUT", "DEBIT_SPREAD", "CALL_DEBIT_SPREAD", "PUT_DEBIT_SPREAD"}

ASSET_FAMILY_LIMITS: dict[str, dict[str, Any]] = {
    "INDEX": {
        "spread_max_pct": 0.020,
        "min_option_volume": 8000,
        "min_oi": 15000,
        "dte_short_premium": (30, 45),
        "dte_long_premium": (21, 45),
        "dte_calendar_front": (21, 35),
        "dte_calendar_back": (45, 75),
        "portfolio_weight_cap": 0.40,
        "event_risk_days_block_short": 2,
        "score_min": 75,
    },
    "ETF": {
        "spread_max_pct": 0.025,
        "min_option_volume": 4000,
        "min_oi": 9000,
        "dte_short_premium": (28, 45),
        "dte_long_premium": (21, 45),
        "dte_calendar_front": (21, 35),
        "dte_calendar_back": (45, 75),
        "portfolio_weight_cap": 0.30,
        "event_risk_days_block_short": 3,
        "score_min": 76,
    },
    "LARGE_CAP_EQUITY": {
        "spread_max_pct": 0.030,
        "min_option_volume": 2000,
        "min_oi": 6000,
        "dte_short_premium": (30, 45),
        "dte_long_premium": (21, 45),
        "dte_calendar_front": (21, 35),
        "dte_calendar_back": (45, 75),
        "portfolio_weight_cap": 0.20,
        "event_risk_days_block_short": 7,
        "score_min": 80,
    },
    "HIGH_BETA_EQUITY": {
        "spread_max_pct": 0.035,
        "min_option_volume": 1500,
        "min_oi": 4000,
        "dte_short_premium": (30, 40),
        "dte_long_premium": (21, 35),
        "dte_calendar_front": (21, 30),
        "dte_calendar_back": (45, 60),
        "portfolio_weight_cap": 0.10,
        "event_risk_days_block_short": 10,
        "score_min": 84,
    },
    "SECTOR_ETF": {
        "spread_max_pct": 0.028,
        "min_option_volume": 2500,
        "min_oi": 7000,
        "dte_short_premium": (28, 45),
        "dte_long_premium": (21, 45),
        "dte_calendar_front": (21, 35),
        "dte_calendar_back": (45, 75),
        "portfolio_weight_cap": 0.20,
        "event_risk_days_block_short": 3,
        "score_min": 78,
    },
    "EVENT_DRIVEN_EQUITY": {
        "spread_max_pct": 0.040,
        "min_option_volume": 1000,
        "min_oi": 3000,
        "dte_short_premium": (35, 45),
        "dte_long_premium": (21, 35),
        "dte_calendar_front": (21, 30),
        "dte_calendar_back": (45, 60),
        "portfolio_weight_cap": 0.08,
        "event_risk_days_block_short": 14,
        "score_min": 88,
    },
}

PIPELINE_LIMITS_DEFAULT: dict[str, Any] = {
    "max_trades_per_cycle": 5,
    "max_trades_per_day": 25,
    "max_per_symbol": 1,
    "max_per_sector": 2,
    "max_per_family": 2,
    "max_directional_same_side": 3,
    "max_abs_portfolio_delta": 2.5,
    "max_abs_portfolio_vega": 6.0,
    "max_corr": 0.75,
    "dedupe_window_minutes": 30,
}


@dataclass
class AssetCandidate:
    symbol: str
    asset_family: str
    sector: Optional[str]
    price: float
    beta: Optional[float]
    has_weeklies: bool
    avg_option_volume_20d: float
    avg_open_interest_20d: float
    avg_bid_ask_pct: float
    earnings_date: Optional[str]
    liquidity_score: float


@dataclass
class OptionContract:
    expiry: str
    option_type: str
    strike: float
    delta: float
    bid: float
    ask: float
    open_interest: float
    volume: float
    iv: float

    @property
    def mid(self) -> float:
        if self.bid > 0 and self.ask > 0:
            return (self.bid + self.ask) / 2.0
        return max(self.bid, self.ask, 0.0)

    @property
    def spread_pct(self) -> float:
        m = self.mid
        if m <= 0:
            return 1.0
        return max(0.0, (self.ask - self.bid) / m)


def _safe_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


def _safe_int(v: Any, default: int = 0) -> int:
    try:
        return int(float(v))
    except Exception:
        return default


def _asset_family_from_metadata(meta: dict[str, Any]) -> str:
    kind = str(meta.get("asset_class") or meta.get("type") or "").upper()
    symbol = str(meta.get("symbol") or "").upper()
    beta = _safe_float(meta.get("beta"), 1.0)
    sector = str(meta.get("sector") or "").lower()
    earnings_date = meta.get("earnings_date")
    if kind in {"INDEX", "INDX"} or symbol in {"SPX", "NDX", "RUT", "VIX"}:
        return "INDEX"
    if kind == "ETF":
        if sector and sector != "broad_market":
            return "SECTOR_ETF"
        return "ETF"
    if earnings_date:
        return "EVENT_DRIVEN_EQUITY"
    if beta >= 1.6:
        return "HIGH_BETA_EQUITY"
    return "LARGE_CAP_EQUITY"


def _liquidity_score(avg_vol: float, avg_oi: float, spread_pct: float, has_weeklies: bool) -> float:
    vol_score = min(1.0, avg_vol / 10000.0)
    oi_score = min(1.0, avg_oi / 20000.0)
    spread_score = max(0.0, 1.0 - min(1.0, spread_pct / 0.05))
    weeklies_boost = 0.1 if has_weeklies else 0.0
    return round(100.0 * (0.42 * vol_score + 0.38 * oi_score + 0.20 * spread_score) + weeklies_boost * 100.0, 4)


def build_optionable_universe(atlas: Any) -> list[AssetCandidate]:
    """Construye universo opcionable amplio y filtrado por calidad."""
    raw_symbols = atlas.get_optionable_universe()
    if hasattr(atlas, "filter_liquid_symbols"):
        try:
            raw_symbols = atlas.filter_liquid_symbols(raw_symbols)
        except Exception:
            pass
    symbols = [str(s).upper() for s in (raw_symbols or []) if str(s).strip()]
    candidates: list[AssetCandidate] = []
    for symbol in symbols:
        meta = atlas.get_asset_metadata(symbol) or {}
        avg_option_volume_20d = _safe_float(meta.get("avg_option_volume_20d"), 0.0)
        avg_bid_ask_pct = _safe_float(meta.get("avg_bid_ask_pct"), 1.0)
        avg_open_interest_20d = _safe_float(meta.get("avg_open_interest_20d"), 0.0)
        has_weeklies = bool(meta.get("has_weeklies"))
        asset_family = _asset_family_from_metadata({**meta, "symbol": symbol})
        fam_cfg = ASSET_FAMILY_LIMITS.get(asset_family, ASSET_FAMILY_LIMITS["LARGE_CAP_EQUITY"])
        if avg_option_volume_20d < float(fam_cfg["min_option_volume"]):
            continue
        if avg_open_interest_20d < float(fam_cfg["min_oi"]):
            continue
        if avg_bid_ask_pct > float(fam_cfg["spread_max_pct"]):
            continue
        liq = _liquidity_score(avg_option_volume_20d, avg_open_interest_20d, avg_bid_ask_pct, has_weeklies)
        candidates.append(
            AssetCandidate(
                symbol=symbol,
                asset_family=asset_family,
                sector=(meta.get("sector") or None),
                price=_safe_float(meta.get("price"), 0.0),
                beta=meta.get("beta"),
                has_weeklies=has_weeklies,
                avg_option_volume_20d=avg_option_volume_20d,
                avg_open_interest_20d=avg_open_interest_20d,
                avg_bid_ask_pct=avg_bid_ask_pct,
                earnings_date=meta.get("earnings_date"),
                liquidity_score=liq,
            )
        )
    candidates.sort(key=lambda x: x.liquidity_score, reverse=True)
    return candidates


def _to_contract(raw: dict[str, Any]) -> OptionContract | None:
    try:
        expiry = str(raw.get("expiry") or raw.get("expiration") or "").strip()
        option_type = str(raw.get("type") or raw.get("option_type") or "").lower()
        if not expiry or option_type not in {"call", "put"}:
            return None
        strike = _safe_float(raw.get("strike"), 0.0)
        if strike <= 0:
            return None
        return OptionContract(
            expiry=expiry,
            option_type=option_type,
            strike=strike,
            delta=_safe_float(raw.get("delta"), 0.0),
            bid=_safe_float(raw.get("bid"), 0.0),
            ask=_safe_float(raw.get("ask"), 0.0),
            open_interest=_safe_float(raw.get("open_interest"), 0.0),
            volume=_safe_float(raw.get("volume"), 0.0),
            iv=_safe_float(raw.get("iv"), 0.0),
        )
    except Exception:
        return None


def _parse_chain(chain: Any) -> list[OptionContract]:
    if isinstance(chain, dict):
        rows = chain.get("options") or chain.get("contracts") or []
    else:
        rows = chain or []
    out: list[OptionContract] = []
    for row in rows:
        if not isinstance(row, dict):
            continue
        c = _to_contract(row)
        if c is not None:
            out.append(c)
    return out


def _dte(expiry: str, current_time: datetime) -> int:
    try:
        exp_dt = datetime.fromisoformat(expiry.replace("Z", "+00:00"))
        if exp_dt.tzinfo is None:
            exp_dt = exp_dt.replace(tzinfo=timezone.utc)
        now = current_time if current_time.tzinfo else current_time.replace(tzinfo=timezone.utc)
        return max(0, int((exp_dt.date() - now.date()).days))
    except Exception:
        return 0


def select_expiries_for_strategy(
    *,
    contracts: list[OptionContract],
    strategy: str,
    asset_family: str,
    current_time: datetime,
) -> list[str]:
    fam_cfg = ASSET_FAMILY_LIMITS.get(asset_family, ASSET_FAMILY_LIMITS["LARGE_CAP_EQUITY"])
    s = strategy.upper()
    if s in SHORT_PREMIUM_SET:
        dte_min, dte_max = fam_cfg["dte_short_premium"]
    elif s in LONG_PREMIUM_SET:
        dte_min, dte_max = fam_cfg["dte_long_premium"]
    elif s == "CALENDAR":
        dte_min, dte_max = fam_cfg["dte_calendar_front"][0], fam_cfg["dte_calendar_back"][1]
    else:
        dte_min, dte_max = 21, 45
    expiries = sorted({c.expiry for c in contracts if dte_min <= _dte(c.expiry, current_time) <= dte_max})
    return expiries


def find_option_by_target_delta(
    contracts: Iterable[OptionContract],
    *,
    option_type: str,
    target_delta_abs: float,
    min_oi: float = 100.0,
    max_spread_pct: float = 0.06,
) -> OptionContract | None:
    filtered = [
        c
        for c in contracts
        if c.option_type == option_type
        and c.open_interest >= min_oi
        and c.spread_pct <= max_spread_pct
        and c.mid > 0
    ]
    if not filtered:
        return None
    return min(filtered, key=lambda c: abs(abs(c.delta) - target_delta_abs))


def _leg_from_contract(side: str, c: OptionContract) -> Leg:
    return Leg(side=side, type=c.option_type, strike=float(c.strike), delta=float(c.delta))


def build_iron_condor(
    *,
    symbol: str,
    expiry: str,
    calls: list[OptionContract],
    puts: list[OptionContract],
) -> OptionStructure | None:
    short_call = find_option_by_target_delta(calls, option_type="call", target_delta_abs=0.16)
    long_call = find_option_by_target_delta(calls, option_type="call", target_delta_abs=0.07)
    short_put = find_option_by_target_delta(puts, option_type="put", target_delta_abs=0.16)
    long_put = find_option_by_target_delta(puts, option_type="put", target_delta_abs=0.07)
    if not all([short_call, long_call, short_put, long_put]):
        return None
    if long_call.strike <= short_call.strike or long_put.strike >= short_put.strike:
        return None
    return OptionStructure(
        symbol=symbol,
        strategy="IRON_CONDOR",
        expiry=expiry,
        legs=[
            _leg_from_contract("sell", short_put),
            _leg_from_contract("buy", long_put),
            _leg_from_contract("sell", short_call),
            _leg_from_contract("buy", long_call),
        ],
        contracts=1,
    )


def build_credit_spread(
    *,
    symbol: str,
    expiry: str,
    contracts: list[OptionContract],
    direction: str,
) -> OptionStructure | None:
    d = direction.upper()
    if d == "BULL":
        puts = [c for c in contracts if c.option_type == "put"]
        short_leg = find_option_by_target_delta(puts, option_type="put", target_delta_abs=0.18)
        long_leg = find_option_by_target_delta(puts, option_type="put", target_delta_abs=0.08)
        if not short_leg or not long_leg or long_leg.strike >= short_leg.strike:
            return None
        return OptionStructure(
            symbol=symbol,
            strategy="PUT_CREDIT_SPREAD",
            expiry=expiry,
            legs=[_leg_from_contract("sell", short_leg), _leg_from_contract("buy", long_leg)],
            contracts=1,
        )
    calls = [c for c in contracts if c.option_type == "call"]
    short_leg = find_option_by_target_delta(calls, option_type="call", target_delta_abs=0.18)
    long_leg = find_option_by_target_delta(calls, option_type="call", target_delta_abs=0.08)
    if not short_leg or not long_leg or long_leg.strike <= short_leg.strike:
        return None
    return OptionStructure(
        symbol=symbol,
        strategy="CALL_CREDIT_SPREAD",
        expiry=expiry,
        legs=[_leg_from_contract("sell", short_leg), _leg_from_contract("buy", long_leg)],
        contracts=1,
    )


def build_debit_spread(
    *,
    symbol: str,
    expiry: str,
    contracts: list[OptionContract],
    direction: str,
) -> OptionStructure | None:
    d = direction.upper()
    if d == "BULL":
        calls = [c for c in contracts if c.option_type == "call"]
        long_leg = find_option_by_target_delta(calls, option_type="call", target_delta_abs=0.55)
        short_leg = find_option_by_target_delta(calls, option_type="call", target_delta_abs=0.32)
        if not long_leg or not short_leg or short_leg.strike <= long_leg.strike:
            return None
        strategy = "CALL_DEBIT_SPREAD"
    else:
        puts = [c for c in contracts if c.option_type == "put"]
        long_leg = find_option_by_target_delta(puts, option_type="put", target_delta_abs=0.55)
        short_leg = find_option_by_target_delta(puts, option_type="put", target_delta_abs=0.32)
        if not long_leg or not short_leg or short_leg.strike >= long_leg.strike:
            return None
        strategy = "PUT_DEBIT_SPREAD"
    return OptionStructure(
        symbol=symbol,
        strategy=strategy,
        expiry=expiry,
        legs=[_leg_from_contract("buy", long_leg), _leg_from_contract("sell", short_leg)],
        contracts=1,
    )


def build_calendar(
    *,
    symbol: str,
    front_expiry: str,
    back_expiry: str,
    front_contracts: list[OptionContract],
    back_contracts: list[OptionContract],
    option_type: str = "call",
) -> OptionStructure | None:
    front = find_option_by_target_delta(front_contracts, option_type=option_type, target_delta_abs=0.35)
    if not front:
        return None
    back_candidates = [c for c in back_contracts if c.option_type == option_type]
    if not back_candidates:
        return None
    back = min(back_candidates, key=lambda c: abs(c.strike - front.strike))
    if back.mid <= 0 or front.mid <= 0:
        return None
    return OptionStructure(
        symbol=symbol,
        strategy="CALENDAR",
        expiry=front_expiry,
        legs=[_leg_from_contract("sell", front), _leg_from_contract("buy", back)],
        contracts=1,
    )


def _default_strategy_pool(asset_family: str) -> list[str]:
    family = str(asset_family).upper()
    if family in {"INDEX", "ETF", "SECTOR_ETF"}:
        return ["IRON_CONDOR", "PUT_CREDIT_SPREAD", "CALL_CREDIT_SPREAD", "CALENDAR", "CALL_DEBIT_SPREAD"]
    if family == "HIGH_BETA_EQUITY":
        return ["CALL_DEBIT_SPREAD", "PUT_DEBIT_SPREAD", "LONG_CALL", "LONG_PUT"]
    if family == "EVENT_DRIVEN_EQUITY":
        return ["CALL_DEBIT_SPREAD", "PUT_DEBIT_SPREAD", "LONG_CALL", "LONG_PUT", "CALENDAR"]
    return ["PUT_CREDIT_SPREAD", "CALL_CREDIT_SPREAD", "CALL_DEBIT_SPREAD", "PUT_DEBIT_SPREAD", "CALENDAR"]


def _direction_from_price_regime(price_regime: PriceRegime) -> str:
    b = str(price_regime.breakout_status or "").lower()
    ema = str(price_regime.ema_alignment or "").lower()
    if "up" in b or "bull" in ema:
        return "BULL"
    if "down" in b or "bear" in ema:
        return "BEAR"
    return "NEUTRAL"


def _date_distance_days(current_time: datetime, iso_date: str | None) -> int | None:
    if not iso_date:
        return None
    try:
        d = datetime.fromisoformat(str(iso_date).replace("Z", "+00:00"))
        now = current_time if current_time.tzinfo else current_time.replace(tzinfo=timezone.utc)
        return int((d.date() - now.date()).days)
    except Exception:
        return None


def _entry_price_limit(structure: OptionStructure, contracts: list[OptionContract]) -> float:
    lookup = {(c.option_type, round(c.strike, 8)): c for c in contracts}
    debit = 0.0
    for leg in structure.legs:
        c = lookup.get((leg.type, round(float(leg.strike), 8)))
        if not c:
            continue
        px = c.mid
        if leg.side == "buy":
            debit += px
        else:
            debit -= px
    return round(max(0.01, debit), 4)


def generate_candidate_strategies(
    *,
    symbol: str,
    asset_family: str,
    current_time: datetime,
    chain: Any,
    price_regime: PriceRegime,
    strategy_pool: Optional[list[str]] = None,
) -> list[OptionStructure]:
    contracts = _parse_chain(chain)
    if not contracts:
        return []
    pool = strategy_pool or _default_strategy_pool(asset_family)
    expiries = select_expiries_for_strategy(
        contracts=contracts,
        strategy=pool[0] if pool else "CALENDAR",
        asset_family=asset_family,
        current_time=current_time,
    )
    if not expiries:
        return []
    direction = _direction_from_price_regime(price_regime)
    by_exp: dict[str, list[OptionContract]] = {}
    for c in contracts:
        by_exp.setdefault(c.expiry, []).append(c)
    structures: list[OptionStructure] = []
    for strategy in pool:
        s = strategy.upper()
        if s not in STRATEGY_SET:
            continue
        if s == "IRON_CONDOR":
            for exp in expiries[:2]:
                rows = by_exp.get(exp, [])
                calls = [r for r in rows if r.option_type == "call"]
                puts = [r for r in rows if r.option_type == "put"]
                st = build_iron_condor(symbol=symbol, expiry=exp, calls=calls, puts=puts)
                if st:
                    structures.append(st)
        elif s in {"PUT_CREDIT_SPREAD", "CALL_CREDIT_SPREAD", "CREDIT_SPREAD"}:
            preferred_dir = "BULL" if s.startswith("PUT") else ("BEAR" if s.startswith("CALL") else direction)
            for exp in expiries[:2]:
                st = build_credit_spread(
                    symbol=symbol,
                    expiry=exp,
                    contracts=by_exp.get(exp, []),
                    direction=preferred_dir,
                )
                if st:
                    structures.append(st)
        elif s in {"CALL_DEBIT_SPREAD", "PUT_DEBIT_SPREAD", "DEBIT_SPREAD"}:
            preferred_dir = "BULL" if s.startswith("CALL") else ("BEAR" if s.startswith("PUT") else direction)
            for exp in expiries[:2]:
                st = build_debit_spread(
                    symbol=symbol,
                    expiry=exp,
                    contracts=by_exp.get(exp, []),
                    direction=preferred_dir,
                )
                if st:
                    structures.append(st)
        elif s == "CALENDAR":
            if len(expiries) < 2:
                continue
            front, back = expiries[0], expiries[-1]
            st = build_calendar(
                symbol=symbol,
                front_expiry=front,
                back_expiry=back,
                front_contracts=by_exp.get(front, []),
                back_contracts=by_exp.get(back, []),
                option_type="call" if direction != "BEAR" else "put",
            )
            if st:
                structures.append(st)
        elif s == "LONG_CALL":
            for exp in expiries[:1]:
                rows = by_exp.get(exp, [])
                long_leg = find_option_by_target_delta(rows, option_type="call", target_delta_abs=0.52)
                hedge_leg = find_option_by_target_delta(rows, option_type="call", target_delta_abs=0.08)
                if long_leg and hedge_leg and hedge_leg.strike > long_leg.strike:
                    structures.append(
                        OptionStructure(
                            symbol=symbol,
                            strategy="LONG_CALL",
                            expiry=exp,
                            legs=[_leg_from_contract("buy", long_leg), _leg_from_contract("sell", hedge_leg)],
                            contracts=1,
                        )
                    )
        elif s == "LONG_PUT":
            for exp in expiries[:1]:
                rows = by_exp.get(exp, [])
                long_leg = find_option_by_target_delta(rows, option_type="put", target_delta_abs=0.52)
                hedge_leg = find_option_by_target_delta(rows, option_type="put", target_delta_abs=0.08)
                if long_leg and hedge_leg and hedge_leg.strike < long_leg.strike:
                    structures.append(
                        OptionStructure(
                            symbol=symbol,
                            strategy="LONG_PUT",
                            expiry=exp,
                            legs=[_leg_from_contract("buy", long_leg), _leg_from_contract("sell", hedge_leg)],
                            contracts=1,
                        )
                    )
    return structures


def rank_and_deduplicate_opportunities(
    opportunities: list[dict[str, Any]],
    portfolio_state: dict[str, Any] | None,
    limits: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    cfg = {**PIPELINE_LIMITS_DEFAULT, **(limits or {})}
    open_positions = (portfolio_state or {}).get("open_positions") or []
    open_keys = {
        (
            str(p.get("symbol") or "").upper(),
            str(p.get("strategy") or p.get("strategy_type") or "").upper(),
            str(p.get("expiry") or ""),
        )
        for p in open_positions
        if isinstance(p, dict)
    }
    sorted_opps = sorted(opportunities, key=lambda x: float(x.get("score") or 0.0), reverse=True)
    out: list[dict[str, Any]] = []
    seen_symbol: dict[str, int] = {}
    seen_sector: dict[str, int] = {}
    seen_family: dict[str, int] = {}
    directional_counts = {"BULL": 0, "BEAR": 0, "NEUTRAL": 0}
    dedupe_signatures: set[tuple[str, str, str]] = set()

    for opp in sorted_opps:
        symbol = str(opp.get("symbol") or "").upper()
        strategy = str(opp.get("strategy") or "").upper()
        expiry = str(opp.get("expiry") or "")
        sector = str(opp.get("sector") or "unknown")
        family = str(opp.get("asset_family") or "LARGE_CAP_EQUITY")
        direction = str(opp.get("direction") or "NEUTRAL").upper()
        sig = (symbol, strategy, expiry)
        if sig in dedupe_signatures:
            continue
        if sig in open_keys:
            continue
        if seen_symbol.get(symbol, 0) >= int(cfg["max_per_symbol"]):
            continue
        if seen_sector.get(sector, 0) >= int(cfg["max_per_sector"]):
            continue
        if seen_family.get(family, 0) >= int(cfg["max_per_family"]):
            continue
        if direction in {"BULL", "BEAR"} and directional_counts[direction] >= int(cfg["max_directional_same_side"]):
            continue
        out.append(opp)
        dedupe_signatures.add(sig)
        seen_symbol[symbol] = seen_symbol.get(symbol, 0) + 1
        seen_sector[sector] = seen_sector.get(sector, 0) + 1
        seen_family[family] = seen_family.get(family, 0) + 1
        directional_counts[direction] = directional_counts.get(direction, 0) + 1
        if len(out) >= int(cfg["max_trades_per_cycle"]):
            break
    return out


def _apply_correlation_filter(
    opportunities: list[dict[str, Any]],
    correlation_matrix: dict[str, dict[str, float]] | None,
    max_corr: float,
) -> list[dict[str, Any]]:
    if not opportunities or not correlation_matrix:
        return opportunities
    selected: list[dict[str, Any]] = []
    for opp in opportunities:
        sym = str(opp.get("symbol") or "").upper()
        blocked = False
        for sel in selected:
            other = str(sel.get("symbol") or "").upper()
            corr = _safe_float((correlation_matrix.get(sym) or {}).get(other), 0.0)
            corr = max(corr, _safe_float((correlation_matrix.get(other) or {}).get(sym), corr))
            if abs(corr) > max_corr:
                blocked = True
                break
        if not blocked:
            selected.append(opp)
    return selected


def build_exit_rules_for_strategy(opp: dict[str, Any], global_regime: GlobalRegime, asset_metadata: dict[str, Any]) -> dict[str, Any]:
    strategy = str(opp.get("strategy") or "").upper()
    family = str(opp.get("asset_family") or "")
    earnings_days = opp.get("earnings_days")
    base = {
        "tp": {"type": "credit_pct", "value": 0.55},
        "sl": {"type": "risk_multiple", "value": 2.0},
        "time_cutoff": {"dte": 21},
        "dynamic_triggers": ["macro_event_in_24h"] if global_regime.event_risk else [],
        "roll_policy": {"enabled": False, "target_dte": None},
    }
    if strategy in SHORT_PREMIUM_SET:
        base["dynamic_triggers"].extend(["vrp_flip_negative", "delta_breach_0.35", "gamma_flip_positive"])
        if family.endswith("EQUITY"):
            base["dynamic_triggers"].append("earnings_in_3d")
        if earnings_days is not None and int(earnings_days) <= 3 and "earnings_in_3d" not in base["dynamic_triggers"]:
            base["dynamic_triggers"].append("earnings_in_3d")
        return base
    if strategy in LONG_PREMIUM_SET:
        return {
            "tp": {"type": "debit_multiple", "value": 2.2},
            "sl": {"type": "debit_pct", "value": 0.55},
            "time_cutoff": {"dte": 12},
            "dynamic_triggers": ["trend_invalidated", "vol_crush", "macro_event_in_24h"],
            "roll_policy": {"enabled": False, "target_dte": None},
        }
    if strategy == "CALENDAR":
        triggers = ["front_iv_collapse", "term_structure_inversion", "macro_event_in_24h"]
        if family.endswith("EQUITY"):
            triggers.append("earnings_in_3d")
        return {
            "tp": {"type": "debit_pct_profit", "value": 0.5},
            "sl": {"type": "debit_pct_loss", "value": 0.5},
            "time_cutoff": {"dte": 14},
            "dynamic_triggers": triggers,
            "roll_policy": {"enabled": False, "target_dte": None},
        }
    return base


def build_risk_metadata(opp: dict[str, Any], sizing: dict[str, Any], portfolio_state: dict[str, Any]) -> dict[str, Any]:
    return {
        "risk_per_trade_pct": sizing.get("risk_pct", 0.0),
        "risk_per_trade_usd": sizing.get("risk_dollars", 0.0),
        "portfolio_delta_before": portfolio_state.get("net_delta", 0.0),
        "portfolio_vega_before": portfolio_state.get("net_vega", 0.0),
        "asset_family_cap": ASSET_FAMILY_LIMITS.get(str(opp.get("asset_family") or ""), {}).get("portfolio_weight_cap"),
        "symbol_exposure_before": portfolio_state.get("symbol_exposure", {}).get(opp.get("symbol"), 0.0),
        "sector_exposure_before": portfolio_state.get("sector_exposure", {}).get(opp.get("sector"), 0.0),
    }


def validate_and_create_position(atlas: Any, position: dict[str, Any], exit_rules: dict[str, Any]) -> dict[str, Any]:
    structure = OptionStructure(**position)
    payload = {
        "symbol": structure.symbol,
        "strategy": structure.strategy,
        "expiry": structure.expiry,
        "legs": [leg.model_dump() for leg in structure.legs],
        "contracts": structure.contracts,
        "mode": "paper",
    }
    try:
        return atlas.create_position(payload, exit_rules=exit_rules)
    except TypeError:
        # Compatibilidad con integraciones legacy que aceptan solo un payload.
        payload_with_exit = dict(payload)
        payload_with_exit["exit_rules"] = dict(exit_rules)
        return atlas.create_position(payload_with_exit)


def _to_global_regime(raw: dict[str, Any]) -> GlobalRegime:
    return GlobalRegime(event_risk=bool(raw.get("event_risk", False)), regime_id=str(raw.get("regime_id") or "unknown"))


def _safe_atlas_call(atlas: Any, method_name: str, *args: Any, **kwargs: Any) -> Any:
    fn = getattr(atlas, method_name, None)
    if not callable(fn):
        return None
    try:
        return fn(*args, **kwargs)
    except Exception:
        return None


def _evaluate_symbol(
    *,
    atlas: Any,
    candidate: AssetCandidate,
    global_regime: GlobalRegime,
    current_time: datetime,
) -> list[dict[str, Any]]:
    symbol = candidate.symbol
    vol_raw = atlas.get_vol_regime(symbol) or {}
    gex_raw = atlas.get_gex_surface(symbol) or {}
    oi_raw = atlas.get_oi_flow(symbol) or {}
    price_raw = atlas.get_price_regime(symbol) or {}
    seasonal_factor = _safe_float(atlas.get_seasonal_multiplier(symbol, current_time), 1.0)
    vol_data = VolData(
        iv_rank=_safe_float(vol_raw.get("iv_rank"), 0.0),
        vrp_20d=_safe_float(vol_raw.get("vrp_20d"), 0.0),
        term_structure_slope=_safe_float(vol_raw.get("term_structure_slope"), 0.0),
        skew_25d=_safe_float(vol_raw.get("skew_25d"), 0.0),
    )
    gex_data = GexData(
        net_gex=_safe_float(gex_raw.get("net_gex"), 0.0),
        gamma_flip_distance_pct=_safe_float(gex_raw.get("gamma_flip_distance_pct"), 0.0),
        max_pain_distance_pct=_safe_float(gex_raw.get("max_pain_distance_pct"), 0.0),
    )
    oi_data = OiData(
        oi_change_1d_pct=_safe_float(oi_raw.get("oi_change_1d_pct"), 0.0),
        call_put_volume_ratio=_safe_float(oi_raw.get("call_put_volume_ratio"), 1.0),
        max_pain_distance_pct=_safe_float(oi_raw.get("max_pain_distance_pct"), 0.0),
    )
    price_regime = PriceRegime(
        adx=_safe_float(price_raw.get("adx"), 0.0),
        ema_alignment=str(price_raw.get("ema_alignment") or "mixed"),
        breakout_status=str(price_raw.get("breakout_status") or "range"),
    )
    vol_score = calculate_vol_score(vol_data, global_regime)
    gamma_score = calculate_gamma_score(gex_data, global_regime)
    oi_score = calculate_oi_score(oi_data, global_regime)
    price_score = calculate_price_score(price_regime, global_regime)
    score = combine_scores(
        vol_score=vol_score,
        gamma_score=gamma_score,
        oi_score=oi_score,
        price_score=price_score,
        asset_family=candidate.asset_family,  # type: ignore[arg-type]
    )
    total_score = score * max(0.6, min(1.4, seasonal_factor))
    score_min = max(
        get_min_total_score_for_asset_family(candidate.asset_family),
        float(ASSET_FAMILY_LIMITS.get(candidate.asset_family, {}).get("score_min", 0.0)),
    )
    earnings_days = _date_distance_days(current_time, candidate.earnings_date)
    if total_score < score_min:
        atlas.log_event(
            {
                "event_type": "signal_skipped",
                "symbol": symbol,
                "reason": "score_below_threshold",
                "score": total_score,
                "threshold": score_min,
                "asset_family": candidate.asset_family,
            }
        )
        return []

    chain = atlas.get_option_chain(symbol)
    structures = generate_candidate_strategies(
        symbol=symbol,
        asset_family=candidate.asset_family,
        current_time=current_time,
        chain=chain,
        price_regime=price_regime,
    )
    if not structures:
        atlas.log_event({"event_type": "signal_skipped", "symbol": symbol, "reason": "no_valid_structure"})
        return []

    opps: list[dict[str, Any]] = []
    for st in structures:
        if st.strategy not in STRATEGY_SET:
            continue
        ok = matches_context_strategy_map(
            strategy=st.strategy,
            vol_data=vol_data,
            gex_data=gex_data,
            oi_data=oi_data,
            price_regime=price_regime,
            global_regime=global_regime,
            asset_family=candidate.asset_family,
            earnings_days=earnings_days,
            spread_pct=candidate.avg_bid_ask_pct,
        )
        if not ok:
            atlas.log_event(
                {
                    "event_type": "signal_skipped",
                    "symbol": symbol,
                    "reason": "context_strategy_mismatch",
                    "strategy": st.strategy,
                }
            )
            continue
        opps.append(
            {
                "symbol": symbol,
                "asset_family": candidate.asset_family,
                "sector": candidate.sector or "unknown",
                "price": candidate.price,
                "score": round(total_score, 6),
                "strategy": st.strategy,
                "expiry": st.expiry,
                "structure": st,
                "direction": _direction_from_price_regime(price_regime),
                "feature_snapshot": {
                    "vol": vol_data.__dict__,
                    "gamma": gex_data.__dict__,
                    "oi": oi_data.__dict__,
                    "price": price_regime.__dict__,
                    "liquidity": candidate.__dict__,
                },
                "regime": {"regime_id": global_regime.regime_id, "event_risk": global_regime.event_risk},
                "entry_reason": f"score={round(total_score,2)} family={candidate.asset_family}",
                "earnings_days": earnings_days,
                "chain": _parse_chain(chain),
            }
        )
    if not opps:
        atlas.log_event({"event_type": "signal_skipped", "symbol": symbol, "reason": "context_strategy_mismatch"})
    return opps


def _has_recent_signal(atlas: Any, symbol: str, strategy: str, expiry: str, within_minutes: int) -> bool:
    fn = getattr(atlas, "has_recent_signal", None)
    if callable(fn):
        try:
            return bool(fn(symbol, strategy, expiry, within_minutes=within_minutes))
        except Exception:
            return False
    return False


def _recent_journal_summary(atlas: Any) -> dict[str, Any]:
    return (
        _safe_atlas_call(atlas, "get_recent_journal_summary")
        or _safe_atlas_call(atlas, "summarize_recent_journal")
        or _safe_atlas_call(atlas, "get_recent_options_journal_summary")
        or {}
    )


def run_options_trading_pipeline(
    current_time: datetime,
    atlas: Any,
    *,
    limits: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Ejecuta ciclo completo del engine en modo paper para universo amplio."""
    limits_cfg = {**PIPELINE_LIMITS_DEFAULT, **(limits or {})}
    regime_raw = atlas.get_global_regime() or {}
    global_regime = _to_global_regime(regime_raw)
    tradable_today = bool(regime_raw.get("tradable_today", True))
    if not tradable_today:
        atlas.log_event(
            {
                "event_type": "signal_skipped",
                "reason": "market_not_tradable",
                "regime_id": global_regime.regime_id,
                "timestamp": current_time.isoformat(),
            }
        )
        return {"ok": True, "emitted": 0, "created": 0, "blocked": 0, "skipped": "market_not_tradable"}

    universe = build_optionable_universe(atlas)
    if not universe:
        atlas.log_event({"event_type": "signal_skipped", "reason": "liquidity_rejected"})
        return {"ok": True, "emitted": 0, "created": 0, "blocked": 0, "skipped": "empty_universe"}

    opportunities: list[dict[str, Any]] = []
    with ThreadPoolExecutor(max_workers=min(16, max(4, len(universe)))) as pool:
        future_map = {
            pool.submit(
                _evaluate_symbol,
                atlas=atlas,
                candidate=c,
                global_regime=global_regime,
                current_time=current_time,
            ): c.symbol
            for c in universe
        }
        for fut in as_completed(future_map):
            symbol = future_map[fut]
            try:
                opportunities.extend(fut.result() or [])
            except Exception as exc:
                atlas.log_event({"event_type": "signal_skipped", "symbol": symbol, "reason": f"symbol_eval_error:{exc}"})

    portfolio_state = atlas.get_portfolio_state() or {}
    selected = rank_and_deduplicate_opportunities(opportunities, portfolio_state, limits_cfg)
    corr_matrix = _safe_atlas_call(atlas, "get_correlation_matrix") or {}
    selected = _apply_correlation_filter(selected, corr_matrix, max_corr=float(limits_cfg["max_corr"]))
    grok_provider = GrokExpertReviewProvider()
    recent_journal_summary = _recent_journal_summary(atlas)
    emitted = 0
    created = 0
    blocked = 0
    for opp in selected:
        structure: OptionStructure = opp["structure"]
        if _has_recent_signal(
            atlas,
            structure.symbol,
            structure.strategy,
            structure.expiry,
            within_minutes=int(limits_cfg["dedupe_window_minutes"]),
        ):
            atlas.log_event(
                {
                    "event_type": "signal_skipped",
                    "symbol": structure.symbol,
                    "strategy": structure.strategy,
                    "reason": "duplicate_signal_window",
                }
            )
            continue
        decision_pack = build_decision_pack(opp, global_regime, portfolio_state, recent_journal_summary)
        grok_review = grok_provider.review_trade(decision_pack)
        atlas.log_event(
            {
                "event_type": "grok_review",
                "symbol": opp.get("symbol"),
                "strategy": opp.get("strategy"),
                "provider_status": grok_review.get("status"),
                "verdict": grok_review.get("verdict"),
                "score_adjustment": grok_review.get("score_adjustment"),
                "contracts_multiplier": grok_review.get("contracts_multiplier"),
                "prefer_strategy": grok_review.get("prefer_strategy"),
                "error": grok_review.get("error"),
                "rationale": grok_review.get("rationale"),
            }
        )
        policy_result = apply_grok_review_policy(opp, grok_review)
        opp = policy_result["opportunity"]
        if policy_result["blocked"]:
            blocked += 1
            atlas.log_event(
                {
                    "event_type": "entry_blocked",
                    "symbol": structure.symbol,
                    "strategy": structure.strategy,
                    "reason": policy_result["block_reason"],
                    "provider": "grok",
                    "asset_family": opp.get("asset_family"),
                    "score": opp.get("score"),
                }
            )
            continue
        sizing = _safe_atlas_call(
            atlas,
            "calculate_position_sizing",
            opportunity=opp,
            portfolio_state=portfolio_state,
            limits=limits_cfg,
            risk_per_trade_pct=0.008,
            asset_family=opp.get("asset_family"),
            mode="paper",
        ) or {}
        contracts = _safe_int(
            sizing.get("contracts")
            if isinstance(sizing, dict)
            else None,
            _safe_int(sizing.get("position_size_units") if isinstance(sizing, dict) else 0, 0),
        )
        contracts_multiplier = _safe_float(opp.get("grok_contracts_multiplier"), None)
        if contracts_multiplier is not None:
            contracts = int(contracts * max(0.0, min(1.0, contracts_multiplier)))
        if contracts <= 0:
            blocked += 1
            atlas.log_event(
                {
                    "event_type": "entry_blocked",
                    "symbol": structure.symbol,
                    "strategy": structure.strategy,
                    "reason": "contracts_zero",
                    "asset_family": opp.get("asset_family"),
                    "score": opp.get("score"),
                }
            )
            continue
        structure = OptionStructure(
            symbol=structure.symbol,
            strategy=structure.strategy,
            expiry=structure.expiry,
            legs=structure.legs,
            contracts=contracts,
        )
        exit_rules = build_exit_rules_for_strategy(opp, global_regime, atlas.get_asset_metadata(structure.symbol) or {})
        chain_contracts = opp.get("chain") or []
        signal = {
            "type": "options_entry_signal",
            "timestamp": current_time.isoformat(),
            "symbol": structure.symbol,
            "strategy": structure.strategy,
            "asset_family": opp.get("asset_family"),
            "expiry": structure.expiry,
            "legs": [leg.model_dump() for leg in structure.legs],
            "contracts": structure.contracts,
            "entry_price_limit": _entry_price_limit(structure, chain_contracts),
            "score": opp.get("score"),
            "entry_reason": opp.get("entry_reason"),
            "regime": opp.get("regime"),
            "feature_snapshot": opp.get("feature_snapshot"),
            "risk_metadata": build_risk_metadata(opp, sizing if isinstance(sizing, dict) else {}, portfolio_state),
            "grok_review": grok_review,
            "paper_only": True,
        }
        atlas.emit_signal(signal)
        emitted += 1
        atlas.log_event(
            {
                "event_type": "signal_emitted",
                "symbol": structure.symbol,
                "strategy": structure.strategy,
                "asset_family": opp.get("asset_family"),
                "score": opp.get("score"),
                "feature_snapshot": opp.get("feature_snapshot"),
                "risk_metadata": signal["risk_metadata"],
            }
        )
        position_payload = {
            "symbol": structure.symbol,
            "strategy": structure.strategy,
            "expiry": structure.expiry,
            "legs": [leg.model_dump() for leg in structure.legs],
            "contracts": structure.contracts,
        }
        created_pos = validate_and_create_position(atlas, position_payload, exit_rules=exit_rules)
        created += 1
        atlas.log_event(
            {
                "event_type": "entry_execution",
                "symbol": structure.symbol,
                "strategy": structure.strategy,
                "asset_family": opp.get("asset_family"),
                "position_id": (created_pos or {}).get("position_id"),
                "regime_id": global_regime.regime_id,
                "entry_reason": opp.get("entry_reason"),
                "feature_snapshot": opp.get("feature_snapshot"),
                "exit_rules": exit_rules,
            }
        )

    return {
        "ok": True,
        "universe_size": len(universe),
        "opportunities": len(opportunities),
        "selected": len(selected),
        "emitted": emitted,
        "created": created,
        "blocked": blocked,
        "timestamp": current_time.isoformat(),
    }

