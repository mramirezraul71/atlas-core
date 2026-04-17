"""Winning probability engine for options trading with Tradier.

This module is intentionally conservative:
- Uses official Tradier REST market-data endpoints.
- Blends realized volatility with current chain IV.
- Adds a light Markov regime layer over Monte Carlo paths.
- Returns a probability estimate and the assumptions used.

It does not place trades. It is designed to be called before execution.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass
from datetime import date, datetime, timedelta, timezone
from typing import Any, Literal
from zoneinfo import ZoneInfo

import numpy as np
import requests

from atlas_code_quant.config.settings import settings


SUPPORTED_STRATEGIES = (
    "long_call",
    "long_put",
    "covered_call",
    "cash_secured_put",
    "bull_call_debit_spread",
    "bear_put_debit_spread",
    "bear_call_credit_spread",
    "bull_put_credit_spread",
    "call_debit_butterfly",
    "put_debit_butterfly",
    "call_credit_butterfly",
    "put_credit_butterfly",
    "call_debit_condor",
    "put_debit_condor",
    "call_credit_condor",
    "put_credit_condor",
    "long_straddle",
    "long_strangle",
    "iron_condor",
    "iron_butterfly",
    "calendar_spread",
    "call_calendar_spread",
    "put_calendar_spread",
    "call_diagonal_debit_spread",
    "put_diagonal_debit_spread",
)


StrategyType = Literal[
    "long_call",
    "long_put",
    "covered_call",
    "cash_secured_put",
    "bull_call_debit_spread",
    "bear_put_debit_spread",
    "bear_call_credit_spread",
    "bull_put_credit_spread",
    "call_debit_butterfly",
    "put_debit_butterfly",
    "call_credit_butterfly",
    "put_credit_butterfly",
    "call_debit_condor",
    "put_debit_condor",
    "call_credit_condor",
    "put_credit_condor",
    "long_straddle",
    "long_strangle",
    "iron_condor",
    "iron_butterfly",
    "calendar_spread",
    "call_calendar_spread",
    "put_calendar_spread",
    "call_diagonal_debit_spread",
    "put_diagonal_debit_spread",
]

TradierScope = Literal["live", "paper"]


@dataclass
class StrategyLeg:
    side: Literal["long", "short"]
    option_type: Literal["call", "put"]
    strike: float
    premium_mid: float
    expiration: str | None
    dte: int | None
    symbol: str
    bid: float
    ask: float
    volume: float
    open_interest: float
    implied_volatility: float | None

    def to_dict(self) -> dict[str, Any]:
        return {
            "side": self.side,
            "option_type": self.option_type,
            "strike": self.strike,
            "premium_mid": self.premium_mid,
            "expiration": self.expiration,
            "dte": self.dte,
            "symbol": self.symbol,
            "bid": self.bid,
            "ask": self.ask,
            "volume": self.volume,
            "open_interest": self.open_interest,
            "implied_volatility": self.implied_volatility,
        }


@dataclass
class WinningProbabilityResult:
    symbol: str
    strategy_type: str
    win_rate_pct: float
    expected_pnl: float
    expected_roi_pct: float
    net_premium: float
    selected_contract: dict[str, Any]
    selected_legs: list[dict[str, Any]]
    market_snapshot: dict[str, Any]
    markov_snapshot: dict[str, Any]
    assumptions: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return {
            "symbol": self.symbol,
            "strategy_type": self.strategy_type,
            "win_rate_pct": round(self.win_rate_pct, 2),
            "expected_pnl": round(self.expected_pnl, 4),
            "expected_roi_pct": round(self.expected_roi_pct, 2),
            "net_premium": round(self.net_premium, 4),
            "selected_contract": self.selected_contract,
            "selected_legs": self.selected_legs,
            "market_snapshot": self.market_snapshot,
            "markov_snapshot": self.markov_snapshot,
            "assumptions": self.assumptions,
        }


class TradierClient:
    """Tradier REST client for market data and account operations."""

    def __init__(
        self,
        scope: TradierScope | None = None,
        token: str | None = None,
        base_url: str | None = None,
        timeout_sec: int | None = None,
    ) -> None:
        self.scope: TradierScope = self._resolve_scope(scope, base_url)
        self.token = self._resolve_token(scope=self.scope, token=token)
        if not self.token:
            raise ValueError(f"Missing Tradier token for scope '{self.scope}'")
        self.base_url = self._resolve_base_url(scope=self.scope, base_url=base_url)
        self.timeout_sec = int(timeout_sec or settings.tradier_timeout_sec)
        self.session = requests.Session()
        self.session.headers.update(
            {
                "Authorization": f"Bearer {self.token}",
                "Accept": "application/json",
            }
        )

    @staticmethod
    def _resolve_scope(scope: TradierScope | None, base_url: str | None) -> TradierScope:
        if scope in {"live", "paper"}:
            return scope
        if base_url and "sandbox.tradier.com" in base_url:
            return "paper"
        return settings.tradier_default_scope  # type: ignore[return-value]

    @staticmethod
    def _resolve_token(scope: TradierScope, token: str | None) -> str:
        if token:
            return token.strip()
        if scope == "paper":
            return (settings.tradier_paper_token or os.getenv("TRADIER_PAPER_TOKEN") or "").strip()
        return (settings.tradier_live_token or os.getenv("TRADIER_LIVE_TOKEN") or "").strip()

    @staticmethod
    def _resolve_base_url(scope: TradierScope, base_url: str | None) -> str:
        if base_url:
            return base_url.rstrip("/")
        if scope == "paper":
            return settings.tradier_paper_base_url
        return settings.tradier_live_base_url

    @staticmethod
    def _as_list(value: Any) -> list[Any]:
        if value is None:
            return []
        if isinstance(value, str) and value.strip().lower() in {"null", "none", ""}:
            return []
        if isinstance(value, list):
            return value
        return [value]

    @staticmethod
    def _as_dict(value: Any) -> dict[str, Any]:
        if value is None:
            return {}
        if isinstance(value, str) and value.strip().lower() in {"null", "none", ""}:
            return {}
        if isinstance(value, dict):
            return value
        return {}

    def _get(self, path: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        response = self.session.get(
            f"{self.base_url}{path}",
            params=params or {},
            timeout=self.timeout_sec,
        )
        self._raise_for_status(response)
        return response.json()

    def _post(self, path: str, data: dict[str, Any] | None = None) -> dict[str, Any]:
        response = self.session.post(
            f"{self.base_url}{path}",
            data=data or {},
            timeout=self.timeout_sec,
        )
        self._raise_for_status(response)
        return response.json()

    @staticmethod
    def _raise_for_status(response: requests.Response) -> None:
        try:
            response.raise_for_status()
        except requests.HTTPError as exc:
            detail = response.text.strip()
            if not detail:
                try:
                    detail = str(response.json())
                except ValueError:
                    detail = ""
            suffix = f" Tradier detail: {detail[:1000]}" if detail else ""
            raise requests.HTTPError(f"{exc}.{suffix}", response=response) from exc

    def user_profile(self) -> dict[str, Any]:
        return self._get("/user/profile")

    def accounts(self) -> list[dict[str, Any]]:
        payload = self.user_profile()
        accounts = ((payload.get("profile") or {}).get("account")) or []
        return self._as_list(accounts)

    def balances(self, account_id: str) -> dict[str, Any]:
        payload = self._get(f"/accounts/{account_id}/balances")
        return self._as_dict(payload.get("balances"))

    def historical_balances(self, account_id: str) -> list[dict[str, Any]]:
        payload = self._get(f"/accounts/{account_id}/historical-balances")
        history = self._as_dict(payload.get("history")).get("day") or self._as_dict(payload.get("historical_balances")).get("day") or []
        return self._as_list(history)

    def positions(self, account_id: str) -> list[dict[str, Any]]:
        payload = self._get(f"/accounts/{account_id}/positions")
        positions = self._as_dict(payload.get("positions")).get("position") or []
        return self._as_list(positions)

    def orders(self, account_id: str) -> list[dict[str, Any]]:
        payload = self._get(f"/accounts/{account_id}/orders")
        orders = self._as_dict(payload.get("orders")).get("order") or []
        return self._as_list(orders)

    def order(self, account_id: str, order_id: str) -> dict[str, Any]:
        payload = self._get(f"/accounts/{account_id}/orders/{order_id}")
        return self._as_dict(payload.get("order"))

    def place_order(self, account_id: str, data: dict[str, Any]) -> dict[str, Any]:
        payload = self._post(f"/accounts/{account_id}/orders", data)
        return (payload.get("order") or payload)

    def gainloss(self, account_id: str) -> dict[str, Any]:
        payload = self._get(f"/accounts/{account_id}/gainloss")
        return self._as_dict(payload.get("gainloss"))

    def account_history(
        self,
        account_id: str,
        *,
        history_type: str | None = None,
        start: date | None = None,
        end: date | None = None,
    ) -> list[dict[str, Any]]:
        params: dict[str, Any] = {}
        if history_type:
            params["type"] = history_type
        if start:
            params["start"] = start.isoformat()
        if end:
            params["end"] = end.isoformat()
        payload = self._get(f"/accounts/{account_id}/history", params)
        history = self._as_dict(payload.get("history")).get("event") or self._as_dict(payload.get("events")).get("event") or []
        return self._as_list(history)

    def quote(self, symbol: str) -> dict[str, Any]:
        payload = self._get("/markets/quotes", {"symbols": symbol, "greeks": "false"})
        quote = self._as_dict(payload.get("quotes")).get("quote") or {}
        if isinstance(quote, list):
            return quote[0] if quote else {}
        return quote

    def quotes(self, symbols: list[str], *, greeks: bool = False) -> list[dict[str, Any]]:
        payload = self._get("/markets/quotes", {"symbols": ",".join(symbols), "greeks": str(greeks).lower()})
        quotes = self._as_dict(payload.get("quotes")).get("quote") or []
        return self._as_list(quotes)

    @staticmethod
    def _dt_param(value: datetime | date | None) -> str | None:
        if value is None:
            return None
        if isinstance(value, datetime):
            if value.tzinfo is not None:
                value = value.astimezone(ZoneInfo("America/New_York")).replace(tzinfo=None)
            return value.strftime("%Y-%m-%d %H:%M")
        return value.isoformat()

    def history(
        self,
        symbol: str,
        start: date,
        end: date,
        interval: str = "daily",
    ) -> list[dict[str, Any]]:
        payload = self._get(
            "/markets/history",
            {
                "symbol": symbol,
                "interval": interval,
                "start": start.isoformat(),
                "end": end.isoformat(),
            },
        )
        history = self._as_dict(payload.get("history")).get("day") or []
        return history if isinstance(history, list) else [history]

    def timesales(
        self,
        symbol: str,
        *,
        interval: str = "1min",
        start: datetime | date | None = None,
        end: datetime | date | None = None,
        session_filter: str = "open",
    ) -> list[dict[str, Any]]:
        params: dict[str, Any] = {
            "symbol": symbol,
            "interval": interval,
        }
        start_param = self._dt_param(start)
        end_param = self._dt_param(end)
        if start_param:
            params["start"] = start_param
        if end_param:
            params["end"] = end_param
        if session_filter:
            params["session_filter"] = session_filter
        payload = self._get("/markets/timesales", params)
        series = self._as_dict(payload.get("series"))
        data = series.get("data") or payload.get("data") or []
        return self._as_list(data)

    def expirations(self, symbol: str) -> list[str]:
        payload = self._get("/markets/options/expirations", {"symbol": symbol, "includeAllRoots": "true"})
        dates = self._as_dict(payload.get("expirations")).get("date") or []
        return dates if isinstance(dates, list) else [dates]

    def chain(self, symbol: str, expiration: str) -> list[dict[str, Any]]:
        payload = self._get(
            "/markets/options/chains",
            {
                "symbol": symbol,
                "expiration": expiration,
                "greeks": "true",
            },
        )
        options = self._as_dict(payload.get("options")).get("option") or []
        return options if isinstance(options, list) else [options]

    def get_option_expirations(self, symbol: str, scope: str | None = None) -> list[str]:
        del scope
        return self.expirations(symbol)

    def get_option_chain(self, symbol: str, expiration: str, scope: str | None = None) -> list[dict[str, Any]]:
        del scope
        return self.chain(symbol, expiration)


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _pick_spot_price(quote: dict[str, Any]) -> float:
    for key in ("last", "close", "bid", "ask", "prevclose"):
        value = _safe_float(quote.get(key), 0.0)
        if value > 0:
            return value
    raise ValueError("Unable to determine spot price from quote payload")


def _compute_realized_vol(history: list[dict[str, Any]]) -> tuple[float, np.ndarray]:
    closes = np.array([_safe_float(row.get("close"), 0.0) for row in history], dtype=float)
    closes = closes[closes > 0]
    if closes.size < 20:
        raise ValueError("Insufficient history to compute realized volatility")
    log_returns = np.diff(np.log(closes))
    realized_vol = float(np.std(log_returns, ddof=1) * math.sqrt(252))
    return realized_vol, log_returns


def _markov_state(value: float, threshold: float) -> int:
    if value < -threshold:
        return 0  # down
    if value > threshold:
        return 2  # up
    return 1      # flat


def _build_markov_transition(log_returns: np.ndarray) -> tuple[np.ndarray, int, float]:
    daily_sigma = max(float(np.std(log_returns, ddof=1)), 1e-8)
    threshold = max(daily_sigma * 0.35, 1e-4)
    states = np.array([_markov_state(value, threshold) for value in log_returns], dtype=int)
    matrix = np.ones((3, 3), dtype=float)
    for current_state, next_state in zip(states[:-1], states[1:]):
        matrix[current_state, next_state] += 1.0
    matrix = matrix / matrix.sum(axis=1, keepdims=True)
    last_state = int(states[-1])
    return matrix, last_state, threshold


def _nearest_expiration(expirations: list[str], min_dte: int = 14, max_dte: int = 45) -> tuple[str, int]:
    today = datetime.now(timezone.utc).date()
    candidates: list[tuple[str, int]] = []
    for expiration in expirations:
        try:
            exp_date = date.fromisoformat(expiration)
        except ValueError:
            continue
        dte = (exp_date - today).days
        if min_dte <= dte <= max_dte:
            candidates.append((expiration, dte))
    if not candidates:
        future = []
        for expiration in expirations:
            try:
                exp_date = date.fromisoformat(expiration)
            except ValueError:
                continue
            dte = (exp_date - today).days
            if dte > 0:
                future.append((expiration, dte))
        if not future:
            raise ValueError("No future expiration found")
        candidates = future
    return sorted(candidates, key=lambda item: item[1])[0]


def _calendar_expirations(
    expirations: list[str],
    min_dte: int = 14,
    max_dte: int = 45,
    min_gap: int = 7,
) -> tuple[tuple[str, int], tuple[str, int]]:
    today = datetime.now(timezone.utc).date()
    future: list[tuple[str, int]] = []
    for expiration in expirations:
        try:
            exp_date = date.fromisoformat(expiration)
        except ValueError:
            continue
        dte = (exp_date - today).days
        if dte > 0:
            future.append((expiration, dte))
    if len(future) < 2:
        raise ValueError("Calendar spread requires at least two future expirations")
    future.sort(key=lambda item: item[1])
    front = next((item for item in future if min_dte <= item[1] <= max_dte), future[0])
    back = next((item for item in future if item[1] >= front[1] + min_gap), None)
    if back is None:
        later = [item for item in future if item[1] > front[1]]
        if not later:
            raise ValueError("No later expiration found for calendar spread")
        back = later[0]
    return front, back


def _mid_price(contract: dict[str, Any]) -> float:
    bid = _safe_float(contract.get("bid"))
    ask = _safe_float(contract.get("ask"))
    last = _safe_float(contract.get("last"))
    if bid > 0 and ask > 0:
        return (bid + ask) / 2.0
    if ask > 0:
        return ask
    if bid > 0:
        return bid
    return last


def _extract_iv(contract: dict[str, Any]) -> float | None:
    greek_block = contract.get("greeks") or {}
    candidates = [
        greek_block.get("mid_iv"),
        greek_block.get("smv_vol"),
        contract.get("greeks_iv"),
        contract.get("implied_volatility"),
        contract.get("iv"),
    ]
    for candidate in candidates:
        value = _safe_float(candidate, 0.0)
        if value > 0:
            if value > 3:
                return value / 100.0
            return value
    return None


def _normalize_contracts(chain: list[dict[str, Any]], option_type: str) -> list[dict[str, Any]]:
    filtered = [
        contract
        for contract in chain
        if str(contract.get("option_type", "")).lower() == option_type and _safe_float(contract.get("strike")) > 0
    ]
    filtered.sort(key=lambda contract: _safe_float(contract.get("strike")))
    if not filtered:
        raise ValueError(f"No {option_type} contracts found in chain")
    return filtered


def _contract_to_leg(
    contract: dict[str, Any],
    side: Literal["long", "short"],
    expiration: str | None = None,
    dte: int | None = None,
) -> StrategyLeg:
    return StrategyLeg(
        side=side,
        option_type=str(contract.get("option_type", "")).lower() or "call",
        strike=_safe_float(contract.get("strike")),
        premium_mid=_mid_price(contract),
        expiration=expiration,
        dte=dte,
        symbol=str(contract.get("symbol") or ""),
        bid=_safe_float(contract.get("bid")),
        ask=_safe_float(contract.get("ask")),
        volume=_safe_float(contract.get("volume")),
        open_interest=_safe_float(contract.get("open_interest")),
        implied_volatility=_extract_iv(contract),
    )


def _nearest_index(contracts: list[dict[str, Any]], target: float) -> int:
    return min(range(len(contracts)), key=lambda idx: abs(_safe_float(contracts[idx].get("strike")) - target))


def _first_index_ge(contracts: list[dict[str, Any]], target: float) -> int:
    for idx, contract in enumerate(contracts):
        if _safe_float(contract.get("strike")) >= target:
            return idx
    return len(contracts) - 1


def _first_index_gt(contracts: list[dict[str, Any]], target: float) -> int:
    for idx, contract in enumerate(contracts):
        if _safe_float(contract.get("strike")) > target:
            return idx
    return len(contracts) - 1


def _last_index_le(contracts: list[dict[str, Any]], target: float) -> int:
    candidate = 0
    for idx, contract in enumerate(contracts):
        if _safe_float(contract.get("strike")) <= target:
            candidate = idx
        else:
            break
    return candidate


def _last_index_lt(contracts: list[dict[str, Any]], target: float) -> int:
    candidate = 0
    for idx, contract in enumerate(contracts):
        if _safe_float(contract.get("strike")) < target:
            candidate = idx
        else:
            break
    return candidate


def _common_strikes(calls: list[dict[str, Any]], puts: list[dict[str, Any]]) -> list[float]:
    call_strikes = {_safe_float(contract.get("strike")) for contract in calls}
    put_strikes = {_safe_float(contract.get("strike")) for contract in puts}
    strikes = sorted(strike for strike in call_strikes.intersection(put_strikes) if strike > 0)
    if not strikes:
        raise ValueError("No common strikes found between calls and puts")
    return strikes


def _contract_by_strike(contracts: list[dict[str, Any]], strike: float) -> dict[str, Any]:
    for contract in contracts:
        if math.isclose(_safe_float(contract.get("strike")), strike, rel_tol=0.0, abs_tol=1e-8):
            return contract
    raise ValueError(f"No contract found at strike {strike}")


def _consecutive_slice(
    contracts: list[dict[str, Any]],
    *,
    spot: float,
    width: int,
) -> list[dict[str, Any]]:
    if len(contracts) < width:
        raise ValueError(f"Need at least {width} strikes")
    anchor = _first_index_ge(contracts, spot)
    start = max(min(anchor - (width // 2), len(contracts) - width), 0)
    return contracts[start:start + width]


def _build_butterfly_legs(
    contracts: list[dict[str, Any]],
    *,
    spot: float,
    expiration: str,
    dte: int,
    credit: bool,
) -> list[StrategyLeg]:
    body_idx = _nearest_index(contracts, spot)
    low_idx = max(body_idx - 1, 0)
    high_idx = min(body_idx + 1, len(contracts) - 1)
    if low_idx == body_idx or high_idx == body_idx:
        raise ValueError("Insufficient strikes for butterfly")
    wing_side: Literal["long", "short"] = "short" if credit else "long"
    body_side: Literal["long", "short"] = "long" if credit else "short"
    low = _contract_to_leg(contracts[low_idx], wing_side, expiration=expiration, dte=dte)
    body = _contract_to_leg(contracts[body_idx], body_side, expiration=expiration, dte=dte)
    high = _contract_to_leg(contracts[high_idx], wing_side, expiration=expiration, dte=dte)
    return [low, body, _contract_to_leg(contracts[body_idx], body_side, expiration=expiration, dte=dte), high]


def _build_condor_legs(
    contracts: list[dict[str, Any]],
    *,
    spot: float,
    expiration: str,
    dte: int,
    credit: bool,
) -> list[StrategyLeg]:
    window = _consecutive_slice(contracts, spot=spot, width=4)
    wing_side: Literal["long", "short"] = "short" if credit else "long"
    body_side: Literal["long", "short"] = "long" if credit else "short"
    return [
        _contract_to_leg(window[0], wing_side, expiration=expiration, dte=dte),
        _contract_to_leg(window[1], body_side, expiration=expiration, dte=dte),
        _contract_to_leg(window[2], body_side, expiration=expiration, dte=dte),
        _contract_to_leg(window[3], wing_side, expiration=expiration, dte=dte),
    ]


def _build_strategy_legs(
    chain: list[dict[str, Any]],
    spot: float,
    strategy_type: StrategyType,
    expiration: str,
    dte: int,
) -> list[StrategyLeg]:
    calls = _normalize_contracts(chain, "call")
    puts = _normalize_contracts(chain, "put")

    if strategy_type == "long_call":
        call_idx = _first_index_ge(calls, spot)
        return [_contract_to_leg(calls[call_idx], "long", expiration=expiration, dte=dte)]

    if strategy_type == "long_put":
        put_idx = _nearest_index(puts, spot)
        return [_contract_to_leg(puts[put_idx], "long", expiration=expiration, dte=dte)]

    if strategy_type == "covered_call":
        call_idx = _first_index_ge(calls, spot)
        return [_contract_to_leg(calls[call_idx], "short", expiration=expiration, dte=dte)]

    if strategy_type == "cash_secured_put":
        put_idx = _last_index_le(puts, spot)
        return [_contract_to_leg(puts[put_idx], "short", expiration=expiration, dte=dte)]

    if strategy_type == "bull_call_debit_spread":
        long_idx = _first_index_ge(calls, spot)
        short_idx = min(long_idx + 1, len(calls) - 1)
        if short_idx == long_idx:
            raise ValueError("Insufficient strikes for bull_call_debit_spread")
        return [
            _contract_to_leg(calls[long_idx], "long", expiration=expiration, dte=dte),
            _contract_to_leg(calls[short_idx], "short", expiration=expiration, dte=dte),
        ]

    if strategy_type == "bear_put_debit_spread":
        long_idx = _first_index_ge(puts, spot)
        short_idx = max(long_idx - 1, 0)
        if short_idx == long_idx:
            raise ValueError("Insufficient strikes for bear_put_debit_spread")
        return [
            _contract_to_leg(puts[long_idx], "long", expiration=expiration, dte=dte),
            _contract_to_leg(puts[short_idx], "short", expiration=expiration, dte=dte),
        ]

    if strategy_type == "bear_call_credit_spread":
        short_idx = _first_index_ge(calls, spot)
        long_idx = min(short_idx + 1, len(calls) - 1)
        if short_idx == long_idx:
            raise ValueError("Insufficient strikes for bear_call_credit_spread")
        return [
            _contract_to_leg(calls[short_idx], "short", expiration=expiration, dte=dte),
            _contract_to_leg(calls[long_idx], "long", expiration=expiration, dte=dte),
        ]

    if strategy_type == "bull_put_credit_spread":
        short_idx = _last_index_le(puts, spot)
        long_idx = max(short_idx - 1, 0)
        if short_idx == long_idx:
            raise ValueError("Insufficient strikes for bull_put_credit_spread")
        return [
            _contract_to_leg(puts[short_idx], "short", expiration=expiration, dte=dte),
            _contract_to_leg(puts[long_idx], "long", expiration=expiration, dte=dte),
        ]

    if strategy_type == "call_debit_butterfly":
        return _build_butterfly_legs(calls, spot=spot, expiration=expiration, dte=dte, credit=False)

    if strategy_type == "put_debit_butterfly":
        return _build_butterfly_legs(puts, spot=spot, expiration=expiration, dte=dte, credit=False)

    if strategy_type == "call_credit_butterfly":
        return _build_butterfly_legs(calls, spot=spot, expiration=expiration, dte=dte, credit=True)

    if strategy_type == "put_credit_butterfly":
        return _build_butterfly_legs(puts, spot=spot, expiration=expiration, dte=dte, credit=True)

    if strategy_type == "call_debit_condor":
        return _build_condor_legs(calls, spot=spot, expiration=expiration, dte=dte, credit=False)

    if strategy_type == "put_debit_condor":
        return _build_condor_legs(puts, spot=spot, expiration=expiration, dte=dte, credit=False)

    if strategy_type == "call_credit_condor":
        return _build_condor_legs(calls, spot=spot, expiration=expiration, dte=dte, credit=True)

    if strategy_type == "put_credit_condor":
        return _build_condor_legs(puts, spot=spot, expiration=expiration, dte=dte, credit=True)

    if strategy_type == "long_straddle":
        strike = min(_common_strikes(calls, puts), key=lambda value: abs(value - spot))
        return [
            _contract_to_leg(_contract_by_strike(calls, strike), "long", expiration=expiration, dte=dte),
            _contract_to_leg(_contract_by_strike(puts, strike), "long", expiration=expiration, dte=dte),
        ]

    if strategy_type == "long_strangle":
        call_idx = _first_index_gt(calls, spot)
        put_idx = _last_index_lt(puts, spot)
        return [
            _contract_to_leg(calls[call_idx], "long", expiration=expiration, dte=dte),
            _contract_to_leg(puts[put_idx], "long", expiration=expiration, dte=dte),
        ]

    if strategy_type == "iron_condor":
        short_put_idx = _last_index_le(puts, spot)
        long_put_idx = max(short_put_idx - 1, 0)
        short_call_idx = _first_index_ge(calls, spot)
        long_call_idx = min(short_call_idx + 1, len(calls) - 1)
        if short_put_idx == long_put_idx or short_call_idx == long_call_idx:
            raise ValueError("Insufficient strikes for iron_condor")
        return [
            _contract_to_leg(puts[short_put_idx], "short", expiration=expiration, dte=dte),
            _contract_to_leg(puts[long_put_idx], "long", expiration=expiration, dte=dte),
            _contract_to_leg(calls[short_call_idx], "short", expiration=expiration, dte=dte),
            _contract_to_leg(calls[long_call_idx], "long", expiration=expiration, dte=dte),
        ]

    if strategy_type == "iron_butterfly":
        strike = min(_common_strikes(calls, puts), key=lambda value: abs(value - spot))
        atm_put_idx = next(idx for idx, contract in enumerate(puts) if math.isclose(_safe_float(contract.get("strike")), strike))
        atm_call_idx = next(idx for idx, contract in enumerate(calls) if math.isclose(_safe_float(contract.get("strike")), strike))
        wing_put_idx = max(atm_put_idx - 1, 0)
        wing_call_idx = min(atm_call_idx + 1, len(calls) - 1)
        if wing_put_idx == atm_put_idx or wing_call_idx == atm_call_idx:
            raise ValueError("Insufficient strikes for iron_butterfly")
        return [
            _contract_to_leg(puts[atm_put_idx], "short", expiration=expiration, dte=dte),
            _contract_to_leg(calls[atm_call_idx], "short", expiration=expiration, dte=dte),
            _contract_to_leg(puts[wing_put_idx], "long", expiration=expiration, dte=dte),
            _contract_to_leg(calls[wing_call_idx], "long", expiration=expiration, dte=dte),
        ]

    raise ValueError(f"Unsupported strategy_type: {strategy_type}")


def _build_calendar_spread_legs(
    front_chain: list[dict[str, Any]],
    back_chain: list[dict[str, Any]],
    spot: float,
    front_expiration: str,
    front_dte: int,
    back_expiration: str,
    back_dte: int,
    *,
    option_type: Literal["call", "put"] = "call",
) -> list[StrategyLeg]:
    front_contracts = _normalize_contracts(front_chain, option_type)
    back_contracts = _normalize_contracts(back_chain, option_type)
    front_strikes = {_safe_float(contract.get("strike")) for contract in front_contracts}
    back_strikes = {_safe_float(contract.get("strike")) for contract in back_contracts}
    common = sorted(strike for strike in front_strikes.intersection(back_strikes) if strike > 0)
    if not common:
        raise ValueError(f"No common {option_type} strikes found across calendar expirations")
    strike = min(common, key=lambda value: abs(value - spot))
    return [
        _contract_to_leg(_contract_by_strike(back_contracts, strike), "long", expiration=back_expiration, dte=back_dte),
        _contract_to_leg(_contract_by_strike(front_contracts, strike), "short", expiration=front_expiration, dte=front_dte),
    ]


def _build_diagonal_spread_legs(
    front_chain: list[dict[str, Any]],
    back_chain: list[dict[str, Any]],
    spot: float,
    front_expiration: str,
    front_dte: int,
    back_expiration: str,
    back_dte: int,
    *,
    option_type: Literal["call", "put"],
) -> list[StrategyLeg]:
    front_contracts = _normalize_contracts(front_chain, option_type)
    back_contracts = _normalize_contracts(back_chain, option_type)
    if option_type == "call":
        short_idx = _first_index_ge(front_contracts, spot)
        short_strike = _safe_float(front_contracts[short_idx].get("strike"))
        candidate_indices = [idx for idx, contract in enumerate(back_contracts) if _safe_float(contract.get("strike")) < short_strike]
        if not candidate_indices:
            raise ValueError("Insufficient strikes for call_diagonal_debit_spread")
        long_idx = candidate_indices[-1]
    else:
        short_idx = _last_index_le(front_contracts, spot)
        short_strike = _safe_float(front_contracts[short_idx].get("strike"))
        candidate_indices = [idx for idx, contract in enumerate(back_contracts) if _safe_float(contract.get("strike")) > short_strike]
        if not candidate_indices:
            raise ValueError("Insufficient strikes for put_diagonal_debit_spread")
        long_idx = candidate_indices[0]
    return [
        _contract_to_leg(back_contracts[long_idx], "long", expiration=back_expiration, dte=back_dte),
        _contract_to_leg(front_contracts[short_idx], "short", expiration=front_expiration, dte=front_dte),
    ]


def _simulate_terminal_prices(
    spot: float,
    annual_vol: float,
    annual_drift: float,
    dte: int,
    n_paths: int,
    transition_matrix: np.ndarray,
    last_state: int,
    random_seed: int,
) -> np.ndarray:
    rng = np.random.default_rng(random_seed)
    sigma_daily = annual_vol / math.sqrt(252)
    drift_daily = annual_drift / 252
    regime_shift = np.array([-0.40, 0.00, 0.40], dtype=float) * sigma_daily
    prices = np.full(n_paths, float(spot), dtype=float)
    states = np.full(n_paths, int(last_state), dtype=int)

    for _ in range(max(int(dte), 1)):
        next_states = np.empty_like(states)
        for idx, state in enumerate(states):
            next_states[idx] = rng.choice(3, p=transition_matrix[state])
        shock = rng.normal(0.0, 1.0, size=n_paths)
        returns = drift_daily + regime_shift[next_states] + sigma_daily * shock
        prices *= np.exp(returns)
        states = next_states
    return prices


def _leg_intrinsic(terminal_price: float, leg: StrategyLeg) -> float:
    if leg.option_type == "call":
        return max(terminal_price - leg.strike, 0.0)
    return max(leg.strike - terminal_price, 0.0)


def _leg_payoff(terminal_price: float, leg: StrategyLeg) -> float:
    intrinsic = _leg_intrinsic(terminal_price, leg)
    if leg.side == "long":
        return intrinsic - leg.premium_mid
    return leg.premium_mid - intrinsic


def _standard_normal_cdf(value: float) -> float:
    return 0.5 * (1.0 + math.erf(value / math.sqrt(2.0)))


def _black_scholes_price(
    spot: float,
    strike: float,
    time_years: float,
    annual_vol: float,
    option_type: Literal["call", "put"],
    risk_free_rate: float = 0.0,
) -> float:
    if time_years <= 0 or annual_vol <= 0:
        intrinsic = max(spot - strike, 0.0) if option_type == "call" else max(strike - spot, 0.0)
        return intrinsic
    sigma_sqrt_t = annual_vol * math.sqrt(time_years)
    if sigma_sqrt_t <= 0:
        intrinsic = max(spot - strike, 0.0) if option_type == "call" else max(strike - spot, 0.0)
        return intrinsic
    d1 = (math.log(max(spot, 1e-9) / max(strike, 1e-9)) + (risk_free_rate + 0.5 * annual_vol ** 2) * time_years) / sigma_sqrt_t
    d2 = d1 - sigma_sqrt_t
    if option_type == "call":
        return spot * _standard_normal_cdf(d1) - strike * math.exp(-risk_free_rate * time_years) * _standard_normal_cdf(d2)
    return strike * math.exp(-risk_free_rate * time_years) * _standard_normal_cdf(-d2) - spot * _standard_normal_cdf(-d1)


def _net_premium(legs: list[StrategyLeg]) -> float:
    return float(sum(leg.premium_mid if leg.side == "long" else -leg.premium_mid for leg in legs))


def _strategy_payoff(strategy_type: StrategyType, terminal_price: float, legs: list[StrategyLeg], spot: float) -> float:
    if strategy_type in {
        "calendar_spread",
        "call_calendar_spread",
        "put_calendar_spread",
        "call_diagonal_debit_spread",
        "put_diagonal_debit_spread",
    }:
        long_leg = next(leg for leg in legs if leg.side == "long")
        short_leg = next(leg for leg in legs if leg.side == "short")
        remaining_dte = max((long_leg.dte or 0) - (short_leg.dte or 0), 0)
        remaining_years = remaining_dte / 365.0
        long_mark_to_model = _black_scholes_price(
            spot=terminal_price,
            strike=long_leg.strike,
            time_years=remaining_years,
            annual_vol=max(long_leg.implied_volatility or 0.25, 0.05),
            option_type=long_leg.option_type,
        )
        short_expiry_pnl = _leg_payoff(terminal_price, short_leg)
        long_mark_pnl = long_mark_to_model - long_leg.premium_mid
        return short_expiry_pnl + long_mark_pnl
    pnl = float(sum(_leg_payoff(terminal_price, leg) for leg in legs))
    if strategy_type == "covered_call":
        pnl += terminal_price - spot
    return pnl


def _max_adjacent_width(legs: list[StrategyLeg]) -> float:
    strikes = sorted({_safe_float(leg.strike) for leg in legs if _safe_float(leg.strike) > 0})
    if len(strikes) < 2:
        return 0.0
    return max(b - a for a, b in zip(strikes[:-1], strikes[1:]))


def _capital_at_risk(strategy_type: StrategyType, legs: list[StrategyLeg], spot: float) -> float:
    net_debit = _net_premium(legs)
    if strategy_type in {
        "long_call",
        "long_put",
        "bull_call_debit_spread",
        "bear_put_debit_spread",
        "call_debit_butterfly",
        "put_debit_butterfly",
        "call_debit_condor",
        "put_debit_condor",
        "long_straddle",
        "long_strangle",
        "calendar_spread",
        "call_calendar_spread",
        "put_calendar_spread",
        "call_diagonal_debit_spread",
        "put_diagonal_debit_spread",
    }:
        return max(net_debit, 1e-6)
    if strategy_type == "covered_call":
        return max(spot + min(net_debit, 0.0), 1e-6)
    if strategy_type == "cash_secured_put":
        short_put = next(leg for leg in legs if leg.option_type == "put" and leg.side == "short")
        return max(short_put.strike + min(net_debit, 0.0), 1e-6)
    if strategy_type in {
        "bear_call_credit_spread",
        "bull_put_credit_spread",
        "call_credit_butterfly",
        "put_credit_butterfly",
        "call_credit_condor",
        "put_credit_condor",
    }:
        width = _max_adjacent_width(legs)
        return max(width + min(net_debit, 0.0), 1e-6)
    if strategy_type in {"iron_condor", "iron_butterfly"}:
        call_legs = sorted((leg for leg in legs if leg.option_type == "call"), key=lambda leg: leg.strike)
        put_legs = sorted((leg for leg in legs if leg.option_type == "put"), key=lambda leg: leg.strike)
        call_width = call_legs[-1].strike - call_legs[0].strike
        put_width = put_legs[-1].strike - put_legs[0].strike
        return max(max(call_width, put_width) + min(net_debit, 0.0), 1e-6)
    return max(abs(net_debit), 1e-6)


def _evaluate_probability(
    *,
    symbol: str,
    strategy_type: StrategyType,
    client: TradierClient,
    quote: dict[str, Any],
    log_returns: np.ndarray,
    realized_vol: float,
    transition_matrix: np.ndarray,
    last_state: int,
    state_threshold: float,
    legs: list[StrategyLeg],
    dte: int,
    n_paths: int,
    random_seed: int,
) -> WinningProbabilityResult:
    spot = _pick_spot_price(quote)
    lead_leg = legs[0]
    implied_vols = [leg.implied_volatility for leg in legs if leg.implied_volatility is not None]
    average_implied_vol = float(sum(implied_vols) / len(implied_vols)) if implied_vols else None
    blended_vol = realized_vol if average_implied_vol is None else (realized_vol * 0.45 + average_implied_vol * 0.55)
    annual_drift = float(np.mean(log_returns) * 252)
    terminal_prices = _simulate_terminal_prices(
        spot=spot,
        annual_vol=blended_vol,
        annual_drift=annual_drift,
        dte=dte,
        n_paths=n_paths,
        transition_matrix=transition_matrix,
        last_state=last_state,
        random_seed=random_seed,
    )

    payoffs = np.array([_strategy_payoff(strategy_type, terminal_price, legs, spot) for terminal_price in terminal_prices], dtype=float)
    win_rate_pct = float((payoffs > 0).mean() * 100.0)
    net_premium = _net_premium(legs)
    capital_at_risk = _capital_at_risk(strategy_type, legs, spot)
    expected_pnl = float(np.mean(payoffs))
    expected_roi_pct = float((expected_pnl / max(capital_at_risk, 1e-6)) * 100.0)
    horizon_dte = max(min((leg.dte for leg in legs if leg.dte is not None), default=dte), 1)

    return WinningProbabilityResult(
        symbol=symbol,
        strategy_type=strategy_type,
        win_rate_pct=win_rate_pct,
        expected_pnl=expected_pnl,
        expected_roi_pct=expected_roi_pct,
        net_premium=net_premium,
        selected_contract={
            "symbol": lead_leg.symbol,
            "expiration": lead_leg.expiration,
            "dte": lead_leg.dte,
            "option_type": lead_leg.option_type,
            "strike": lead_leg.strike,
            "premium_mid": lead_leg.premium_mid,
            "bid": lead_leg.bid,
            "ask": lead_leg.ask,
            "volume": lead_leg.volume,
            "open_interest": lead_leg.open_interest,
            "implied_volatility": lead_leg.implied_volatility,
        },
        selected_legs=[leg.to_dict() for leg in legs],
        market_snapshot={
            "spot": spot,
            "quote": {
                "last": _safe_float(quote.get("last")),
                "bid": _safe_float(quote.get("bid")),
                "ask": _safe_float(quote.get("ask")),
                "change_percentage": _safe_float(quote.get("change_percentage")),
                "prevclose": _safe_float(quote.get("prevclose")),
            },
            "realized_volatility": realized_vol,
            "blended_volatility": blended_vol,
            "average_implied_volatility": average_implied_vol,
        },
        markov_snapshot={
            "states": ["down", "flat", "up"],
            "last_state_index": last_state,
            "transition_matrix": transition_matrix.round(4).tolist(),
            "state_threshold": state_threshold,
        },
        assumptions={
            "pricing_model": "markov_regime_monte_carlo",
            "paths": n_paths,
            "history_days_used": int(log_returns.size + 1),
            "annual_drift_from_history": annual_drift,
            "tradier_base_url": client.base_url,
            "tradier_scope": client.scope,
            "strategy_legs": len(legs),
            "horizon_dte": horizon_dte,
            "premium_mode": "positive=debit negative=credit",
            "note": (
                "Use this as a probability gate, not as a sole execution trigger. "
                "Combine with liquidity, spread, slippage, margin and assignment constraints."
            ),
            "calendar_spread_model": (
                "front-expiry payoff with back-leg mark-to-model"
                if strategy_type in {
                    "calendar_spread",
                    "call_calendar_spread",
                    "put_calendar_spread",
                    "call_diagonal_debit_spread",
                    "put_diagonal_debit_spread",
                }
                else None
            ),
            "supported_strategies": list(SUPPORTED_STRATEGIES),
        },
    )


def get_winning_probability(
    symbol: str,
    strategy_type: StrategyType,
    *,
    account_scope: TradierScope | None = None,
    tradier_token: str | None = None,
    tradier_base_url: str | None = None,
    history_days: int = 252,
    min_dte: int = 14,
    max_dte: int = 45,
    n_paths: int = 10000,
    random_seed: int = 42,
) -> WinningProbabilityResult:
    """Estimate the probability that an options strategy finishes profitable.

    Data sources:
    - Underlying quote: Tradier /markets/quotes
    - Historical pricing: Tradier /markets/history
    - Expirations: Tradier /markets/options/expirations
    - Option chain with IV/Greeks: Tradier /markets/options/chains

    The implementation is intentionally generic. It can be used by:
    - Trading AI as a confirmation signal
    - Atlas Code-Quant as a pre-trade execution gate
    """
    client = TradierClient(scope=account_scope, token=tradier_token, base_url=tradier_base_url)
    quote = client.quote(symbol)

    end_date = datetime.now(timezone.utc).date()
    start_date = end_date - timedelta(days=max(history_days * 2, 400))
    history = client.history(symbol, start=start_date, end=end_date, interval="daily")
    realized_vol, log_returns = _compute_realized_vol(history)
    transition_matrix, last_state, state_threshold = _build_markov_transition(log_returns)
    spot = _pick_spot_price(quote)

    expirations = client.expirations(symbol)
    if strategy_type in {
        "calendar_spread",
        "call_calendar_spread",
        "put_calendar_spread",
        "call_diagonal_debit_spread",
        "put_diagonal_debit_spread",
    }:
        (front_expiration, front_dte), (back_expiration, back_dte) = _calendar_expirations(
            expirations,
            min_dte=min_dte,
            max_dte=max_dte,
        )
        front_chain = client.chain(symbol, front_expiration)
        back_chain = client.chain(symbol, back_expiration)
        if strategy_type in {"calendar_spread", "call_calendar_spread"}:
            legs = _build_calendar_spread_legs(
                front_chain=front_chain,
                back_chain=back_chain,
                spot=spot,
                front_expiration=front_expiration,
                front_dte=front_dte,
                back_expiration=back_expiration,
                back_dte=back_dte,
                option_type="call",
            )
        elif strategy_type == "put_calendar_spread":
            legs = _build_calendar_spread_legs(
                front_chain=front_chain,
                back_chain=back_chain,
                spot=spot,
                front_expiration=front_expiration,
                front_dte=front_dte,
                back_expiration=back_expiration,
                back_dte=back_dte,
                option_type="put",
            )
        elif strategy_type == "call_diagonal_debit_spread":
            legs = _build_diagonal_spread_legs(
                front_chain=front_chain,
                back_chain=back_chain,
                spot=spot,
                front_expiration=front_expiration,
                front_dte=front_dte,
                back_expiration=back_expiration,
                back_dte=back_dte,
                option_type="call",
            )
        else:
            legs = _build_diagonal_spread_legs(
                front_chain=front_chain,
                back_chain=back_chain,
                spot=spot,
                front_expiration=front_expiration,
                front_dte=front_dte,
                back_expiration=back_expiration,
                back_dte=back_dte,
                option_type="put",
            )
        expiration = front_expiration
        dte = front_dte
    else:
        expiration, dte = _nearest_expiration(expirations, min_dte=min_dte, max_dte=max_dte)
        chain = client.chain(symbol, expiration)
        legs = _build_strategy_legs(chain, spot=spot, strategy_type=strategy_type, expiration=expiration, dte=dte)
    return _evaluate_probability(
        symbol=symbol,
        strategy_type=strategy_type,
        client=client,
        quote=quote,
        log_returns=log_returns,
        realized_vol=realized_vol,
        transition_matrix=transition_matrix,
        last_state=last_state,
        state_threshold=state_threshold,
        legs=legs,
        dte=dte,
        n_paths=n_paths,
        random_seed=random_seed,
    )


def estimate_probability_from_legs(
    symbol: str,
    strategy_type: StrategyType,
    legs: list[StrategyLeg],
    *,
    account_scope: TradierScope | None = None,
    tradier_token: str | None = None,
    tradier_base_url: str | None = None,
    history_days: int = 252,
    n_paths: int = 10000,
    random_seed: int = 42,
) -> WinningProbabilityResult:
    """Recalculate winning probability from actual live legs instead of model-selected legs."""
    if not legs:
        raise ValueError("At least one leg is required")
    client = TradierClient(scope=account_scope, token=tradier_token, base_url=tradier_base_url)
    quote = client.quote(symbol)
    end_date = datetime.now(timezone.utc).date()
    start_date = end_date - timedelta(days=max(history_days * 2, 400))
    history = client.history(symbol, start=start_date, end=end_date, interval="daily")
    realized_vol, log_returns = _compute_realized_vol(history)
    transition_matrix, last_state, state_threshold = _build_markov_transition(log_returns)
    horizon_dte = max(min((leg.dte for leg in legs if leg.dte is not None), default=30), 1)
    return _evaluate_probability(
        symbol=symbol,
        strategy_type=strategy_type,
        client=client,
        quote=quote,
        log_returns=log_returns,
        realized_vol=realized_vol,
        transition_matrix=transition_matrix,
        last_state=last_state,
        state_threshold=state_threshold,
        legs=legs,
        dte=horizon_dte,
        n_paths=n_paths,
        random_seed=random_seed,
    )
