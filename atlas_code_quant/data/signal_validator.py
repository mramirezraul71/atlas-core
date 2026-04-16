from __future__ import annotations

import json
import math
from dataclasses import dataclass
from datetime import datetime, time
from pathlib import Path
from typing import Any

try:
    from atlas_code_quant.backtesting.winning_probability import SUPPORTED_STRATEGIES
except ModuleNotFoundError:  # pragma: no cover - fallback for cwd=atlas_code_quant
    from backtesting.winning_probability import SUPPORTED_STRATEGIES

try:
    from atlas_code_quant.config.settings import settings
except ModuleNotFoundError:  # pragma: no cover - fallback for cwd=atlas_code_quant
    from config.settings import settings


_DEFAULT_SUPPORTED_STRATEGIES = {
    "equity_long",
    "equity_short",
    "rebuild_equity_closed",
    "rebuild_option_closed",
    *SUPPORTED_STRATEGIES,
}


@dataclass(frozen=True)
class ValidationResult:
    is_valid: bool
    reason: str
    severity: str


class SignalValidator:
    """Rejects corrupted signal and journal payloads before they reach models or storage."""

    def __init__(
        self,
        *,
        state_path: Path | None = None,
        supported_strategies: set[str] | None = None,
    ) -> None:
        self.state_path = state_path or (settings.data_dir.parent / "operation" / "operation_center_state.json")
        self.supported_strategies = {
            str(item or "").strip().lower()
            for item in (supported_strategies or _DEFAULT_SUPPORTED_STRATEGIES)
            if str(item or "").strip()
        }
        self._last_price_by_symbol: dict[str, float] = {}

    def is_enabled(self) -> bool:
        try:
            payload = json.loads(self.state_path.read_text(encoding="utf-8"))
        except Exception:
            return True
        if not isinstance(payload, dict):
            return True
        return bool(payload.get("signal_validator_enabled", True))

    def validate_price_data(
        self,
        symbol: Any,
        price: Any,
        volume: Any,
        bid: Any,
        ask: Any,
    ) -> ValidationResult:
        if not self.is_enabled():
            return ValidationResult(True, "", "warn")

        normalized_symbol = str(symbol or "").strip().upper()
        if not normalized_symbol:
            return ValidationResult(False, "symbol vacío", "error")

        parsed_price = self._to_float(price)
        if parsed_price is None or parsed_price <= 0.0:
            return ValidationResult(False, "price <= 0", "critical")

        parsed_bid = self._to_float(bid)
        parsed_ask = self._to_float(ask)
        if parsed_bid is not None and parsed_ask is not None:
            if parsed_bid > parsed_ask:
                return ValidationResult(False, "bid > ask (crossed market)", "critical")
            spread_base = max(abs(parsed_price), parsed_bid, parsed_ask, 1e-9)
            spread_pct = (parsed_ask - parsed_bid) / spread_base
            if spread_pct > 0.05:
                return ValidationResult(False, f"spread > 5% ({spread_pct:.2%})", "error")

        parsed_volume = self._to_float(volume)
        if parsed_volume is not None and parsed_volume <= 0.0 and self._market_hours_active():
            return ValidationResult(False, "volume = 0 en horario de mercado", "warn")

        previous_price = self._last_price_by_symbol.get(normalized_symbol)
        if previous_price and previous_price > 0.0:
            gap_pct = abs(parsed_price - previous_price) / previous_price
            if gap_pct > 0.15:
                return ValidationResult(False, f"gap vs precio previo > 15% ({gap_pct:.2%})", "warn")

        self._last_price_by_symbol[normalized_symbol] = parsed_price
        return ValidationResult(True, "", "warn")

    def validate_trade_entry(self, entry: Any) -> ValidationResult:
        if not self.is_enabled():
            return ValidationResult(True, "", "warn")

        payload = entry if isinstance(entry, dict) else {}
        symbol = str(payload.get("symbol") or payload.get("underlying") or "").strip().upper()
        if not symbol:
            return ValidationResult(False, "symbol vacío o None", "error")

        strategy_type = str(payload.get("strategy_type") or "").strip().lower()
        if strategy_type not in self.supported_strategies:
            return ValidationResult(False, f"strategy_type no reconocido: {strategy_type or 'empty'}", "error")

        signed_qty = self._extract_signed_qty(payload)
        if signed_qty == 0.0:
            return ValidationResult(False, "signed_qty == 0", "error")

        entry_price = self._to_float(payload.get("entry_price"))
        if entry_price is not None and entry_price <= 0.0:
            return ValidationResult(False, "entry_price <= 0", "critical")

        return ValidationResult(True, "", "warn")

    def validate_journal_write(self, record: Any) -> ValidationResult:
        if not self.is_enabled():
            return ValidationResult(True, "", "warn")

        payload = record if isinstance(record, dict) else {}
        symbol = str(payload.get("symbol") or payload.get("underlying") or "").strip().upper()
        if not symbol:
            return ValidationResult(False, "record sin underlying válido", "error")

        entry_price = self._to_float(payload.get("entry_price"))
        if entry_price is not None and entry_price < 0.0:
            return ValidationResult(False, "record con entry_price < 0", "critical")

        return ValidationResult(True, "", "warn")

    @staticmethod
    def _to_float(value: Any) -> float | None:
        if value in {None, ""}:
            return None
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return None
        if math.isnan(parsed) or math.isinf(parsed):
            return None
        return parsed

    @staticmethod
    def _extract_signed_qty(payload: dict[str, Any]) -> float:
        direct_qty = SignalValidator._to_float(payload.get("signed_qty"))
        if direct_qty not in {None, 0.0}:
            return float(direct_qty)

        positions = payload.get("positions") or payload.get("legs") or []
        if isinstance(positions, list):
            total_abs = 0.0
            for item in positions:
                if not isinstance(item, dict):
                    continue
                qty = SignalValidator._to_float(item.get("signed_qty"))
                if qty is None:
                    continue
                total_abs += abs(qty)
            if total_abs > 0.0:
                return total_abs

        fallback_qty = SignalValidator._to_float(payload.get("quantity"))
        return float(fallback_qty or 0.0)

    @staticmethod
    def _market_hours_active(now: datetime | None = None) -> bool:
        current = now or datetime.now()
        if current.weekday() >= 5:
            return False
        market_open = time(9, 30)
        market_close = time(16, 0)
        return market_open <= current.time() <= market_close
