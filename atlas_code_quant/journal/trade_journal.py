"""Trade Journal — F9.

Registro en memoria (con persistencia opcional JSONL) de entradas
``trade_open`` y ``trade_close``. Cada entrada lleva el ``trace_id``
end-to-end para reconstruir el ciclo completo del trade.

Diseño:
- ``record_open(position)`` y ``record_close(position)`` aceptan instancias
  :class:`PaperOpenPosition`.
- ``entries(trace_id)`` filtra por trace_id; ``all_entries()`` devuelve todo.
- Persistencia opcional: si ``log_path`` se pasa, cada entrada se vuelca como
  línea JSON al archivo (append-only).
"""
from __future__ import annotations

import json
import os
import time
import uuid
from dataclasses import dataclass, field, asdict
from typing import Any

from atlas_code_quant.execution.paper_broker import PaperOpenPosition


@dataclass(slots=True)
class JournalEntry:
    """Una entrada del journal: trade_open o trade_close."""

    entry_id: str
    event: str  # trade_open | trade_close
    trace_id: str
    position_id: str
    symbol: str
    strategy: str
    ts: float
    payload: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class TradeJournal:
    """Journal en memoria con persistencia JSONL opcional."""

    def __init__(self, log_path: str | None = None) -> None:
        self._entries: list[JournalEntry] = []
        self.log_path = log_path

    # ── API pública ────────────────────────────────────────────────────────
    def record_open(self, position: PaperOpenPosition) -> JournalEntry:
        entry = JournalEntry(
            entry_id=f"JE-{uuid.uuid4().hex[:10]}",
            event="trade_open",
            trace_id=position.trace_id,
            position_id=position.position_id,
            symbol=position.symbol,
            strategy=position.strategy,
            ts=time.time(),
            payload={
                "qty": position.qty,
                "entry_price": position.entry_price,
                "take_profit_price": position.take_profit_price,
                "stop_loss_price": position.stop_loss_price,
                "time_stop_at": position.time_stop_at,
                "direction": position.direction,
                "plan_max_loss_usd": position.plan_max_loss_usd,
                "plan_notional_usd": position.plan_notional_usd,
            },
        )
        return self._append(entry)

    def record_close(self, position: PaperOpenPosition) -> JournalEntry:
        entry = JournalEntry(
            entry_id=f"JE-{uuid.uuid4().hex[:10]}",
            event="trade_close",
            trace_id=position.trace_id,
            position_id=position.position_id,
            symbol=position.symbol,
            strategy=position.strategy,
            ts=time.time(),
            payload={
                "exit_price": position.exit_price,
                "exit_reason": position.exit_reason,
                "realized_pnl_usd": position.realized_pnl_usd,
                "opened_at": position.opened_at,
                "closed_at": position.closed_at,
                "qty": position.qty,
                "entry_price": position.entry_price,
            },
        )
        return self._append(entry)

    def entries(self, trace_id: str | None = None) -> list[JournalEntry]:
        if not trace_id:
            return list(self._entries)
        return [e for e in self._entries if e.trace_id == trace_id]

    def all_entries(self) -> list[JournalEntry]:
        return list(self._entries)

    def has_complete_cycle(self, trace_id: str) -> bool:
        evs = {e.event for e in self.entries(trace_id)}
        return "trade_open" in evs and "trade_close" in evs

    # ── interno ────────────────────────────────────────────────────────────
    def _append(self, entry: JournalEntry) -> JournalEntry:
        self._entries.append(entry)
        if self.log_path:
            try:
                os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
                with open(self.log_path, "a", encoding="utf-8") as fh:
                    fh.write(json.dumps(entry.to_dict(), ensure_ascii=False) + "\n")
            except Exception:
                # Persistencia no bloqueante
                pass
        return entry
