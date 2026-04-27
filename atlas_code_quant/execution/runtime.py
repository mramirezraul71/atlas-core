"""Singletons de runtime paper-first compartidos por el endpoint E2E y métricas.

Diseño:
- Una instancia única de ``PaperBroker`` para todo el proceso uvicorn.
- Una instancia única de ``TradeJournal`` (en memoria + JSONL opcional via
  ``ATLAS_JOURNAL_LOG_PATH``).

Se accede únicamente vía las funciones ``get_paper_broker()`` y
``get_trade_journal()`` para permitir reset en tests.
"""
from __future__ import annotations

import os
import threading

from atlas_code_quant.execution.paper_broker import PaperBroker
from atlas_code_quant.journal import TradeJournal

_lock = threading.Lock()
_broker: PaperBroker | None = None
_journal: TradeJournal | None = None


def get_paper_broker() -> PaperBroker:
    global _broker
    with _lock:
        if _broker is None:
            _broker = PaperBroker()
        return _broker


def get_trade_journal() -> TradeJournal:
    global _journal
    with _lock:
        if _journal is None:
            log_path = os.environ.get("ATLAS_JOURNAL_LOG_PATH") or None
            _journal = TradeJournal(log_path=log_path)
        return _journal


def reset_runtime() -> None:
    """Reset usado por tests; no usar en producción."""
    global _broker, _journal
    with _lock:
        _broker = None
        _journal = None
