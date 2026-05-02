"""
state/journal.py — Journal append-only para auditoría y observabilidad.

Persistimos en JSONL (un evento por línea) bajo ``log_dir``:

- ``radar_decisions.jsonl``: cada :class:`BrainDecision` con ensemble
  y readout completo.
- ``radar_orders.jsonl``: ya generado por ``executor_v2`` (audit).
- ``radar_fills.jsonl``: fills individuales con slippage real.
- ``radar_exits.jsonl``: cierres y resultado PnL.
- ``radar_risk.jsonl``: cambios de breaker / kill / safe-mode.
- ``radar_sizing.jsonl``: intentos con tamaño 0 o bloqueo post-gate.
- ``radar_reconcile.jsonl``: resultados periódicos de ``reconcile()`` (live).

El journal es la fuente de verdad para los reportes y para
calcular métricas (PnL bruto/neto, hit-rate, profit factor,
expectancy, drawdown).
"""
from __future__ import annotations

import json
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


class Journal:
    """Wrapper thread-safe para apppend en JSONL."""

    def __init__(self, log_dir: Path) -> None:
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self.max_bytes = 8 * 1024 * 1024
        self.keep_rotated = 5

    def write(self, kind: str, payload: dict[str, Any]) -> None:
        path = self.log_dir / f"radar_{kind}.jsonl"
        line = json.dumps({
            "ts": datetime.now(timezone.utc).isoformat(),
            **payload,
        }, default=str)
        with self._lock:
            self._rotate_if_needed(path)
            with open(path, "a", encoding="utf-8") as f:
                f.write(line + "\n")

    def read(self, kind: str, limit: int = 1000) -> list[dict]:
        path = self.log_dir / f"radar_{kind}.jsonl"
        if not path.exists():
            return []
        rows = []
        with open(path, "r", encoding="utf-8") as f:
            for line in f.readlines()[-limit:]:
                try:
                    rows.append(json.loads(line))
                except Exception:
                    continue
        return rows

    def _rotate_if_needed(self, path: Path) -> None:
        if not path.exists():
            return
        try:
            if path.stat().st_size < self.max_bytes:
                return
            ts = datetime.now(timezone.utc).strftime("%Y%m%d%H%M%S")
            rotated = path.with_suffix(f".jsonl.{ts}")
            path.replace(rotated)
            self._cleanup_rotated(path)
        except Exception:
            return

    def _cleanup_rotated(self, path: Path) -> None:
        pattern = f"{path.name}.*"
        rotated = sorted(path.parent.glob(pattern), key=lambda p: p.stat().st_mtime, reverse=True)
        for old in rotated[self.keep_rotated :]:
            try:
                old.unlink(missing_ok=True)
            except Exception:
                continue
