from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from pydantic import BaseModel

from .signals import EnsembleWeights


class EventStats(BaseModel):
    decisions: int = 0
    exits: int = 0
    wins: int = 0
    losses: int = 0
    pnl_cents: int = 0
    avg_edge: float = 0.0


class LearningSnapshot(BaseModel):
    event_buckets: dict[str, EventStats] = {}
    global_exits: int = 0
    global_wins: int = 0
    global_losses: int = 0
    global_pnl_cents: int = 0
    suggested_weights: dict[str, float] = {}


@dataclass
class LearningEngine:
    """Aprendizaje incremental por evento para autonomía operativa.

    - Agrupa eventos por bucket (ticker/source simple).
    - Aprende de decisiones (edge promedio) y resultados (PnL/win-loss).
    - Sugiere ajustes de pesos del ensemble con guardrails.
    """

    state_path: Path
    min_exits_for_adjust: int = 20
    _snap: LearningSnapshot = field(default_factory=LearningSnapshot)

    def __post_init__(self) -> None:
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        self._load()

    # ---------------- persistence ----------------
    def _load(self) -> None:
        if not self.state_path.exists():
            return
        try:
            data = json.loads(self.state_path.read_text(encoding="utf-8"))
            self._snap = LearningSnapshot.model_validate(data)
        except Exception:
            self._snap = LearningSnapshot()

    def save(self) -> None:
        self.state_path.write_text(
            json.dumps(self._snap.model_dump(), ensure_ascii=True, indent=2),
            encoding="utf-8",
        )

    # ---------------- ingestion ----------------
    @staticmethod
    def bucket_for(ticker: str) -> str:
        t = (ticker or "").upper()
        if t.startswith("POLY:"):
            return "polymarket"
        if any(k in t for k in ("WEATHER", "TEMP", "RAIN")):
            return "weather"
        if any(k in t for k in ("CPI", "INFL", "FED", "RATE", "ECON")):
            return "macro"
        if any(k in t for k in ("ELECT", "PRES", "SENATE", "HOUSE")):
            return "politics"
        return "kalshi_misc"

    def on_decision(self, ticker: str, edge: float) -> None:
        bucket = self.bucket_for(ticker)
        stats = self._snap.event_buckets.setdefault(bucket, EventStats())
        stats.decisions += 1
        # moving average robusta
        alpha = 0.05
        stats.avg_edge = (1 - alpha) * stats.avg_edge + alpha * float(edge)

    def on_exit(self, ticker: str, pnl_cents: int) -> None:
        bucket = self.bucket_for(ticker)
        stats = self._snap.event_buckets.setdefault(bucket, EventStats())
        stats.exits += 1
        stats.pnl_cents += int(pnl_cents)
        self._snap.global_exits += 1
        self._snap.global_pnl_cents += int(pnl_cents)
        if pnl_cents >= 0:
            stats.wins += 1
            self._snap.global_wins += 1
        else:
            stats.losses += 1
            self._snap.global_losses += 1

    # ---------------- policy ----------------
    def suggest_weights(self, base: EnsembleWeights) -> EnsembleWeights:
        if self._snap.global_exits < self.min_exits_for_adjust:
            return base.normalized()

        win_rate = self._snap.global_wins / max(1, self._snap.global_exits)
        pnl = self._snap.global_pnl_cents

        w = base.normalized()
        # regla simple y estable:
        # - si desempeño pobre: bajar llm, subir micro/markov
        # - si desempeño bueno: subir llm gradualmente
        if win_rate < 0.5 or pnl < 0:
            w.llm = max(0.20, w.llm - 0.05)
            w.micro = min(0.45, w.micro + 0.03)
            w.markov = min(0.35, w.markov + 0.02)
        else:
            w.llm = min(0.55, w.llm + 0.03)
            w.micro = max(0.20, w.micro - 0.02)
            w.markov = max(0.15, w.markov - 0.01)
        w = w.normalized()
        self._snap.suggested_weights = {
            "micro": w.micro,
            "markov": w.markov,
            "llm": w.llm,
            "momentum": w.momentum,
        }
        return w

    def snapshot(self) -> dict:
        return self._snap.model_dump()

