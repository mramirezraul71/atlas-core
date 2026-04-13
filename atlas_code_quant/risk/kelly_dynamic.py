"""Kelly Dinámico — Fase 0 Semana 3.

Conecta el KellySizer con el journal real (trading_journal.sqlite3) para
calcular el tamaño de posición óptimo usando el historial de trades de ATLAS.

Diferencia con kelly_sizer.py (estático):
- Este módulo lee PnLs directamente del journal SQLite
- Segmenta por strategy_type y symbol para Kelly específico por estrategia
- Aplica circuit breakers automáticos basados en drawdown reciente
- Feature flag: kelly_dynamic_enabled en operation_center_state.json
- Se integra con operation_center para exponer el snapshot kelly_dynamic

Uso:
    from atlas_code_quant.risk.kelly_dynamic import KellyDynamic
    kd = KellyDynamic(db_path=settings.journal_db_path)
    result = kd.recommend_size("SPY", "bull_put_credit_spread", capital=50000)
"""
from __future__ import annotations

import json
import logging
import sqlite3
from dataclasses import dataclass, asdict
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Optional

from atlas_code_quant.execution.kelly_sizer import KellySizer, KellyResult

logger = logging.getLogger("quant.risk.kelly_dynamic")

# ── Constantes ────────────────────────────────────────────────────────────────
_DEFAULT_WINDOW_DAYS = 60       # Ventana de historial para calcular Kelly
_DEFENSIVE_MODE_THRESHOLD = 3   # Pérdidas consecutivas → modo defensivo
_DRAWDOWN_REDUCE_THRESHOLD = 0.03   # 3% drawdown acumulado → reducir 50%
_DRAWDOWN_HALT_THRESHOLD = 0.05     # 5% drawdown → circuit breaker total
_FALLBACK_POSITION_PCT = 0.05       # 5% si no hay historial suficiente


@dataclass
class KellyDynamicResult:
    """Resultado completo del sizing dinámico con contexto del journal."""
    symbol: str
    strategy: str
    kelly: KellyResult
    recommended_pct: float          # Porcentaje final recomendado del capital
    recommended_usd: float          # USD a arriesgar dado el capital
    circuit_breaker: str            # "none" | "defensive" | "halted"
    consecutive_losses: int
    recent_drawdown_pct: float
    window_days: int
    note: str

    def to_dict(self) -> dict:
        d = asdict(self)
        d["kelly"] = asdict(self.kelly)
        return d


class KellyDynamic:
    """Position sizer dinámico que lee historial real del journal.

    Args:
        db_path: Ruta al trading_journal.sqlite3.
        window_days: Días de historial a considerar para el cálculo.
        fraction: Fracción de Kelly (0.25 = Quarter-Kelly).
        max_position_pct: Cap máximo por posición.
    """

    def __init__(
        self,
        db_path: Optional[Path] = None,
        window_days: int = _DEFAULT_WINDOW_DAYS,
        fraction: float = 0.25,
        max_position_pct: float = 0.20,
    ) -> None:
        if db_path is None:
            base = Path(__file__).parent.parent
            db_path = base / "data" / "journal" / "trading_journal.sqlite3"
        self.db_path = Path(db_path)
        self.window_days = window_days
        self._sizer = KellySizer(fraction=fraction, max_position_pct=max_position_pct)

    # ── Público ───────────────────────────────────────────────────────────────

    def recommend_size(
        self,
        symbol: str,
        strategy: str,
        capital: float,
        price: float = 1.0,
        atr: Optional[float] = None,
    ) -> KellyDynamicResult:
        """Calcula el tamaño de posición óptimo para un símbolo/estrategia.

        Pasos:
        1. Leer PnLs históricos del journal para esta estrategia
        2. Verificar circuit breakers (drawdown, pérdidas consecutivas)
        3. Calcular Kelly con el historial disponible
        4. Aplicar modificadores de circuit breaker al tamaño final

        Args:
            symbol: Ticker del activo (ej: "SPY").
            strategy: Nombre de la estrategia (ej: "bull_put_credit_spread").
            capital: Capital total disponible en USD.
            price: Precio actual del activo (para calcular unidades).
            atr: ATR actual para stops dinámicos (opcional).

        Returns:
            KellyDynamicResult con sizing recomendado y estado de circuit breakers.
        """
        pnls = self._load_pnls(strategy=strategy, symbol=symbol)
        recent_pnls = self._load_pnls(strategy=strategy, symbol=symbol, days=10)

        # Circuit breakers
        consecutive_losses = self._count_consecutive_losses(strategy=strategy)
        recent_drawdown = self._calculate_recent_drawdown(recent_pnls, capital)

        circuit_breaker = "none"
        modifier = 1.0

        if recent_drawdown >= _DRAWDOWN_HALT_THRESHOLD:
            circuit_breaker = "halted"
            modifier = 0.0
            logger.warning(
                "[KellyDynamic] CIRCUIT BREAKER ACTIVADO — drawdown=%.2f%% >= %.0f%%",
                recent_drawdown * 100, _DRAWDOWN_HALT_THRESHOLD * 100
            )
        elif recent_drawdown >= _DRAWDOWN_REDUCE_THRESHOLD or consecutive_losses >= _DEFENSIVE_MODE_THRESHOLD:
            circuit_breaker = "defensive"
            modifier = 0.5
            logger.info(
                "[KellyDynamic] Modo defensivo — drawdown=%.2f%% losses_consec=%d",
                recent_drawdown * 100, consecutive_losses
            )

        # Calcular Kelly con historial completo
        kelly_result = self._sizer.compute(symbol=symbol, strategy=strategy, pnls=pnls)

        # Aplicar modifier
        final_pct = kelly_result.position_pct * modifier
        recommended_usd = capital * final_pct

        notes = []
        if not pnls:
            notes.append("sin historial — usando default conservador")
        if circuit_breaker == "defensive":
            notes.append(f"modo defensivo (50% reducción)")
        elif circuit_breaker == "halted":
            notes.append(f"HALTED por drawdown {recent_drawdown:.1%}")
        if kelly_result.note:
            notes.append(kelly_result.note)

        result = KellyDynamicResult(
            symbol=symbol,
            strategy=strategy,
            kelly=kelly_result,
            recommended_pct=round(final_pct, 6),
            recommended_usd=round(recommended_usd, 2),
            circuit_breaker=circuit_breaker,
            consecutive_losses=consecutive_losses,
            recent_drawdown_pct=round(recent_drawdown, 6),
            window_days=self.window_days,
            note=" | ".join(notes) if notes else "OK",
        )

        logger.info(
            "[KellyDynamic] %s/%s → pct=%.2f%% usd=%.0f cb=%s (n=%d)",
            symbol, strategy,
            final_pct * 100, recommended_usd,
            circuit_breaker, len(pnls),
        )
        return result

    def get_portfolio_snapshot(self, capital: float) -> dict:
        """Retorna un snapshot del estado Kelly para el portfolio completo."""
        try:
            strategies = self._load_active_strategies()
        except Exception:
            strategies = []

        return {
            "kelly_dynamic_enabled": True,
            "capital": capital,
            "window_days": self.window_days,
            "fraction": self._sizer.fraction,
            "active_strategies": len(strategies),
            "strategies": strategies,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }

    # ── Privado ───────────────────────────────────────────────────────────────

    def _load_pnls(
        self,
        strategy: Optional[str] = None,
        symbol: Optional[str] = None,
        days: Optional[int] = None,
    ) -> list[float]:
        """Carga PnLs del journal filtrado por estrategia, símbolo y ventana."""
        if not self.db_path.exists():
            logger.warning("Journal DB no encontrada: %s", self.db_path)
            return []

        window = days or self.window_days
        cutoff = (datetime.now() - timedelta(days=window)).strftime("%Y-%m-%d")

        conditions = [
            "status = 'closed'",
            "realized_pnl IS NOT NULL",
            f"DATE(entry_time, 'localtime') >= '{cutoff}'",
        ]
        if strategy:
            conditions.append(f"strategy_type = '{strategy}'")
        if symbol:
            conditions.append(f"symbol = '{symbol}'")

        sql = f"SELECT realized_pnl FROM trading_journal WHERE {' AND '.join(conditions)} ORDER BY entry_time"

        try:
            with sqlite3.connect(str(self.db_path), timeout=5) as conn:
                rows = conn.execute(sql).fetchall()
            return [float(r[0]) for r in rows if r[0] is not None]
        except Exception as exc:
            logger.error("Error leyendo PnLs del journal: %s", exc)
            return []

    def _count_consecutive_losses(self, strategy: Optional[str] = None) -> int:
        """Cuenta las pérdidas consecutivas más recientes."""
        if not self.db_path.exists():
            return 0

        conditions = ["status = 'closed'", "realized_pnl IS NOT NULL"]
        if strategy:
            conditions.append(f"strategy_type = '{strategy}'")

        sql = f"""
            SELECT realized_pnl FROM trading_journal
            WHERE {' AND '.join(conditions)}
            ORDER BY entry_time DESC LIMIT 10
        """
        try:
            with sqlite3.connect(str(self.db_path), timeout=5) as conn:
                rows = conn.execute(sql).fetchall()
            count = 0
            for (pnl,) in rows:
                if float(pnl) < 0:
                    count += 1
                else:
                    break
            return count
        except Exception:
            return 0

    def _calculate_recent_drawdown(self, recent_pnls: list[float], capital: float) -> float:
        """Calcula el drawdown acumulado reciente como porcentaje del capital."""
        if not recent_pnls or capital <= 0:
            return 0.0
        total_loss = sum(p for p in recent_pnls if p < 0)
        return abs(total_loss) / capital

    def _load_active_strategies(self) -> list[dict]:
        """Carga estrategias con al menos un trade en la ventana."""
        if not self.db_path.exists():
            return []

        cutoff = (datetime.now() - timedelta(days=self.window_days)).strftime("%Y-%m-%d")
        sql = f"""
            SELECT strategy_type, COUNT(*) as trades,
                   SUM(CASE WHEN realized_pnl > 0 THEN 1 ELSE 0 END) as wins,
                   ROUND(SUM(realized_pnl), 2) as total_pnl
            FROM trading_journal
            WHERE status = 'closed'
              AND DATE(entry_time, 'localtime') >= '{cutoff}'
            GROUP BY strategy_type
            ORDER BY total_pnl DESC
        """
        try:
            with sqlite3.connect(str(self.db_path), timeout=5) as conn:
                rows = conn.execute(sql).fetchall()
            return [
                {
                    "strategy": r[0],
                    "trades": r[1],
                    "wins": r[2],
                    "win_rate": round(r[2] / r[1], 3) if r[1] > 0 else 0.0,
                    "total_pnl": r[3],
                }
                for r in rows
            ]
        except Exception:
            return []
