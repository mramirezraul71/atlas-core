"""Self-Report Semanal Automático — Fase 0 Semana 4.

Genera y envía a Telegram un reporte semanal completo de ATLAS cada domingo.
Incluye: PnL, win rate, profit factor, errores, estado de módulos nuevos
(Signal Validator, VaR, Kelly, Sentiment) y KPIs del Plan Élite Fase 0.

Feature flag: weekly_report_enabled en operation_center_state.json
Schedule: domingo a las 20:00 EDT (el cron se configura por separado)

Uso standalone:
    python -m atlas_code_quant.reports.weekly_report

Uso programático:
    from atlas_code_quant.reports.weekly_report import WeeklyReporter
    reporter = WeeklyReporter(db_path=..., state_path=..., bot_token=..., chat_id=...)
    reporter.send()
"""
from __future__ import annotations

import json
import logging
import os
import sqlite3
import urllib.parse
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Optional

logger = logging.getLogger("quant.reports.weekly_report")

# ── KPIs objetivo Fase 0 ──────────────────────────────────────────────────────
_TARGET_WIN_RATE = 0.40      # 40%
_TARGET_PROFIT_FACTOR = 1.2
_TARGET_TRADES = 100


@dataclass
class WeeklyStats:
    period_start: str
    period_end: str
    total_trades: int
    wins: int
    losses: int
    win_rate: float
    total_pnl: float
    avg_pnl: float
    gross_wins: float
    gross_losses: float
    profit_factor: float
    best_trade: float
    worst_trade: float
    # Histórico acumulado (todos los tiempos)
    lifetime_trades: int
    lifetime_closed: int


class WeeklyReporter:
    """Genera el reporte semanal de ATLAS y lo envía a Telegram.

    Args:
        db_path: Ruta al trading_journal.sqlite3.
        state_path: Ruta al operation_center_state.json.
        bot_token: Token del bot de Telegram.
        chat_id: Chat ID de Telegram.
    """

    def __init__(
        self,
        db_path: Optional[Path] = None,
        state_path: Optional[Path] = None,
        bot_token: Optional[str] = None,
        chat_id: Optional[str] = None,
    ) -> None:
        base = Path(__file__).parent.parent

        self.db_path = Path(db_path) if db_path else base / "data" / "journal" / "trading_journal.sqlite3"
        self.state_path = Path(state_path) if state_path else base / "data" / "operation" / "operation_center_state.json"

        # Leer credenciales de .env si no se pasan explícitamente
        self.bot_token = bot_token or os.getenv("BOT_TOKEN", "")
        self.chat_id = chat_id or os.getenv("CHAT_ID", "")

    # ── Público ───────────────────────────────────────────────────────────────

    def send(self) -> bool:
        """Genera el reporte semanal y lo envía a Telegram.

        Returns:
            True si el envío fue exitoso, False si hubo error.
        """
        try:
            stats = self._compute_weekly_stats()
            state = self._load_state()
            message = self._build_message(stats, state)
            return self._send_telegram(message)
        except Exception as exc:
            logger.error("Error generando reporte semanal: %s", exc)
            return False

    def compute_stats(self) -> WeeklyStats:
        """Expone el cálculo de stats para tests sin necesitar envío."""
        return self._compute_weekly_stats()

    # ── Privado ───────────────────────────────────────────────────────────────

    def _compute_weekly_stats(self) -> WeeklyStats:
        """Calcula estadísticas de la semana actual (lunes a hoy)."""
        now = datetime.now()
        # Semana actual: último lunes hasta hoy
        days_since_monday = now.weekday()
        week_start = (now - timedelta(days=days_since_monday)).replace(
            hour=0, minute=0, second=0, microsecond=0
        )
        week_end = now

        period_start = week_start.strftime("%Y-%m-%d")
        period_end = week_end.strftime("%Y-%m-%d")

        if not self.db_path.exists():
            logger.warning("Journal DB no encontrada: %s", self.db_path)
            return WeeklyStats(
                period_start=period_start, period_end=period_end,
                total_trades=0, wins=0, losses=0, win_rate=0.0,
                total_pnl=0.0, avg_pnl=0.0, gross_wins=0.0, gross_losses=0.0,
                profit_factor=0.0, best_trade=0.0, worst_trade=0.0,
                lifetime_trades=0, lifetime_closed=0,
            )

        with sqlite3.connect(str(self.db_path), timeout=5) as conn:
            # Stats de la semana
            rows = conn.execute("""
                SELECT realized_pnl FROM trading_journal
                WHERE status = 'closed'
                  AND DATE(entry_time, 'localtime') >= ?
                  AND DATE(entry_time, 'localtime') <= ?
            """, (period_start, period_end)).fetchall()

            pnls = [float(r[0]) for r in rows if r[0] is not None]

            # Stats de toda la vida
            lifetime_row = conn.execute(
                "SELECT COUNT(*) FROM trading_journal"
            ).fetchone()
            lifetime_closed_row = conn.execute(
                "SELECT COUNT(*) FROM trading_journal WHERE status='closed'"
            ).fetchone()

        lifetime_total = lifetime_row[0] if lifetime_row else 0
        lifetime_closed = lifetime_closed_row[0] if lifetime_closed_row else 0

        if not pnls:
            return WeeklyStats(
                period_start=period_start, period_end=period_end,
                total_trades=0, wins=0, losses=0, win_rate=0.0,
                total_pnl=0.0, avg_pnl=0.0, gross_wins=0.0, gross_losses=0.0,
                profit_factor=0.0, best_trade=0.0, worst_trade=0.0,
                lifetime_trades=lifetime_total, lifetime_closed=lifetime_closed,
            )

        wins_list = [p for p in pnls if p > 0]
        loss_list = [p for p in pnls if p <= 0]

        total = len(pnls)
        wins = len(wins_list)
        losses = len(loss_list)
        win_rate = wins / total if total > 0 else 0.0

        gross_wins = sum(wins_list)
        gross_losses = abs(sum(loss_list)) if loss_list else 0.0
        profit_factor = gross_wins / gross_losses if gross_losses > 0 else (float("inf") if gross_wins > 0 else 0.0)

        return WeeklyStats(
            period_start=period_start,
            period_end=period_end,
            total_trades=total,
            wins=wins,
            losses=losses,
            win_rate=win_rate,
            total_pnl=round(sum(pnls), 2),
            avg_pnl=round(sum(pnls) / total, 2),
            gross_wins=round(gross_wins, 2),
            gross_losses=round(gross_losses, 2),
            profit_factor=round(profit_factor, 3),
            best_trade=round(max(pnls), 2),
            worst_trade=round(min(pnls), 2),
            lifetime_trades=lifetime_total,
            lifetime_closed=lifetime_closed,
        )

    def _load_state(self) -> dict:
        """Carga el state de OperationCenter."""
        if not self.state_path.exists():
            return {}
        try:
            return json.loads(self.state_path.read_text(encoding="utf-8"))
        except Exception:
            return {}

    def _build_message(self, stats: WeeklyStats, state: dict) -> str:
        """Construye el mensaje de Telegram con el reporte semanal."""
        now = datetime.now(timezone.utc)

        # Indicadores de KPI Fase 0
        wr_icon = "✅" if stats.win_rate >= _TARGET_WIN_RATE else "⚠️"
        pf_icon = "✅" if stats.profit_factor >= _TARGET_PROFIT_FACTOR else "⚠️"
        trades_pct = min(100, int((stats.lifetime_closed / _TARGET_TRADES) * 100))

        # Estado módulos nuevos
        signal_val = "✅ activo" if state.get("signal_validator_enabled") else "⚪ inactivo"
        var_mon = "✅ activo" if state.get("var_monitor_enabled") else "⚪ inactivo"
        kelly_dyn = "✅ activo" if state.get("kelly_dynamic_enabled") else "⚪ inactivo"
        sentiment_src = state.get("sentiment_source", "manual")
        sentiment_score = state.get("sentiment_score", 0.0)
        sentiment_icon = "🟢" if sentiment_score > 0.1 else ("🔴" if sentiment_score < -0.1 else "🟡")

        fail_safe = "🔴 ACTIVO" if state.get("fail_safe_active") else "✅ inactivo"
        errors = state.get("operational_error_count", 0)
        mode = state.get("auton_mode", "unknown")

        # Profit factor display
        if stats.profit_factor == float("inf"):
            pf_str = "∞ (sin pérdidas)"
        elif stats.profit_factor == 0 and stats.total_trades == 0:
            pf_str = "N/A (sin trades)"
        else:
            pf_str = f"{stats.profit_factor:.2f}"

        msg = f"""📊 *ATLAS SELF-REPORT SEMANAL*
{stats.period_start} → {stats.period_end}

🎯 *Rendimiento de la Semana*
Trades: {stats.total_trades} | Ganadores: {stats.wins} | Perdedores: {stats.losses}
Win Rate: {stats.win_rate:.1%} {wr_icon} _(meta: {_TARGET_WIN_RATE:.0%})_
Profit Factor: {pf_str} {pf_icon} _(meta: {_TARGET_PROFIT_FACTOR})_
PnL total: ${stats.total_pnl:+.2f}
PnL promedio: ${stats.avg_pnl:+.2f}
Mejor trade: ${stats.best_trade:+.2f}
Peor trade: ${stats.worst_trade:+.2f}

📈 *Progreso Fase 0*
Trades cerrados (lifetime): {stats.lifetime_closed}/{_TARGET_TRADES} ({trades_pct}%)
▓{'█' * (trades_pct // 10)}{'░' * (10 - trades_pct // 10)} {trades_pct}%

⚙️ *Estado del Sistema*
Modo: `{mode}`
Fail-safe: {fail_safe}
Errores operacionales: {errors}

🔧 *Módulos Élite*
Signal Validator: {signal_val}
VaR Monitor: {var_mon}
Kelly Dinámico: {kelly_dyn}
Sentiment: {sentiment_icon} {sentiment_score:+.3f} _(fuente: {sentiment_src})_

_Generado: {now.strftime('%Y-%m-%d %H:%M')} UTC_
_Meta Fase 0: 100 trades limpios, WR > 40%, PF > 1.2_"""

        return msg

    def _send_telegram(self, message: str) -> bool:
        """Envía el mensaje al bot de Telegram."""
        if not self.bot_token or not self.chat_id:
            logger.error("BOT_TOKEN o CHAT_ID no configurados — no se puede enviar reporte")
            return False

        url = f"https://api.telegram.org/bot{self.bot_token}/sendMessage"
        payload = {
            "chat_id": self.chat_id,
            "text": message,
            "parse_mode": "Markdown",
        }

        try:
            data = urllib.parse.urlencode(payload).encode("utf-8")
            req = urllib.request.Request(url, data=data, method="POST")
            with urllib.request.urlopen(req, timeout=10) as resp:
                result = json.loads(resp.read().decode("utf-8"))
                if result.get("ok"):
                    logger.info("Reporte semanal enviado exitosamente a Telegram")
                    return True
                else:
                    logger.error("Telegram API error: %s", result)
                    return False
        except Exception as exc:
            logger.error("Error enviando reporte a Telegram: %s", exc)
            return False


# ── Entry point standalone ────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys
    from pathlib import Path

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    # Intentar cargar .env si existe
    env_path = Path(__file__).parent.parent.parent / ".env"
    if env_path.exists():
        for line in env_path.read_text().splitlines():
            if "=" in line and not line.startswith("#"):
                k, v = line.split("=", 1)
                os.environ.setdefault(k.strip(), v.strip())

    reporter = WeeklyReporter()
    success = reporter.send()
    sys.exit(0 if success else 1)
