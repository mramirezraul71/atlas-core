"""Módulo PDT-Aware Controller — ATLAS-Quant v0.7

Pattern Day Trader compliance layer para cuentas reales y simuladas.

Regla FINRA PDT:
  - 4+ day trades en 5 días laborables → cuenta marcada como PDT → margin call.
  - Cuentas < $25 000: máximo 3 day trades / ventana de 5 días.
  - Cuentas >= $25 000: sin límite, pero aplicamos límites propios de riesgo.

Un "day trade" = apertura Y cierre del mismo valor el mismo día calendario.

Comportamiento por modo:
  paper : Nunca bloquear. Registrar y simular conteo PDT en logs / diario.
  live  : Bloquear entradas cuando el presupuesto PDT se agota o se acerca
          al límite. Cierres de riesgo (SL/trailing/EOD) siempre permitidos.

Variables de entorno:
  ATLAS_PDT_MAX_DAY_TRADES       int,   default 2  (límite conservador; FINRA = 3)
  ATLAS_PDT_MIN_SCORE_NEAR_LIMIT float, default 0.75  (score mínimo al quedar 1 DT)
  ATLAS_PDT_WINDOW_DAYS          int,   default 5

Uso::

    pdt = PDTController(mode="live", account_id="TRD123456")
    dec = pdt.can_open("AAPL", signal_score=0.82, is_swing=False)
    if not dec.allowed:
        logger.warning("PDT bloqueado: %s", dec.reason)
        return None

    # … ejecutar orden …
    pdt.record_open("AAPL")

    # Al cerrar:
    dec = pdt.can_close("AAPL", is_risk_close=False)
    if dec.allowed:
        # ejecutar cierre
        pdt.record_close("AAPL")
"""

from __future__ import annotations

import datetime
import logging
import os
import threading
import time
from dataclasses import dataclass, field

logger = logging.getLogger("atlas.execution.pdt")

# ── Configuración de entorno ──────────────────────────────────────────────────
_MAX_DAY_TRADES    = int(os.getenv("ATLAS_PDT_MAX_DAY_TRADES",       "2"))
_MIN_SCORE_NEAR    = float(os.getenv("ATLAS_PDT_MIN_SCORE_NEAR_LIMIT", "0.75"))
_WINDOW_DAYS       = int(os.getenv("ATLAS_PDT_WINDOW_DAYS",           "5"))

# Tipos de cierre que siempre se consideran "cierres de riesgo"
# (protectores — nunca bloqueados por PDT):
RISK_CLOSE_KINDS = frozenset({
    "CLOSE_SL",
    "CLOSE_TRAIL",
    "CLOSE_EOD",
    "CLOSE_REVERSE",
})


# ── Estructuras de datos ──────────────────────────────────────────────────────

@dataclass
class PDTDecision:
    """Resultado de una decisión PDT-aware."""
    allowed:              bool
    reason:               str
    day_trades_used:      int
    day_trades_remaining: int
    min_score_required:   float = 0.0    # umbral de calidad exigido (si aplica)
    is_simulated:         bool  = False  # True en modo paper (sin bloqueo real)


@dataclass
class PDTSessionState:
    """Estado de sesión por día (reset a las 00:00 local)."""
    date_str:           str   = ""
    session_dt_count:   int   = 0       # DTs confirmados en esta sesión
    dt_symbols_today:   set   = field(default_factory=set)   # símbolos DT'd hoy
    # symbol → open_timestamp (posiciones abiertas hoy)
    session_opens:      dict  = field(default_factory=dict)
    blocks_by_pdt:      int   = 0       # señales bloqueadas por PDT hoy
    allowed_by_pdt:     int   = 0       # señales permitidas hoy


# ── PDT Controller ────────────────────────────────────────────────────────────

class PDTController:
    """Controlador PDT-aware: gestiona presupuesto de day trades por sesión.

    Integra con `tradier_pdt_ledger.count_intraday_day_trades()` para leer
    el conteo real del broker. El conteo de sesión actúa como fallback.
    """

    def __init__(
        self,
        mode:              str,            # "paper" | "live"
        account_id:        str = "",       # ID de cuenta Tradier (para ledger)
        max_day_trades:    int   = _MAX_DAY_TRADES,
        min_score_near:    float = _MIN_SCORE_NEAR,
        window_days:       int   = _WINDOW_DAYS,
    ) -> None:
        self.mode            = mode.strip().lower()
        self.account_id      = account_id
        self._max_day_trades = max_day_trades
        self._min_score_near = min_score_near
        self._window_days    = window_days
        self._state          = PDTSessionState(date_str=datetime.date.today().isoformat())
        self._lock           = threading.Lock()

        logger.info(
            "PDTController iniciado — modo=%s cuenta=%s max_DT=%d score_near=%.2f",
            self.mode.upper(), account_id or "(sin cuenta)", max_day_trades, min_score_near,
        )

    # ── API principal ─────────────────────────────────────────────────────────

    @staticmethod
    def is_pdt_exempt(symbol: str, asset_class: str = "equity_stock") -> bool:
        """True si el activo está exento de la regla PDT (FINRA Rule 4210).

        Índices broad-based (SPX, NDX, RUT) no son securities para PDT.
        Crypto y futuros tampoco.
        """
        exempt_classes = {"index_option", "crypto", "future", "forex"}
        if asset_class in exempt_classes:
            return True
        # Por símbolo para cuando asset_class no viene informado
        exempt_symbols = {"SPX", "SPXW", "NDX", "RUT", "VIX", "XSP"}
        return symbol.upper() in exempt_symbols

    def can_open(
        self,
        symbol:       str,
        signal_score: float,
        is_swing:     bool = False,
        asset_class:  str  = "equity_stock",
    ) -> PDTDecision:
        """Gate de apertura de posición.

        Args:
            symbol:       Ticker del activo.
            signal_score: Score de la señal (0–1, p.ej. regime.confidence).
            is_swing:     True si la posición se planea mantener overnight
                          (no es un day trade; no consume presupuesto PDT).

        Returns:
            PDTDecision con allowed=True/False y razón detallada.
        """
        with self._lock:
            self._refresh_day()
            dt_used      = self._count_dt_total()
            dt_remaining = max(0, self._max_day_trades - dt_used)

            # Activos exentos de PDT (índices, crypto, futuros, forex)
            if self.is_pdt_exempt(symbol, asset_class):
                self._state.allowed_by_pdt += 1
                return PDTDecision(
                    allowed=True, reason="pdt_exempt_asset_class",
                    day_trades_used=dt_used, day_trades_remaining=dt_remaining,
                )

            # Modo paper: nunca bloquear, solo registrar
            if self.mode == "paper":
                self._state.allowed_by_pdt += 1
                dec = PDTDecision(
                    allowed=True, reason="paper_mode_no_block",
                    day_trades_used=dt_used, day_trades_remaining=dt_remaining,
                    is_simulated=True,
                )
                if dt_remaining <= 0:
                    logger.info(
                        "[PDT PAPER] Apertura %s — límite PDT alcanzado (%d/%d) "
                        "pero en paper se permite. score=%.2f",
                        symbol, dt_used, self._max_day_trades, signal_score,
                    )
                return dec

            # Swings no consumen DT
            if is_swing:
                self._state.allowed_by_pdt += 1
                return PDTDecision(
                    allowed=True, reason="swing_no_pdt_concern",
                    day_trades_used=dt_used, day_trades_remaining=dt_remaining,
                )

            # ── Verificaciones intraday ──────────────────────────────────────

            # 1. Límite PDT agotado
            if dt_remaining <= 0:
                self._state.blocks_by_pdt += 1
                reason = (
                    f"PDT limit agotado — {dt_used}/{self._max_day_trades} DTs "
                    f"usados en ventana de {self._window_days} días"
                )
                logger.warning("[PDT] BLOQUEADO apertura %s: %s", symbol, reason)
                return PDTDecision(
                    allowed=False, reason=reason,
                    day_trades_used=dt_used, day_trades_remaining=0,
                )

            # 2. Símbolo ya DT'd hoy → no repetir en la misma sesión
            if symbol in self._state.dt_symbols_today:
                self._state.blocks_by_pdt += 1
                reason = f"{symbol} ya fue day-traded hoy — evitando scalping repetido"
                logger.warning("[PDT] BLOQUEADO apertura %s: %s", symbol, reason)
                return PDTDecision(
                    allowed=False, reason=reason,
                    day_trades_used=dt_used, day_trades_remaining=dt_remaining,
                )

            # 3. Último DT disponible → exigir score alto
            if dt_remaining == 1 and signal_score < self._min_score_near:
                self._state.blocks_by_pdt += 1
                reason = (
                    f"Último DT disponible — score {signal_score:.2f} "
                    f"< mínimo requerido {self._min_score_near:.2f}"
                )
                logger.warning("[PDT] BLOQUEADO apertura %s: %s", symbol, reason)
                return PDTDecision(
                    allowed=False, reason=reason,
                    day_trades_used=dt_used, day_trades_remaining=dt_remaining,
                    min_score_required=self._min_score_near,
                )

            # ── Apertura permitida ───────────────────────────────────────────
            self._state.allowed_by_pdt += 1
            qualifier = "último" if dt_remaining == 1 else "ok"
            logger.info(
                "[PDT] PERMITIDO apertura %s — DTs=%d/%d (%s) score=%.2f",
                symbol, dt_used, self._max_day_trades, qualifier, signal_score,
            )
            return PDTDecision(
                allowed=True,
                reason=f"pdt_{qualifier}",
                day_trades_used=dt_used,
                day_trades_remaining=dt_remaining,
            )

    def can_close(
        self,
        symbol:        str,
        signal_kind:   str  = "",     # SignalKind value para detectar cierres de riesgo
        is_risk_close: bool = False,  # override explícito
    ) -> PDTDecision:
        """Gate de cierre de posición.

        Cierres de riesgo (SL, trailing, EOD, REVERSE) siempre se permiten.
        Cierres voluntarios (TP, TIME) están sujetos al límite PDT.

        Args:
            symbol:       Ticker.
            signal_kind:  Valor de SignalKind (ej. "CLOSE_SL").
            is_risk_close: Override para marcar como cierre de riesgo.

        Returns:
            PDTDecision.
        """
        with self._lock:
            self._refresh_day()
            dt_used      = self._count_dt_total()
            dt_remaining = max(0, self._max_day_trades - dt_used)

            # Determinar si es cierre de riesgo
            is_risk = is_risk_close or (signal_kind.upper() in RISK_CLOSE_KINDS)

            # Cierres de riesgo: siempre permitidos (stop-loss, emergencia, EOD)
            if is_risk:
                return PDTDecision(
                    allowed=True, reason="risk_close_always_allowed",
                    day_trades_used=dt_used, day_trades_remaining=dt_remaining,
                )

            # Modo paper: permitir sin restricciones
            if self.mode == "paper":
                return PDTDecision(
                    allowed=True, reason="paper_mode_no_block",
                    day_trades_used=dt_used, day_trades_remaining=dt_remaining,
                    is_simulated=True,
                )

            # ¿Es este cierre un day trade?
            open_ts = self._state.session_opens.get(symbol)
            if open_ts is None:
                # Posición no abierta en esta sesión (swing desde días previos)
                return PDTDecision(
                    allowed=True, reason="swing_close_no_dt",
                    day_trades_used=dt_used, day_trades_remaining=dt_remaining,
                )

            open_date = datetime.date.fromtimestamp(open_ts).isoformat()
            today     = datetime.date.today().isoformat()

            if open_date != today:
                # Abierta ayer o antes → cierre es swing, no day trade
                return PDTDecision(
                    allowed=True, reason="overnight_close_no_dt",
                    day_trades_used=dt_used, day_trades_remaining=dt_remaining,
                )

            # Este cierre SÍ crea un day trade
            if dt_remaining <= 0:
                self._state.blocks_by_pdt += 1
                reason = (
                    f"PDT limit agotado — cierre de {symbol} crearía DT "
                    f"({dt_used}/{self._max_day_trades}). Use SL para cierre de riesgo."
                )
                logger.warning("[PDT] BLOQUEADO cierre voluntario %s: %s", symbol, reason)
                return PDTDecision(
                    allowed=False, reason=reason,
                    day_trades_used=dt_used, day_trades_remaining=0,
                )

            return PDTDecision(
                allowed=True, reason="pdt_day_trade_allowed",
                day_trades_used=dt_used, day_trades_remaining=dt_remaining,
            )

    # ── Registro de eventos ───────────────────────────────────────────────────

    def record_open(self, symbol: str, ts: float | None = None) -> None:
        """Registra apertura de posición (llamar tras ejecución exitosa)."""
        with self._lock:
            self._refresh_day()
            open_ts = ts if ts is not None else time.time()
            self._state.session_opens[symbol] = open_ts
            logger.debug("[PDT] Apertura registrada: %s @ %s", symbol,
                         datetime.datetime.fromtimestamp(open_ts).strftime("%H:%M:%S"))

    def record_close(self, symbol: str, ts: float | None = None) -> None:
        """Registra cierre de posición. Si es same-day, cuenta como day trade."""
        with self._lock:
            self._refresh_day()
            close_ts  = ts if ts is not None else time.time()
            close_date = datetime.date.fromtimestamp(close_ts).isoformat()
            today      = datetime.date.today().isoformat()

            open_ts = self._state.session_opens.pop(symbol, None)

            if open_ts is not None:
                open_date = datetime.date.fromtimestamp(open_ts).isoformat()
                if open_date == today:
                    # Day trade completado
                    self._state.dt_symbols_today.add(symbol)
                    self._state.session_dt_count += 1
                    logger.info(
                        "[PDT] Day trade registrado: %s. DTs sesión=%d/%d",
                        symbol, self._state.session_dt_count, self._max_day_trades,
                    )
                    self._notify_if_near_limit()

    # ── Consultas ─────────────────────────────────────────────────────────────

    def day_trades_remaining(self) -> int:
        """DTs disponibles para el resto de la sesión."""
        with self._lock:
            self._refresh_day()
            return max(0, self._max_day_trades - self._count_dt_total())

    def is_pdt_limited(self) -> bool:
        """True si no quedan DTs disponibles."""
        return self.day_trades_remaining() == 0

    def session_summary(self) -> dict:
        """Resumen del estado PDT de la sesión."""
        with self._lock:
            self._refresh_day()
            dt_used = self._count_dt_total()
            return {
                "mode":                  self.mode,
                "date":                  self._state.date_str,
                "day_trades_used":       dt_used,
                "day_trades_remaining":  max(0, self._max_day_trades - dt_used),
                "max_day_trades":        self._max_day_trades,
                "dt_symbols_today":      list(self._state.dt_symbols_today),
                "open_positions_today":  list(self._state.session_opens.keys()),
                "session_dt_count":      self._state.session_dt_count,
                "blocks_by_pdt":         self._state.blocks_by_pdt,
                "allowed_by_pdt":        self._state.allowed_by_pdt,
            }

    # ── Privados ──────────────────────────────────────────────────────────────

    def _refresh_day(self) -> None:
        """Reset de estado al detectar cambio de día (llamar dentro del lock)."""
        today = datetime.date.today().isoformat()
        if self._state.date_str != today:
            prev_date = self._state.date_str
            self._state = PDTSessionState(date_str=today)
            logger.info(
                "[PDT] Nuevo día %s (anterior: %s) — contadores reseteados",
                today, prev_date,
            )

    def _count_dt_total(self) -> int:
        """Cuenta DTs del ledger del broker + sesión actual (usa el mayor)."""
        ledger_count = 0
        if self.account_id:
            try:
                from atlas_code_quant.execution.tradier_pdt_ledger import (
                    count_intraday_day_trades,
                )
                ledger_count = count_intraday_day_trades(
                    self.account_id, n_days=self._window_days
                )
            except Exception as exc:
                logger.debug("[PDT] ledger no disponible: %s", exc)

        # El conteo real del ledger incluye trades de sesiones anteriores.
        # Usamos el máximo para ser conservadores.
        return max(ledger_count, self._state.session_dt_count)

    def _notify_if_near_limit(self) -> None:
        """Alerta cuando quedan 1 o 0 DTs disponibles."""
        dt_used      = self._count_dt_total()
        dt_remaining = self._max_day_trades - dt_used
        if dt_remaining <= 1:
            msg = (
                f"⚠ PDT ALERT: {dt_used}/{self._max_day_trades} day trades usados. "
                f"Quedan {dt_remaining}. "
                f"{'Solo señales de alta calidad.' if dt_remaining == 1 else 'Sin más day trades disponibles.'}"
            )
            logger.warning(msg)
            try:
                from atlas_code_quant.production.telegram_alerts import TelegramAlerts
                TelegramAlerts.send_static(msg)
            except Exception:
                pass
