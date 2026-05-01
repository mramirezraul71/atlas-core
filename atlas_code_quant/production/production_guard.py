# ATLAS-Quant — Módulo 9A: Production Guard
"""Capa de seguridad de producción para trading live.

Responsabilidades:
  1. Verificar que test_runner aprobó el sistema (ready_for_live.json)
  2. Daily loss limit: 2% equity → pausa automática del día
  3. Max position size: 5% del portfolio por orden
  4. Forzar Tradier preview=True antes de cualquier orden real
  5. Double confirmation: voz + círculo físico ×3 + F12 humano
     → requerida SÓLO antes del PRIMER trade live del día

Reset diario: a las 09:30 ET (apertura de mercado).
"""
from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

logger = logging.getLogger("atlas.production.guard")

# ── Constantes de seguridad ───────────────────────────────────────────────────
_DAILY_LOSS_LIMIT_PCT   = float(os.getenv("ATLAS_DAILY_LOSS_PCT",   "2.0"))  # %
_WEEKLY_LOSS_LIMIT_PCT  = float(os.getenv("ATLAS_WEEKLY_LOSS_PCT",  "5.0"))  # %
_MONTHLY_LOSS_LIMIT_PCT = float(os.getenv("ATLAS_MONTHLY_LOSS_PCT", "10.0")) # %
_MAX_POSITION_SIZE_PCT  = float(os.getenv("ATLAS_MAX_POS_PCT",      "5.0"))  # %
_MAX_TRADES_PER_DAY     = int(os.getenv("ATLAS_MAX_TRADES_PER_DAY", "6"))    # trades
_CIRCLE_RADIUS_PX       = 80
_CIRCLE_STEPS           = 36
_DOUBLE_CONFIRM_CIRCLES = 3      # círculos de mouse para confirmación
_F12_WAIT_S             = 30.0   # tiempo máximo esperando F12
_READY_FOR_LIVE_FILE    = Path(os.getenv(
    "ATLAS_READY_FILE",
    "data/operation/ready_for_live.json"
))

# pyautogui / pynput opcionales
try:
    import pyautogui
    pyautogui.FAILSAFE = False
    pyautogui.PAUSE    = 0.02
    _GUI_OK = True
except ImportError:
    _GUI_OK = False

try:
    from pynput import keyboard as _kb
    _PYNPUT_OK = True
except ImportError:
    _PYNPUT_OK = False


@dataclass
class GuardState:
    """Estado del guardián de producción (reset diario en apertura)."""
    date_str:         str   = ""           # "2026-03-22"
    opening_equity:   float = 0.0
    daily_pnl:        float = 0.0
    daily_paused:     bool  = False
    first_live_today: bool  = True         # True → requiere double confirmation
    trades_today:     int   = 0
    blocks_today:     int   = 0
    # Seguimiento semanal/mensual (persiste a través de resets diarios)
    week_str:         str   = ""           # "2026-W12"
    weekly_pnl:       float = 0.0
    weekly_paused:    bool  = False
    month_str:        str   = ""           # "2026-03"
    monthly_pnl:      float = 0.0
    monthly_paused:   bool  = False


class ProductionGuard:
    """Guardián de seguridad para el primer día de trading live.

    Uso::

        guard = ProductionGuard(voice=vf, hid=hid_ctrl)
        ok, reason = guard.check_ready_for_live()
        if not ok:
            raise SystemExit(reason)

        # Antes de cada orden:
        ok, reason = guard.gate_order(order_value=2400, portfolio_value=100_000)
        if not ok:
            logger.warning("Orden bloqueada: %s", reason)
    """

    def __init__(
        self,
        voice=None,
        hid=None,
        daily_loss_pct:   float = _DAILY_LOSS_LIMIT_PCT,
        weekly_loss_pct:  float = _WEEKLY_LOSS_LIMIT_PCT,
        monthly_loss_pct: float = _MONTHLY_LOSS_LIMIT_PCT,
        max_position_pct: float = _MAX_POSITION_SIZE_PCT,
        max_trades_day:   int   = _MAX_TRADES_PER_DAY,
    ) -> None:
        self.voice              = voice
        self.hid                = hid
        self.daily_loss_pct     = daily_loss_pct
        self.weekly_loss_pct    = weekly_loss_pct
        self.monthly_loss_pct   = monthly_loss_pct
        self.max_pos_pct        = max_position_pct
        self.max_trades_day     = max_trades_day
        self._state             = GuardState()
        self._lock              = threading.Lock()
        self._f12_event         = threading.Event()
        self._f12_listener      = None

    # ── Verificación de elegibilidad LIVE ─────────────────────────────────────

    @staticmethod
    def force_paper_mode() -> None:
        """Fuerza ATLAS_MODE=paper y ATLAS_FORCE_LIVE_PREVIEW=true en el proceso actual.

        Llamar al inicio de immediate_start / start_immediate_dynamic para
        garantizar que ningún subcomponente pueda ejecutar órdenes reales.
        """
        import os as _os
        _os.environ["ATLAS_MODE"]               = "paper"
        _os.environ["ATLAS_FORCE_LIVE_PREVIEW"] = "true"
        logger.warning(
            "force_paper_mode(): ATLAS_MODE=paper ATLAS_FORCE_LIVE_PREVIEW=true — "
            "órdenes reales BLOQUEADAS"
        )

    def announce_paper_mode(self) -> None:
        """Anuncia por voz y Telegram que el sistema está en modo PAPER."""
        msg = (
            "Sistema en modo PAPER. Mañana Open Test activo. "
            "Todas las órdenes son simuladas en sandbox Tradier. "
            "Sin riesgo de capital real."
        )
        self._speak(msg)
        self._notify_telegram(
            "🟡 *ProductionGuard — PAPER mode*\n"
            "SIMULACIÓN PAPER — Mañana Open Test\n"
            "Órdenes simuladas | Sin capital real"
        )
        logger.info("[PAPER MODE] %s", msg)

    def check_ready_for_live(self) -> tuple[bool, str]:
        """Lee ready_for_live.json y verifica que el sistema pasó el test runner."""
        for path in [_READY_FOR_LIVE_FILE,
                     Path("reports/ready_for_live.json"),
                     Path("data/ready_for_live.json")]:
            if path.exists():
                try:
                    with open(path, encoding="utf-8") as f:
                        data = json.load(f)
                    if not data.get("ready_for_live", False):
                        return False, (
                            f"Test runner no aprobó LIVE — "
                            f"Sharpe={data.get('sharpe',0):.2f} "
                            f"DD={data.get('max_drawdown_pct',0):.1f}%"
                        )
                    logger.info(
                        "✓ ready_for_live verificado — Sharpe=%.2f DD=%.1f%%",
                        data.get("sharpe", 0), data.get("max_drawdown_pct", 0)
                    )
                    return True, "ok"
                except Exception as exc:
                    return False, f"Error leyendo ready_for_live.json: {exc}"

        return False, (
            f"Archivo ready_for_live.json no encontrado. "
            f"Ejecuta primero: python -m atlas_code_quant.calibration.test_runner"
        )

    # ── Gate de órdenes ───────────────────────────────────────────────────────

    def gate_order(
        self,
        order_value:     float,
        portfolio_value: float,
        equity:          float | None = None,
        is_close:        bool = False,   # True → skip trade-count limit (cierres siempre pasan)
    ) -> tuple[bool, str]:
        """Verifica si una orden puede ejecutarse. Retorna (ok, razón)."""
        with self._lock:
            self._refresh_daily_state(equity or portfolio_value)

            # 1. Pausas activas (diaria / semanal / mensual)
            if self._state.daily_paused:
                self._state.blocks_today += 1
                return False, f"Pausa diaria activa — pérdida diaria supera {self.daily_loss_pct}%"

            if self._state.weekly_paused:
                self._state.blocks_today += 1
                return False, f"Pausa semanal activa — pérdida semanal supera {self.weekly_loss_pct}%"

            if self._state.monthly_paused:
                self._state.blocks_today += 1
                return False, f"Pausa mensual activa — pérdida mensual supera {self.monthly_loss_pct}%"

            ref_equity = max(self._state.opening_equity, 1.0)

            # 2. Daily loss limit
            if equity is not None and self._state.daily_pnl < 0:
                daily_loss_pct = abs(self._state.daily_pnl) / ref_equity * 100
                if daily_loss_pct >= self.daily_loss_pct:
                    self._state.daily_paused = True
                    self._state.blocks_today += 1
                    msg = (
                        f"Daily loss limit: {daily_loss_pct:.1f}% ≥ {self.daily_loss_pct}%. "
                        f"Trading pausado hasta mañana."
                    )
                    logger.warning(msg)
                    self._speak(msg, urgent=True)
                    self._notify_telegram(msg)
                    return False, msg

            # 3. Weekly loss limit
            if self._state.weekly_pnl < 0:
                weekly_loss_pct = abs(self._state.weekly_pnl) / ref_equity * 100
                if weekly_loss_pct >= self.weekly_loss_pct:
                    self._state.weekly_paused = True
                    self._state.blocks_today += 1
                    msg = (
                        f"Weekly loss limit: {weekly_loss_pct:.1f}% ≥ {self.weekly_loss_pct}%. "
                        f"Trading pausado hasta la próxima semana."
                    )
                    logger.warning(msg)
                    self._speak(msg, urgent=True)
                    self._notify_telegram(msg)
                    return False, msg

            # 4. Monthly loss limit
            if self._state.monthly_pnl < 0:
                monthly_loss_pct = abs(self._state.monthly_pnl) / ref_equity * 100
                if monthly_loss_pct >= self.monthly_loss_pct:
                    self._state.monthly_paused = True
                    self._state.blocks_today += 1
                    msg = (
                        f"Monthly loss limit: {monthly_loss_pct:.1f}% ≥ {self.monthly_loss_pct}%. "
                        f"Trading pausado hasta el próximo mes."
                    )
                    logger.warning(msg)
                    self._speak(msg, urgent=True)
                    self._notify_telegram(msg)
                    return False, msg

            # 5. Max trades per day (solo para aperturas)
            if not is_close and self._state.trades_today >= self.max_trades_day:
                self._state.blocks_today += 1
                return False, (
                    f"Máximo de trades diarios alcanzado: "
                    f"{self._state.trades_today}/{self.max_trades_day}"
                )

            # 6. Max position size
            if portfolio_value > 0:
                pos_pct = order_value / portfolio_value * 100
                if pos_pct > self.max_pos_pct:
                    self._state.blocks_today += 1
                    return False, (
                        f"Posición {pos_pct:.1f}% supera máximo {self.max_pos_pct}%. "
                        f"Reduce a ${portfolio_value * self.max_pos_pct / 100:,.0f}"
                    )

        return True, "ok"

    def record_trade_pnl(self, pnl: float) -> None:
        """Registra PnL de trade terminado (diario + semanal + mensual)."""
        with self._lock:
            self._state.daily_pnl    += pnl
            self._state.weekly_pnl   += pnl
            self._state.monthly_pnl  += pnl
            self._state.trades_today += 1

    # ── Double confirmation (primer trade live del día) ────────────────────────

    def require_double_confirmation(self) -> bool:
        """Requiere confirmación doble: voz + círculo ×3 + tecla F12.

        Retorna True si el humano confirmó, False si rechazó o timeout.
        Sólo se llama una vez al día (first_live_today).
        """
        with self._lock:
            if not self._state.first_live_today:
                return True   # ya confirmado hoy

        logger.warning("DOUBLE CONFIRMATION requerida para primer trade live del día")

        self._speak(
            "Atención. ATLAS está listo para ejecutar el primer trade real del día. "
            "Se requiere confirmación física. "
            "El robot moverá el mouse en tres círculos. "
            "Presiona F12 para confirmar o Escape para cancelar.",
            urgent=True
        )
        time.sleep(3.0)

        # Círculos físicos ×3
        for i in range(_DOUBLE_CONFIRM_CIRCLES):
            self._speak(f"Círculo {i+1} de {_DOUBLE_CONFIRM_CIRCLES}")
            confirmed = self._execute_circle()
            if not confirmed:
                self._speak("Error en confirmación física. Cancelado.", urgent=True)
                return False
            time.sleep(0.8)

        # Esperar F12 del humano
        self._speak(
            "Círculos completados. Presiona F12 para confirmar el inicio de trading real. "
            f"Tienes {int(_F12_WAIT_S)} segundos.",
            urgent=True
        )

        confirmed = self._wait_for_f12(timeout=_F12_WAIT_S)

        if confirmed:
            with self._lock:
                self._state.first_live_today = False
            logger.warning("DOUBLE CONFIRMATION APROBADA — trading live autorizado")
            self._speak("Confirmación aceptada. Iniciando trading real.", urgent=True)
            self._notify_telegram("✅ ATLAS: Double confirmation aprobada — trading LIVE iniciado")
        else:
            logger.warning("DOUBLE CONFIRMATION RECHAZADA o timeout")
            self._speak("Confirmación no recibida. Permaneciendo en paper.", urgent=True)

        return confirmed

    def _execute_circle(self) -> bool:
        """Mueve el mouse en un círculo de confirmación."""
        if not _GUI_OK:
            logger.info("pyautogui no disponible — simulando círculo")
            time.sleep(1.2)
            return True
        try:
            cx, cy = pyautogui.position()
            for step in range(_CIRCLE_STEPS + 1):
                angle = 2 * math.pi * step / _CIRCLE_STEPS
                x = int(cx + _CIRCLE_RADIUS_PX * math.cos(angle))
                y = int(cy + _CIRCLE_RADIUS_PX * math.sin(angle))
                pyautogui.moveTo(x, y, duration=0.025)
            pyautogui.moveTo(cx, cy, duration=0.08)
            return True
        except Exception as exc:
            logger.error("Error en círculo: %s", exc)
            return False

    def _wait_for_f12(self, timeout: float = _F12_WAIT_S) -> bool:
        """Espera pulsación de F12 dentro del timeout. Esc = cancel."""
        if not _PYNPUT_OK:
            # Sin pynput: pedir por stdin
            print(f"\n{'='*50}")
            print("⚠  ATLAS QUANT — CONFIRMACIÓN DE TRADING LIVE")
            print(f"{'='*50}")
            print(f"Escribe 'CONFIRMO' y Enter (timeout {int(timeout)}s):")
            try:
                import signal as _sig
                def _timeout_handler(s, f):
                    raise TimeoutError()
                _sig.signal(_sig.SIGALRM, _timeout_handler)
                _sig.alarm(int(timeout))
                resp = input("> ").strip()
                _sig.alarm(0)
                return resp == "CONFIRMO"
            except (TimeoutError, Exception):
                return False

        self._f12_event.clear()
        _cancelled = threading.Event()

        def on_press(key):
            if key == _kb.Key.f12:
                self._f12_event.set()
            elif key == _kb.Key.esc:
                _cancelled.set()
                self._f12_event.set()   # desbloquear espera

        listener = _kb.Listener(on_press=on_press)
        listener.daemon = True
        listener.start()
        self._f12_event.wait(timeout=timeout)
        listener.stop()

        return self._f12_event.is_set() and not _cancelled.is_set()

    # ── Estado diario ─────────────────────────────────────────────────────────

    def _refresh_daily_state(self, current_equity: float) -> None:
        """Reset del estado al inicio de un nuevo día de trading.

        Preserva PnL semanal/mensual y sus flags de pausa a través de días.
        Resetea PnL semanal si cambió la semana ISO; mensual si cambió el mes.
        """
        import datetime as _dt
        today      = _dt.date.today()
        today_str  = today.isoformat()
        week_str   = f"{today.isocalendar()[0]}-W{today.isocalendar()[1]:02d}"
        month_str  = today.strftime("%Y-%m")

        if self._state.date_str == today_str:
            return   # mismo día, nada que resetear

        logger.info("Nuevo día de trading — reset de estado ProductionGuard")

        # Preservar acumuladores semanales/mensuales (reset si cambió la semana/mes)
        prev_weekly_pnl    = self._state.weekly_pnl   if self._state.week_str  == week_str  else 0.0
        prev_monthly_pnl   = self._state.monthly_pnl  if self._state.month_str == month_str else 0.0
        prev_weekly_pause  = self._state.weekly_paused  if self._state.week_str  == week_str  else False
        prev_monthly_pause = self._state.monthly_paused if self._state.month_str == month_str else False

        self._state = GuardState(
            date_str         = today_str,
            opening_equity   = current_equity,
            first_live_today = True,
            week_str         = week_str,
            weekly_pnl       = prev_weekly_pnl,
            weekly_paused    = prev_weekly_pause,
            month_str        = month_str,
            monthly_pnl      = prev_monthly_pnl,
            monthly_paused   = prev_monthly_pause,
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _speak(self, text: str, urgent: bool = False) -> None:
        if self.voice is not None:
            self.voice.speak(text, urgent=urgent)
        else:
            logger.info("TTS: %s", text)

    def _notify_telegram(self, message: str) -> None:
        try:
            from atlas_code_quant.production.telegram_alerts import TelegramAlerts
            TelegramAlerts.send_static(message)
        except Exception:
            pass

    def status(self) -> dict:
        with self._lock:
            s = self._state
            return {
                "date":              s.date_str,
                "daily_pnl":         round(s.daily_pnl, 2),
                "daily_paused":      s.daily_paused,
                "weekly_pnl":        round(s.weekly_pnl, 2),
                "weekly_paused":     s.weekly_paused,
                "monthly_pnl":       round(s.monthly_pnl, 2),
                "monthly_paused":    s.monthly_paused,
                "first_live_today":  s.first_live_today,
                "trades_today":      s.trades_today,
                "blocks_today":      s.blocks_today,
                "opening_equity":    s.opening_equity,
                "max_trades_day":    self.max_trades_day,
            }
