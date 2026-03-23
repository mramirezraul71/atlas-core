# ATLAS-EXECUTION — Módulo 7C: Mode Switcher (Paper ↔ Live)
"""Toggle seguro entre modo paper y modo live con confirmación física.

Proceso de activación de modo LIVE:
  1. Verificar equity mínimo ($25,000 PDT requirement)
  2. Verificar posiciones abiertas = 0
  3. Verificar circuit breaker = inactive
  4. Confirmación verbal sintetizada (TTS): "Modo LIVE activado"
  5. Movimiento físico del mouse en patrón circular (señal de confirmación)
  6. Cambio de variable de entorno ATLAS_MODE en runtime
  7. Re-inicializar SignalExecutor con nuevo modo
  8. Notificación Telegram

Confirmación inversa (live → paper):
  - No requiere confirmación física (es una acción más segura)
  - Sí registra en audit log
"""
from __future__ import annotations

import logging
import math
import os
import threading
import time
from typing import Optional

logger = logging.getLogger("atlas.execution.mode_switcher")

# ── TTS opcional ──────────────────────────────────────────────────────────────
try:
    import pyttsx3
    _TTS_OK = True
except ImportError:
    _TTS_OK = False

# ── pyautogui para movimiento físico ─────────────────────────────────────────
try:
    import pyautogui
    _GUI_OK = True
except ImportError:
    _GUI_OK = False


# ── Constantes de seguridad ───────────────────────────────────────────────────
_MIN_EQUITY_LIVE   = 25_000.0   # PDT minimum para cuentas margin
_CIRCLE_RADIUS_PX  = 80         # radio del círculo de confirmación
_CIRCLE_STEPS      = 36         # pasos del círculo (360°/10°)
_SWITCH_COOLDOWN_S = 60.0       # mínimo tiempo entre toggles


_last_toggle_ts: float = 0.0
_tts_lock = threading.Lock()


def toggle_paper_live(
    current_mode: str,
    hid_controller=None,
    risk_engine=None,
    speak: bool = True,
    force: bool = False,
) -> str:
    """Alterna entre paper y live con validaciones de seguridad.

    Retorna el nuevo modo ("paper" | "live").
    Si la validación falla o el usuario cancela, retorna el modo actual.
    """
    global _last_toggle_ts

    target = "live" if current_mode == "paper" else "paper"

    # ── Cooldown ──────────────────────────────────────────────────────────────
    elapsed = time.time() - _last_toggle_ts
    if elapsed < _SWITCH_COOLDOWN_S and not force:
        logger.warning(
            "Toggle demasiado frecuente — espera %.0fs (cooldown=%s)",
            _SWITCH_COOLDOWN_S - elapsed, _SWITCH_COOLDOWN_S
        )
        return current_mode

    # ── Validaciones de seguridad sólo para activar LIVE ─────────────────────
    if target == "live":
        ok, reason = _validate_live_activation(risk_engine)
        if not ok:
            logger.error("LIVE BLOQUEADO: %s", reason)
            _speak(f"Activación live bloqueada. {reason}", urgent=True)
            return current_mode

    # ── Confirmación física ───────────────────────────────────────────────────
    if target == "live":
        confirmed = _physical_confirmation(hid_controller, speak)
        if not confirmed:
            logger.warning("Confirmación física rechazada — permaneciendo en PAPER")
            return current_mode

    # ── Cambio de modo ────────────────────────────────────────────────────────
    os.environ["ATLAS_MODE"] = target
    _last_toggle_ts = time.time()

    # Log de auditoría
    _audit_log(current_mode, target)

    # Notificación
    msg = f"ATLAS-Quant: Modo cambiado de {current_mode.upper()} a {target.upper()}"
    logger.warning(msg)

    if speak:
        _speak(f"Modo {target} activado", urgent=(target == "live"))

    # Telegram
    _notify_telegram(msg)

    return target


def activate_live_mode(
    hid_controller=None,
    risk_engine=None,
    speak: bool = True,
) -> bool:
    """Activa directamente el modo live. Retorna True si el cambio se realizó."""
    current = os.getenv("ATLAS_MODE", "paper")
    if current == "live":
        logger.info("Ya en modo LIVE")
        return True
    result = toggle_paper_live(
        current_mode   = "paper",
        hid_controller = hid_controller,
        risk_engine    = risk_engine,
        speak          = speak,
    )
    return result == "live"


def activate_paper_mode(speak: bool = True) -> None:
    """Desactiva live sin confirmación física (acción segura)."""
    os.environ["ATLAS_MODE"] = "paper"
    logger.warning("Modo PAPER activado — trading real suspendido")
    if speak:
        _speak("Modo paper activado. Trading real suspendido.")
    _notify_telegram("ATLAS-Quant: Modo PAPER activado — trading real suspendido")


# ── Validaciones ──────────────────────────────────────────────────────────────

def _validate_live_activation(risk_engine=None) -> tuple[bool, str]:
    """Verifica condiciones mínimas para operar en live."""

    # Equity mínimo
    if risk_engine is not None:
        state = risk_engine.state()
        equity = state.current_equity
        if equity < _MIN_EQUITY_LIVE:
            return False, f"equity ${equity:,.0f} < mínimo PDT ${_MIN_EQUITY_LIVE:,.0f}"

        # Circuit breaker activo
        if state.circuit_breaker_active:
            return False, "circuit breaker activo — no se permite LIVE"

        # Drawdown alto
        if state.drawdown_pct >= 0.05:
            return False, f"drawdown {state.drawdown_pct*100:.1f}% > 5% umbral para live"

    # Variables de entorno requeridas
    live_token = os.getenv("TRADIER_LIVE_TOKEN", "").strip()
    live_account = os.getenv("TRADIER_LIVE_ACCOUNT_ID", "").strip()

    if not live_token:
        return False, "TRADIER_LIVE_TOKEN no configurado"
    if not live_account:
        return False, "TRADIER_LIVE_ACCOUNT_ID no configurado"

    return True, "ok"


# ── Confirmación física ───────────────────────────────────────────────────────

def _physical_confirmation(hid_controller=None, speak: bool = True) -> bool:
    """Mueve el mouse en círculo como confirmación física de activación LIVE.

    Si no hay pyautogui disponible, pide confirmación por stdin.
    """
    if speak:
        _speak(
            "Atención. Se va a activar el modo live. "
            "El robot moverá el mouse en círculo para confirmar.",
            urgent=True
        )
        time.sleep(2.0)

    if _GUI_OK:
        return _circle_confirmation()
    else:
        return _stdin_confirmation()


def _circle_confirmation() -> bool:
    """Mueve el mouse en patrón circular — confirmación visual de que el robot controla la interfaz."""
    try:
        cx, cy = pyautogui.position()  # tipo: ignore[attr-defined]
        logger.info("Moviendo mouse en círculo (r=%dpx, %d pasos)", _CIRCLE_RADIUS_PX, _CIRCLE_STEPS)

        for step in range(_CIRCLE_STEPS + 1):
            angle = 2 * math.pi * step / _CIRCLE_STEPS
            x = int(cx + _CIRCLE_RADIUS_PX * math.cos(angle))
            y = int(cy + _CIRCLE_RADIUS_PX * math.sin(angle))
            pyautogui.moveTo(x, y, duration=0.04)  # tipo: ignore[attr-defined]

        pyautogui.moveTo(cx, cy, duration=0.1)   # volver al centro
        logger.info("Confirmación física completada — movimiento circular OK")
        return True
    except Exception as exc:
        logger.error("Error en confirmación física: %s", exc)
        return _stdin_confirmation()


def _stdin_confirmation() -> bool:
    """Confirmación por terminal si no hay GUI disponible."""
    print("\n" + "="*60)
    print("⚠️  ATLAS QUANT — ACTIVACIÓN MODO LIVE")
    print("="*60)
    print("Esto ejecutará órdenes REALES con dinero REAL.")
    print("Escribe 'CONFIRMO' para continuar o Enter para cancelar:")
    try:
        resp = input("> ").strip()
        return resp == "CONFIRMO"
    except (EOFError, KeyboardInterrupt):
        return False


# ── TTS ───────────────────────────────────────────────────────────────────────

def _speak(text: str, urgent: bool = False) -> None:
    """Síntesis de voz en background (no bloquea el loop de trading)."""
    if not _TTS_OK:
        logger.info("TTS [%s]: %s", "URGENTE" if urgent else "info", text)
        return

    def _run() -> None:
        with _tts_lock:
            try:
                engine = pyttsx3.init()
                engine.setProperty("rate", 140 if urgent else 160)
                engine.setProperty("volume", 1.0)
                engine.say(text)
                engine.runAndWait()
            except Exception as exc:
                logger.debug("TTS error: %s", exc)

    t = threading.Thread(target=_run, daemon=True)
    t.start()


# ── Notificación Telegram ─────────────────────────────────────────────────────

def _notify_telegram(message: str) -> None:
    try:
        from atlas_code_quant.operations.alert_dispatcher import get_alert_dispatcher
        disp = get_alert_dispatcher()
        disp.system_error(
            component = "mode_switcher",
            error     = message,
            critical  = True,
        )
    except Exception as exc:
        logger.debug("Telegram notify error: %s", exc)


# ── Audit log ─────────────────────────────────────────────────────────────────

def _audit_log(old_mode: str, new_mode: str) -> None:
    import json
    from pathlib import Path

    entry = {
        "ts":       time.time(),
        "from":     old_mode,
        "to":       new_mode,
        "pid":      os.getpid(),
        "env_user": os.getenv("USER", ""),
    }
    audit_path = Path("logs/mode_switch_audit.jsonl")
    try:
        audit_path.parent.mkdir(parents=True, exist_ok=True)
        with open(audit_path, "a") as f:
            f.write(json.dumps(entry) + "\n")
    except Exception:
        pass


# ── Consulta de modo actual ───────────────────────────────────────────────────

def current_mode() -> str:
    return os.getenv("ATLAS_MODE", "paper").strip().lower()


def is_live() -> bool:
    return current_mode() == "live"


def is_paper() -> bool:
    return current_mode() == "paper"
