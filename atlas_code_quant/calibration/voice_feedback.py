# ATLAS-Quant — Módulo 8B: Voice Feedback
"""Síntesis de voz en español neutro para feedback operativo del robot.

Motor: pyttsx3 con backend espeak-ng (óptimo para Jetson ARM64).
Diseño: no bloqueante — cada mensaje corre en thread daemon propio.

Mensajes predefinidos cubriendo los estados críticos del sistema:
  - Activación de modo LIVE
  - Detección y ejecución de señales
  - Gestión de riesgo (drawdown, Kelly)
  - Emergency stop
  - Estado OCR / cámara
  - Self-healing (reparación de subsistemas)
  - Calibración física
  - Resumen de test runner
"""
from __future__ import annotations

import logging
import os
import queue
import threading
import time
from typing import Optional

logger = logging.getLogger("atlas.calibration.voice")

# ── Backend TTS ───────────────────────────────────────────────────────────────
try:
    import pyttsx3
    _TTS_BACKEND = "pyttsx3"
except ImportError:
    pyttsx3 = None  # type: ignore[assignment]
    _TTS_BACKEND = "none"
    logger.warning("pyttsx3 no disponible — voz deshabilitada (instala: pip install pyttsx3)")

# ── Parámetros de voz ─────────────────────────────────────────────────────────
_DEFAULT_RATE    = int(os.getenv("ATLAS_TTS_RATE", "145"))      # palabras/minuto
_URGENT_RATE     = int(os.getenv("ATLAS_TTS_RATE_URGENT", "130"))
_DEFAULT_VOLUME  = float(os.getenv("ATLAS_TTS_VOLUME", "0.95"))
_LANG            = os.getenv("ATLAS_TTS_LANG", "es")            # español


class VoiceFeedback:
    """Capa de síntesis de voz para ATLAS-Quant.

    Uso::

        vf = VoiceFeedback()
        vf.start()                              # inicia worker thread
        vf.speak("Sistema iniciado")
        vf.announce_live_activated()
        vf.announce_signal("AAPL", "BUY", 2)   # "Señal alcista detectada..."
        vf.stop()
    """

    # ── Mensajes predefinidos (español neutro) ─────────────────────────────
    MSG = {
        # Sistema
        "startup":            "Sistema ATLAS-Quant iniciado correctamente.",
        "live_activated":     "Atención. Modo LIVE activado. Trading real en progreso.",
        "paper_activated":    "Modo paper activado. Trading simulado.",
        "emergency_stop":     "Parada de emergencia activada. Todas las órdenes suspendidas.",
        "shutdown":           "Sistema ATLAS-Quant deteniendo. Hasta pronto.",

        # Señales
        "signal_bull":        "Señal alcista detectada en {symbol}. Ejecutando orden de compra.",
        "signal_bear":        "Señal bajista detectada en {symbol}. Ejecutando orden de venta.",
        "signal_exit":        "Cerrando posición en {symbol}. Razón: {reason}.",
        "signal_blocked":     "Señal en {symbol} bloqueada por control de riesgo.",

        # Riesgo
        "drawdown_warning":   "Atención. Drawdown {pct} por ciento. Kelly reducido a modo defensivo.",
        "drawdown_critical":  "Alerta crítica. Drawdown supera umbral máximo. Activando circuit breaker.",
        "kelly_reduced":      "Tamaño de posición reducido. Fracción Kelly al {pct} por ciento.",
        "circuit_breaker":    "Circuit breaker activado. Pausa de {minutes} minutos.",

        # OCR / Cámara
        "ocr_confidence":     "OCR precisión {pct} por ciento. Validación visual activa.",
        "camera_degraded":    "Cámara degradada. Operando sin confirmación visual.",
        "camera_restored":    "Cámara restaurada. Validación visual reactivada.",

        # Healing
        "repair_start":       "Reparando {subsystem}. Estimado {seconds} segundos.",
        "repair_done":        "Reparación de {subsystem} completada. Sistema nominal.",
        "repair_failed":      "Error reparando {subsystem}. Intervención manual requerida.",

        # Calibración
        "calibrate_start":    "Iniciando calibración física. Apunta la cámara al monitor principal.",
        "calibrate_move":     "Moviendo mouse en círculo. Confirma que el robot controla la pantalla.",
        "calibrate_done":     "Calibración completada. Mapa de pantalla guardado.",
        "calibrate_retry":    "Precisión insuficiente. Repitiendo calibración.",

        # Test runner
        "test_start":         "Iniciando test runner. {cycles} ciclos en modo paper.",
        "test_done":          "Test completado. Sharpe {sharpe}. Drawdown máximo {drawdown} por ciento. {verdict}",
        "test_hid":           "Se usaron {count} ejecuciones HID de respaldo durante el test.",
    }

    def __init__(self, enabled: bool = True) -> None:
        self.enabled = enabled and (_TTS_BACKEND != "none")
        self._queue: queue.Queue[Optional[tuple[str, bool]]] = queue.Queue(maxsize=20)
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._engine = None   # creado dentro del worker (thread-local en pyttsx3)

    # ── Ciclo de vida ─────────────────────────────────────────────────────────

    def start(self) -> None:
        """Inicia el worker thread de TTS (no bloquea el loop principal)."""
        if not self.enabled:
            return
        self._running = True
        self._thread  = threading.Thread(
            target=self._worker, daemon=True, name="atlas-tts"
        )
        self._thread.start()
        logger.info("VoiceFeedback iniciado (backend=%s rate=%d)", _TTS_BACKEND, _DEFAULT_RATE)

    def stop(self) -> None:
        self._running = False
        self._queue.put(None)   # sentinel
        if self._thread:
            self._thread.join(timeout=3.0)

    def _worker(self) -> None:
        """Worker que procesa la cola TTS en serie para evitar colisiones."""
        if pyttsx3 is None:
            return

        try:
            engine = pyttsx3.init()
            engine.setProperty("rate",   _DEFAULT_RATE)
            engine.setProperty("volume", _DEFAULT_VOLUME)
            # Intentar voz en español si está disponible
            voices = engine.getProperty("voices")
            for v in voices or []:
                if hasattr(v, "languages") and any(
                    b"es" in (lang if isinstance(lang, bytes) else lang.encode())
                    for lang in (v.languages or [])
                ):
                    engine.setProperty("voice", v.id)
                    break
        except Exception as exc:
            logger.warning("pyttsx3 init error: %s", exc)
            return

        while self._running:
            item = self._queue.get()
            if item is None:
                break
            text, urgent = item
            try:
                rate = _URGENT_RATE if urgent else _DEFAULT_RATE
                engine.setProperty("rate", rate)
                engine.say(text)
                engine.runAndWait()
            except Exception as exc:
                logger.debug("TTS error: %s", exc)

    # ── API pública ───────────────────────────────────────────────────────────

    def speak(self, text: str, urgent: bool = False) -> None:
        """Encola un mensaje de voz. No bloquea si la cola está llena."""
        if not self.enabled or not self._running:
            logger.info("TTS[%s]: %s", "URGENTE" if urgent else "info", text)
            return
        try:
            self._queue.put_nowait((text, urgent))
        except queue.Full:
            logger.debug("TTS queue llena — mensaje descartado: %s", text[:40])

    def _msg(self, key: str, **kwargs) -> str:
        template = self.MSG.get(key, key)
        try:
            return template.format(**kwargs)
        except KeyError:
            return template

    # ── Mensajes semánticos ───────────────────────────────────────────────────

    def announce_startup(self) -> None:
        self.speak(self._msg("startup"))

    def announce_live_activated(self) -> None:
        self.speak(self._msg("live_activated"), urgent=True)

    def announce_paper_activated(self) -> None:
        self.speak(self._msg("paper_activated"))

    def announce_emergency_stop(self) -> None:
        self.speak(self._msg("emergency_stop"), urgent=True)

    def announce_signal(self, symbol: str, side: str, confidence: float) -> None:
        """Anuncia una señal de trading ejecutada."""
        key = "signal_bull" if side.upper() in ("BUY", "LONG") else "signal_bear"
        self.speak(self._msg(key, symbol=symbol), urgent=False)

    def announce_exit(self, symbol: str, reason: str = "objetivo alcanzado") -> None:
        self.speak(self._msg("signal_exit", symbol=symbol, reason=reason))

    def announce_signal_blocked(self, symbol: str) -> None:
        self.speak(self._msg("signal_blocked", symbol=symbol))

    def announce_drawdown(self, pct: float, critical: bool = False) -> None:
        key = "drawdown_critical" if critical else "drawdown_warning"
        self.speak(self._msg(key, pct=f"{pct:.1f}"), urgent=critical)

    def announce_kelly_reduced(self, pct: float) -> None:
        self.speak(self._msg("kelly_reduced", pct=f"{pct*100:.0f}"))

    def announce_circuit_breaker(self, minutes: int = 30) -> None:
        self.speak(self._msg("circuit_breaker", minutes=minutes), urgent=True)

    def announce_ocr_confidence(self, pct: float) -> None:
        self.speak(self._msg("ocr_confidence", pct=f"{pct:.0f}"))

    def announce_camera_degraded(self) -> None:
        self.speak(self._msg("camera_degraded"), urgent=True)

    def announce_camera_restored(self) -> None:
        self.speak(self._msg("camera_restored"))

    def announce_repair(self, subsystem: str, eta_seconds: int) -> None:
        self.speak(self._msg("repair_start", subsystem=subsystem, seconds=eta_seconds))

    def announce_repair_done(self, subsystem: str) -> None:
        self.speak(self._msg("repair_done", subsystem=subsystem))

    def announce_repair_failed(self, subsystem: str) -> None:
        self.speak(self._msg("repair_failed", subsystem=subsystem), urgent=True)

    def announce_calibrate_start(self) -> None:
        self.speak(self._msg("calibrate_start"), urgent=True)

    def announce_calibrate_move(self) -> None:
        self.speak(self._msg("calibrate_move"))

    def announce_calibrate_done(self) -> None:
        self.speak(self._msg("calibrate_done"))

    def announce_calibrate_retry(self) -> None:
        self.speak(self._msg("calibrate_retry"), urgent=True)

    def announce_test_start(self, cycles: int) -> None:
        self.speak(self._msg("test_start", cycles=cycles))

    def announce_test_done(
        self,
        sharpe: float,
        drawdown_pct: float,
        ready_for_live: bool,
    ) -> None:
        verdict = "Listo para modo LIVE." if ready_for_live else "Se recomienda continuar en paper."
        self.speak(
            self._msg(
                "test_done",
                sharpe=f"{sharpe:.2f}",
                drawdown=f"{drawdown_pct:.1f}",
                verdict=verdict,
            ),
            urgent=ready_for_live,
        )

    def announce_test_hid(self, count: int) -> None:
        if count > 0:
            self.speak(self._msg("test_hid", count=count))

    # ── Integración con ErrorMemoryAgent ─────────────────────────────────────

    def make_error_agent_callback(self):
        """Retorna callback compatible con ErrorMemoryAgent para anunciar reparaciones."""
        def _on_repair_action(subsystem: str, action: str, eta_s: int = 10) -> None:
            self.announce_repair(subsystem, eta_s)

        def _on_repair_done(subsystem: str, success: bool) -> None:
            if success:
                self.announce_repair_done(subsystem)
            else:
                self.announce_repair_failed(subsystem)

        return _on_repair_action, _on_repair_done
