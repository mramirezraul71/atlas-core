"""ATLAS-Quant-Core — Orquestador Principal del Robot de Trading.

Integra los 7 módulos en un loop de operación autónoma:

    M1 Hardware → M2 Pipeline → M3 Régimen → M4 Señales → M5 Riesgo → M6 Healing → M7 Ejecución

Ciclo de operación (cada 5 segundos, via LiveLoop):
  1. Captura frame Insta360 → OCR → visual state  (throttled a 1s si CPU >80%)
  2. Recibe quotes/trades Tradier vía WebSocket
  3. Actualiza indicadores (CVD, IV Rank, técnicos)
  4. Clasifica régimen (XGBoost + LSTM)
  5. Genera señales para cada símbolo del scanner
  6. Calcula posición (Kelly + inv vol)
  7. Ejecuta vía SignalExecutor → API Tradier (o fallback HID)
  8. Monitorea self-healing en background

Ejecución:
    python -m atlas_code_quant.atlas_quant_core              # paper (defecto)
    python -m atlas_code_quant.atlas_quant_core --mode live  # trading real
    # o desde start_quant.bat
"""

from __future__ import annotations

import argparse
import logging
import signal
import time
from pathlib import Path

logger = logging.getLogger("atlas.quant.core")


def _setup_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


class ATLASQuantCore:
    """Cerebro central del robot de trading ATLAS.

    Inicializa todos los módulos y corre el loop de operación autónoma.
    """

    CYCLE_INTERVAL_S = 5.0     # segundos entre ciclos de evaluación
    MIN_OCR_CONFIDENCE = 0.92
    MAX_LATENCY_MS = 200.0

    def __init__(self, mode: str = "paper") -> None:
        # Importaciones lazy para evitar fallos si algún módulo no está instalado
        self._initialized = False
        self._running = False
        self.mode = mode.strip().lower()

        # Módulos (inicializados en setup())
        self.camera = None
        self.hid = None
        self.ros2 = None
        self.stream_client = None
        self.cvd_calc = None
        self.iv_rank = None
        self.tech_indicators: dict = {}
        self.regime_clf = None
        self.signal_gen = None
        self.risk_engine = None
        self.error_agent = None
        self.healer = None

        # Módulo 7: Ejecución (inicializados en setup())
        self.executor = None
        self.live_loop = None

        # Estado
        self._symbols: list[str] = []
        self._latest_quotes: dict[str, object] = {}
        self._cycle_count = 0

    # ── Setup ──────────────────────────────────────────────────────────────────

    def setup(self) -> None:
        """Inicializa todos los módulos. Debe llamarse antes de start()."""
        from atlas_code_quant.config.settings import settings
        from atlas_code_quant.hardware.camera_interface import CameraInterface
        from atlas_code_quant.hardware.control_interface import HIDController, TradierWebController
        from atlas_code_quant.hardware.ros2_bridge import ATLASRos2Bridge
        from atlas_code_quant.pipeline.tradier_stream import TradierStreamClient
        from atlas_code_quant.pipeline.indicators import (
            CVDCalculator, IVRankCalculator, TechnicalIndicators
        )
        from atlas_code_quant.models.regime_classifier import RegimeClassifier
        from atlas_code_quant.strategy.signal_generator import SignalGenerator
        from atlas_code_quant.strategy.visual_triggers import VisualTriggerValidator
        from atlas_code_quant.risk.kelly_engine import KellyRiskEngine
        from atlas_code_quant.healing.error_agent import ErrorMemoryAgent
        from atlas_code_quant.healing.self_healing import SelfHealingOrchestrator
        from atlas_code_quant.execution.signal_executor import SignalExecutor
        from atlas_code_quant.execution.live_loop import LiveLoop

        logger.info("=== ATLAS-Quant-Core SETUP — modo=%s ===", self.mode.upper())

        self._symbols = settings.scanner_universe[:20]  # límite inicial

        # ── Módulo 1: Hardware ────────────────────────────────────────────────
        rtmp_url = "rtmp://192.168.1.10/live/atlas"  # ajustar a IP de la Insta360
        self.camera = CameraInterface(rtmp_url=rtmp_url, use_gpu=True)
        self.camera.on_degraded(self._on_camera_degraded)

        self.hid = HIDController(armed=False)   # papel por defecto
        self.hid.start_hotkey_listener()

        self.ros2 = ATLASRos2Bridge()
        self.ros2.on_emergency_stop(self._emergency_stop)
        self.ros2.start()

        logger.info("✓ Módulo 1 (Hardware) inicializado")

        # ── Módulo 2: Pipeline Tradier ────────────────────────────────────────
        # Usar token live solo si el modo lo requiere Y el token está configurado
        if self.mode == "live":
            import os
            live_token = os.getenv("TRADIER_LIVE_TOKEN", "").strip()
            token = live_token if live_token else settings.tradier_paper_token
            sandbox = not bool(live_token)
            if not live_token:
                logger.warning("TRADIER_LIVE_TOKEN no configurado — usando sandbox paper")
        else:
            token   = settings.tradier_paper_token
            sandbox = True

        self.stream_client = TradierStreamClient(
            token=token,
            sandbox=sandbox,
        )
        self.stream_client.subscribe(self._symbols)
        self.stream_client.on_quote(self._on_quote)
        self.stream_client.on_trade(self._on_trade)
        self.stream_client.on_error(self._on_stream_error)
        self.stream_client.connect()

        self.cvd_calc = CVDCalculator()
        self.iv_rank  = IVRankCalculator()
        for sym in self._symbols:
            self.tech_indicators[sym] = TechnicalIndicators(sym)

        logger.info("✓ Módulo 2 (Pipeline) inicializado — %d símbolos", len(self._symbols))

        # ── Módulo 3: Clasificador de Régimen ─────────────────────────────────
        self.regime_clf = RegimeClassifier(use_gpu=True)
        self.regime_clf.load_or_train(symbols=self._symbols[:5])
        logger.info("✓ Módulo 3 (Régimen ML) inicializado")

        # ── Módulo 4: Generador de Señales ────────────────────────────────────
        self.signal_gen = SignalGenerator()
        self.visual_trigger = VisualTriggerValidator()
        logger.info("✓ Módulo 4 (Señales) inicializado")

        # ── Módulo 5: Risk Engine ─────────────────────────────────────────────
        self.risk_engine = KellyRiskEngine(initial_capital=100_000.0)
        logger.info("✓ Módulo 5 (Risk Engine) inicializado")

        # ── Módulo 6: Self-Healing ────────────────────────────────────────────
        self.error_agent = ErrorMemoryAgent()
        self.error_agent.initialize()

        self.healer = SelfHealingOrchestrator(
            memory_agent=self.error_agent,
            alert_callback=self._send_alert,
        )

        # Registrar checks de subsistemas
        self.healer.register_check(
            "camera",
            health_fn=lambda: (
                self.camera is not None and
                self.camera.get_ocr_confidence() >= self.MIN_OCR_CONFIDENCE
            ),
            error_type="ocr_confidence_low",
            critical=False,
        )
        self.healer.register_check(
            "tradier_stream",
            health_fn=lambda: (
                self.stream_client is not None and
                self.stream_client._running and
                self.stream_client._reconnect_count < 5
            ),
            error_type="stream_latency_high",
            critical=True,
        )
        self.healer.register_check(
            "risk_engine",
            health_fn=lambda: (
                self.risk_engine is not None and
                not self.risk_engine.state().circuit_breaker_active
            ),
            error_type="drawdown_exceeded",
            critical=True,
        )

        self.healer.start()
        logger.info("✓ Módulo 6 (Self-Healing) inicializado")

        # ── Módulo 7: Ejecución ───────────────────────────────────────────────
        self.executor = SignalExecutor(
            mode           = self.mode,
            hid_controller = self.hid,
            error_agent    = self.error_agent,
        )
        self.live_loop = LiveLoop(
            camera          = self.camera,
            stream          = self.stream_client,
            tech_indicators = self.tech_indicators,
            cvd_calc        = self.cvd_calc,
            iv_rank         = self.iv_rank,
            regime_clf      = self.regime_clf,
            signal_gen      = self.signal_gen,
            risk_engine     = self.risk_engine,
            executor        = self.executor,
            healer          = self.healer,
            ros2            = self.ros2,
            hid             = self.hid,
            symbols         = self._symbols,
            mode            = self.mode,
        )
        logger.info(
            "✓ Módulo 7 (Ejecución) inicializado — modo=%s executor=%s",
            self.mode.upper(), self.executor.__class__.__name__,
        )

        # Iniciar cámara
        self.camera.start()
        logger.info("✓ Cámara Insta360 iniciada")

        self._initialized = True
        logger.info("=== ATLAS-Quant-Core LISTO — %s ===", self.mode.upper())

    # ── Loop de operación ─────────────────────────────────────────────────────

    def start(self) -> None:
        """Inicia el loop de trading autónomo vía LiveLoop. Bloquea hasta señal de parada."""
        if not self._initialized:
            self.setup()

        self._running = True
        logger.info(
            "▶ ATLAS-Quant iniciando — modo=%s símbolos=%d",
            self.mode.upper(), len(self._symbols)
        )

        # LiveLoop maneja SIGINT/SIGTERM internamente
        self.live_loop.start()

        try:
            self.live_loop.join()   # bloquea hasta que el loop termine
        except KeyboardInterrupt:
            self.live_loop.stop("KeyboardInterrupt")
        finally:
            self._shutdown()

    # ── Auditoría Final (Módulo 9) ────────────────────────────────────────────

    def final_audit(self) -> dict:
        """Ejecuta verificación completa de todos los módulos y reporta estado.

        Imprime resumen en consola, reproduce voz y retorna dict con resultados.

        Uso::

            core = ATLASQuantCore()
            result = core.final_audit()
            # → {"status": "ready" | "pending" | "critical", "issues": [...]}
        """
        import importlib
        from pathlib import Path

        checks = []
        issues = []

        def _check(name: str, fn, critical: bool = False) -> bool:
            try:
                ok = fn()
                checks.append({"name": name, "ok": ok, "critical": critical})
                if not ok and critical:
                    issues.append(f"❌ CRÍTICO: {name}")
                elif not ok:
                    issues.append(f"⚠️  PENDIENTE: {name}")
                return bool(ok)
            except Exception as exc:
                checks.append({"name": name, "ok": False, "critical": critical, "error": str(exc)})
                tag = "❌ CRÍTICO" if critical else "⚠️  ERROR"
                issues.append(f"{tag}: {name} → {exc}")
                return False

        # M0 — Config
        _check("settings importable", lambda: bool(importlib.import_module("atlas_code_quant.config.settings")), critical=True)
        _check("TRADIER_PAPER_TOKEN configurado", lambda: bool(os.getenv("TRADIER_PAPER_TOKEN", "")))

        # M1 — Hardware
        _check("camera_interface importable", lambda: bool(importlib.import_module("atlas_code_quant.hardware.camera_interface")))
        _check("control_interface importable", lambda: bool(importlib.import_module("atlas_code_quant.hardware.control_interface")))
        _check("pyttsx3 disponible", lambda: bool(importlib.import_module("pyttsx3")))
        _check("pyautogui disponible", lambda: bool(importlib.import_module("pyautogui")))

        # M2 — Pipeline
        _check("tradier_stream importable", lambda: bool(importlib.import_module("atlas_code_quant.pipeline.tradier_stream")))
        _check("indicators importable",     lambda: bool(importlib.import_module("atlas_code_quant.pipeline.indicators")))

        # M3+M4 — ML
        _check("regime_classifier importable", lambda: bool(importlib.import_module("atlas_code_quant.models.regime_classifier")))
        _check("signal_generator importable",  lambda: bool(importlib.import_module("atlas_code_quant.strategy.signal_generator")))

        # M5+M6 — Risk + Healing
        _check("kelly_engine importable",  lambda: bool(importlib.import_module("atlas_code_quant.risk.kelly_engine")))
        _check("error_agent importable",   lambda: bool(importlib.import_module("atlas_code_quant.healing.error_agent")))
        _check("self_healing importable",  lambda: bool(importlib.import_module("atlas_code_quant.healing.self_healing")))

        # M7 — Execution
        _check("signal_executor importable", lambda: bool(importlib.import_module("atlas_code_quant.execution.signal_executor")))
        _check("live_loop importable",       lambda: bool(importlib.import_module("atlas_code_quant.execution.live_loop")))
        _check("ATLAS_FORCE_LIVE_PREVIEW activo",
               lambda: os.getenv("ATLAS_FORCE_LIVE_PREVIEW", "true").lower() != "false",
               critical=True)

        # M8 — Calibration
        _check("voice_feedback importable",      lambda: bool(importlib.import_module("atlas_code_quant.calibration.voice_feedback")))
        _check("physical_calibration importable", lambda: bool(importlib.import_module("atlas_code_quant.calibration.physical_calibration")))
        _check("test_runner importable",          lambda: bool(importlib.import_module("atlas_code_quant.calibration.test_runner")))
        _check("Mapa de calibración existe",
               lambda: Path(os.getenv("ATLAS_SCREEN_MAP", "calibration/atlas_screen_map.json")).exists())
        _check("ready_for_live.json existe",
               lambda: (
                   Path("data/operation/ready_for_live.json").exists() or
                   Path("reports/ready_for_live.json").exists()
               ))

        # M9 — Production
        _check("production_guard importable", lambda: bool(importlib.import_module("atlas_code_quant.production.production_guard")))
        _check("telegram_alerts importable",  lambda: bool(importlib.import_module("atlas_code_quant.production.telegram_alerts")))
        _check("grafana_dashboard importable", lambda: bool(importlib.import_module("atlas_code_quant.production.grafana_dashboard")))
        _check("live_activation importable",  lambda: bool(importlib.import_module("atlas_code_quant.production.live_activation")))
        _check("Dashboard PRO generado",
               lambda: Path("grafana/dashboards/atlas_pro_2026.json").exists())
        _check("TELEGRAM_BOT_TOKEN configurado", lambda: bool(os.getenv("TELEGRAM_BOT_TOKEN", "")))

        total   = len(checks)
        passed  = sum(1 for c in checks if c["ok"])
        critical_fails = [c for c in checks if not c["ok"] and c.get("critical")]
        pending = [c for c in checks if not c["ok"] and not c.get("critical")]

        status  = "critical" if critical_fails else ("pending" if pending else "ready")

        # ── Impresión de resultados ───────────────────────────────────────────
        print("\n" + "═"*62)
        print("   ATLAS-QUANT — AUDITORÍA FINAL DEL SISTEMA")
        print("═"*62)
        for c in checks:
            sym = "✅" if c["ok"] else ("❌" if c.get("critical") else "⚠️ ")
            print(f"  {sym} {c['name']}")
        print("─"*62)
        print(f"  Resultado: {passed}/{total} checks pasados")
        if issues:
            print(f"\n  Issues ({len(issues)}):")
            for iss in issues:
                print(f"    {iss}")
        print("─"*62)

        if status == "ready":
            msg = "✅ ATLAS 100% listo para LIVE — Todos los sistemas nominales."
            print(f"\n  {msg}")
        elif status == "pending":
            msg = f"⚠️  ATLAS casi listo — {len(pending)} pendientes no bloqueantes."
            print(f"\n  {msg}")
        else:
            msg = f"❌ ATLAS NO listo — {len(critical_fails)} issue(s) crítico(s) a resolver."
            print(f"\n  {msg}")

        print("═"*62 + "\n")

        # ── Voz ───────────────────────────────────────────────────────────────
        try:
            from atlas_code_quant.calibration.voice_feedback import VoiceFeedback
            vf = VoiceFeedback()
            vf.start()
            if status == "ready":
                vf.speak("Sistema completamente auditado y profesional. ATLAS listo para trading real.", urgent=True)
            else:
                vf.speak(f"Auditoría completada. {len(issues)} problemas encontrados. Revisar informe.", urgent=(status == "critical"))
            import time; time.sleep(4.0)
            vf.stop()
        except Exception:
            pass

        # ── Generar informe + dashboard PRO ───────────────────────────────────
        try:
            from atlas_code_quant.production.grafana_pro import save_pro_dashboard
            save_pro_dashboard()
        except Exception as exc:
            logger.debug("Dashboard PRO: %s", exc)

        return {
            "status":         status,
            "passed":         passed,
            "total":          total,
            "issues":         issues,
            "critical_fails": [c["name"] for c in critical_fails],
            "pending":        [c["name"] for c in pending],
        }

    # ── Activación LIVE completa (Módulo 9) ──────────────────────────────────

    def activate_live(
        self,
        symbols: list[str] | None = None,
        cycles: int = 50,
        rtmp_url: str = "rtmp://192.168.1.10/live/atlas",
        force_rerun_test: bool = False,
        skip_confirmation: bool = False,
    ) -> bool:
        """Ejecuta el protocolo completo de activación LIVE (Módulo 9).

        Flujo: calibración → test runner → ProductionGuard →
               Tradier preview test → double confirmation → launch live.

        Retorna True si ATLAS se lanzó en LIVE, False si se canceló.

        Uso::

            core = ATLASQuantCore()
            core.activate_live(symbols=["SPY", "AAPL"])
        """
        from atlas_code_quant.production.live_activation import activate

        rc = activate(
            symbols           = symbols or self._symbols or ["SPY", "QQQ", "AAPL"],
            cycles            = cycles,
            rtmp_url          = rtmp_url,
            force_rerun_test  = force_rerun_test,
            skip_confirmation = skip_confirmation,
            mode              = "live",
        )
        return rc == 0

    # ── Calibración física ────────────────────────────────────────────────────

    def calibrate(self, force: bool = False) -> bool:
        """Ejecuta calibración física de pantalla y guarda mapa de coordenadas.

        Si ya existe un mapa válido y force=False, lo carga sin recalibrar.
        Retorna True si la calibración fue exitosa o el mapa ya existía.

        Uso::

            core = ATLASQuantCore(mode="paper")
            core.calibrate()          # calibra o carga mapa existente
            core.setup()
            core.start()
        """
        from atlas_code_quant.calibration.physical_calibration import PhysicalCalibrator
        from atlas_code_quant.calibration.voice_feedback import VoiceFeedback

        voice = VoiceFeedback()
        voice.start()

        cal = PhysicalCalibrator(
            rtmp_url = "rtmp://192.168.1.10/live/atlas",
            voice    = voice,
            hid      = self.hid,
            ros2     = self.ros2,
        )

        if not force and PhysicalCalibrator.map_exists():
            screen_map = PhysicalCalibrator.load_map()
            if screen_map is not None:
                logger.info(
                    "Mapa de calibración existente cargado (%d monitores)",
                    len(screen_map.monitors)
                )
                voice.speak("Mapa de calibración cargado. Listo para operar.")
                voice.stop()
                return True

        logger.info("Iniciando calibración física…")
        try:
            screen_map = cal.calibrate()
            saved_path = cal.save_map(screen_map)
            logger.info("✓ Calibración guardada: %s", saved_path)
            voice.stop()
            return True
        except Exception as exc:
            logger.error("Error en calibración: %s", exc)
            voice.announce_repair_failed("calibración")
            voice.stop()
            return False

    # ── Morning Market Open Test (2026-03-23) ────────────────────────────────

    def start_market_open_test(
        self,
        skip_wait: bool = False,
        now_mode:  bool = False,
    ) -> int:
        """Inicia el flujo autónomo completo del Morning Market Open Test.

        Fuerza modo PAPER, activa voz + Telegram + Grafana y delega a
        MorningMarketTest para orquestar las 4 fases del día.

        Args:
            skip_wait: si True, no espera al horario ET (útil para CI/test rápido).
            now_mode:  si True, ejecuta todas las fases inmediatamente.

        Retorna código de salida: 0 = éxito, 1 = cancelado/error.
        """
        import os as _os
        _os.environ["ATLAS_MODE"]                 = "paper"
        _os.environ["ATLAS_FORCE_LIVE_PREVIEW"]   = "true"
        self.mode = "paper"

        logger.info("▶ start_market_open_test — modo=PAPER skip_wait=%s now=%s",
                    skip_wait, now_mode)

        try:
            # Importación lazy para no romper entornos sin el script
            import importlib.util, sys as _sys
            spec = importlib.util.spec_from_file_location(
                "morning_market_test",
                str(Path(__file__).resolve().parent.parent / "morning_market_test.py"),
            )
            mod = importlib.util.module_from_spec(spec)
            _sys.modules.setdefault("morning_market_test", mod)
            spec.loader.exec_module(mod)

            test = mod.MorningMarketTest(skip_wait=skip_wait, now_mode=now_mode)
            # Compartir sub-sistemas ya inicializados si están disponibles
            if self._initialized:
                test._core = self
            return test.run()
        except Exception as exc:
            logger.exception("Error en start_market_open_test: %s", exc)
            return 1

    # ── Full Autonomy (M10) ───────────────────────────────────────────────────

    def start_full_autonomy(
        self,
        symbols: list[str] | None = None,
        fullscreen: bool = True,
    ) -> bool:
        """Lanza el pipeline completo de autonomía: TradingView → calibración → escaneo → señales paper.

        Secuencia:
          1. Abre 4 pestañas TradingView gratuito (Chrome + pyautogui)
          2. Ejecuta calibración física (carga mapa de pantalla)
          3. Inicializa stream de datos (Tradier sandbox)
          4. Arranca LiveLoop en modo PAPER

        Retorna True si todos los pasos se completaron.
        """
        import os as _os
        _os.environ["ATLAS_MODE"]               = "paper"
        _os.environ["ATLAS_FORCE_LIVE_PREVIEW"] = "true"
        self.mode = "paper"

        logger.info("🤖 start_full_autonomy — modo=PAPER, símbolos=%s", symbols)

        # 1. TradingView FREE — M10
        chart_launcher = None
        try:
            from atlas_code_quant.chart_launcher import launch_free_tradingview
            chart_launcher = launch_free_tradingview(
                symbols=symbols,
                fullscreen=fullscreen,
            )
            logger.info("✅ TradingView FREE activo")
        except Exception as exc:
            logger.warning("ChartLauncher no disponible: %s — continuando", exc)

        # 2. Calibración física
        calib_ok = self.calibrate(force=False)
        if not calib_ok:
            logger.warning("⚠️  Calibración no completada — mapa de pantalla no disponible")

        # 3. Setup de módulos (si no inicializados)
        if not self._initialized:
            try:
                self.setup(symbols=symbols)
            except Exception as exc:
                logger.error("Error en setup: %s", exc)
                return False

        # 4. LiveLoop PAPER
        try:
            self.run(cycles=0)   # 0 = loop infinito hasta señal de parada
        except KeyboardInterrupt:
            logger.info("Full autonomy detenida por usuario")
        except Exception as exc:
            logger.error("Error en run: %s", exc)
            return False

        return True

    # ── Ejecución delegada a SignalExecutor (vía LiveLoop) ───────────────────

    # ── Callbacks de eventos ──────────────────────────────────────────────────

    def _on_quote(self, quote) -> None:
        self._latest_quotes[quote.symbol] = quote
        # Alimentar LiveLoop para que su _evaluate_and_execute tenga datos frescos
        if self.live_loop is not None:
            self.live_loop.update_quote(quote.symbol, quote)
        # Actualizar IV Rank si hay datos de opciones
        if hasattr(quote, "iv") and quote.iv > 0:
            self.iv_rank.update_iv(quote.symbol, quote.iv)
        self.cvd_calc.update_quote(quote.bid, quote.ask)

    def _on_trade(self, trade) -> None:
        self.cvd_calc.update_trade(trade.price, trade.size, trade.timestamp)

    def _on_stream_error(self, error: str) -> None:
        self.error_agent.record_error(
            error_type="api_disconnect",
            description=f"Error en stream Tradier: {error}",
        )

    def _on_camera_degraded(self, reason: str, value: float) -> None:
        self.error_agent.record_error(
            error_type="ocr_confidence_low" if "ocr" in reason else "stream_latency_high",
            description=f"Cámara degradada: {reason}={value:.2f}",
        )

    def _send_alert(self, subsystem: str, message: str) -> None:
        """Envía alerta Telegram vía AlertDispatcher existente de Atlas."""
        try:
            from atlas_code_quant.operations.alert_dispatcher import get_alert_dispatcher
            dispatcher = get_alert_dispatcher()
            dispatcher.system_error(
                component=f"quant_core.{subsystem}",
                error=message,
                critical=True,
            )
        except Exception as exc:
            logger.error("Error enviando alerta: %s", exc)

    # ── Gestión de ciclo de vida ──────────────────────────────────────────────

    def _emergency_stop(self) -> None:
        logger.critical("EMERGENCY STOP — cerrando todas las posiciones")
        self._running = False
        if self.live_loop is not None:
            self.live_loop.emergency_stop()

    def _graceful_stop(self) -> None:
        logger.info("Deteniendo ATLAS-Quant-Core…")
        self._running = False
        if self.live_loop is not None:
            self.live_loop.stop("graceful")

    def _shutdown(self) -> None:
        logger.info("Shutdown: liberando recursos…")
        if self.camera:
            self.camera.stop()
        if self.stream_client:
            self.stream_client.disconnect()
        if self.ros2:
            self.ros2.stop()
        if self.healer:
            self.healer.stop()
        if self.hid:
            self.hid.stop_hotkey_listener()
        logger.info("=== ATLAS-Quant-Core DETENIDO ===")

    def status(self) -> dict:
        return {
            "running":         self._running,
            "mode":            self.mode,
            "cycle_count":     self.live_loop.status().get("cycle_count", 0) if self.live_loop else 0,
            "symbols":         len(self._symbols),
            "open_positions":  len(self.signal_gen.open_positions()) if self.signal_gen else 0,
            "risk":            self.risk_engine.status_dict() if self.risk_engine else {},
            "healing":         self.healer.health_summary() if self.healer else {},
            "stream":          self.stream_client.stats() if self.stream_client else {},
            "execution":       self.executor.stats() if self.executor else {},
            "live_loop":       self.live_loop.status() if self.live_loop else {},
        }


# ── Entrypoint ────────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ATLAS-Quant-Core — Robot de Trading Autónomo")
    parser.add_argument(
        "--mode",
        choices=["paper", "live"],
        default="paper",
        help="Modo de operación: 'paper' (simulado, defecto) o 'live' (trading real)",
    )
    parser.add_argument(
        "--symbols",
        nargs="*",
        default=None,
        help="Lista de símbolos a operar (sobreescribe scanner_universe del settings)",
    )
    parser.add_argument(
        "--cycle",
        type=float,
        default=None,
        help="Duración del ciclo en segundos (defecto: 5)",
    )
    parser.add_argument(
        "--final-audit",
        action="store_true",
        default=False,
        help="Ejecuta auditoría completa de todos los módulos y sale",
    )
    return parser.parse_args()


if __name__ == "__main__":
    _setup_logging()
    args = _parse_args()

    import os

    if args.final_audit:
        core = ATLASQuantCore(mode="paper")
        result = core.final_audit()
        import sys
        sys.exit(0 if result["status"] != "critical" else 1)

    if args.mode == "live":
        os.environ.setdefault("ATLAS_MODE", "live")

    if args.cycle:
        os.environ.setdefault("ATLAS_CYCLE_S", str(args.cycle))

    core = ATLASQuantCore(mode=args.mode)

    # Sobreescribir símbolos si se pasaron por CLI
    if args.symbols:
        core._symbols = args.symbols

    core.setup()
    core.start()
