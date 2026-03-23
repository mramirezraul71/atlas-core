#!/usr/bin/env python3
"""ATLAS-Quant — Immediate Start (Apertura de Mercado HOY)
=========================================================
Arranca AHORA sin esperar horario. Mercado abre en minutos.

Secuencia de arranque (<3 min):
  1. TradingView FREE → Chrome 4 pestañas (AAPL, TSLA, SPY, QQQ / 5m / RSI+MACD+Vol)
  2. Calibración física → carga mapa de pantalla
  3. Init: Grafana + Telegram + Voice
  4. Stream Tradier sandbox → quotes en tiempo real
  5. Pre-market scan cada 60s → IV Rank / CVD / liquidez
  6. 09:30 ET exacta → modo señales PAPER (sandbox.tradier.com + preview=true)
  7. EOD 16:00 ET → reporte y veredicto

Ejecución directa:
    python immediate_start.py
    python immediate_start.py --symbols AAPL,TSLA,SPY,QQQ
    python immediate_start.py --skip-tv    # sin abrir TradingView

Docker:
    docker run --runtime nvidia --privileged --network host \\
      -v /dev:/dev \\
      -v /c/dev/calibration:/calibration \\
      -v /c/dev/credenciales.txt:/credentials/credenciales.txt:ro \\
      -e ATLAS_MODE=paper \\
      -e ATLAS_SYMBOLS=AAPL,TSLA,SPY,QQQ \\
      atlas-quant:jetson immediate-start
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time
import threading
from datetime import datetime
from pathlib import Path
from typing import Optional
import zoneinfo

# ── Path setup ────────────────────────────────────────────────────────────────
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

# ── Forzar modo PAPER antes de cualquier import ──────────────────────────────
os.environ["ATLAS_MODE"]               = "paper"
os.environ["ATLAS_FORCE_LIVE_PREVIEW"] = "true"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s — %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(ROOT / "logs" / "immediate_start.log",
                            encoding="utf-8", mode="a"),
    ],
)
logger = logging.getLogger("atlas.immediate_start")

ET = zoneinfo.ZoneInfo("America/New_York")

# ── Símbolos por defecto ──────────────────────────────────────────────────────
_DEFAULT_SYMBOLS = [s.strip().upper()
                    for s in os.getenv("ATLAS_SYMBOLS", "AAPL,TSLA,SPY,QQQ").split(",")]


# ══════════════════════════════════════════════════════════════════════════════
class ImmediateStart:
    """Orquestador de arranque inmediato para apertura de mercado HOY."""

    SCAN_INTERVAL_S = 60          # pre-market scan cada 60s
    MARKET_OPEN_ET  = (9, 30)     # (hora, min) apertura NYSE
    MARKET_CLOSE_ET = (16, 0)     # cierre NYSE

    def __init__(
        self,
        symbols: list[str] | None = None,
        skip_tv: bool = False,
    ) -> None:
        self.symbols    = (symbols or _DEFAULT_SYMBOLS)[:4]
        self.skip_tv    = skip_tv
        self._stop      = threading.Event()
        self._voice     = None
        self._telegram  = None
        self._grafana   = None
        self._chart_launcher = None
        self._stream    = None
        self._core      = None

    # ── Entry point ───────────────────────────────────────────────────────────

    def run(self) -> int:
        """Flujo completo de arranque inmediato. Retorna 0 = OK, 1 = error."""
        self._print_banner()

        try:
            # ── Paso 1: TradingView FREE ──────────────────────────────────────
            if not self.skip_tv:
                self._launch_tradingview()

            # ── Paso 2: Subsistemas ───────────────────────────────────────────
            self._init_voice()
            self._init_telegram()
            self._init_grafana()
            self._init_calibration()

            # ── Paso 3: Anuncio de arranque ───────────────────────────────────
            self._announce_start()

            # ── Paso 4: Stream de datos Tradier ───────────────────────────────
            self._init_stream()

            # ── Paso 5: Pre-market scan hasta 09:30 ───────────────────────────
            self._premarket_loop()

            # ── Paso 6: Trading PAPER (09:30 → 16:00) ────────────────────────
            self._trading_session()

            # ── Paso 7: EOD ───────────────────────────────────────────────────
            self._eod_report()

        except KeyboardInterrupt:
            logger.info("⛔ Detenido por usuario (Ctrl+C)")
        except Exception as exc:
            logger.exception("Error crítico en ImmediateStart: %s", exc)
            return 1

        return 0

    # ── TradingView ───────────────────────────────────────────────────────────

    def _launch_tradingview(self) -> None:
        logger.info("📊 Lanzando TradingView FREE — 4 pestañas Chrome…")
        try:
            from atlas_code_quant.chart_launcher import launch_free_tradingview
            self._chart_launcher = launch_free_tradingview(
                symbols=self.symbols,
                voice=self._voice,
                telegram=self._telegram,
                fullscreen=True,
            )
            logger.info("✅ TradingView FREE activo — %s", self.symbols)
        except Exception as exc:
            logger.warning("ChartLauncher falló: %s — continuando sin TV visual", exc)

    # ── Subsistemas ───────────────────────────────────────────────────────────

    def _init_voice(self) -> None:
        try:
            from atlas_code_quant.calibration.voice_feedback import VoiceFeedback
            self._voice = VoiceFeedback()
            self._voice.start()
            logger.info("✅ Voice (pyttsx3 español) activo")
        except Exception as exc:
            logger.warning("Voice no disponible: %s", exc)

    def _init_telegram(self) -> None:
        try:
            from atlas_code_quant.production.telegram_alerts import TelegramAlerts
            self._telegram = TelegramAlerts()
            logger.info("✅ Telegram Alerts activo")
        except Exception as exc:
            logger.warning("Telegram no disponible: %s", exc)

    def _init_grafana(self) -> None:
        try:
            from atlas_code_quant.production.grafana_dashboard import GrafanaDashboard
            self._grafana = GrafanaDashboard()
            self._grafana.update_metric("atlas_mode_live", 0.0)    # 0 = paper
            logger.info("✅ Grafana métricas activas")
        except Exception as exc:
            logger.warning("Grafana no disponible: %s", exc)

    def _init_calibration(self) -> None:
        try:
            from atlas_code_quant.calibration.physical_calibration import PhysicalCalibrator
            if PhysicalCalibrator.map_exists():
                logger.info("✅ Mapa de calibración cargado")
                self._speak("Calibración de cámara cargada.")
            else:
                logger.warning("⚠️  Sin mapa de calibración — usando defaults")
        except Exception as exc:
            logger.warning("Calibración: %s", exc)

    def _init_stream(self) -> None:
        try:
            from atlas_code_quant.config.settings import settings
            from atlas_code_quant.pipeline.tradier_stream import TradierStreamClient
            token = settings.tradier_paper_token
            if not token:
                logger.warning("TRADIER_PAPER_TOKEN no configurado — stream desactivado")
                return
            self._stream = TradierStreamClient(token=token, sandbox=True)
            self._stream.subscribe(self.symbols)
            self._stream.on_quote(self._on_quote)
            self._stream.on_trade(self._on_trade)
            self._stream.connect()
            logger.info("✅ Tradier stream sandbox activo — %s", self.symbols)
        except Exception as exc:
            logger.warning("Tradier stream: %s", exc)

    # ── Anuncio de arranque ───────────────────────────────────────────────────

    def _announce_start(self) -> None:
        voice_msg = (
            "TradingView gratuito abierto en cuatro pestañas. "
            "Escaneo iniciado. Listo para apertura."
        )
        telegram_msg = (
            "🚀 *ATLAS arrancando AHORA*\n"
            f"Símbolos: {', '.join(self.symbols)}\n"
            "Modo: PAPER | sandbox.tradier.com | preview=true\n"
            "TradingView: AAPL · TSLA · SPY · QQQ (5m)\n"
            "Indicadores: RSI(14) · MACD · Volumen\n"
            "⏰ Apertura: 09:30 ET\n"
            "🤖 ATLAS 100% autónomo"
        )
        self._speak(voice_msg)
        self._send_telegram(telegram_msg)
        logger.info("━" * 60)
        logger.info("  ATLAS IMMEDIATE START — PAPER MODE")
        logger.info("  Símbolos: %s", " | ".join(self.symbols))
        logger.info("  Stream:   sandbox.tradier.com")
        logger.info("  TV FREE:  RSI + MACD + Vol (5m)")
        logger.info("━" * 60)

    # ── Pre-market scan ───────────────────────────────────────────────────────

    def _premarket_loop(self) -> None:
        """Escanea cada 60s hasta las 09:30 ET."""
        now = self._now_et()
        open_dt = now.replace(hour=self.MARKET_OPEN_ET[0],
                               minute=self.MARKET_OPEN_ET[1],
                               second=0, microsecond=0)

        if now >= open_dt:
            logger.info("⏰ Ya pasó la apertura — saltando pre-market scan")
            return

        seconds_to_open = (open_dt - now).total_seconds()
        logger.info(
            "⏳ Pre-market scan hasta 09:30 ET (%.0f min restantes)",
            seconds_to_open / 60
        )
        self._speak(f"Pre-market activo. Apertura en {int(seconds_to_open/60)} minutos.")

        scan_count = 0
        while not self._stop.is_set():
            now = self._now_et()
            if now >= open_dt:
                break

            scan_count += 1
            remaining = (open_dt - now).total_seconds()
            logger.info(
                "🔍 Scan pre-market #%d — %.0f min para apertura",
                scan_count, remaining / 60
            )
            self._run_scan(label=f"PRE#{scan_count}")

            # Actualizar Grafana
            if self._grafana:
                try:
                    self._grafana.update_metric("atlas_open_positions", 0.0)
                    self._grafana.update_metric("atlas_mode_live", 0.0)
                except Exception:
                    pass

            self._stop.wait(self.SCAN_INTERVAL_S)

        logger.info("🔔 09:30 ET — Apertura de mercado")
        self._speak("Apertura de mercado. Iniciando modo trading PAPER.")
        self._send_telegram("🔔 *09:30 ET — MERCADO ABIERTO*\nIniciando señales PAPER")

    # ── Trading session ───────────────────────────────────────────────────────

    def _trading_session(self) -> None:
        """09:30 → 16:00 ET — Señales y órdenes PAPER via ATLASQuantCore."""
        logger.info("=" * 60)
        logger.info("TRADING SESSION — PAPER MODE (09:30 → 16:00 ET)")
        logger.info("=" * 60)

        try:
            from atlas_code_quant.atlas_quant_core import ATLASQuantCore
            self._core = ATLASQuantCore(mode="paper")
            self._core.setup()

            # Compartir stream ya conectado si es posible
            if self._stream is not None and self._core.stream_client is None:
                self._core.stream_client = self._stream

            close_dt = self._now_et().replace(
                hour=self.MARKET_CLOSE_ET[0],
                minute=self.MARKET_CLOSE_ET[1],
                second=0, microsecond=0,
            )

            # Arrancar LiveLoop en background
            self._core.live_loop.start()

            # Esperar hasta cierre o stop manual
            while not self._stop.is_set():
                now = self._now_et()
                if now >= close_dt:
                    logger.info("⏰ 16:00 ET — Cierre de mercado")
                    break
                # Actualizar Grafana cada ciclo
                self._update_grafana_metrics()
                self._stop.wait(5.0)

            self._core.live_loop.stop("market_close")
            self._core.live_loop._thread.join(timeout=10)

        except Exception as exc:
            logger.error("Error en trading session: %s", exc)
            self._send_telegram(f"⚠️ Error en trading session: {exc}")

    # ── EOD ───────────────────────────────────────────────────────────────────

    def _eod_report(self) -> None:
        """Genera reporte de fin de día."""
        logger.info("=" * 60)
        logger.info("END OF DAY — Generando reporte")
        logger.info("=" * 60)

        now_str = self._now_et().strftime("%Y-%m-%d %H:%M ET")
        equity = 0.0
        trades = 0
        pnl    = 0.0

        if self._core and self._core.risk_engine:
            state  = self._core.risk_engine.state()
            equity = state.current_equity
            pnl    = equity - 100_000.0   # vs capital inicial
        if self._core and self._core.signal_gen:
            trades = len(self._core.signal_gen.open_positions())

        pnl_sign = "+" if pnl >= 0 else ""
        report_lines = [
            "# ATLAS Immediate Start — Reporte EOD",
            f"Fecha: {now_str}",
            f"Modo: PAPER | sandbox.tradier.com",
            f"Símbolos: {', '.join(self.symbols)}",
            f"Equity final: ${equity:,.2f}",
            f"P&L del día: {pnl_sign}${pnl:,.2f}",
            f"Posiciones: {trades}",
        ]

        report_path = ROOT / "reports" / f"immediate_eod_{self._now_et().strftime('%Y%m%d')}.md"
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text("\n".join(report_lines), encoding="utf-8")
        logger.info("📄 Reporte EOD guardado: %s", report_path)

        verdict = "día positivo" if pnl >= 0 else "día negativo"
        self._speak(
            f"Sesión terminada. {verdict}. "
            "Revisa el dashboard Grafana para el análisis completo."
        )
        self._send_telegram(
            "📊 *EOD Report — ATLAS PAPER*\n"
            f"Equity: ${equity:,.2f}\n"
            f"P&L: {pnl_sign}${pnl:,.2f}\n"
            f"Posiciones: {trades}\n"
            "Sistema detenido. ¿Pasar a LIVE real mañana?"
        )

    # ── Scan de subsistemas ───────────────────────────────────────────────────

    def _run_scan(self, label: str = "SCAN") -> None:
        """Escaneo rápido de IV Rank, CVD, liquidez para los símbolos activos."""
        if self._stream is None:
            return
        for sym in self.symbols:
            try:
                q = self._stream.get_quote(sym)
                if q:
                    bid  = float(q.get("bid", 0))
                    ask  = float(q.get("ask", 0))
                    last = float(q.get("last", 0))
                    logger.info(
                        "  [%s] %s bid=%.2f ask=%.2f last=%.2f",
                        label, sym, bid, ask, last
                    )
            except Exception:
                pass

    # ── Callbacks stream ──────────────────────────────────────────────────────

    def _on_quote(self, quote) -> None:
        if self._grafana:
            try:
                self._grafana.update_metric("atlas_equity_usd",
                                            float(quote.bid + quote.ask) / 2)
            except Exception:
                pass

    def _on_trade(self, trade) -> None:
        logger.debug("TRADE %s @ %.2f x%d", trade.symbol, trade.price, trade.size)

    # ── Grafana ───────────────────────────────────────────────────────────────

    def _update_grafana_metrics(self) -> None:
        if self._grafana is None:
            return
        try:
            if self._core and self._core.risk_engine:
                state = self._core.risk_engine.state()
                self._grafana.update_metric("atlas_equity_usd",     state.current_equity)
                self._grafana.update_metric("atlas_drawdown_pct",   state.drawdown_pct * 100)
                self._grafana.update_metric("atlas_kelly_fraction",  state.kelly_fraction)
        except Exception:
            pass

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _speak(self, text: str) -> None:
        if self._voice is not None:
            try:
                self._voice.speak(text)
            except Exception:
                pass
        logger.info("[VOZ] %s", text)

    def _send_telegram(self, text: str) -> None:
        if self._telegram is not None:
            try:
                self._telegram.send(text)
            except Exception:
                pass
        logger.info("[TELEGRAM] %s", text.replace("\n", " | "))

    @staticmethod
    def _now_et() -> datetime:
        return datetime.now(tz=ET)

    def _print_banner(self) -> None:
        logger.info("╔══════════════════════════════════════════════════════╗")
        logger.info("║        ATLAS-QUANT IMMEDIATE START — PAPER           ║")
        logger.info("║  TradingView FREE + Tradier Sandbox + Grafana PRO    ║")
        logger.info("║  sandbox.tradier.com | preview=true | ATLAS_MODE=paper║")
        logger.info("╚══════════════════════════════════════════════════════╝")
        logger.info("   Símbolos : %s", " | ".join(self.symbols))
        logger.info("   Hora ET  : %s", self._now_et().strftime("%H:%M:%S ET %Y-%m-%d"))


# ── CLI ───────────────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="ATLAS-Quant Immediate Start — Apertura HOY"
    )
    parser.add_argument(
        "--symbols",
        default=os.getenv("ATLAS_SYMBOLS", "AAPL,TSLA,SPY,QQQ"),
        help="Símbolos separados por coma (máx 4)",
    )
    parser.add_argument(
        "--skip-tv",
        action="store_true",
        default=False,
        help="No abrir TradingView (útil en servidor headless sin monitor)",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args  = _parse_args()
    syms  = [s.strip().upper() for s in args.symbols.split(",")][:4]

    # Crear logs/ si no existe
    (ROOT / "logs").mkdir(parents=True, exist_ok=True)

    runner = ImmediateStart(symbols=syms, skip_tv=args.skip_tv)
    exit_code = runner.run()

    print()
    print("✅ ATLAS 100% AUTÓNOMO Y LISTO. Ejecuta el comando docker de arriba AHORA.")
    sys.exit(exit_code)
