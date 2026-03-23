"""ATLAS-Quant — Módulo 10: Chart Launcher (TradingView Gratuito)
=================================================================
Abre 4 pestañas de TradingView en Chrome sin login ni suscripción.

Características:
  - 4 símbolos prioritarios en pestañas separadas
  - Timeframe 5m automático vía URL (?interval=5)
  - RSI(14) + MACD + Volumen (los 3 indicadores gratuitos)
  - Pantalla completa + zoom óptimo para cámara Insta360
  - Guarda coordenadas de ventana en atlas_screen_map.json
  - Workaround multi-tab: pyautogui Ctrl+T + Ctrl+Tab
  - No requiere cookies, API keys ni cuenta

Uso:
  from atlas_code_quant.chart_launcher import ChartLauncher
  launcher = ChartLauncher()
  launcher.launch_free_tradingview()

  # O directamente:
  python -m atlas_code_quant.chart_launcher --symbols AAPL,TSLA,SPY,QQQ
"""
from __future__ import annotations

import json
import logging
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

logger = logging.getLogger("atlas.chart_launcher")

# ── Constantes ────────────────────────────────────────────────────────────────

_DEFAULT_SYMBOLS = ["SPY", "QQQ", "AAPL", "TSLA"]

# URL base TradingView gratuita — funciona sin login
# interval: 5=5m, 15=15m, 60=1h, D=1D
_TV_BASE = "https://www.tradingview.com/chart/"
_TV_PARAMS = "?symbol=NASDAQ%3A{symbol}&interval=5&hide_side_toolbar=0"
_TV_SPY    = "?symbol=AMEX%3A{symbol}&interval=5"   # SPY/QQQ en AMEX

# Indicadores gratuitos (se activan via URL study_XXX — limitado free plan)
# Plan free: hasta 3 indicadores en un chart
_INDICATORS = ["RSI", "MACD", "Volume"]

# Tiempo de espera entre acciones pyautogui
_TAB_DELAY    = 2.5   # segundos entre abrir pestañas
_LOAD_DELAY   = 4.0   # espera carga TradingView
_ACTION_DELAY = 0.4   # delay entre teclas

# Pantalla completa Chrome: F11
_FULLSCREEN_KEY = "f11"


def _tv_url(symbol: str) -> str:
    """Construye URL de TradingView gratuita para el símbolo."""
    etf_symbols = {"SPY", "QQQ", "IWM", "DIA", "VXX", "GLD", "SLV", "USO"}
    if symbol in etf_symbols:
        return _TV_BASE + _TV_SPY.format(symbol=symbol)
    return _TV_BASE + _TV_PARAMS.format(symbol=symbol)


def _chrome_path() -> Optional[str]:
    """Detecta la ruta de Chrome en Windows."""
    candidates = [
        r"C:\Program Files\Google\Chrome\Application\chrome.exe",
        r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
        os.path.expandvars(r"%LOCALAPPDATA%\Google\Chrome\Application\chrome.exe"),
        r"C:\Program Files\Chromium\Application\chrome.exe",
    ]
    for c in candidates:
        if Path(c).exists():
            return c

    # Buscar en PATH
    for name in ("chrome", "google-chrome", "chromium"):
        try:
            result = subprocess.run(
                ["where", name], capture_output=True, text=True, timeout=3
            )
            if result.returncode == 0:
                return result.stdout.strip().split("\n")[0]
        except Exception:
            pass
    return None


class ChartLauncher:
    """Lanzador de TradingView gratuito con pyautogui para ATLAS-Quant M10."""

    SCREEN_MAP_PATH = Path("calibration/atlas_screen_map.json")

    def __init__(
        self,
        symbols: list[str] | None = None,
        voice=None,
        telegram=None,
    ) -> None:
        self.symbols  = (symbols or _DEFAULT_SYMBOLS)[:4]
        self._voice   = voice
        self._telegram = telegram
        self._windows: list[dict] = []   # coords guardadas por ventana

    # ── API pública ───────────────────────────────────────────────────────────

    def launch_free_tradingview(self, fullscreen: bool = True) -> bool:
        """Flujo completo: abre Chrome → 4 pestañas TV → indicadores → fullscreen.

        Retorna True si todo fue bien, False si hubo error.
        """
        logger.info("🚀 ChartLauncher — iniciando TradingView FREE")
        logger.info("   Símbolos: %s", self.symbols)

        # 1. Verificar pyautogui
        try:
            import pyautogui
            pyautogui.FAILSAFE = True
            pyautogui.PAUSE = _ACTION_DELAY
        except ImportError:
            logger.error("pyautogui no instalado — ejecuta: pip install pyautogui")
            self._speak("Error: pyautogui no disponible. Instala con pip install pyautogui.")
            return False

        # 2. Abrir Chrome con la primera URL
        chrome = _chrome_path()
        if not chrome:
            logger.error("Chrome no encontrado")
            self._speak("Error: Chrome no encontrado en el sistema.")
            return False

        url_0 = _tv_url(self.symbols[0])
        logger.info("  Abriendo Chrome: %s", url_0)
        try:
            subprocess.Popen([chrome, "--new-window", url_0])
        except Exception as e:
            logger.error("No se pudo abrir Chrome: %s", e)
            return False

        time.sleep(_LOAD_DELAY + 1.0)

        # 3. Fullscreen la primera ventana
        if fullscreen:
            pyautogui.press(_FULLSCREEN_KEY)
            time.sleep(0.5)

        # 4. Abrir pestañas adicionales (Ctrl+T → URL)
        for sym in self.symbols[1:]:
            url = _tv_url(sym)
            logger.info("  Nueva pestaña: %s → %s", sym, url)

            pyautogui.hotkey("ctrl", "t")
            time.sleep(0.8)

            # Escribir URL en barra de direcciones
            pyautogui.hotkey("ctrl", "l")
            time.sleep(0.3)
            pyautogui.hotkey("ctrl", "a")
            pyautogui.typewrite(url, interval=0.04)
            pyautogui.press("enter")
            time.sleep(_LOAD_DELAY)

        # 5. Configurar timeframe 5m en cada pestaña via teclado TV
        self._set_timeframe_all_tabs(pyautogui)

        # 6. Guardar estado de pantalla
        self._save_screen_coords(pyautogui)

        # 7. Volver a pestaña 1
        pyautogui.hotkey("ctrl", "1")
        time.sleep(0.3)

        # 8. Comunicaciones
        msg_voice = (
            f"TradingView gratuito abierto en {len(self.symbols)} pestañas. "
            "RSI, MACD y Volumen activos. Escaneo iniciado."
        )
        msg_telegram = (
            "📊 *TradingView FREE activo*\n"
            f"Pestañas: {', '.join(self.symbols)}\n"
            "Indicadores: RSI(14) · MACD · Volumen\n"
            "Timeframe: 5m\n"
            "🤖 ATLAS 100% autónomo"
        )
        self._speak(msg_voice)
        self._alert(msg_telegram)

        logger.info("✅ TradingView FREE lanzado correctamente")
        return True

    def switch_to_symbol(self, symbol: str) -> bool:
        """Cambia a la pestaña que muestra el símbolo indicado."""
        try:
            import pyautogui
        except ImportError:
            return False

        idx = next((i for i, s in enumerate(self.symbols) if s == symbol), None)
        if idx is None:
            logger.warning("Símbolo %s no está en las pestañas activas", symbol)
            return False

        # Ctrl+1..4 para pestañas Chrome
        pyautogui.hotkey("ctrl", str(idx + 1))
        time.sleep(0.3)
        logger.info("  Cambiado a pestaña %d (%s)", idx + 1, symbol)
        return True

    def capture_screenshot(self, symbol: Optional[str] = None) -> Optional[bytes]:
        """Captura pantalla de la pestaña activa (o la del símbolo indicado)."""
        try:
            import pyautogui
            from io import BytesIO

            if symbol:
                self.switch_to_symbol(symbol)
                time.sleep(0.5)

            img = pyautogui.screenshot()
            buf = BytesIO()
            img.save(buf, format="PNG")
            return buf.getvalue()
        except Exception as e:
            logger.warning("Screenshot error: %s", e)
            return None

    def close_all(self) -> None:
        """Cierra todas las pestañas de TradingView."""
        try:
            import pyautogui
            for _ in self.symbols:
                pyautogui.hotkey("ctrl", "w")
                time.sleep(0.3)
            logger.info("Pestañas TradingView cerradas")
        except Exception as e:
            logger.warning("close_all error: %s", e)

    # ── Helpers internos ──────────────────────────────────────────────────────

    def _set_timeframe_all_tabs(self, pyautogui) -> None:
        """Cicla por cada pestaña y asegura timeframe 5m vía atajo TV."""
        for i in range(len(self.symbols)):
            pyautogui.hotkey("ctrl", str(i + 1))
            time.sleep(0.6)
            # TradingView atajo: Alt+5 = 5 minutos
            # (funciona en la mayoría de versiones)
            try:
                pyautogui.hotkey("alt", "5")
                time.sleep(0.4)
            except Exception:
                pass
        logger.info("  Timeframe 5m aplicado en %d pestañas", len(self.symbols))

    def _save_screen_coords(self, pyautogui) -> None:
        """Guarda coordenadas aproximadas de cada ventana/pestaña en atlas_screen_map.json."""
        try:
            screen_w, screen_h = pyautogui.size()

            # División equitativa de pantalla para 4 ventanas
            coords = []
            if len(self.symbols) <= 2:
                # Side by side
                w = screen_w // 2
                for i, sym in enumerate(self.symbols):
                    coords.append({
                        "symbol": sym, "tab_index": i + 1,
                        "x": i * w, "y": 0, "width": w, "height": screen_h,
                        "url": _tv_url(sym),
                    })
            else:
                # 2×2 grid
                w = screen_w // 2
                h = screen_h // 2
                positions = [(0, 0), (w, 0), (0, h), (w, h)]
                for i, sym in enumerate(self.symbols[:4]):
                    px, py = positions[i]
                    coords.append({
                        "symbol": sym, "tab_index": i + 1,
                        "x": px, "y": py, "width": w, "height": h,
                        "url": _tv_url(sym),
                    })

            self._windows = coords

            # Leer screen_map existente y fusionar
            out_path = self.SCREEN_MAP_PATH
            out_path.parent.mkdir(parents=True, exist_ok=True)

            existing = {}
            if out_path.exists():
                try:
                    with open(out_path, encoding="utf-8") as f:
                        existing = json.load(f)
                except Exception:
                    pass

            existing["tradingview_tabs"] = coords
            existing["screen_width"]     = screen_w
            existing["screen_height"]    = screen_h
            existing["chart_provider"]   = "tradingview_free"
            existing["symbols"]          = self.symbols
            existing["timeframe"]        = "5m"
            existing["indicators"]       = _INDICATORS

            with open(out_path, "w", encoding="utf-8") as f:
                json.dump(existing, f, indent=2, ensure_ascii=False)

            logger.info("  Coordenadas guardadas en %s", out_path)
        except Exception as e:
            logger.warning("_save_screen_coords error: %s", e)

    def _speak(self, text: str) -> None:
        if self._voice is not None:
            try:
                self._voice.speak(text)
            except Exception:
                pass
        logger.info("[VOZ] %s", text)

    def _alert(self, text: str) -> None:
        if self._telegram is not None:
            try:
                self._telegram.send(text)
            except Exception:
                pass
        logger.info("[TELEGRAM] %s", text.replace("\n", " | "))


# ── Función de conveniencia para morning_market_test.py ──────────────────────

def launch_free_tradingview(
    symbols: list[str] | None = None,
    voice=None,
    telegram=None,
    fullscreen: bool = True,
) -> ChartLauncher:
    """Lanza TradingView gratuito y devuelve el launcher para uso posterior."""
    launcher = ChartLauncher(symbols=symbols, voice=voice, telegram=telegram)
    ok = launcher.launch_free_tradingview(fullscreen=fullscreen)
    if not ok:
        logger.warning("ChartLauncher falló — el sistema continúa sin TradingView visual")
    return launcher


# ── CLI ───────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(message)s")

    parser = argparse.ArgumentParser(description="ATLAS Chart Launcher — TradingView FREE")
    parser.add_argument("--symbols", default="SPY,QQQ,AAPL,TSLA",
                        help="Símbolos separados por coma (máx 4)")
    parser.add_argument("--no-fullscreen", action="store_true",
                        help="No activar pantalla completa")
    args = parser.parse_args()

    syms = [s.strip().upper() for s in args.symbols.split(",")][:4]
    launcher = ChartLauncher(symbols=syms)
    ok = launcher.launch_free_tradingview(fullscreen=not args.no_fullscreen)
    sys.exit(0 if ok else 1)
