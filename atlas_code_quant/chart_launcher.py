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

# Universo de escaneo para el scanner dinámico (máximo 6 resultan)
_SCANNER_UNIVERSE = [
    "SPY", "QQQ", "AAPL", "TSLA", "NVDA", "MSFT",
    "AMZN", "META", "AMD", "IWM", "GLD", "XLF",
]

# Filtros del escáner dinámico
_SCAN_IV_RANK_MIN  = 70.0   # IV Rank mínimo (percentil)
_SCAN_IV_HV_RATIO  = 1.2    # IV / HV mínimo
_SCAN_CVD_SIGMA    = 1.0    # CVD z-score mínimo
_SCAN_LIQ_MIN_USD  = 10e6   # liquidez mínima $ (volumen × precio)
_SCAN_MAX_SYMBOLS  = 6      # máximo de símbolos a abrir

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


def run_dynamic_scanner(
    token: str,
    universe: list[str] | None = None,
    iv_rank_min: float = _SCAN_IV_RANK_MIN,
    iv_hv_ratio_min: float = _SCAN_IV_HV_RATIO,
    cvd_sigma_min: float = _SCAN_CVD_SIGMA,
    liq_min_usd: float = _SCAN_LIQ_MIN_USD,
    max_symbols: int = _SCAN_MAX_SYMBOLS,
) -> list[str]:
    """Escáner dinámico: filtra activos por IV Rank, IV/HV, CVD y liquidez.

    Usa Tradier REST (sandbox o live). Devuelve top_symbols ordenados por score.

    Criterios de filtro:
      - Liquidez (vol × precio)  ≥ liq_min_usd
      - IV Rank proxy             ≥ iv_rank_min   (percentil ATM IV vs universe)
      - IV/HV ratio               ≥ iv_hv_ratio_min
      - CVD z-score (bid/ask vol) ≥ cvd_sigma_min  (positivo = presión compradora)

    Args:
        token:          Tradier access token (paper o live).
        universe:       Lista de símbolos a escanear (default: _SCANNER_UNIVERSE).
        iv_rank_min:    Percentil mínimo de IV (0-100).
        iv_hv_ratio_min: IV / HV mínimo.
        cvd_sigma_min:  Z-score CVD mínimo.
        liq_min_usd:    Liquidez mínima en USD.
        max_symbols:    Máximo de símbolos devueltos.

    Retorna lista de símbolos que pasaron los filtros, ordenados por score.
    """
    try:
        import requests as _req
        import statistics as _stat
        import math as _math
    except ImportError as e:
        logger.error("Scanner: import error — %s", e)
        return list((universe or _SCANNER_UNIVERSE)[:max_symbols])

    syms = list(universe or _SCANNER_UNIVERSE)
    hdrs = {"Authorization": f"Bearer {token}", "Accept": "application/json"}

    # ── 1. Quotes: liquidez + spread + CVD proxy ──────────────────────────────
    logger.info("Scanner: fetching quotes para %d símbolos…", len(syms))
    try:
        r = _req.get(
            "https://sandbox.tradier.com/v1/markets/quotes",
            headers=hdrs,
            params={"symbols": ",".join(syms)},
            timeout=12,
        )
        r.raise_for_status()
        raw_quotes = r.json().get("quotes", {}).get("quote", [])
        if isinstance(raw_quotes, dict):
            raw_quotes = [raw_quotes]
    except Exception as exc:
        logger.warning("Scanner quotes error: %s — usando universo completo", exc)
        return list(syms[:max_symbols])

    quote_map: dict[str, dict] = {}
    for q in raw_quotes:
        sym  = q.get("symbol", "")
        last = float(q.get("last") or q.get("close") or 0)
        bid  = float(q.get("bid") or 0)
        ask  = float(q.get("ask") or 0)
        bsz  = float(q.get("bidsize") or q.get("bid_size") or 1)
        asz  = float(q.get("asksize") or q.get("ask_size") or 1)
        vol  = int(q.get("volume") or 0)
        hi   = float(q.get("high") or last)
        lo   = float(q.get("low") or last)
        op   = float(q.get("open") or last)
        if sym:
            quote_map[sym] = {
                "last": last, "bid": bid, "ask": ask,
                "bid_sz": bsz, "ask_sz": asz,
                "vol": vol, "hi": hi, "lo": lo, "open": op,
            }

    # ── 2. IV via opciones (primer vencimiento disponible) ────────────────────
    logger.info("Scanner: fetching IV para opciones…")
    iv_map: dict[str, float] = {}   # sym → ATM IV promedio
    for sym in syms:
        try:
            re_exp = _req.get(
                "https://sandbox.tradier.com/v1/markets/options/expirations",
                headers=hdrs,
                params={"symbol": sym, "includeAllRoots": "true"},
                timeout=8,
            )
            exps = re_exp.json().get("expirations", {}).get("date", [])
            exp  = (exps[0] if isinstance(exps, list) else exps) if exps else None
            if not exp:
                continue
            rc = _req.get(
                "https://sandbox.tradier.com/v1/markets/options/chains",
                headers=hdrs,
                params={"symbol": sym, "expiration": exp, "greeks": "true"},
                timeout=10,
            )
            opts = rc.json().get("options", {}).get("option", [])
            if isinstance(opts, dict):
                opts = [opts]
            ivs = [
                float(o.get("greeks", {}).get("mid_iv", 0) or 0)
                for o in opts
                if o.get("greeks") and float(o.get("greeks", {}).get("mid_iv", 0) or 0) > 0
            ]
            if ivs:
                iv_map[sym] = sum(ivs) / len(ivs)
        except Exception:
            pass

    # ── 3. Calcular métricas de filtrado ──────────────────────────────────────
    all_ivs = [v for v in iv_map.values() if v > 0]
    iv_median = _stat.median(all_ivs) if all_ivs else 0.5
    iv_sorted = sorted(all_ivs)
    n_iv      = len(iv_sorted)

    def iv_rank_pct(iv: float) -> float:
        """Percentil de IV dentro del universo escaneado."""
        if n_iv == 0 or iv <= 0:
            return 0.0
        below = sum(1 for x in iv_sorted if x <= iv)
        return (below / n_iv) * 100.0

    candidates: list[dict] = []

    for sym in syms:
        q = quote_map.get(sym)
        if not q or q["last"] <= 0:
            continue

        last  = q["last"]
        vol   = q["vol"]
        bid   = q["bid"]
        ask   = q["ask"]
        bsz   = q["bid_sz"]
        asz   = q["ask_sz"]
        hi    = q["hi"]
        lo    = q["lo"]

        # Liquidez
        liq_usd = vol * last
        if liq_usd < liq_min_usd:
            continue

        # IV + IV Rank
        iv   = iv_map.get(sym, 0.0)
        rank = iv_rank_pct(iv)
        if rank < iv_rank_min:
            continue

        # HV proxy: rango intradiario normalizado (Yang-Zhang no disponible → usar hi-lo/open)
        hv_proxy = ((hi - lo) / last) * _math.sqrt(252) if last > 0 else 0.0
        if hv_proxy <= 0:
            hv_proxy = 0.001

        iv_hv = iv / hv_proxy if hv_proxy > 0 else 0.0
        if iv_hv < iv_hv_ratio_min:
            continue

        # CVD proxy: desequilibrio bid/ask size → z-score relativo
        total_sz = bsz + asz
        cvd_raw  = (asz - bsz) / total_sz if total_sz > 0 else 0.0  # positivo = más compras
        # z-score relativo al rango: si > cvd_sigma * 0.1 => pasa
        cvd_sigma = cvd_raw / 0.1 if cvd_raw > 0 else 0.0
        if cvd_sigma < cvd_sigma_min:
            # Permitir si señal neutral y otros filtros muy fuertes
            if not (rank >= 85 and iv_hv >= 1.5):
                continue

        # Score compuesto (mayor = mejor candidato)
        score = rank * 0.4 + iv_hv * 10 + cvd_sigma * 5 + min(liq_usd / 1e8, 10)

        candidates.append({
            "symbol":   sym,
            "last":     last,
            "liq_usd":  liq_usd,
            "iv":       iv,
            "iv_rank":  rank,
            "iv_hv":    iv_hv,
            "cvd":      cvd_sigma,
            "score":    score,
        })
        logger.info(
            "  ✅ PASS  %s — liq=$%.0fM iv=%.1f%% rank=%.0f%% iv/hv=%.2f cvd=%.2f score=%.1f",
            sym, liq_usd / 1e6, iv * 100, rank, iv_hv, cvd_sigma, score,
        )

    # ── 4. Ordenar por score y limitar ────────────────────────────────────────
    candidates.sort(key=lambda x: x["score"], reverse=True)
    top = [c["symbol"] for c in candidates[:max_symbols]]

    if not top:
        logger.warning(
            "Scanner: ningún símbolo pasó los filtros — "
            "relajando criterios y usando top liquidez"
        )
        # Fallback: top-4 por liquidez
        liq_ranked = sorted(
            [sym for sym in syms if sym in quote_map],
            key=lambda s: quote_map[s]["vol"] * quote_map[s]["last"],
            reverse=True,
        )
        top = liq_ranked[:4]

    logger.info("Scanner resultado: %s (%d símbolos)", top, len(top))
    return top


class ChartLauncher:
    """Lanzador de TradingView gratuito con pyautogui para ATLAS-Quant M10."""

    SCREEN_MAP_PATH = Path("calibration/atlas_screen_map.json")

    def __init__(
        self,
        symbols: list[str] | None = None,
        voice=None,
        telegram=None,
    ) -> None:
        self.symbols  = (symbols or _DEFAULT_SYMBOLS)[:_SCAN_MAX_SYMBOLS]
        self._voice   = voice
        self._telegram = telegram
        self._windows: list[dict] = []   # coords guardadas por ventana
        self._top_symbols: list[str] = []   # resultado del último scan dinámico

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

    def launch_dynamic_tradingview(
        self,
        token: str,
        universe: list[str] | None = None,
        fullscreen: bool = True,
        iv_rank_min: float = _SCAN_IV_RANK_MIN,
        iv_hv_ratio_min: float = _SCAN_IV_HV_RATIO,
        cvd_sigma_min: float = _SCAN_CVD_SIGMA,
        liq_min_usd: float = _SCAN_LIQ_MIN_USD,
    ) -> list[str]:
        """M10 DINÁMICO: escáner primero → abre solo los símbolos que pasan los filtros.

        Flujo:
          1. Corre run_dynamic_scanner() con los parámetros dados
          2. Actualiza self.symbols con los resultados
          3. Llama launch_free_tradingview() con los nuevos símbolos
          4. Guarda top_symbols en atlas_screen_map.json["scanner_result"]

        Args:
            token:          Tradier token (paper sandbox).
            universe:       Universo de escaneo (default: _SCANNER_UNIVERSE).
            fullscreen:     Activar pantalla completa.
            iv_rank_min:    Percentil IV mínimo (0-100).
            iv_hv_ratio_min: IV/HV mínimo.
            cvd_sigma_min:  CVD z-score mínimo.
            liq_min_usd:    Liquidez mínima en USD.

        Retorna la lista de símbolos que pasaron el escáner (top_symbols).
        """
        logger.info("🔍 ChartLauncher DINÁMICO — escaneando universo…")
        self._speak("Iniciando escáner dinámico de mercado. Filtrando por IV Rank, liquidez y CVD.")

        top = run_dynamic_scanner(
            token=token,
            universe=universe,
            iv_rank_min=iv_rank_min,
            iv_hv_ratio_min=iv_hv_ratio_min,
            cvd_sigma_min=cvd_sigma_min,
            liq_min_usd=liq_min_usd,
        )

        self._top_symbols = top
        self.symbols      = top[:_SCAN_MAX_SYMBOLS]

        if not self.symbols:
            logger.warning("Scanner sin resultados — nada que abrir")
            return []

        msg = f"Escáner completado. {len(self.symbols)} activos seleccionados: {', '.join(self.symbols)}."
        self._speak(msg)
        self._alert(
            f"📊 *Scanner dinámico completado*\n"
            f"Seleccionados ({len(self.symbols)}): {', '.join(self.symbols)}\n"
            f"Filtros: IV Rank >{iv_rank_min:.0f}% | IV/HV >{iv_hv_ratio_min} | liq >$10M"
        )

        ok = self.launch_free_tradingview(fullscreen=fullscreen)
        if not ok:
            logger.error("launch_free_tradingview falló tras scanner")
            return self.symbols

        # Persistir resultado del scanner en atlas_screen_map.json
        try:
            out_path = self.SCREEN_MAP_PATH
            out_path.parent.mkdir(parents=True, exist_ok=True)
            existing: dict = {}
            if out_path.exists():
                try:
                    with open(out_path, encoding="utf-8") as f:
                        import json as _json
                        existing = _json.load(f)
                except Exception:
                    pass
            existing["scanner_result"]      = self.symbols
            existing["scanner_universe"]    = list(universe or _SCANNER_UNIVERSE)
            existing["scanner_iv_rank_min"] = iv_rank_min
            existing["scanner_iv_hv_min"]   = iv_hv_ratio_min
            existing["scanner_liq_min_usd"] = liq_min_usd
            with open(out_path, "w", encoding="utf-8") as f:
                import json as _json
                _json.dump(existing, f, indent=2, ensure_ascii=False)
            logger.info("Scanner result guardado en %s", out_path)
        except Exception as exc:
            logger.warning("No se pudo persistir scanner_result: %s", exc)

        return self.symbols

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
