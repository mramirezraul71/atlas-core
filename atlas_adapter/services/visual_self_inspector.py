"""
ATLAS Visual Self Inspector v1.0
=================================
Inspección autónoma del dashboard SPA v4 usando cámara física Insta360 X4/X5.

Arquitectura:
    - CameraInterface existente: RTMP 2880×1440 → EasyOCR pipeline (threads propios)
    - VisualSelfInspector: navega hash routes del SPA, espera frame RTMP fresco,
      parsea métricas de raw_texts (ya procesados), valida umbrales por módulo.
    - AtlasDoctor: task asyncio INDEPENDIENTE, NUNCA dentro del lock del nervous loop.

Flujo por módulo:
    1. webbrowser.open(f"http://127.0.0.1:8791/ui#{hash}") → SPA carga módulo
    2. Esperar frame CameraInterface con timestamp > t_navegación
    3. raw_texts → _parse_metrics() → dict métrico
    4. _validate_module() → VisualModuleResult
    5. Anomalías → doctor.report_visual_anomalies()

Consideraciones de seguridad:
    - RTMP URL no hardcodeado: ATLAS_VISUAL_RTMP_URL en env
    - Dashboard URL no hardcodeado: ATLAS_DASHBOARD_URL en env
    - Sin credenciales en código

CLI:
    python -m atlas_adapter.services.visual_self_inspector --module health
    python -m atlas_adapter.services.visual_self_inspector --daemon
    python -m atlas_adapter.services.visual_self_inspector --list
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import re
import subprocess
import sys
import time
import webbrowser
from statistics import fmean
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

# Asegurar import estable cuando se ejecuta como script directo
_PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

# Cargar atlas.env antes de leer env vars (cuando se corre como script CLI)
try:
    from dotenv import load_dotenv as _load_dotenv
    _load_dotenv(str(_PROJECT_ROOT / "config" / "atlas.env"), override=False)
except Exception:
    pass

logger = logging.getLogger("atlas.visual_inspector")

# ── Configuración desde entorno ────────────────────────────────────────────────
def _parse_camera_source(raw: str) -> "str | int":
    """Convierte env var a int si es un índice, o deja como URL RTMP."""
    return int(raw) if raw.lstrip("-").isdigit() else raw

_RTMP_URL: "str | int" = _parse_camera_source(
    os.getenv("ATLAS_VISUAL_RTMP_URL", "rtmp://192.168.1.10/live/atlas")
)
_DASHBOARD_URL = os.getenv(
    "ATLAS_DASHBOARD_URL",
    "http://127.0.0.1:8791/ui",
)
_PAGE_LOAD_WAIT = float(os.getenv("ATLAS_VISUAL_PAGE_LOAD_S", "3.0"))
_FRAME_TIMEOUT = float(os.getenv("ATLAS_VISUAL_FRAME_TIMEOUT_S", "10.0"))
_OCR_CONF_THRESHOLD = float(os.getenv("ATLAS_VISUAL_OCR_CONF_MIN", "0.55"))
_USE_HID_NAV = os.getenv("ATLAS_VISUAL_USE_HID_NAV", "false").strip().lower() in (
    "1", "true", "yes", "y", "on"
)

# ── Módulos SPA y sus hashes de ruta ──────────────────────────────────────────
# Orden: módulos críticos primero para detección rápida de fallos graves
MODULE_HASHES: Dict[str, str] = {
    "health":       "#/health",
    "trading":      "#/trading",
    "atlas_quant":  "#/atlas_quant",
    "memory":       "#/memory",
    "bitacora":     "#/bitacora",
    "audit":        "#/audit",
    "events":       "#/events",
    "healing":      "#/healing",
    "autonomy":     "#/autonomy",
    "approvals":    "#/approvals",
    "config":       "#/config",
    "voice":        "#/voice",
    "comms":        "#/comms",
    "api_explorer": "#/api_explorer",
    "learning":     "#/learning",
    "doctor":       "#/doctor",
    "cognitive":    "#/cognitive",
    "vision":       "#/vision",
    "chat":         "#/chat",
    "body_module":  "#/body_module",
}

# ── Reglas de validación por módulo ───────────────────────────────────────────
# Cada regla: metric_key → callable(valor: float) -> bool
# Solo métricas OCR-legibles con certeza (valores numéricos grandes o con %)
ModuleRules = Dict[str, Callable[[float], bool]]

MODULE_VALIDATION_RULES: Dict[str, ModuleRules] = {
    "health": {
        "cpu": lambda x: x < 90.0,
        "ram": lambda x: x < 90.0,
    },
    "trading": {
        "sharpe": lambda x: x > 0.0,
    },
    "atlas_quant": {
        "sharpe": lambda x: x > 0.0,
    },
    "memory": {
        "collections": lambda x: x >= 1.0,
    },
}

# ── Patrones de extracción OCR ────────────────────────────────────────────────
# Tupla: (regex_busqueda, metric_key)
# Los patrones son case-insensitive. Capturan el valor numérico.
OCR_METRIC_PATTERNS: List[Tuple[str, str]] = [
    (r"cpu\s*[:\s]\s*([\d.]+)\s*%?",          "cpu"),
    (r"ram\s*[:\s]\s*([\d.]+)\s*%?",          "ram"),
    (r"mem(?:oria)?\s*[:\s]\s*([\d.]+)\s*%?", "ram"),
    (r"sharpe\s*[:\s]\s*([+-]?[\d.]+)",        "sharpe"),
    (r"latencia?\s*[:\s]\s*([\d.]+)",          "latency_ms"),
    (r"latency\s*[:\s]\s*([\d.]+)",            "latency_ms"),
    (r"collections?\s*[:\s]\s*(\d+)",          "collections"),
    (r"score\s*[:\s]\s*([\d.]+)",              "score"),
    (r"p&?l\s*[:\s]\s*\$?\s*([+-]?[\d.]+)",   "pnl"),
    (r"win\s*rate\s*[:\s]\s*([\d.]+)\s*%?",   "win_rate"),
    (r"(\d{1,3}\.?\d*)\s*%\s+cpu",            "cpu"),   # "23.4% CPU" format
    (r"(\d{1,3}\.?\d*)\s*%\s+ram",            "ram"),
]


# ── Dataclasses ───────────────────────────────────────────────────────────────

@dataclass
class VisualModuleResult:
    """
    Resultado de inspección visual de un módulo SPA.

    Attributes:
        module:          Nombre del módulo (ej: 'health')
        ok:              True si todas las reglas pasan (o no hay reglas)
        metrics:         Métricas numéricas extraídas por OCR
        issues:          Lista de descripciones de fallos
        ocr_confidence:  Confianza promedio del OCR (0.0–1.0)
        frame_timestamp: Unix timestamp del frame de cámara usado
        tier:            Tier del Doctor (3=WARNING por defecto para visual)
    """
    module: str
    ok: bool
    metrics: Dict[str, float] = field(default_factory=dict)
    issues: List[str] = field(default_factory=list)
    ocr_confidence: float = 0.0
    frame_timestamp: float = field(default_factory=time.time)
    tier: int = 3

    def to_dict(self) -> Dict[str, Any]:
        return {
            "module":          self.module,
            "ok":              self.ok,
            "metrics":         self.metrics,
            "issues":          self.issues,
            "ocr_confidence":  round(self.ocr_confidence, 4),
            "frame_ts":        self.frame_timestamp,
            "tier":            self.tier,
        }


# ── Navegación autónoma (función de módulo, compartida por ambos inspectores) ──

def _autonomous_navigate(url: str) -> None:
    """
    Abre el browser de forma autónoma en Windows y lo trae al primer plano.

    Estrategia por capas (fallback progresivo):
      1. Ctrl+L via pyautogui: navega dentro del browser ya abierto (sin nueva pestaña).
      2. PowerShell Start-Process: abre/enfoca Chrome/Edge con la URL.
      3. cmd 'start': abre en browser default del sistema.
      4. webbrowser.open(): fallback estándar multiplataforma.

    Args:
        url: URL completa con hash (ej: http://127.0.0.1:8791/ui#/health)
    """
    if sys.platform == "win32":
        # Nivel 1: PowerShell Start-Process (fuerza primer plano)
        try:
            subprocess.Popen(
                ["powershell", "-WindowStyle", "Hidden", "-Command",
                 f'Start-Process "{url}"'],
                creationflags=subprocess.CREATE_NO_WINDOW,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            logging.getLogger("atlas.visual_inspector").debug(
                "[Navigate] PowerShell Start-Process → %s", url
            )
            return
        except Exception:
            pass

        # Nivel 2: cmd start
        try:
            subprocess.Popen(
                ["cmd", "/c", "start", "", url],
                creationflags=subprocess.CREATE_NO_WINDOW,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            logging.getLogger("atlas.visual_inspector").debug(
                "[Navigate] cmd start → %s", url
            )
            return
        except Exception:
            pass

    # Nivel 3 (opt-in): HID Ctrl+L + pegar desde clipboard
    if _USE_HID_NAV:
        try:
            import pyautogui  # type: ignore[import]
            try:
                import pyperclip  # type: ignore[import]
                pyperclip.copy(url)
            except Exception:
                subprocess.run(
                    ["clip"],
                    input=url.encode("utf-8"),
                    check=False,
                    capture_output=True,
                )

            pyautogui.hotkey("ctrl", "l")
            time.sleep(0.15)
            pyautogui.hotkey("ctrl", "a")
            pyautogui.hotkey("ctrl", "v")
            pyautogui.press("enter")
            logging.getLogger("atlas.visual_inspector").debug(
                "[Navigate] HID clipboard → %s", url
            )
            return
        except Exception:
            pass

    # Nivel 4: webbrowser estándar (cualquier OS)
    webbrowser.open(url, new=0, autoraise=True)
    logging.getLogger("atlas.visual_inspector").debug(
        "[Navigate] webbrowser.open → %s", url
    )


# ── Inspector principal ────────────────────────────────────────────────────────

class VisualSelfInspector:
    """
    Inspector visual autónomo del dashboard SPA v4 usando cámara física Insta360.

    Reutiliza CameraInterface existente (atlas_code_quant/hardware/camera_interface.py)
    que ya corre EasyOCR en background threads sobre el stream RTMP.
    No instancia un segundo OCR reader — usa latest_result().raw_texts.

    Args:
        rtmp_url:        URL RTMP de la Insta360. Default: ATLAS_VISUAL_RTMP_URL env.
        doctor:          Instancia AtlasDoctor. None = modo standalone/test.
        dashboard_url:   URL base del SPA. Default: ATLAS_DASHBOARD_URL env.
        page_load_wait_s: Segundos a esperar tras navegación browser.
        frame_timeout_s:  Timeout esperando frame fresco (post-navegación).
        modules:         Módulos a inspeccionar. None = todos (MODULE_HASHES).
    """

    def __init__(
        self,
        rtmp_url: "str | int" = _RTMP_URL,
        doctor: Optional[Any] = None,   # Any evita import circular en runtime
        dashboard_url: str = _DASHBOARD_URL,
        page_load_wait_s: float = _PAGE_LOAD_WAIT,
        frame_timeout_s: float = _FRAME_TIMEOUT,
        modules: Optional[List[str]] = None,
    ) -> None:
        # Importación lazy para no romper carga si cv2/easyocr no disponibles
        from atlas_code_quant.hardware.camera_interface import CameraInterface

        self._doctor = doctor
        self._dashboard_url = dashboard_url.rstrip("/")
        self._page_load_wait_s = page_load_wait_s
        self._frame_timeout_s = frame_timeout_s
        self._modules: List[str] = modules or list(MODULE_HASHES.keys())
        self._camera_started = False

        self.camera = CameraInterface(
            rtmp_url=rtmp_url,
            ocr_languages=["es", "en"],
            use_gpu=False,          # Inspección no requiere GPU
            frame_queue_size=10,    # Cola pequeña — solo necesitamos último frame
        )

        self._last_results: List[VisualModuleResult] = []
        self._last_run_ts: Optional[float] = None

        logger.debug(
            "[VisualInspector] Inicializado. rtmp=%s dashboard=%s módulos=%d",
            rtmp_url, dashboard_url, len(self._modules),
        )

    # ── Ciclo de vida ─────────────────────────────────────────────────────────

    def start_camera(self) -> None:
        """
        Inicia el stream RTMP y el pipeline OCR en background.
        Bloquea ~2s para warm-up del primer frame.
        """
        if self._camera_started:
            return
        self.camera.start()
        self._camera_started = True
        time.sleep(2.0)  # Warm-up: primer frame antes de inspeccionar
        logger.info("[VisualInspector] Cámara RTMP iniciada → %s", self.camera.rtmp_url)

    def stop_camera(self) -> None:
        """Detiene stream RTMP y libera recursos."""
        if self._camera_started:
            self.camera.stop()
            self._camera_started = False
            logger.info("[VisualInspector] Cámara RTMP detenida.")

    # ── Entry point asíncrono ─────────────────────────────────────────────────

    async def run_full_inspection(self) -> List[VisualModuleResult]:
        """
        Ejecuta la inspección completa de todos los módulos configurados.

        Diseñado para llamarse como asyncio.Task independiente, FUERA del lock
        del nervous_system_loop del Doctor.

        Returns:
            Lista de VisualModuleResult, uno por módulo inspeccionado.
        """
        logger.info(
            "[VisualInspector] === INICIO INSPECCIÓN FÍSICA (%d módulos) ===",
            len(self._modules),
        )

        if not self._camera_started:
            self.start_camera()

        # Bloqueo sincrónico pesado (browser + sleeps) → thread separado
        results: List[VisualModuleResult] = await asyncio.to_thread(
            self._physical_inspection_loop
        )

        self._last_results = results
        self._last_run_ts = time.time()

        anomalies = [r for r in results if not r.ok]
        if anomalies and self._doctor is not None:
            self._report_to_doctor(anomalies)

        ok_count = sum(1 for r in results if r.ok)
        logger.info(
            "[VisualInspector] === FIN | %d/%d OK | %d anomalías ===",
            ok_count, len(results), len(anomalies),
        )
        return results

    # ── Loop sincrónico (corre en thread) ─────────────────────────────────────

    def _physical_inspection_loop(self) -> List[VisualModuleResult]:
        """
        Navega cada módulo, captura frame RTMP, extrae métricas y valida.
        Ejecutado en asyncio.to_thread — no bloquea el event loop.

        Returns:
            Lista de VisualModuleResult (incluye tanto OK como anomalías).
        """
        results: List[VisualModuleResult] = []

        for module in self._modules:
            try:
                result = self._inspect_single_module(module)
                results.append(result)

                # Log estructurado — sin print(), siempre logger
                status = "✓" if result.ok else "✗"
                metrics_str = " | ".join(
                    f"{k}={v:.2f}" for k, v in result.metrics.items()
                ) or "sin métricas OCR"
                issues_str = f" → {result.issues}" if result.issues else ""
                logger.info(
                    "[VisualInspector] %s %-16s  conf=%.2f  %s%s",
                    status, module,
                    result.ocr_confidence,
                    metrics_str,
                    issues_str,
                )

            except Exception as exc:
                logger.error(
                    "[VisualInspector] Excepción en módulo '%s': %s",
                    module, exc, exc_info=True,
                )
                results.append(VisualModuleResult(
                    module=module,
                    ok=False,
                    issues=[f"exception_during_inspection: {exc}"],
                    tier=3,
                ))

        return results

    # ── Inspección de un módulo ────────────────────────────────────────────────

    def _inspect_single_module(self, module: str) -> VisualModuleResult:
        """
        Navega al módulo SPA, espera frame fresco de la cámara, extrae y valida.

        Args:
            module: Nombre del módulo (debe estar en MODULE_HASHES o se usa #/{module})

        Returns:
            VisualModuleResult con el resultado de validación.
        """
        hash_path = MODULE_HASHES.get(module, f"#{module}")
        url = f"{self._dashboard_url}{hash_path}"
        t_navigate = time.time()

        # Navegación autónoma Windows: PowerShell Start-Process → cmd start → webbrowser
        _autonomous_navigate(url)
        time.sleep(self._page_load_wait_s)   # SPA necesita tiempo de hidratación

        # Esperar frame de cámara posterior a la navegación
        ocr_result = self._wait_for_fresh_frame(after_ts=t_navigate)
        if ocr_result is None:
            return VisualModuleResult(
                module=module,
                ok=False,
                issues=["camera_timeout: no llegó frame fresco dentro del timeout"],
                frame_timestamp=t_navigate,
                tier=3,
            )

        # Parsear métricas desde raw_texts (ya procesados por CameraInterface)
        metrics = self._parse_metrics(ocr_result.raw_texts)

        return self._validate_module(
            module=module,
            metrics=metrics,
            ocr_confidence=ocr_result.ocr_confidence,
            frame_timestamp=ocr_result.timestamp,
        )

    # ── Freshness de frame ────────────────────────────────────────────────────

    def _wait_for_fresh_frame(
        self,
        after_ts: float,
        poll_interval_s: float = 0.2,
    ) -> Optional[Any]:  # Optional[OCRResult]
        """
        Espera hasta obtener un OCRResult con timestamp > after_ts.

        Garantiza que el frame capturado es posterior a la navegación,
        no un frame cacheado del módulo anterior.

        Args:
            after_ts:        Unix timestamp mínimo del frame deseado.
            poll_interval_s: Intervalo de polling (s).

        Returns:
            OCRResult si llega dentro del timeout; None si agota el tiempo.
        """
        deadline = time.monotonic() + self._frame_timeout_s
        while time.monotonic() < deadline:
            result = self.camera.latest_result()
            if result is not None and result.timestamp > after_ts:
                return result
            time.sleep(poll_interval_s)

        logger.warning(
            "[VisualInspector] Timeout esperando frame (after_ts=%.2f, timeout=%.0fs)",
            after_ts, self._frame_timeout_s,
        )
        return None

    # ── Parsing OCR ──────────────────────────────────────────────────────────

    def _parse_metrics(self, raw_texts: List[str]) -> Dict[str, float]:
        """
        Extrae métricas numéricas de los textos OCR del dashboard.

        Combina todos los textos en un string y aplica cada patrón regex.
        Los patrones son tolerantes: 'CPU: 23.4', 'CPU 23.4%', '23.4% CPU'.

        Args:
            raw_texts: Lista de strings extraídos por EasyOCR (CameraInterface)

        Returns:
            Dict {metric_key: float}. Solo incluye métricas detectadas.
        """
        metrics: Dict[str, float] = {}
        if not raw_texts:
            return metrics

        # Una sola cadena para matching cross-word (ej: "CPU : 23")
        combined = " ".join(raw_texts)

        for pattern, metric_key in OCR_METRIC_PATTERNS:
            if metric_key in metrics:
                continue   # Ya encontrado por patrón anterior
            match = re.search(pattern, combined, re.IGNORECASE)
            if not match:
                continue
            try:
                raw_val = match.group(1).strip().replace(",", ".").lstrip("+")
                metrics[metric_key] = float(raw_val)
            except (ValueError, IndexError):
                logger.debug(
                    "[VisualInspector] No se pudo parsear '%s' para métrica '%s'",
                    match.group(0), metric_key,
                )

        return metrics

    # ── Validación por módulo ─────────────────────────────────────────────────

    def _validate_module(
        self,
        module: str,
        metrics: Dict[str, float],
        ocr_confidence: float,
        frame_timestamp: float,
    ) -> VisualModuleResult:
        """
        Aplica las reglas de MODULE_VALIDATION_RULES para el módulo dado.

        Si la confianza OCR es menor al umbral configurado Y hay reglas definidas,
        la inspección se marca como 'indeterminate' (ok=True, issue logged)
        para evitar falsos positivos por mala captura.

        Args:
            module:          Nombre del módulo SPA.
            metrics:         Métricas extraídas por _parse_metrics.
            ocr_confidence:  Confianza promedio EasyOCR del frame.
            frame_timestamp: Unix timestamp del frame.

        Returns:
            VisualModuleResult con ok=False si alguna regla falla.
        """
        rules = MODULE_VALIDATION_RULES.get(module, {})
        issues: List[str] = []

        # Umbral mínimo de confianza para aplicar validación
        if rules and ocr_confidence < _OCR_CONF_THRESHOLD:
            return VisualModuleResult(
                module=module,
                ok=True,   # No declaramos fallo — imagen de baja calidad
                metrics=metrics,
                issues=[
                    f"ocr_confidence_baja: {ocr_confidence:.2f} "
                    f"< umbral {_OCR_CONF_THRESHOLD} — validación omitida"
                ],
                ocr_confidence=ocr_confidence,
                frame_timestamp=frame_timestamp,
                tier=3,
            )

        for metric_key, rule_fn in rules.items():
            value = metrics.get(metric_key)
            if value is None:
                issues.append(f"metrica_no_detectada: '{metric_key}'")
            elif not rule_fn(value):
                issues.append(
                    f"{metric_key}={value:.3f} viola umbral definido"
                )

        return VisualModuleResult(
            module=module,
            ok=len(issues) == 0,
            metrics=metrics,
            issues=issues,
            ocr_confidence=ocr_confidence,
            frame_timestamp=frame_timestamp,
            tier=3,
        )

    # ── Reporte al Doctor ─────────────────────────────────────────────────────

    def _report_to_doctor(self, anomalies: List[VisualModuleResult]) -> None:
        """
        Convierte VisualModuleResults en anomalías y las registra en AtlasDoctor.

        Args:
            anomalies: Módulos con ok=False.
        """
        if self._doctor is None:
            return
        try:
            payload = [r.to_dict() for r in anomalies]
            self._doctor.report_visual_anomalies(payload)
            logger.info(
                "[VisualInspector] %d anomalías enviadas al Doctor", len(anomalies)
            )
        except Exception as exc:
            logger.error(
                "[VisualInspector] Error reportando al Doctor: %s", exc
            )

    # ── API pública de consulta ───────────────────────────────────────────────

    def summary(self) -> Dict[str, Any]:
        """
        Resumen de la última inspección.

        Returns:
            Dict con estadísticas: total, ok, anomalías, confianza promedio.
        """
        if not self._last_results:
            return {"completed": False}

        total = len(self._last_results)
        ok_count = sum(1 for r in self._last_results if r.ok)
        confs = [r.ocr_confidence for r in self._last_results if r.ocr_confidence > 0]
        avg_conf = float(fmean(confs)) if confs else 0.0

        return {
            "completed":          True,
            "last_run_ts":        self._last_run_ts,
            "modules_total":      total,
            "modules_ok":         ok_count,
            "modules_anomaly":    total - ok_count,
            "avg_ocr_confidence": round(avg_conf, 4),
            "anomalies": [
                r.to_dict() for r in self._last_results if not r.ok
            ],
        }


# ── CLI standalone ────────────────────────────────────────────────────────────

def _setup_logging(level: str = "INFO") -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


async def _cli_inspect(
    modules: Optional[List[str]],
    rtmp_url: str,
    dashboard_url: str,
) -> None:
    inspector = VisualSelfInspector(
        rtmp_url=rtmp_url,
        doctor=None,  # Standalone: sin reporte al Doctor
        dashboard_url=dashboard_url,
        modules=modules,
    )
    results = await inspector.run_full_inspection()
    inspector.stop_camera()
    print(json.dumps([r.to_dict() for r in results], indent=2, ensure_ascii=False))


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="python -m atlas_adapter.services.visual_self_inspector",
        description="ATLAS Visual Self Inspector — Inspección de dashboard via Insta360",
    )
    parser.add_argument(
        "--module", metavar="MODULE",
        help="Inspeccionar solo este módulo (ej: health)",
    )
    parser.add_argument(
        "--modules", metavar="M1,M2",
        help="Lista de módulos separados por coma",
    )
    parser.add_argument(
        "--list", action="store_true",
        help="Listar módulos disponibles y salir",
    )
    parser.add_argument(
        "--daemon", action="store_true",
        help="Correr como daemon: inspección cada 15 min",
    )
    parser.add_argument(
        "--rtmp", default=_RTMP_URL,
        help=f"URL RTMP Insta360 (default: {_RTMP_URL})",
    )
    parser.add_argument(
        "--dashboard", default=_DASHBOARD_URL,
        help=f"URL del dashboard SPA (default: {_DASHBOARD_URL})",
    )
    parser.add_argument(
        "--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Nivel de logging",
    )
    args = parser.parse_args()
    _setup_logging(args.log_level)

    if args.list:
        print("Módulos disponibles:")
        for name, hash_path in MODULE_HASHES.items():
            rules = MODULE_VALIDATION_RULES.get(name, {})
            rule_keys = list(rules.keys()) or ["sin reglas"]
            print(f"  {name:<18} {hash_path:<22} valida: {rule_keys}")
        sys.exit(0)

    modules: Optional[List[str]] = None
    if args.module:
        if args.module not in MODULE_HASHES:
            print(f"Módulo '{args.module}' no encontrado. Use --list para ver disponibles.")
            sys.exit(1)
        modules = [args.module]
    elif args.modules:
        modules = [m.strip() for m in args.modules.split(",") if m.strip()]

    if args.daemon:
        async def _daemon_loop() -> None:
            inspector = VisualSelfInspector(
                rtmp_url=args.rtmp,
                doctor=None,
                dashboard_url=args.dashboard,
                modules=modules,
            )
            inspector.start_camera()
            try:
                while True:
                    await inspector.run_full_inspection()
                    summary = inspector.summary()
                    logger.info(
                        "[Daemon] Ciclo completo. OK=%d/%d anomalías=%d conf=%.2f",
                        summary.get("modules_ok", 0),
                        summary.get("modules_total", 0),
                        summary.get("modules_anomaly", 0),
                        summary.get("avg_ocr_confidence", 0.0),
                    )
                    await asyncio.sleep(15 * 60)  # 15 minutos
            except (KeyboardInterrupt, asyncio.CancelledError):
                logger.info("[Daemon] Deteniendo inspector visual...")
            finally:
                inspector.stop_camera()

        asyncio.run(_daemon_loop())
    else:
        asyncio.run(
            _cli_inspect(
                modules=modules,
                rtmp_url=args.rtmp,
                dashboard_url=args.dashboard,
            )
        )


if __name__ == "__main__":
    main()
