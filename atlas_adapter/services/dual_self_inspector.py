"""
ATLAS Dual Self Inspector v1.0
================================
Validación cruzada del dashboard SPA v4 usando DOS fuentes de captura simultáneas:

  1. PYTHON SCREENSHOT: mss captura la pantalla localmente → EasyOCR → métricas digitales
  2. FÍSICA INSTA360:   CameraInterface RTMP (cámara apuntando al monitor) →
                        raw_texts ya procesados por CameraInterface → métricas visuales

Si las métricas difieren >10% → TIER2 DEGRADED (posible error de rendering o
lag de stream RTMP).

Arquitectura de concurrencia:
    Ambas capturas se lanzan en PARALELO por módulo via asyncio.gather().
    Tiempo por módulo ≈ max(t_screenshot, t_físico) en lugar de la suma.
    El loop completo (20 módulos) corre en asyncio.Task independiente del
    nervous_system_loop del Doctor.

Consideraciones de seguridad:
    - Credenciales/URLs nunca hardcodeadas: usar env ATLAS_VISUAL_RTMP_URL,
      ATLAS_DASHBOARD_URL.
    - Un solo easyocr.Reader a nivel de módulo (singleton lazy) evita
      cargas repetidas (~2GB RAM, ~30s init).

CLI:
    python -m atlas_adapter.services.dual_self_inspector --test health
    python -m atlas_adapter.services.dual_self_inspector --daemon
    python -m atlas_adapter.services.dual_self_inspector --list
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import subprocess
import sys
import time
import webbrowser
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# Asegurar que C:\ATLAS_PUSH esté en sys.path cuando se corre como script directo
_PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

import numpy as np

# Cargar atlas.env antes de leer env vars (cuando se corre como script CLI)
try:
    from dotenv import load_dotenv as _load_dotenv
    _load_dotenv(str(_PROJECT_ROOT / "config" / "atlas.env"), override=False)
except Exception:
    pass

# Importar constantes compartidas del inspector visual (sin importar la clase)
from atlas_adapter.services.visual_self_inspector import (
    MODULE_HASHES,
    OCR_METRIC_PATTERNS,
    _OCR_CONF_THRESHOLD,
)

logger = logging.getLogger("atlas.dual_inspector")

# ── Configuración desde entorno ────────────────────────────────────────────────
def _parse_camera_source(raw: str) -> "str | int":
    return int(raw) if raw.lstrip("-").isdigit() else raw

_RTMP_URL: "str | int" = _parse_camera_source(
    os.getenv("ATLAS_VISUAL_RTMP_URL", "rtmp://192.168.1.10/live/atlas")
)
_DASHBOARD_URL = os.getenv("ATLAS_DASHBOARD_URL", "http://127.0.0.1:8791/ui")
_PAGE_LOAD_WAIT = float(os.getenv("ATLAS_DUAL_PAGE_LOAD_S", "3.0"))
_FRAME_TIMEOUT = float(os.getenv("ATLAS_DUAL_FRAME_TIMEOUT_S", "10.0"))
_MISMATCH_THRESHOLD = float(os.getenv("ATLAS_DUAL_MISMATCH_THRESHOLD", "0.10"))  # 10%
# Índice del monitor mss que muestra el dashboard (1 = primer monitor físico)
_MSS_MONITOR_IDX = int(os.getenv("ATLAS_DUAL_MONITOR_IDX", "1"))

# ── Singleton EasyOCR para capturas de screenshot ─────────────────────────────
# Un solo reader por proceso: la carga tarda ~30s y consume ~2GB RAM.
_OCR_READER: Optional[Any] = None
_OCR_READER_LOCK = asyncio.Lock()    # async lock para init lazy thread-safe


def _get_ocr_reader_sync() -> Optional[Any]:
    """
    Obtiene (o crea) el singleton EasyOCR. Llamar desde thread (asyncio.to_thread).
    No thread-safe por sí solo — el caller debe garantizar que solo un thread
    llama a esto a la vez durante el init (gestionado por _ensure_ocr_reader).
    """
    global _OCR_READER
    if _OCR_READER is not None:
        return _OCR_READER
    try:
        import easyocr  # type: ignore[import]
        _OCR_READER = easyocr.Reader(["es", "en"], gpu=False, verbose=False)
        logger.info("[DualInspector] EasyOCR reader inicializado (cpu)")
    except ImportError:
        logger.warning("[DualInspector] easyocr no disponible — screenshot OCR deshabilitado")
    return _OCR_READER


async def _ensure_ocr_reader() -> Optional[Any]:
    """Garantiza que el singleton EasyOCR se inicializa una sola vez en modo async."""
    async with _OCR_READER_LOCK:
        if _OCR_READER is None:
            await asyncio.to_thread(_get_ocr_reader_sync)
    return _OCR_READER


# ── Utilidades ────────────────────────────────────────────────────────────────

def _relative_diff(a: float, b: float) -> float:
    """
    Diferencia porcentual simétrica entre dos valores (sMAPE single-point).
    Evita división por cero cuando ambos son 0.

    Returns:
        Valor en [0, 2.0]. 0.0 = idénticos; 1.0 = 100% diferencia.
    """
    denom = (abs(a) + abs(b)) / 2.0
    if denom < 1e-9:
        return 0.0   # Ambos cero → sin divergencia
    return abs(a - b) / denom


def _parse_metrics(raw_texts: List[str]) -> Dict[str, float]:
    """
    Extrae métricas numéricas de textos OCR usando OCR_METRIC_PATTERNS compartidos.

    Args:
        raw_texts: Strings crudos del OCR (de mss+easyocr o de CameraInterface)

    Returns:
        Dict {metric_key: float}
    """
    import re
    metrics: Dict[str, float] = {}
    if not raw_texts:
        return metrics
    combined = " ".join(raw_texts)
    for pattern, metric_key in OCR_METRIC_PATTERNS:
        if metric_key in metrics:
            continue
        match = re.search(pattern, combined, re.IGNORECASE)
        if not match:
            continue
        try:
            raw_val = match.group(1).strip().replace(",", ".").lstrip("+")
            metrics[metric_key] = float(raw_val)
        except (ValueError, IndexError):
            pass
    return metrics


# ── Dataclasses ───────────────────────────────────────────────────────────────

@dataclass
class DualCaptureResult:
    """
    Métricas capturadas por UNA fuente (screenshot o física).

    Attributes:
        source:     'screenshot' | 'physical'
        metrics:    Métricas numéricas extraídas
        confidence: Confianza OCR promedio (0.0–1.0)
        elapsed_ms: Tiempo de captura en ms
        error:      Descripción del error si la captura falló
    """
    source: str
    metrics: Dict[str, float] = field(default_factory=dict)
    confidence: float = 0.0
    elapsed_ms: float = 0.0
    error: Optional[str] = None

    @property
    def ok(self) -> bool:
        return self.error is None


@dataclass
class DualModuleResult:
    """
    Resultado de la validación cruzada de un módulo SPA.

    Attributes:
        module:      Nombre del módulo
        match:       True si las dos fuentes concuerdan (diff < threshold)
        screenshot:  Resultado de la captura Python/mss
        physical:    Resultado de la captura Insta360
        divergences: Dict {metric_key: diff_porcentual} para claves comunes
        max_diff:    Diferencia máxima encontrada entre fuentes
        tier:        Tier del Doctor (2 = DEGRADED para mismatch)
    """
    module: str
    match: bool
    screenshot: DualCaptureResult
    physical: DualCaptureResult
    divergences: Dict[str, float] = field(default_factory=dict)
    max_diff: float = 0.0
    tier: int = 2

    def to_dict(self) -> Dict[str, Any]:
        return {
            "module":       self.module,
            "match":        self.match,
            "max_diff":     round(self.max_diff, 4),
            "divergences":  {k: round(v, 4) for k, v in self.divergences.items()},
            "screenshot":   {
                "metrics":    self.screenshot.metrics,
                "confidence": round(self.screenshot.confidence, 4),
                "elapsed_ms": round(self.screenshot.elapsed_ms, 1),
                "error":      self.screenshot.error,
            },
            "physical": {
                "metrics":    self.physical.metrics,
                "confidence": round(self.physical.confidence, 4),
                "elapsed_ms": round(self.physical.elapsed_ms, 1),
                "error":      self.physical.error,
            },
            "tier": self.tier,
        }


# ── Inspector principal ────────────────────────────────────────────────────────

class DualSelfInspector:
    """
    Inspector de validación cruzada: screenshot local vs cámara física Insta360.

    Para cada módulo SPA:
      1. Navega la URL hash en el browser
      2. Lanza en PARALELO: captura mss+OCR  y  lectura RTMP CameraInterface
      3. Parsea métricas de ambas fuentes con los mismos patrones
      4. Calcula divergencia simétrica por métrica
      5. TIER2 si max_diff > _MISMATCH_THRESHOLD

    Args:
        rtmp_url:         URL RTMP Insta360. Default: ATLAS_VISUAL_RTMP_URL env.
        doctor:           Instancia AtlasDoctor. None = modo standalone.
        dashboard_url:    URL base del SPA.
        page_load_wait_s: Segundos a esperar tras navegación browser.
        frame_timeout_s:  Timeout esperando frame fresco de cámara física.
        mismatch_threshold: Umbral de divergencia (default 0.10 = 10%).
        monitor_idx:      Índice mss del monitor con el dashboard.
        modules:          Módulos a inspeccionar. None = todos.
    """

    def __init__(
        self,
        rtmp_url: "str | int" = _RTMP_URL,
        doctor: Optional[Any] = None,
        dashboard_url: str = _DASHBOARD_URL,
        page_load_wait_s: float = _PAGE_LOAD_WAIT,
        frame_timeout_s: float = _FRAME_TIMEOUT,
        mismatch_threshold: float = _MISMATCH_THRESHOLD,
        monitor_idx: int = _MSS_MONITOR_IDX,
        modules: Optional[List[str]] = None,
    ) -> None:
        # CameraInterface: import lazy para tolerancia a entornos sin cv2
        from atlas_code_quant.hardware.camera_interface import CameraInterface

        self._doctor = doctor
        self._dashboard_url = dashboard_url.rstrip("/")
        self._page_load_wait_s = page_load_wait_s
        self._frame_timeout_s = frame_timeout_s
        self._mismatch_threshold = mismatch_threshold
        self._monitor_idx = monitor_idx
        self._modules: List[str] = modules or list(MODULE_HASHES.keys())
        self._camera_started = False

        self.camera = CameraInterface(
            rtmp_url=rtmp_url,
            ocr_languages=["es", "en"],
            use_gpu=False,
            frame_queue_size=5,
        )

        self._last_results: List[DualModuleResult] = []
        self._last_run_ts: Optional[float] = None

        logger.debug(
            "[DualInspector] Inicializado. rtmp=%s dashboard=%s módulos=%d threshold=%.0f%%",
            rtmp_url, dashboard_url, len(self._modules), mismatch_threshold * 100,
        )

    # ── Ciclo de vida ─────────────────────────────────────────────────────────

    def start_camera(self) -> None:
        """Inicia el stream RTMP + pipeline OCR de CameraInterface."""
        if self._camera_started:
            return
        self.camera.start()
        self._camera_started = True
        time.sleep(2.0)  # Warm-up
        logger.info("[DualInspector] Cámara RTMP iniciada → %s", self.camera.rtmp_url)

    def stop_camera(self) -> None:
        if self._camera_started:
            self.camera.stop()
            self._camera_started = False

    # ── Entry point asíncrono ─────────────────────────────────────────────────

    async def run_dual_inspection(self) -> List[DualModuleResult]:
        """
        Ejecuta la inspección dual de todos los módulos configurados.

        Diseñado para llamarse como asyncio.Task independiente, FUERA del
        lock del nervous_system_loop del Doctor.

        Returns:
            Lista de DualModuleResult, uno por módulo.
        """
        logger.info(
            "[DualInspector] === INICIO DUAL INSPECTION (%d módulos, threshold=%.0f%%) ===",
            len(self._modules), self._mismatch_threshold * 100,
        )

        if not self._camera_started:
            self.start_camera()

        # Asegurar EasyOCR listo antes del loop (init tarda ~30s, mejor hacerlo ahora)
        await _ensure_ocr_reader()

        results: List[DualModuleResult] = []
        for module in self._modules:
            try:
                result = await self._inspect_module(module)
                results.append(result)
                self._log_result(result)
            except Exception as exc:
                logger.error(
                    "[DualInspector] Excepción en módulo '%s': %s",
                    module, exc, exc_info=True,
                )
                empty = DualCaptureResult(source="error", error=str(exc))
                results.append(DualModuleResult(
                    module=module,
                    match=False,
                    screenshot=empty,
                    physical=empty,
                    tier=2,
                ))

        self._last_results = results
        self._last_run_ts = time.time()

        mismatches = [r for r in results if not r.match]
        if mismatches and self._doctor is not None:
            self._report_to_doctor(mismatches)

        ok_count = sum(1 for r in results if r.match)
        logger.info(
            "[DualInspector] === FIN | %d/%d OK | %d mismatches ===",
            ok_count, len(results), len(mismatches),
        )
        return results

    # ── Inspección de un módulo ────────────────────────────────────────────────

    async def _inspect_module(self, module: str) -> DualModuleResult:
        """
        Navega al módulo y lanza SIMULTÁNEAMENTE las dos capturas en threads.

        El parallelismo real se consigue con asyncio.gather sobre asyncio.to_thread:
        - Thread 1: mss screenshot + EasyOCR (bloqueante ~2–10s)
        - Thread 2: esperar frame RTMP fresco + parsear raw_texts (bloqueante ≤10s)

        Args:
            module: Nombre del módulo SPA

        Returns:
            DualModuleResult con resultados de ambas fuentes y validación cruzada.
        """
        hash_path = MODULE_HASHES.get(module, f"#{module}")
        url = f"{self._dashboard_url}{hash_path}"
        t_navigate = time.time()

        # Navegación autónoma: abre el browser en primer plano
        await asyncio.to_thread(self._autonomous_navigate, url)
        await asyncio.sleep(self._page_load_wait_s)

        # CAPTURA PARALELA: screenshot + física
        screenshot_capture, physical_capture = await asyncio.gather(
            asyncio.to_thread(self._capture_screenshot),
            asyncio.to_thread(self._capture_physical, t_navigate),
            return_exceptions=False,
        )

        # Validación cruzada
        return self._validate_dual(module, screenshot_capture, physical_capture)

    # ── Navegación autónoma ───────────────────────────────────────────────────

    def _autonomous_navigate(self, url: str) -> None:
        """
        Abre el browser de forma autónoma en Windows, trayéndolo al primer plano.

        Estrategia por capas (fallback progresivo):
          1. PowerShell Start-Process: abre/enfoca Chrome/Edge con la URL.
             Fuerza la ventana al frente — más fiable que webbrowser en Windows.
          2. Si falla: cmd 'start' (abre en browser default del sistema).
          3. Fallback final: webbrowser.open() estándar.

        También intenta usar las manos HID (Ctrl+L + URL) para navegar dentro
        de una ventana ya abierta, evitando abrir una nueva pestaña cada vez.

        Args:
            url: URL completa con hash (ej: http://127.0.0.1:8791/ui#/health)
        """
        # Intentar navegar con Ctrl+L en ventana existente (más rápido, sin nueva pestaña)
        _navigated_hid = self._navigate_via_hid(url)
        if _navigated_hid:
            logger.debug("[DualInspector] HID Ctrl+L → %s", url)
            return

        # Fallback 1: PowerShell Start-Process (fuerza primer plano en Windows)
        if sys.platform == "win32":
            try:
                subprocess.Popen(
                    ["powershell", "-WindowStyle", "Hidden", "-Command",
                     f'Start-Process "{url}"'],
                    creationflags=subprocess.CREATE_NO_WINDOW,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                logger.debug("[DualInspector] PowerShell Start-Process → %s", url)
                return
            except Exception as exc:
                logger.debug("[DualInspector] PowerShell navigate failed: %s", exc)

            # Fallback 2: cmd start (default browser, trae al frente)
            try:
                subprocess.Popen(
                    ["cmd", "/c", "start", "", url],
                    creationflags=subprocess.CREATE_NO_WINDOW,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                logger.debug("[DualInspector] cmd start → %s", url)
                return
            except Exception as exc:
                logger.debug("[DualInspector] cmd start failed: %s", exc)

        # Fallback 3: webbrowser estándar (cualquier OS)
        webbrowser.open(url, new=0, autoraise=True)
        logger.debug("[DualInspector] webbrowser.open → %s", url)

    def _navigate_via_hid(self, url: str) -> bool:
        """
        Navega a una URL dentro de la ventana activa usando Ctrl+L (barra de dirección).
        Solo funciona si el browser ya está abierto y en primer plano.

        Args:
            url: URL de destino

        Returns:
            True si la navegación HID se ejecutó sin error; False si no disponible.
        """
        try:
            import pyautogui  # type: ignore[import]
            # Abrir barra de dirección (Chrome/Edge/Firefox)
            pyautogui.hotkey("ctrl", "l")
            time.sleep(0.2)
            # Escribir URL y confirmar
            pyautogui.typewrite(url, interval=0.02)
            pyautogui.press("enter")
            return True
        except Exception:
            return False

    # ── Captura screenshot (thread) ───────────────────────────────────────────

    def _capture_screenshot(self) -> DualCaptureResult:
        """
        Captura la pantalla con mss y corre EasyOCR.
        Ejecutado en thread por asyncio.to_thread.

        Returns:
            DualCaptureResult con métricas extraídas del screenshot.
        """
        t0 = time.monotonic()
        ocr_reader = _OCR_READER   # Ya inicializado por _ensure_ocr_reader

        if ocr_reader is None:
            return DualCaptureResult(
                source="screenshot",
                error="easyocr no disponible",
            )

        try:
            import mss
            import numpy as np_local
            from PIL import Image

            with mss.mss() as sct:
                if self._monitor_idx >= len(sct.monitors):
                    logger.warning(
                        "[DualInspector] Monitor idx=%d no existe (disponibles: %d). "
                        "Usando monitor 1.",
                        self._monitor_idx, len(sct.monitors) - 1,
                    )
                    monitor_idx = min(1, len(sct.monitors) - 1)
                else:
                    monitor_idx = self._monitor_idx

                monitor_def = sct.monitors[monitor_idx]
                shot = sct.grab(monitor_def)

            # BGRA → numpy RGB para EasyOCR
            img = Image.frombytes("RGB", shot.size, shot.bgra, "raw", "BGRX")
            img_array = np_local.array(img)

        except Exception as exc:
            logger.error("[DualInspector] mss capture error: %s", exc)
            return DualCaptureResult(
                source="screenshot",
                elapsed_ms=(time.monotonic() - t0) * 1000,
                error=f"mss_error: {exc}",
            )

        try:
            ocr_out = ocr_reader.readtext(img_array)
            texts = [text for (_, text, _) in ocr_out if text.strip()]
            confs = [conf for (_, _, conf) in ocr_out]
            avg_conf = float(np.mean(confs)) if confs else 0.0
            metrics = _parse_metrics(texts)
        except Exception as exc:
            logger.error("[DualInspector] OCR screenshot error: %s", exc)
            return DualCaptureResult(
                source="screenshot",
                elapsed_ms=(time.monotonic() - t0) * 1000,
                error=f"ocr_error: {exc}",
            )

        return DualCaptureResult(
            source="screenshot",
            metrics=metrics,
            confidence=avg_conf,
            elapsed_ms=(time.monotonic() - t0) * 1000,
        )

    # ── Captura física Insta360 (thread) ──────────────────────────────────────

    def _capture_physical(self, after_ts: float) -> DualCaptureResult:
        """
        Espera un frame fresco de CameraInterface y parsea sus raw_texts.
        Ejecutado en thread por asyncio.to_thread.

        NO corre EasyOCR de nuevo — usa raw_texts ya procesados por el
        pipeline interno de CameraInterface (evita doble carga).

        Args:
            after_ts: Unix timestamp mínimo del frame (posterior a navegación)

        Returns:
            DualCaptureResult con métricas extraídas del frame RTMP.
        """
        t0 = time.monotonic()
        deadline = time.monotonic() + self._frame_timeout_s
        poll_interval = 0.2

        while time.monotonic() < deadline:
            result = self.camera.latest_result()
            if result is not None and result.timestamp > after_ts:
                metrics = _parse_metrics(result.raw_texts)
                return DualCaptureResult(
                    source="physical",
                    metrics=metrics,
                    confidence=result.ocr_confidence,
                    elapsed_ms=(time.monotonic() - t0) * 1000,
                )
            time.sleep(poll_interval)

        logger.warning(
            "[DualInspector] Timeout esperando frame físico (after=%.2f, timeout=%.0fs)",
            after_ts, self._frame_timeout_s,
        )
        return DualCaptureResult(
            source="physical",
            elapsed_ms=(time.monotonic() - t0) * 1000,
            error=f"camera_timeout: no frame after {self._frame_timeout_s:.0f}s",
        )

    # ── Validación cruzada ────────────────────────────────────────────────────

    def _validate_dual(
        self,
        module: str,
        screenshot: DualCaptureResult,
        physical: DualCaptureResult,
    ) -> DualModuleResult:
        """
        Compara las métricas de ambas fuentes usando diferencia simétrica.

        Casos especiales:
        - Si alguna captura tiene error → match=True (no podemos comparar)
        - Si no hay métricas en común → match=True (OCR no detectó datos)
        - Si diff > threshold en CUALQUIER métrica común → match=False

        Args:
            module:     Nombre del módulo
            screenshot: Resultado de captura local mss
            physical:   Resultado de captura RTMP Insta360

        Returns:
            DualModuleResult con divergencias calculadas.
        """
        # Si alguna fuente falló, no podemos validar → no marcamos como mismatch
        if not screenshot.ok or not physical.ok:
            return DualModuleResult(
                module=module,
                match=True,   # Indeterminate — no penalizar por fallo de captura
                screenshot=screenshot,
                physical=physical,
                divergences={},
                max_diff=0.0,
                tier=2,
            )

        common_keys = set(screenshot.metrics) & set(physical.metrics)
        if not common_keys:
            # Sin métricas comunes → OCR no encontró datos comparables
            return DualModuleResult(
                module=module,
                match=True,
                screenshot=screenshot,
                physical=physical,
                divergences={},
                max_diff=0.0,
                tier=2,
            )

        divergences: Dict[str, float] = {}
        for key in sorted(common_keys):
            divergences[key] = _relative_diff(
                screenshot.metrics[key],
                physical.metrics[key],
            )

        max_diff = max(divergences.values())
        match = max_diff < self._mismatch_threshold

        return DualModuleResult(
            module=module,
            match=match,
            screenshot=screenshot,
            physical=physical,
            divergences=divergences,
            max_diff=max_diff,
            tier=2,
        )

    # ── Logging estructurado ──────────────────────────────────────────────────

    def _log_result(self, result: DualModuleResult) -> None:
        """Emite una línea de log estructurada por módulo inspeccionado."""
        status = "OK  " if result.match else "WARN"
        ss_m = result.screenshot.metrics
        ph_m = result.physical.metrics

        # Construir comparativa de las métricas comunes
        common = set(ss_m) & set(ph_m)
        cmp_parts = []
        for k in sorted(common):
            diff_pct = result.divergences.get(k, 0.0) * 100
            cmp_parts.append(f"{k}: ss={ss_m[k]:.2f} vs ph={ph_m[k]:.2f} ({diff_pct:.1f}%)")
        cmp_str = " | ".join(cmp_parts) if cmp_parts else "sin métricas comunes"

        if result.match:
            logger.info("[DualInspector] %s %-16s  %s", status, result.module, cmp_str)
        else:
            logger.warning(
                "[DualInspector] %s %-16s  max_diff=%.1f%%  %s",
                status, result.module, result.max_diff * 100, cmp_str,
            )

    # ── Reporte al Doctor ─────────────────────────────────────────────────────

    def _report_to_doctor(self, mismatches: List[DualModuleResult]) -> None:
        """
        Convierte mismatches en anomalías y los reporta al AtlasDoctor.
        Reutiliza doctor.report_visual_anomalies() con source='dual_inspector'.

        Args:
            mismatches: Módulos con match=False.
        """
        if self._doctor is None:
            return
        try:
            payload = []
            for r in mismatches:
                payload.append({
                    "module":          r.module,
                    "ok":              False,
                    "issues":          [
                        f"dual_mismatch: {k} diff={v*100:.1f}%"
                        for k, v in r.divergences.items()
                        if v >= self._mismatch_threshold
                    ],
                    "ocr_confidence":  min(
                        r.screenshot.confidence, r.physical.confidence
                    ),
                    "metrics":         r.screenshot.metrics,
                    "tier":            r.tier,
                    "source":          "dual_inspector",
                })
            self._doctor.report_visual_anomalies(payload)
            logger.info("[DualInspector] %d mismatches reportados al Doctor", len(mismatches))
        except Exception as exc:
            logger.error("[DualInspector] Error reportando al Doctor: %s", exc)

    # ── Consulta pública ──────────────────────────────────────────────────────

    def summary(self) -> Dict[str, Any]:
        """Resumen de la última inspección dual."""
        if not self._last_results:
            return {"completed": False}
        total = len(self._last_results)
        ok = sum(1 for r in self._last_results if r.match)
        diffs = [r.max_diff for r in self._last_results if r.divergences]
        avg_diff = float(np.mean(diffs)) if diffs else 0.0
        return {
            "completed":      True,
            "last_run_ts":    self._last_run_ts,
            "modules_total":  total,
            "modules_match":  ok,
            "modules_mismatch": total - ok,
            "avg_max_diff":   round(avg_diff, 4),
            "threshold":      self._mismatch_threshold,
            "mismatches": [r.to_dict() for r in self._last_results if not r.match],
        }


# ── CLI ────────────────────────────────────────────────────────────────────────

def _setup_logging(level: str = "INFO") -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


async def _cli_run(
    modules: Optional[List[str]],
    rtmp_url: str,
    dashboard_url: str,
    threshold: float,
    monitor_idx: int,
) -> None:
    inspector = DualSelfInspector(
        rtmp_url=rtmp_url,
        doctor=None,
        dashboard_url=dashboard_url,
        mismatch_threshold=threshold,
        monitor_idx=monitor_idx,
        modules=modules,
    )
    results = await inspector.run_dual_inspection()
    inspector.stop_camera()
    print(json.dumps([r.to_dict() for r in results], indent=2, ensure_ascii=False))


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="python -m atlas_adapter.services.dual_self_inspector",
        description="ATLAS Dual Inspector — validación cruzada screenshot vs Insta360",
    )
    parser.add_argument("--test",    metavar="MODULE",  help="Probar un módulo específico")
    parser.add_argument("--modules", metavar="M1,M2",   help="Lista separada por comas")
    parser.add_argument("--list",    action="store_true", help="Listar módulos y salir")
    parser.add_argument("--daemon",  action="store_true", help="Daemon: cada 15 min")
    parser.add_argument("--rtmp",    default=_RTMP_URL,  help="URL RTMP Insta360")
    parser.add_argument("--dashboard", default=_DASHBOARD_URL, help="URL base SPA")
    parser.add_argument(
        "--threshold", type=float, default=_MISMATCH_THRESHOLD,
        help=f"Umbral de divergencia (default: {_MISMATCH_THRESHOLD})",
    )
    parser.add_argument(
        "--monitor", type=int, default=_MSS_MONITOR_IDX,
        help=f"Índice mss del monitor con el dashboard (default: {_MSS_MONITOR_IDX})",
    )
    parser.add_argument(
        "--log-level", default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
    )
    args = parser.parse_args()
    _setup_logging(args.log_level)

    if args.list:
        print("Módulos disponibles:")
        for name, hash_path in MODULE_HASHES.items():
            print(f"  {name:<18} {hash_path}")
        sys.exit(0)

    modules: Optional[List[str]] = None
    if args.test:
        if args.test not in MODULE_HASHES:
            print(f"Módulo '{args.test}' desconocido. Use --list para ver disponibles.")
            sys.exit(1)
        modules = [args.test]
    elif args.modules:
        modules = [m.strip() for m in args.modules.split(",") if m.strip()]

    if args.daemon:
        async def _daemon() -> None:
            inspector = DualSelfInspector(
                rtmp_url=args.rtmp,
                doctor=None,
                dashboard_url=args.dashboard,
                mismatch_threshold=args.threshold,
                monitor_idx=args.monitor,
                modules=modules,
            )
            inspector.start_camera()
            try:
                while True:
                    await inspector.run_dual_inspection()
                    s = inspector.summary()
                    logger.info(
                        "[Daemon] match=%d/%d mismatches=%d avg_diff=%.1f%%",
                        s.get("modules_match", 0), s.get("modules_total", 0),
                        s.get("modules_mismatch", 0),
                        s.get("avg_max_diff", 0.0) * 100,
                    )
                    await asyncio.sleep(15 * 60)
            except (KeyboardInterrupt, asyncio.CancelledError):
                logger.info("[Daemon] Deteniendo dual inspector...")
            finally:
                inspector.stop_camera()
        asyncio.run(_daemon())
    else:
        asyncio.run(_cli_run(
            modules=modules,
            rtmp_url=args.rtmp,
            dashboard_url=args.dashboard,
            threshold=args.threshold,
            monitor_idx=args.monitor,
        ))


if __name__ == "__main__":
    main()
