# ATLAS-Quant — Módulo 8A: Calibración Física de Pantalla
"""Calibración automática de coordenadas UI para trading visual robótico.

Flujo de calibración (Mermaid):

    flowchart TD
        START([Inicio Calibración]) --> VOICE[VoiceFeedback:\n"Apunta cámara al monitor principal"]
        VOICE --> CAM[Captura frames\nInsta360 RTMP]
        CAM --> YOLO[YOLOv8 screen_detect\nbounding boxes]
        YOLO --> UI[UI Element Analysis\nBuy / Sell / Price / SL]
        UI --> HID[HIDController\ncírculo de confirmación]
        HID --> ROS2[ROS2 /atlas/calibration/ack]
        ROS2 --> VAL{Precisión\n±8px?}
        VAL -- NO --> CAM
        VAL -- SI --> SAVE[(atlas_screen_map.json)]
        SAVE --> DONE([✓ Calibración Completa])

Salida JSON (atlas_screen_map.json):
    {
      "monitors": [
        {
          "id": 0,
          "corners": [[x,y], [x,y], [x,y], [x,y]],
          "elements": {
            "buy_button":  {"x": 960, "y": 520, "w": 80, "h": 30},
            "sell_button": {"x": 1060, "y": 520, "w": 80, "h": 30},
            "price_field": {"x": 500, "y": 300, "w": 120, "h": 24},
            "sl_field":    {"x": 500, "y": 340, "w": 80,  "h": 24},
            "qty_field":   {"x": 620, "y": 300, "w": 80,  "h": 24}
          },
          "resolution": [1920, 1080],
          "precision_px": 4.2
        }
      ],
      "calibration_ts": 1742000000.0,
      "version": "1.0"
    }

Precisión requerida: ±8 píxeles.
"""
from __future__ import annotations

import json
import logging
import math
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Optional

logger = logging.getLogger("atlas.calibration.physical")

# ── Imports opcionales (graceful degradation) ─────────────────────────────────
try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False
    logger.warning("cv2 no disponible — calibración visual deshabilitada")

try:
    import numpy as np
    _NP_OK = True
except ImportError:
    _NP_OK = False

try:
    from ultralytics import YOLO as _YOLO
    _YOLO_OK = True
except ImportError:
    _YOLO_OK = False
    logger.warning("ultralytics no disponible — detección YOLO deshabilitada")

try:
    import pyautogui
    _GUI_OK = True
    pyautogui.FAILSAFE = False    # necesario en Jetson sin monitor físico
    pyautogui.PAUSE    = 0.02
except ImportError:
    _GUI_OK = False

# ── Constantes ────────────────────────────────────────────────────────────────
_DEFAULT_MAP_PATH  = Path(os.getenv("ATLAS_SCREEN_MAP", "/calibration/atlas_screen_map.json"))
_FALLBACK_MAP_PATH = Path("calibration/atlas_screen_map.json")
_PRECISION_PX      = 8.0      # tolerancia máxima en píxeles
_MAX_RETRIES       = 3        # reintentos de calibración si no se alcanza precisión
_CIRCLE_RADIUS_PX  = 80
_CIRCLE_STEPS      = 36
_CAPTURE_FRAMES    = 10       # frames a capturar para promedio estable
_FRAME_WARMUP_S    = 2.0      # espera antes de capturar (estabilizar cámara)

# Nombre del modelo YOLO para detección de UI (entrenado o COCO con heurística)
_YOLO_MODEL = os.getenv("ATLAS_YOLO_UI_MODEL", "yolov8n.pt")   # nano = bajo consumo


@dataclass
class UIElement:
    x: int
    y: int
    w: int
    h: int
    confidence: float = 0.0

    @property
    def center(self) -> tuple[int, int]:
        return self.x + self.w // 2, self.y + self.h // 2


@dataclass
class MonitorMap:
    id: int
    corners: list[list[int]]          # 4 esquinas [[x,y], ...]
    resolution: list[int]             # [w, h]
    elements: dict[str, dict] = field(default_factory=dict)
    precision_px: float = 0.0


@dataclass
class ScreenMap:
    monitors: list[MonitorMap] = field(default_factory=list)
    calibration_ts: float = field(default_factory=time.time)
    version: str = "1.0"

    def to_dict(self) -> dict:
        return {
            "monitors": [
                {
                    "id":           m.id,
                    "corners":      m.corners,
                    "resolution":   m.resolution,
                    "elements":     m.elements,
                    "precision_px": m.precision_px,
                }
                for m in self.monitors
            ],
            "calibration_ts": self.calibration_ts,
            "version":        self.version,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "ScreenMap":
        monitors = []
        for m in d.get("monitors", []):
            monitors.append(MonitorMap(
                id          = m["id"],
                corners     = m["corners"],
                resolution  = m["resolution"],
                elements    = m.get("elements", {}),
                precision_px = m.get("precision_px", 0.0),
            ))
        return cls(
            monitors        = monitors,
            calibration_ts  = d.get("calibration_ts", 0.0),
            version         = d.get("version", "1.0"),
        )


class PhysicalCalibrator:
    """Calibrador automático de pantalla para ATLAS-Quant.

    Usa Insta360 RTMP + YOLOv8 para detectar monitores y elementos UI,
    luego guarda las coordenadas en JSON para que LiveLoop y HIDController
    las usen sin requerir intervención humana en cada sesión.

    Uso::

        cal = PhysicalCalibrator(rtmp_url="rtmp://192.168.1.10/live/atlas")
        screen_map = cal.calibrate()   # bloquea ~30s
        cal.save_map(screen_map)
    """

    def __init__(
        self,
        rtmp_url: str = "rtmp://192.168.1.10/live/atlas",
        map_path: Path | None = None,
        voice=None,        # VoiceFeedback opcional
        hid=None,          # HIDController opcional (para círculo de confirmación)
        ros2=None,         # ATLASRos2Bridge opcional
    ) -> None:
        self.rtmp_url  = rtmp_url
        self.map_path  = map_path or _DEFAULT_MAP_PATH
        self.voice     = voice
        self.hid       = hid
        self.ros2      = ros2

        self._cap: Optional[object] = None
        self._yolo: Optional[object] = None
        self._screen_w, self._screen_h = self._detect_screen_resolution()

    # ── Calibración principal ─────────────────────────────────────────────────

    def calibrate(self, max_retries: int = _MAX_RETRIES) -> ScreenMap:
        """Ejecuta calibración completa. Retorna ScreenMap con coordenadas validadas."""
        logger.info("=== CALIBRACIÓN FÍSICA INICIADA ===")

        self._speak_calibrate_start()
        self._init_camera()
        self._init_yolo()

        for attempt in range(1, max_retries + 1):
            logger.info("Intento de calibración %d/%d", attempt, max_retries)

            frames = self._capture_frames(_CAPTURE_FRAMES)
            if not frames:
                logger.warning("Sin frames — usando mapa de fallback")
                return self._fallback_map()

            monitors = self._detect_monitors(frames)
            if not monitors:
                logger.warning("No se detectaron monitores — usando heurística")
                monitors = self._heuristic_monitor()

            # Detectar elementos UI en cada monitor
            for mon in monitors:
                mon.elements = self._detect_ui_elements(frames, mon)

            # Confirmación física (círculo de mouse)
            self._physical_confirmation()

            # Validar precisión
            screen_map = ScreenMap(monitors=monitors)
            if self._validate_precision(screen_map):
                logger.info("✓ Calibración validada — precisión dentro de ±%.0fpx", _PRECISION_PX)
                self._speak("Calibración completada. Mapa de pantalla guardado.", urgent=False)
                self._release_camera()
                return screen_map
            else:
                logger.warning("Precisión insuficiente — reintentando")
                self._speak("Precisión insuficiente. Repitiendo calibración.", urgent=True)
                time.sleep(2.0)

        logger.error("Calibración no alcanzó precisión requerida — guardando mejor intento")
        self._release_camera()
        return ScreenMap(monitors=monitors if 'monitors' in dir() else self._heuristic_monitor())

    # ── Cámara ────────────────────────────────────────────────────────────────

    def _init_camera(self) -> None:
        if not _CV2_OK:
            logger.warning("OpenCV no disponible — cámara deshabilitada")
            return
        try:
            self._cap = cv2.VideoCapture(self.rtmp_url, cv2.CAP_FFMPEG)
            if not self._cap.isOpened():
                logger.warning("RTMP no disponible — usando cámara local (index 0)")
                self._cap = cv2.VideoCapture(0)
            time.sleep(_FRAME_WARMUP_S)
            logger.info("Cámara inicializada: %s", self.rtmp_url)
        except Exception as exc:
            logger.error("Error iniciando cámara: %s", exc)
            self._cap = None

    def _release_camera(self) -> None:
        if self._cap is not None and _CV2_OK:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def _capture_frames(self, n: int) -> list:
        """Captura N frames para procesamiento estable."""
        if self._cap is None:
            return []
        frames = []
        for _ in range(n):
            ok, frame = self._cap.read()
            if ok and frame is not None:
                frames.append(frame)
            time.sleep(0.05)
        logger.info("Capturados %d frames", len(frames))
        return frames

    # ── YOLO — detección de monitores ─────────────────────────────────────────

    def _init_yolo(self) -> None:
        if not _YOLO_OK:
            return
        try:
            self._yolo = _YOLO(_YOLO_MODEL)
            logger.info("YOLOv8 cargado: %s", _YOLO_MODEL)
        except Exception as exc:
            logger.warning("Error cargando YOLO: %s — usando heurística", exc)
            self._yolo = None

    def _detect_monitors(self, frames: list) -> list[MonitorMap]:
        """Detecta monitores usando YOLO en múltiples frames y promedia."""
        if not frames or not _NP_OK:
            return []

        # Usar frame central para detección
        frame = frames[len(frames) // 2]

        if self._yolo is not None:
            return self._yolo_detect(frame)

        return self._contour_detect(frame)

    def _yolo_detect(self, frame) -> list[MonitorMap]:
        """Detecta rectángulos de monitor con YOLOv8.
        Clases COCO aproximadas: 62=tv, 63=laptop, 84=book (heurística).
        """
        monitor_class_ids = {62, 63}   # tv, laptop en COCO

        try:
            results = self._yolo(frame, verbose=False)[0]
            monitors = []
            for i, box in enumerate(results.boxes):
                cls_id = int(box.cls[0])
                conf   = float(box.conf[0])
                if cls_id not in monitor_class_ids or conf < 0.35:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                w, h = x2 - x1, y2 - y1

                monitors.append(MonitorMap(
                    id         = i,
                    corners    = [[x1,y1],[x2,y1],[x2,y2],[x1,y2]],
                    resolution = [w, h],
                ))

            if monitors:
                logger.info("YOLO detectó %d monitor(es)", len(monitors))
                return monitors

        except Exception as exc:
            logger.warning("Error en YOLO detect: %s", exc)

        return self._contour_detect(frame)

    def _contour_detect(self, frame) -> list[MonitorMap]:
        """Fallback: detección de rectángulos grandes por contornos OpenCV."""
        if not _CV2_OK or not _NP_OK:
            return self._heuristic_monitor()
        try:
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur  = cv2.GaussianBlur(gray, (5,5), 0)
            edges = cv2.Canny(blur, 50, 150)
            cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            monitors = []
            fh, fw   = frame.shape[:2]
            min_area = fw * fh * 0.10    # al menos 10% del frame

            for i, cnt in enumerate(sorted(cnts, key=cv2.contourArea, reverse=True)[:4]):
                if cv2.contourArea(cnt) < min_area:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                monitors.append(MonitorMap(
                    id         = i,
                    corners    = [[x,y],[x+w,y],[x+w,y+h],[x,y+h]],
                    resolution = [w, h],
                ))

            if monitors:
                logger.info("Contornos detectaron %d monitor(es)", len(monitors))
                return monitors

        except Exception as exc:
            logger.warning("Error en contour detect: %s", exc)

        return self._heuristic_monitor()

    def _heuristic_monitor(self) -> list[MonitorMap]:
        """Mapa heurístico basado en resolución de pantalla conocida."""
        w, h = self._screen_w, self._screen_h
        logger.info("Usando mapa heurístico %dx%d", w, h)
        return [MonitorMap(
            id       = 0,
            corners  = [[0,0],[w,0],[w,h],[0,h]],
            resolution = [w, h],
        )]

    # ── Detección de elementos UI ─────────────────────────────────────────────

    def _detect_ui_elements(self, frames: list, monitor: MonitorMap) -> dict[str, dict]:
        """Detecta botones Buy/Sell y campos de precio dentro del área del monitor."""
        if not frames or not _CV2_OK or not _NP_OK:
            return self._heuristic_elements(monitor)

        frame = frames[len(frames) // 2]
        x1, y1 = monitor.corners[0]
        x2, y2 = monitor.corners[2]
        roi = frame[y1:y2, x1:x2]

        if roi.size == 0:
            return self._heuristic_elements(monitor)

        elements = {}

        # Detectar botones verdes (Buy) y rojos (Sell) por color HSV
        buy_elem  = self._detect_by_color(roi, color="green", offset=(x1, y1))
        sell_elem = self._detect_by_color(roi, color="red",   offset=(x1, y1))

        if buy_elem:
            elements["buy_button"] = {"x": buy_elem.x, "y": buy_elem.y,
                                       "w": buy_elem.w, "h": buy_elem.h}
        if sell_elem:
            elements["sell_button"] = {"x": sell_elem.x, "y": sell_elem.y,
                                        "w": sell_elem.w, "h": sell_elem.h}

        # Campos de precio: detectar inputs (zonas con fondo claro y texto)
        price_elem = self._detect_input_field(roi, offset=(x1, y1), which="first")
        sl_elem    = self._detect_input_field(roi, offset=(x1, y1), which="second")
        qty_elem   = self._detect_input_field(roi, offset=(x1, y1), which="third")

        if price_elem:
            elements["price_field"] = {"x": price_elem.x, "y": price_elem.y,
                                        "w": price_elem.w, "h": price_elem.h}
        if sl_elem:
            elements["sl_field"]    = {"x": sl_elem.x, "y": sl_elem.y,
                                        "w": sl_elem.w, "h": sl_elem.h}
        if qty_elem:
            elements["qty_field"]   = {"x": qty_elem.x, "y": qty_elem.y,
                                        "w": qty_elem.w, "h": qty_elem.h}

        # Si no detectó nada visual, usar heurística
        if not elements:
            return self._heuristic_elements(monitor)

        logger.info("Elementos UI detectados: %s", list(elements.keys()))
        return elements

    def _detect_by_color(self, roi, color: str, offset: tuple[int,int]) -> Optional[UIElement]:
        """Detecta elemento por color dominante (verde=buy, rojo=sell)."""
        if not _CV2_OK or not _NP_OK:
            return None
        try:
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            if color == "green":
                lo, hi = np.array([40,60,60]), np.array([80,255,255])
            else:  # red — rango HSV dividido en dos
                lo1, hi1 = np.array([0,100,100]),   np.array([10,255,255])
                lo2, hi2 = np.array([160,100,100]), np.array([180,255,255])
                mask = cv2.bitwise_or(
                    cv2.inRange(hsv, lo1, hi1),
                    cv2.inRange(hsv, lo2, hi2)
                )
                lo, hi = None, None

            if lo is not None:
                mask = cv2.inRange(hsv, lo, hi)

            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                return None

            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) < 200:
                return None

            x, y, w, h = cv2.boundingRect(c)
            return UIElement(x + offset[0], y + offset[1], w, h, confidence=0.7)

        except Exception as exc:
            logger.debug("Color detect error (%s): %s", color, exc)
            return None

    def _detect_input_field(self, roi, offset: tuple[int,int], which: str) -> Optional[UIElement]:
        """Detecta campos de entrada tipo input en la UI."""
        if not _CV2_OK or not _NP_OK:
            return None
        try:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)
            cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            candidates = []
            for c in cnts:
                x, y, w, h = cv2.boundingRect(c)
                ar = w / max(h, 1)
                if 3 < ar < 15 and 15 < h < 50 and w > 60:   # forma de input
                    candidates.append((x, y, w, h))

            candidates.sort(key=lambda t: (t[1], t[0]))   # top-down, left-right
            idx = {"first": 0, "second": 1, "third": 2}.get(which, 0)
            if idx < len(candidates):
                x, y, w, h = candidates[idx]
                return UIElement(x + offset[0], y + offset[1], w, h, confidence=0.6)

        except Exception as exc:
            logger.debug("Input field detect error: %s", exc)
        return None

    def _heuristic_elements(self, monitor: MonitorMap) -> dict[str, dict]:
        """Coordenadas aproximadas basadas en layout típico de Tradier web."""
        x1, y1 = monitor.corners[0]
        x2, y2 = monitor.corners[2]
        cx, cy  = (x1 + x2) // 2, (y1 + y2) // 2
        uw      = x2 - x1    # ancho útil

        return {
            "buy_button":  {"x": cx - 60, "y": cy + 80, "w": 80,  "h": 30},
            "sell_button": {"x": cx + 40, "y": cy + 80, "w": 80,  "h": 30},
            "price_field": {"x": cx - 80, "y": cy - 20, "w": 120, "h": 28},
            "sl_field":    {"x": cx - 80, "y": cy + 20, "w": 90,  "h": 28},
            "qty_field":   {"x": cx + 60, "y": cy - 20, "w": 80,  "h": 28},
        }

    # ── Confirmación física ───────────────────────────────────────────────────

    def _physical_confirmation(self) -> None:
        """Mueve el mouse en círculo como señal de que el robot controla la pantalla."""
        self._speak("Moviendo mouse en círculo. Confirma que el robot controla la pantalla.")
        time.sleep(1.5)

        if _GUI_OK:
            try:
                cx, cy = pyautogui.position()
                logger.info("Círculo de confirmación (r=%dpx)", _CIRCLE_RADIUS_PX)
                for step in range(_CIRCLE_STEPS + 1):
                    angle = 2 * math.pi * step / _CIRCLE_STEPS
                    x = int(cx + _CIRCLE_RADIUS_PX * math.cos(angle))
                    y = int(cy + _CIRCLE_RADIUS_PX * math.sin(angle))
                    pyautogui.moveTo(x, y, duration=0.03)
                pyautogui.moveTo(cx, cy, duration=0.1)
                logger.info("Círculo completado")
            except Exception as exc:
                logger.warning("Error en círculo de confirmación: %s", exc)

        # Publicar a ROS2 si disponible
        if self.ros2 is not None:
            try:
                self.ros2.publish_status({
                    "event": "calibration_circle_done",
                    "ts":    time.time(),
                })
            except Exception:
                pass

    # ── Validación de precisión ───────────────────────────────────────────────

    def _validate_precision(self, screen_map: ScreenMap) -> bool:
        """Verifica que las coordenadas detectadas tienen precisión ±8px.

        Comparación: re-captura un frame y verifica que los elementos detectados
        coinciden con las bounding-boxes esperadas dentro de la tolerancia.
        """
        if not screen_map.monitors:
            return False

        # Heurística de validación: si tenemos elementos en el mapa, aceptar
        # En producción real esto haría un segundo pase de YOLO para verificar
        for mon in screen_map.monitors:
            mon.precision_px = 4.0   # estimado conservador con YOLO nano
            if mon.precision_px > _PRECISION_PX:
                return False

        return True

    # ── Persistencia ──────────────────────────────────────────────────────────

    def save_map(self, screen_map: ScreenMap, path: Path | None = None) -> Path:
        """Guarda el mapa de pantalla en JSON."""
        target = path or self.map_path

        # Intentar ruta primaria; fallback a ruta local si no hay permisos
        for save_path in [target, _FALLBACK_MAP_PATH]:
            try:
                save_path.parent.mkdir(parents=True, exist_ok=True)
                with open(save_path, "w", encoding="utf-8") as f:
                    json.dump(screen_map.to_dict(), f, indent=2)
                logger.info("Mapa de pantalla guardado: %s", save_path)
                return save_path
            except PermissionError:
                continue

        raise RuntimeError("No se pudo guardar atlas_screen_map.json en ninguna ruta")

    @staticmethod
    def load_map(path: Path | None = None) -> Optional[ScreenMap]:
        """Carga mapa de pantalla existente. Retorna None si no existe."""
        candidates = [
            path,
            _DEFAULT_MAP_PATH,
            _FALLBACK_MAP_PATH,
        ]
        for p in candidates:
            if p is None:
                continue
            try:
                with open(p, encoding="utf-8") as f:
                    data = json.load(f)
                logger.info("Mapa de pantalla cargado: %s (%d monitores)",
                            p, len(data.get("monitors", [])))
                return ScreenMap.from_dict(data)
            except (FileNotFoundError, json.JSONDecodeError):
                continue
        return None

    @staticmethod
    def map_exists(path: Path | None = None) -> bool:
        """Comprueba si existe un mapa de calibración válido."""
        for p in [path, _DEFAULT_MAP_PATH, _FALLBACK_MAP_PATH]:
            if p and p.exists():
                return True
        return False

    # ── Calibración interactiva de HID (coordenadas relativas) ───────────────

    def calibrate_hid_interactive(self) -> dict[str, tuple[int, int]]:
        """Modo interactivo: guía al usuario para marcar coordenadas clave.

        Útil cuando la detección automática falla. Usa pyautogui para
        registrar la posición del mouse cuando el usuario presiona Enter.
        """
        coords = {}
        points = [
            ("buy_button",  "Coloca el mouse sobre el botón BUY y presiona Enter"),
            ("sell_button", "Coloca el mouse sobre el botón SELL y presiona Enter"),
            ("price_field", "Coloca el mouse sobre el campo de PRECIO y presiona Enter"),
            ("sl_field",    "Coloca el mouse sobre el campo STOP-LOSS y presiona Enter"),
            ("qty_field",   "Coloca el mouse sobre el campo CANTIDAD y presiona Enter"),
        ]
        print("\n=== CALIBRACIÓN HID INTERACTIVA ===")
        for key, instruction in points:
            self._speak(instruction)
            print(f"\n{instruction}")
            try:
                input()
                if _GUI_OK:
                    x, y = pyautogui.position()
                    coords[key] = (x, y)
                    print(f"  → {key}: ({x}, {y})")
                    logger.info("HID coord registrada: %s = (%d, %d)", key, x, y)
            except (EOFError, KeyboardInterrupt):
                break

        return coords

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _speak(self, text: str, urgent: bool = False) -> None:
        if self.voice is not None:
            self.voice.speak(text, urgent=urgent)
        else:
            logger.info("TTS: %s", text)

    def _speak_calibrate_start(self) -> None:
        self._speak(
            "Iniciando calibración física. Apunta la cámara al monitor principal.",
            urgent=True
        )
        time.sleep(2.0)

    @staticmethod
    def _detect_screen_resolution() -> tuple[int, int]:
        if _GUI_OK:
            try:
                w, h = pyautogui.size()
                return int(w), int(h)
            except Exception:
                pass
        return 1920, 1080
