"""Camera Manager: modulo dedicado para todas las fuentes de video.

Fuentes soportadas:
- USBSource: camaras USB locales (cv2 + DSHOW)
- NetworkSource: camaras IP via RTSP o MJPEG HTTP
- DashCamSource: dashcams con dual stream (frontal/trasera)
- MobileSource: moviles enviando MJPEG push (IP Webcam, DroidCam)

Principios de diseno:
- UNA clase por tipo de fuente, todas heredan de CameraSource
- grab()+retrieve() sin sleep artificial: el hardware marca el ritmo
- MJPEG generator directo: sin cache intermedio, sin polling
- Lifecycle explicito: open/close con cleanup en atexit
"""
from __future__ import annotations

import atexit
import logging
import os
import threading
import time
from abc import ABC, abstractmethod
from typing import Any, Dict, Generator, List, Optional

_log = logging.getLogger("atlas.camera_manager")

# ---------------------------------------------------------------------------
# Base
# ---------------------------------------------------------------------------

class CameraSource(ABC):
    """Fuente de video abstracta."""

    def __init__(self, source_id: str, label: str = ""):
        self.source_id = source_id
        self.label = label or source_id
        self._lock = threading.Lock()
        self._last_jpeg: Optional[bytes] = None
        self._last_ts: float = 0.0
        self._open = False

    @abstractmethod
    def open(self) -> bool:
        """Abre la fuente. Retorna True si exito."""

    @abstractmethod
    def close(self) -> None:
        """Libera recursos."""

    @abstractmethod
    def read_jpeg(self) -> Optional[bytes]:
        """Lee el siguiente frame como JPEG. Bloqueante (espera al hardware)."""

    @property
    def is_open(self) -> bool:
        return self._open

    def snapshot(self) -> Optional[bytes]:
        """Retorna el ultimo JPEG disponible o lee uno nuevo."""
        jpeg = self.read_jpeg()
        if jpeg:
            with self._lock:
                self._last_jpeg = jpeg
                self._last_ts = time.time()
        return jpeg

    def info(self) -> Dict[str, Any]:
        return {
            "source_id": self.source_id,
            "label": self.label,
            "type": type(self).__name__,
            "open": self._open,
            "last_frame_ts": self._last_ts,
        }


# ---------------------------------------------------------------------------
# USB Source
# ---------------------------------------------------------------------------

_JPEG_QUALITY = int(os.getenv("CAM_JPEG_QUALITY", "70"))
_CAM_WIDTH = int(os.getenv("CAM_WIDTH", "640"))
_CAM_HEIGHT = int(os.getenv("CAM_HEIGHT", "480"))


class USBSource(CameraSource):
    """Camara USB local via OpenCV.

    Usa CAP_DSHOW en Windows (mas estable que MSMF).
    grab()+retrieve() sin sleep: el driver regula la velocidad.
    """

    def __init__(self, index: int, label: str = ""):
        super().__init__(source_id=f"usb:{index}", label=label or f"USB Camera {index}")
        self.index = index
        self._cap = None
        self._encode_params: list = []

    def open(self) -> bool:
        import cv2
        if self._cap is not None and self._cap.isOpened():
            self._open = True
            return True
        # Orden de backends: MSMF (estable en Windows para indices), DSHOW, ANY
        backends = []
        if os.name == "nt":
            backends.extend([cv2.CAP_MSMF, cv2.CAP_DSHOW])
        backends.append(cv2.CAP_ANY)
        for backend in backends:
            cap = self._try_open_with_timeout(backend, timeout_sec=5.0)
            if cap is not None:
                self._cap = cap
                self._encode_params = [cv2.IMWRITE_JPEG_QUALITY, max(30, min(95, _JPEG_QUALITY))]
                self._open = True
                _log.info("USBSource %d abierta con backend %s", self.index, backend)
                return True
        self._open = False
        return False

    def _try_open_with_timeout(self, backend, timeout_sec: float = 5.0):
        """Intenta abrir la camara con timeout. Retorna cap o None."""
        import cv2
        result = [None]

        def _attempt():
            try:
                cap = cv2.VideoCapture(self.index, backend)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, _CAM_WIDTH)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, _CAM_HEIGHT)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    cap.set(cv2.CAP_PROP_FPS, 30)
                    ok, _ = cap.read()
                    if ok:
                        result[0] = cap
                        return
                    try:
                        cap.release()
                    except Exception:
                        pass
                else:
                    try:
                        cap.release()
                    except Exception:
                        pass
            except Exception:
                pass

        t = threading.Thread(target=_attempt, daemon=True)
        t.start()
        t.join(timeout=timeout_sec)
        if t.is_alive():
            _log.debug("USBSource %d: backend %s timeout after %.1fs", self.index, backend, timeout_sec)
            return None
        return result[0]

    def close(self) -> None:
        self._open = False
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def read_jpeg(self) -> Optional[bytes]:
        import cv2
        if self._cap is None or not self._cap.isOpened():
            return None
        # grab() descarta frames buffereados, retrieve() obtiene el mas reciente
        if not self._cap.grab():
            return None
        ok, frame = self._cap.retrieve()
        if not ok or frame is None:
            return None
        ok2, buf = cv2.imencode(".jpg", frame, self._encode_params)
        if not ok2:
            return None
        return buf.tobytes()


# ---------------------------------------------------------------------------
# Network Source (RTSP / MJPEG HTTP)
# ---------------------------------------------------------------------------

class NetworkSource(CameraSource):
    """Camara IP via RTSP o MJPEG HTTP."""

    def __init__(self, url: str, source_id: str = "", label: str = ""):
        sid = source_id or f"net:{url[:60]}"
        super().__init__(source_id=sid, label=label or f"Network {url[:40]}")
        self.url = url
        self._cap = None
        self._encode_params: list = []
        self._is_mjpeg_http = self._detect_mjpeg_http(url)

    @staticmethod
    def _detect_mjpeg_http(url: str) -> bool:
        u = (url or "").lower()
        return u.startswith("http") and ("mjpeg" in u or "video" in u or "shot.jpg" not in u)

    def open(self) -> bool:
        import cv2
        if self._is_mjpeg_http:
            self._cap = cv2.VideoCapture(self.url)
        else:
            # RTSP: usar FFMPEG backend si disponible
            self._cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        if self._cap is not None and self._cap.isOpened():
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._encode_params = [cv2.IMWRITE_JPEG_QUALITY, max(30, min(95, _JPEG_QUALITY))]
            self._open = True
            _log.info("NetworkSource abierta: %s", self.url[:60])
            return True
        self._open = False
        return False

    def close(self) -> None:
        self._open = False
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def read_jpeg(self) -> Optional[bytes]:
        import cv2
        if self._cap is None or not self._cap.isOpened():
            return None
        if not self._cap.grab():
            return None
        ok, frame = self._cap.retrieve()
        if not ok or frame is None:
            return None
        ok2, buf = cv2.imencode(".jpg", frame, self._encode_params)
        if not ok2:
            return None
        return buf.tobytes()


# ---------------------------------------------------------------------------
# DashCam Source (extiende Network, dual stream)
# ---------------------------------------------------------------------------

class DashCamSource(NetworkSource):
    """Dashcam con posible dual stream (frontal + trasera).

    La deteccion de URLs se delega a la infraestructura existente
    en nexus/.../network/discoverer.py.
    """

    def __init__(self, url: str, stream_name: str = "frontal", **kwargs):
        label = kwargs.pop("label", f"DashCam ({stream_name})")
        sid = kwargs.pop("source_id", f"dash:{stream_name}:{url[:40]}")
        super().__init__(url=url, source_id=sid, label=label, **kwargs)
        self.stream_name = stream_name


# ---------------------------------------------------------------------------
# Mobile Source (MJPEG push receiver)
# ---------------------------------------------------------------------------

class MobileSource(CameraSource):
    """Recibe JPEG frames via HTTP push desde un movil.

    El movil (IP Webcam, DroidCam) envia frames a /cam/mobile/push.
    Este source almacena el ultimo frame y lo sirve via stream.
    """

    def __init__(self, device_id: str = "mobile-0", label: str = ""):
        super().__init__(source_id=f"mobile:{device_id}", label=label or f"Mobile ({device_id})")
        self._frame_event = threading.Event()

    def open(self) -> bool:
        self._open = True
        return True

    def close(self) -> None:
        self._open = False
        self._frame_event.set()

    def push_frame(self, jpeg_bytes: bytes) -> None:
        """Llamado cuando llega un frame del movil."""
        with self._lock:
            self._last_jpeg = jpeg_bytes
            self._last_ts = time.time()
        self._frame_event.set()

    def read_jpeg(self) -> Optional[bytes]:
        # Espera hasta 2s por un frame nuevo
        self._frame_event.wait(timeout=2.0)
        self._frame_event.clear()
        with self._lock:
            return self._last_jpeg


# ---------------------------------------------------------------------------
# MJPEG Generator (el corazon del streaming)
# ---------------------------------------------------------------------------

def mjpeg_generator(source: CameraSource, max_fps: int = 25) -> Generator[bytes, None, None]:
    """Genera frames MJPEG directamente desde un CameraSource.

    Sin cache intermedio, sin polling. read_jpeg() bloquea hasta
    que el hardware entrega el siguiente frame.
    """
    min_interval = 1.0 / max(1, max_fps)
    consecutive_fails = 0
    max_fails = 60  # ~2s a 30fps sin frame -> terminar

    while source.is_open and consecutive_fails < max_fails:
        t0 = time.monotonic()
        jpeg = source.read_jpeg()
        if jpeg:
            consecutive_fails = 0
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                + jpeg + b"\r\n"
            )
            # Respetar max_fps sin sleep artificial excesivo
            elapsed = time.monotonic() - t0
            if elapsed < min_interval:
                time.sleep(min_interval - elapsed)
        else:
            consecutive_fails += 1
            time.sleep(0.03)


def placeholder_jpeg(text: str = "OFFLINE", width: int = 640, height: int = 360) -> bytes:
    """Genera un JPEG placeholder con texto."""
    try:
        import cv2
        import numpy as np
        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[:] = (35, 35, 40)
        font = cv2.FONT_HERSHEY_SIMPLEX
        text_size = cv2.getTextSize(text, font, 1.0, 2)[0]
        x = (width - text_size[0]) // 2
        y = (height + text_size[1]) // 2
        cv2.putText(img, text, (x, y), font, 1.0, (200, 200, 200), 2)
        _, buf = cv2.imencode(".jpg", img)
        return buf.tobytes()
    except Exception:
        return b""


# ---------------------------------------------------------------------------
# Camera Manager (orquestador central)
# ---------------------------------------------------------------------------

class CameraManager:
    """Gestiona todas las fuentes de video.

    - Mantiene un diccionario de sources activos
    - Registra/descubre camaras
    - Lifecycle: start/stop
    """

    def __init__(self):
        self._sources: Dict[str, CameraSource] = {}
        self._lock = threading.Lock()
        self._started = False

    # -- Lifecycle --

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        _log.info("CameraManager iniciado")
        # Auto-detectar USB cameras al arrancar
        self._auto_detect_usb()

    def stop(self) -> None:
        self._started = False
        with self._lock:
            for src in self._sources.values():
                try:
                    src.close()
                except Exception:
                    pass
            self._sources.clear()
        _log.info("CameraManager detenido")

    # -- Source management --

    def get_source(self, key: str) -> Optional[CameraSource]:
        """Obtiene un source por key. Para USB, key es el indice ('0', '1', ...)."""
        with self._lock:
            # Buscar por key directo
            if key in self._sources:
                src = self._sources[key]
                if src.is_open:
                    return src
                # Intentar reabrir
                if src.open():
                    return src
                return None
            # Si es un numero, buscar/crear USBSource
            try:
                idx = int(key)
                return self._get_or_create_usb(idx)
            except (ValueError, TypeError):
                pass
            return None

    def add_source(self, key: str, source: CameraSource) -> bool:
        """Registra un source manualmente."""
        with self._lock:
            self._sources[key] = source
        if not source.is_open:
            return source.open()
        return True

    def remove_source(self, key: str) -> None:
        with self._lock:
            src = self._sources.pop(key, None)
        if src:
            try:
                src.close()
            except Exception:
                pass

    def list_sources(self) -> List[Dict[str, Any]]:
        with self._lock:
            return [src.info() for src in self._sources.values()]

    # -- USB auto-detection --

    def _auto_detect_usb(self) -> None:
        """Intenta abrir USB 0, 1, 2. Rapido, sin scan exhaustivo."""
        for idx in range(3):
            self._get_or_create_usb(idx)

    def _get_or_create_usb(self, idx: int) -> Optional[CameraSource]:
        key = str(idx)
        with self._lock:
            if key in self._sources:
                src = self._sources[key]
                if src.is_open:
                    return src
        # Crear fuera del lock (open puede tardar)
        src = USBSource(idx)
        if src.open():
            with self._lock:
                self._sources[key] = src
            _log.info("USB camera %d detectada y abierta", idx)
            return src
        return None

    # -- Network cameras --

    def add_network_camera(self, url: str, label: str = "", key: str = "") -> Dict[str, Any]:
        """Agrega una camara de red (RTSP/MJPEG)."""
        auto_key = key or f"net-{len([k for k in self._sources if k.startswith('net-')])}"
        src = NetworkSource(url=url, label=label)
        if src.open():
            with self._lock:
                self._sources[auto_key] = src
            return {"ok": True, "key": auto_key, "label": src.label}
        return {"ok": False, "error": f"No se pudo conectar a {url[:60]}"}

    def add_dashcam(self, url: str, stream_name: str = "frontal", label: str = "", key: str = "") -> Dict[str, Any]:
        """Agrega un stream de dashcam."""
        auto_key = key or f"dash-{stream_name}"
        src = DashCamSource(url=url, stream_name=stream_name, label=label)
        if src.open():
            with self._lock:
                self._sources[auto_key] = src
            return {"ok": True, "key": auto_key, "label": src.label}
        return {"ok": False, "error": f"No se pudo conectar a dashcam {url[:60]}"}

    # -- Mobile --

    def get_or_create_mobile(self, device_id: str = "mobile-0") -> MobileSource:
        """Obtiene o crea un MobileSource para recibir push."""
        key = f"mob-{device_id}"
        with self._lock:
            existing = self._sources.get(key)
            if existing and isinstance(existing, MobileSource):
                return existing
        mob = MobileSource(device_id=device_id)
        mob.open()
        with self._lock:
            self._sources[key] = mob
        return mob

    # -- Discovery --

    def discover_all(self, timeout_sec: float = 10.0) -> Dict[str, Any]:
        """Descubre todas las camaras disponibles (USB + red + ONVIF + mDNS).

        Delega al modulo de auto-discovery existente en nexus.
        """
        results: Dict[str, Any] = {"usb": [], "network": [], "errors": []}

        # USB: ya detectadas en start(), re-scan rapido
        for idx in range(5):
            src = USBSource(idx)
            if src.open():
                key = str(idx)
                with self._lock:
                    if key not in self._sources:
                        self._sources[key] = src
                results["usb"].append({"index": idx, "label": src.label})
            else:
                src.close()

        # Red: intentar usar auto_discovery de nexus
        try:
            import sys
            from pathlib import Path
            nexus_path = Path(__file__).resolve().parent.parent / "nexus" / "atlas_nexus_robot" / "backend"
            if str(nexus_path) not in sys.path:
                sys.path.insert(0, str(nexus_path))
            from vision.cameras.auto_discovery import run_full_discovery
            net_results = run_full_discovery(timeout_sec=timeout_sec)
            for cam in net_results.get("cameras", []):
                url = cam.get("url", "")
                label = cam.get("label", "")
                cam_type = cam.get("type", "network")
                if url:
                    if "dashcam" in cam_type.lower() or "dash" in label.lower():
                        r = self.add_dashcam(url, label=label)
                    else:
                        r = self.add_network_camera(url, label=label)
                    if r.get("ok"):
                        results["network"].append({"url": url, "label": label, "key": r.get("key")})
        except Exception as e:
            _log.debug("Network discovery error (non-fatal): %s", e)
            results["errors"].append(str(e)[:120])

        # mDNS: intentar descubrir moviles (IP Webcam, etc.)
        try:
            from vision.cameras.protocols.mdns import scan_mdns
            mdns_results = scan_mdns(timeout_sec=min(3.0, timeout_sec))
            for cam in mdns_results:
                url = cam.get("url", "")
                label = cam.get("label", cam.get("name", ""))
                if url:
                    r = self.add_network_camera(url, label=label)
                    if r.get("ok"):
                        results["network"].append({"url": url, "label": label, "key": r.get("key"), "via": "mdns"})
        except Exception as e:
            _log.debug("mDNS discovery error (non-fatal): %s", e)
            results["errors"].append(f"mdns: {str(e)[:80]}")

        results["total"] = len(results["usb"]) + len(results["network"])
        return results


# ---------------------------------------------------------------------------
# Singleton global
# ---------------------------------------------------------------------------

_manager: Optional[CameraManager] = None
_manager_lock = threading.Lock()


def get_camera_manager() -> CameraManager:
    """Obtiene el CameraManager singleton. Lo crea y arranca si no existe."""
    global _manager
    if _manager is not None:
        return _manager
    with _manager_lock:
        if _manager is not None:
            return _manager
        _manager = CameraManager()
        _manager.start()
        atexit.register(_manager.stop)
    return _manager
