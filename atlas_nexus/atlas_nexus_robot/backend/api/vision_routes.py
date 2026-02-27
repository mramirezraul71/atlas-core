"""
Vision API Routes
"""

import os
import time
import atexit
import multiprocessing
from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse, Response
from pydantic import BaseModel
from typing import List, Dict
import cv2
import logging

from vision.object_detection import get_detector
from vision.streaming.frame_broadcaster import get_broadcaster, cleanup_broadcasters

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/vision", tags=["vision"])

def _env_float(name: str, default: float, lo: float = 0.1, hi: float = 30.0) -> float:
    try:
        v = float(os.getenv(name, str(default)))
    except Exception:
        v = float(default)
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def _snapshot_worker(
    conn,
    *,
    source: str,
    enhance: str,
    jpeg_quality: int,
    index: int,
    focus_x: float,
    focus_y: float,
    zoom: float,
) -> None:
    """
    Worker aislado para evitar cuelgues del driver de cámara o screen grab.
    Devuelve (ok: bool, payload: bytes|str) por Pipe.
    """
    try:
        if (source or "").strip().lower() == "camera":
            cap = _get_capture_by_index(int(index or 0))
            if not cap or not cap.isOpened():
                conn.send((False, "Camera not available"))
                return
            ret, frame = cap.read()
            try:
                cap.release()
            except Exception:
                pass
            if not ret or frame is None:
                conn.send((False, "Cannot read frame"))
                return
        else:
            # Screen capture: PIL puede colgar en sesiones no interactivas; el aislamiento protege.
            try:
                from PIL import ImageGrab
                import numpy as np
            except Exception as e:
                conn.send((False, f"PIL not available for screen capture: {e}"))
                return
            img = ImageGrab.grab()
            frame = np.array(img)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        frame = _apply_focus_zoom(frame, focus_x=focus_x, focus_y=focus_y, zoom=zoom)
        frame = _enhance_frame(frame, enhance)
        q = int(jpeg_quality) if jpeg_quality is not None else 85
        if q < 30:
            q = 30
        if q > 95:
            q = 95
        ok, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, q])
        if not ok:
            conn.send((False, "Encode failed"))
            return
        conn.send((True, buffer.tobytes()))
    except Exception as e:
        try:
            conn.send((False, str(e)))
        except Exception:
            pass


def _enhance_frame(frame, profile: str = "none"):
    """
    Mejora de imagen (software) para más nitidez/legibilidad.

    Perfiles:
    - none: sin cambios
    - auto: denoise suave + unsharp leve (recomendado por defecto)
    - sharp: contraste local + unsharp más fuerte (más CPU)
    - max: máxima nitidez (más CPU)
    - ocr: binarización/contraste para lectura de texto (reduce color)
    """
    p = (profile or "none").strip().lower()
    if p in ("0", "false", "off", "no"):
        p = "none"
    if p == "max":
        # Alias de "sharp" pero con denoise/unsharp un poco más agresivo
        p = "sharp"
    if p == "none":
        return frame
    try:
        import numpy as np
        if p == "auto":
            out = cv2.fastNlMeansDenoisingColored(frame, None, 3, 3, 7, 21)
        elif p in ("sharp", "ocr"):
            out = cv2.fastNlMeansDenoisingColored(frame, None, 6, 6, 7, 21)
        else:
            out = frame

        if p == "ocr":
            g = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
            g = cv2.GaussianBlur(g, (3, 3), 0)
            g = cv2.adaptiveThreshold(g, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 5)
            return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)

        if p == "sharp":
            lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            l2 = clahe.apply(l)
            out = cv2.cvtColor(cv2.merge((l2, a, b)), cv2.COLOR_LAB2BGR)

        blur = cv2.GaussianBlur(out, (0, 0), 1.0 if p == "auto" else 1.25)
        amount = 0.7 if p == "auto" else 1.35
        sharp = cv2.addWeighted(out, 1.0 + amount, blur, -amount, 0)
        return np.clip(sharp, 0, 255).astype(out.dtype)
    except Exception:
        return frame


def _get_capture():
    """Obtiene captura: factory si existe, sino cv2 por defecto."""
    try:
        from vision.cameras.factory import get_camera
        cam = get_camera()
        if cam and cam.is_opened:
            return cam
    except Exception as e:
        logger.debug("Factory camera fallback to cv2: %s", e)
    cap = cv2.VideoCapture(0)
    return cap if cap.isOpened() else None


@router.get("/camera/test")
async def test_camera():
    """Test de cámara"""
    try:
        # Igual que /snapshot: proceso aislado + timeout duro para evitar cuelgues del driver.
        # Objetivo UI: respuesta rápida. Si la cámara está bloqueada, fallar sin retardo.
        timeout_s = _env_float("VISION_SNAPSHOT_TIMEOUT_SEC", 1.5, lo=0.3, hi=10.0)
        ctx = multiprocessing.get_context("spawn" if os.name == "nt" else None)
        parent_conn, child_conn = ctx.Pipe(duplex=False)
        p = ctx.Process(
            target=_snapshot_worker,
            kwargs={
                "conn": child_conn,
                "source": "camera",
                "enhance": "none",
                "jpeg_quality": 80,
                "index": 0,
                "focus_x": 0.5,
                "focus_y": 0.5,
                "zoom": 1.0,
            },
        )
        p.daemon = True
        p.start()
        try:
            child_conn.close()
        except Exception:
            pass

        ok = False
        payload = "timeout"
        try:
            if parent_conn.poll(timeout_s):
                ok, payload = parent_conn.recv()
        except Exception as e:
            ok, payload = False, str(e)
        finally:
            try:
                parent_conn.close()
            except Exception:
                pass

        # Cleanup duro y corto (no queremos añadir latencia después del timeout)
        try:
            if p.is_alive():
                p.terminate()
        except Exception:
            pass
        try:
            p.join(timeout=0.06)
        except Exception:
            pass

        if not ok or not payload:
            raise HTTPException(503, str(payload or "snapshot_failed"))

        # Decodificar JPEG para obtener resolución/canales (sin tocar driver directamente).
        try:
            import numpy as np
            arr = np.frombuffer(payload, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception:
            frame = None
        if frame is None:
            raise HTTPException(503, "Cannot decode frame")

        h, w, c = frame.shape
        
        return {
            "status": "ok",
            "resolution": f"{w}x{h}",
            "channels": c,
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Camera test error: {e}")
        raise HTTPException(500, str(e))


def _get_capture_by_index(index: int):
    """Obtiene captura por índice de cámara.

    En Windows preferimos DSHOW por estabilidad (MSMF suele fallar intermitente).
    """
    try:
        from vision.cameras.backend import preferred_backends
        backends = preferred_backends()
    except Exception:
        backends = []
        if hasattr(cv2, "CAP_DSHOW"):
            backends.append(cv2.CAP_DSHOW)
        if hasattr(cv2, "CAP_MSMF"):
            backends.append(cv2.CAP_MSMF)
        backends.append(cv2.CAP_ANY)

    # Probe: no basta con "opened"; hay drivers que abren pero no entregan frames.
    for backend in backends:
        cap = None
        try:
            cap = cv2.VideoCapture(index, backend)
            if not cap.isOpened():
                try:
                    cap.release()
                except Exception:
                    pass
                continue
            ok, frame = cap.read()
            if ok and frame is not None:
                return cap
        except Exception:
            pass
        finally:
            # Si no sirvió, liberar
            try:
                if cap is not None and not cap.isOpened():
                    cap.release()
            except Exception:
                pass

    # Último recurso: dejar que OpenCV elija backend (a veces funciona mejor que forzar).
    try:
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            ok, frame = cap.read()
            if ok and frame is not None:
                return cap
        try:
            cap.release()
        except Exception:
            pass
    except Exception:
        pass
    return None


def _clamp(v: float, lo: float, hi: float) -> float:
    try:
        x = float(v)
    except Exception:
        return lo
    return lo if x < lo else hi if x > hi else x


def _apply_focus_zoom(frame, focus_x: float = 0.5, focus_y: float = 0.5, zoom: float = 1.0):
    """
    Enfoque/zoom digital seguro (sin PTZ físico): recorta alrededor de (focus_x, focus_y) en [0..1] y reescala.
    Diseñado para ser controlado por el cerebro (ojos externos gobernados).
    """
    if frame is None:
        return frame
    z = _clamp(zoom, 1.0, 4.0)
    if z <= 1.01:
        return frame
    h, w = frame.shape[:2]
    cx = int(_clamp(focus_x, 0.0, 1.0) * w)
    cy = int(_clamp(focus_y, 0.0, 1.0) * h)
    crop_w = max(16, int(w / z))
    crop_h = max(16, int(h / z))
    x0 = max(0, min(w - crop_w, cx - crop_w // 2))
    y0 = max(0, min(h - crop_h, cy - crop_h // 2))
    crop = frame[y0 : y0 + crop_h, x0 : x0 + crop_w]
    try:
        return cv2.resize(crop, (w, h), interpolation=cv2.INTER_LINEAR)
    except Exception:
        return frame


def generate_frames(
    camera_index: int = 0,
    enhance: str = "auto",
    jpeg_quality: int = 85,
    focus_x: float = 0.5,
    focus_y: float = 0.5,
    zoom: float = 1.0,
):
    """Stream de video robusto con broadcaster compartido por cámara/perfil."""
    try:
        broadcaster = get_broadcaster(
            camera_index=camera_index,
            fps=15,
            enhance=enhance,
            focus_x=focus_x,
            focus_y=focus_y,
            zoom=zoom,
        )
        for frame_data in broadcaster.subscribe():
            frame = frame_data.frame
            q = int(jpeg_quality) if jpeg_quality is not None else 85
            if q < 30:
                q = 30
            if q > 95:
                q = 95
            ret, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, q])
            if not ret:
                continue
            frame_bytes = buffer.tobytes()
            yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n")
    except GeneratorExit:
        logger.info(f"Camera {camera_index}: Stream closed by client")
    except Exception as e:
        logger.error(f"Camera {camera_index}: Stream error: {e}")


def generate_screen_frames(
    enhance: str = "auto",
    jpeg_quality: int = 80,
    focus_x: float = 0.5,
    focus_y: float = 0.5,
    zoom: float = 1.0,
    max_fps: int = 6,
):
    """Stream de captura de pantalla para navegación en PC (cerebro)."""
    try:
        from PIL import ImageGrab
        import numpy as np
    except ImportError:
        logger.warning("PIL/Pillow no disponible para screen capture")
        return

    while True:
        try:
            t0 = time.time()
            img = ImageGrab.grab()
            frame = np.array(img)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            frame = _apply_focus_zoom(frame, focus_x=focus_x, focus_y=focus_y, zoom=zoom)
            frame = _enhance_frame(frame, enhance)
            q = int(jpeg_quality) if jpeg_quality is not None else 80
            if q < 30:
                q = 30
            if q > 95:
                q = 95
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, q])
            if not ret:
                break
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            # Throttle para no consumir CPU 100%
            try:
                fps = int(max_fps) if max_fps is not None else 6
                if fps < 1:
                    fps = 1
                if fps > 30:
                    fps = 30
                dt = time.time() - t0
                sleep_s = max(0.0, (1.0 / fps) - dt)
                if sleep_s > 0:
                    time.sleep(sleep_s)
            except Exception:
                pass
        except Exception as e:
            logger.debug("Screen capture error: %s", e)
            break


def generate_frames_with_detection(
    camera_index: int = 0,
    enhance: str = "none",
    jpeg_quality: int = 85,
    focus_x: float = 0.5,
    focus_y: float = 0.5,
    zoom: float = 1.0,
):
    """Stream de video con detección YOLO sobre broadcaster compartido."""
    detector = get_detector()
    if not detector.is_loaded:
        detector.load_model()

    try:
        broadcaster = get_broadcaster(
            camera_index=camera_index,
            fps=10,
            enhance=enhance,
            focus_x=focus_x,
            focus_y=focus_y,
            zoom=zoom,
        )
        for frame_data in broadcaster.subscribe():
            try:
                frame = frame_data.frame
                detections = detector.detect(frame, confidence=0.5)
                frame_with_detections = detector.draw_detections(frame, detections)
            except Exception as e:
                logger.debug(f"Detection error: {e}")
                frame_with_detections = frame_data.frame
            try:
                q = int(jpeg_quality) if jpeg_quality is not None else 85
                if q < 30:
                    q = 30
                if q > 95:
                    q = 95
                ret, buffer = cv2.imencode(".jpg", frame_with_detections, [cv2.IMWRITE_JPEG_QUALITY, q])
                if not ret:
                    continue
                frame_bytes = buffer.tobytes()
                yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n")
            except Exception as e:
                logger.debug("Encoding detection frame failed: %s", e)
                continue
    except GeneratorExit:
        logger.info(f"Camera {camera_index} (detection): Stream closed by client")
    except Exception as e:
        logger.error(f"Camera {camera_index} (detection): Stream error: {e}")


@router.get("/camera/stream")
async def video_stream(
    index: int = 0,
    enhance: str = "auto",
    jpeg_quality: int = 85,
    focus_x: float = 0.5,
    focus_y: float = 0.5,
    zoom: float = 1.0,
):
    """Stream de video sin detección. index: 0, 1, 2... para cada cámara instalada."""
    return StreamingResponse(
        generate_frames(
            camera_index=index,
            enhance=enhance,
            jpeg_quality=jpeg_quality,
            focus_x=focus_x,
            focus_y=focus_y,
            zoom=zoom,
        ),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )


@router.get("/camera/stream/detection")
async def video_stream_with_detection(
    index: int = 0,
    enhance: str = "none",
    jpeg_quality: int = 85,
    focus_x: float = 0.5,
    focus_y: float = 0.5,
    zoom: float = 1.0,
):
    """Stream de video CON detección YOLO. index: índice de cámara."""
    return StreamingResponse(
        generate_frames_with_detection(
            camera_index=index,
            enhance=enhance,
            jpeg_quality=jpeg_quality,
            focus_x=focus_x,
            focus_y=focus_y,
            zoom=zoom,
        ),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )


def generate_network_camera_frames(cam_id: str):
    """Stream de cámara de red (RTSP o HTTP). OpenCV lee ambos."""
    try:
        from vision.cameras.network import get_network_cameras
        cameras = get_network_cameras()
        cam = next((c for c in cameras if c.get("id") == cam_id), None)
        if not cam:
            return
        url = cam.get("url", "")
        if not url:
            return
        cap = cv2.VideoCapture(url)
        if not cap.isOpened():
            return
        try:
            while True:
                ret, frame = cap.read()
                if not ret or frame is None:
                    break
                r, buf = cv2.imencode('.jpg', frame)
                if not r:
                    break
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
        finally:
            cap.release()
    except Exception as e:
        logger.debug("Network camera stream error: %s", e)


@router.get("/camera/network/stream")
async def network_camera_stream(id: str = ""):
    """Stream de cámara de red por ID."""
    if not id:
        raise HTTPException(400, "Parámetro 'id' requerido")
    return StreamingResponse(
        generate_network_camera_frames(id),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )


@router.get("/screen/stream")
async def screen_stream(
    enhance: str = "auto",
    jpeg_quality: int = 80,
    focus_x: float = 0.5,
    focus_y: float = 0.5,
    zoom: float = 1.0,
    max_fps: int = 6,
):
    """Stream de captura de pantalla para navegación en PC (cerebro)."""
    return StreamingResponse(
        generate_screen_frames(
            enhance=enhance,
            jpeg_quality=jpeg_quality,
            focus_x=focus_x,
            focus_y=focus_y,
            zoom=zoom,
            max_fps=max_fps,
        ),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )


@router.get("/detect/current")
async def detect_current_frame():
    """Detecta objetos en el frame actual"""
    try:
        cap = _get_capture_by_index(0)
        if not cap or not cap.isOpened():
            raise HTTPException(503, "Camera not available")
        ret, frame = False, None
        for _ in range(3):
            try:
                ret, frame = cap.read()
            except Exception:
                ret, frame = False, None
            if ret and frame is not None:
                break
            time.sleep(0.08)
        try:
            cap.release()
        except Exception:
            pass
        if not ret or frame is None:
            raise HTTPException(503, "Cannot read frame")
        
        detector = get_detector()
        detections = detector.detect(frame, confidence=0.5)
        summary = detector.get_summary(detections)
        
        return {
            "detections": detections,
            "summary": summary
        }
        
    except Exception as e:
        logger.error(f"Detection error: {e}")
        raise HTTPException(500, str(e))


@router.post("/detect/initialize")
async def initialize_detector():
    """Inicializa el detector YOLO"""
    try:
        detector = get_detector()
        success = detector.load_model()
        
        return {
            "success": success,
            "model": detector.model_name,
            "loaded": detector.is_loaded
        }
    except Exception as e:
        logger.error(f"Init error: {e}")
        raise HTTPException(500, str(e))


# =============================================================================
# API EXTERNA: OJOS EXTERNOS para ATLAS PUSH
# =============================================================================

@router.get("/snapshot")
async def vision_snapshot(
    source: str = "screen",
    enhance: str = "auto",
    jpeg_quality: int = 85,
    index: int = 0,
    focus_x: float = 0.5,
    focus_y: float = 0.5,
    zoom: float = 1.0,
):
    """
    **Un frame** para ATLAS PUSH (nervio → ojos externos).
    source=camera: cámara; source=screen: captura de pantalla (navegación PC).
    Retorna image/jpeg. Uso: GET .../api/vision/snapshot?source=screen
    """
    try:
        # Hard timeout para evitar cuelgues: cámara/screen grab pueden trabarse por driver/sesión.
        # Objetivo: preview fluido. Si falla, responder rápido.
        timeout_s = _env_float("VISION_SNAPSHOT_TIMEOUT_SEC", 1.5, lo=0.3, hi=10.0)
        ctx = multiprocessing.get_context("spawn" if os.name == "nt" else None)
        parent_conn, child_conn = ctx.Pipe(duplex=False)
        p = ctx.Process(
            target=_snapshot_worker,
            kwargs={
                "conn": child_conn,
                "source": source,
                "enhance": enhance,
                "jpeg_quality": int(jpeg_quality or 85),
                "index": int(index or 0),
                "focus_x": float(focus_x or 0.5),
                "focus_y": float(focus_y or 0.5),
                "zoom": float(zoom or 1.0),
            },
        )
        p.daemon = True
        p.start()
        try:
            child_conn.close()
        except Exception:
            pass

        ok = False
        payload = "timeout"
        try:
            if parent_conn.poll(timeout_s):
                ok, payload = parent_conn.recv()
        except Exception as e:
            ok, payload = False, str(e)
        finally:
            try:
                parent_conn.close()
            except Exception:
                pass

        # Cleanup duro y corto (prioridad: latencia UI).
        try:
            if p.is_alive():
                p.terminate()
        except Exception:
            pass
        try:
            p.join(timeout=0.06)
        except Exception:
            pass

        if not ok or not payload:
            # Fallback en-proceso para cámara: evita 503 cuando el worker aislado
            # falla por timeout de spawn/driver en Windows.
            if (source or "").strip().lower() == "camera":
                cap = None
                try:
                    cap = _get_capture_by_index(int(index or 0))
                    if cap and cap.isOpened():
                        ret, frame = cap.read()
                        if ret and frame is not None:
                            frame = _apply_focus_zoom(frame, focus_x=float(focus_x or 0.5), focus_y=float(focus_y or 0.5), zoom=float(zoom or 1.0))
                            frame = _enhance_frame(frame, enhance)
                            q = int(jpeg_quality or 85)
                            if q < 30:
                                q = 30
                            if q > 95:
                                q = 95
                            ok2, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, q])
                            if ok2:
                                return Response(content=buf.tobytes(), media_type="image/jpeg")
                except Exception:
                    pass
                finally:
                    try:
                        if cap is not None:
                            cap.release()
                    except Exception:
                        pass
            raise HTTPException(503, str(payload or "snapshot_failed"))
        return Response(content=payload, media_type="image/jpeg")
    except HTTPException:
        raise
    except Exception as e:
        logger.debug("Snapshot error: %s", e)
        raise HTTPException(503, str(e))


@router.get("/external/eyes")
def external_eyes():
    """
    **Ojos externos** - Herramienta de visión para ATLAS PUSH.

    ATLAS PUSH puede consumir este endpoint como una de sus herramientas.
    Retorna estado de la cámara, URLs del stream y capacidades.

    Uso desde PUSH:
      GET http://<NEXUS_HOST>:8002/api/vision/external/eyes
    """
    base_url = os.getenv("NEXUS_BASE_URL", "http://localhost:8002")
    try:
        from vision.cameras.factory import get_camera_info
        info = get_camera_info()
    except Exception:
        info = {"active": False, "properties": {}}

    props = info.get("properties", {}) if info.get("active") else {}
    return {
        "ok": True,
        "tool": "ojos_externos",
        "source": "ATLAS_NEXUS",
        "active": info.get("active", False),
        "stream_url": f"{base_url}/api/vision/camera/stream",
        "stream_detection_url": f"{base_url}/api/vision/camera/stream/detection",
        "screen_stream_url": f"{base_url}/api/vision/screen/stream",
        "snapshot_url": f"{base_url}/api/vision/snapshot",
        "model": props.get("model", "Desconocida"),
        "resolution": props.get("resolution", [640, 480]),
        "capabilities": props.get("capabilities", ["video"]),
        "integration": "ATLAS PUSH: consumir snapshot_url (un frame) o stream_url para reconocimiento.",
    }


atexit.register(cleanup_broadcasters)
