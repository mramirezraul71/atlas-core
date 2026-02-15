"""
Vision API Routes
"""

import os
import time
from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse, Response
from pydantic import BaseModel
from typing import List, Dict
import cv2
import logging

from vision.object_detection import get_detector

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/vision", tags=["vision"])


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
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            raise HTTPException(500, "Cannot open camera")
        
        ret, frame = cap.read()
        cap.release()
        
        if not ret:
            raise HTTPException(500, "Cannot read frame")
        
        h, w, c = frame.shape
        
        return {
            "status": "ok",
            "resolution": f"{w}x{h}",
            "channels": c,
        }
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
    """Stream de video robusto. camera_index: 0, 1, 2... para cada cámara instalada."""
    cap = _get_capture_by_index(camera_index)
    if not cap:
        return
    
    retry_count = 0
    max_retries = 3
    consecutive_failures = 0
    max_failures = 10
    
    try:
        while True:
            try:
                success, frame = cap.read()
                
                if not success or frame is None:
                    consecutive_failures += 1
                    logger.warning(f"Camera {camera_index}: Frame read failed #{consecutive_failures}")
                    
                    # Reintentar abrir la cámara si hay muchos fallos
                    if consecutive_failures >= max_failures:
                        logger.warning(f"Camera {camera_index}: Too many failures, reopening...")
                        cap.release()
                        cap = _get_capture_by_index(camera_index)
                        if not cap:
                            logger.error(f"Camera {camera_index}: Failed to reopen")
                            break
                        consecutive_failures = 0
                        retry_count += 1
                        if retry_count >= max_retries:
                            logger.error(f"Camera {camera_index}: Max retries exceeded")
                            break
                        continue
                    
                    # Enviar frame negro si falla la lectura
                    import numpy as np
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(frame, f"Camera {camera_index} - Reconnecting...", (100, 240), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Resetear contador de fallos si el frame es exitoso
                consecutive_failures = 0
                retry_count = 0

                frame = _apply_focus_zoom(frame, focus_x=focus_x, focus_y=focus_y, zoom=zoom)
                frame = _enhance_frame(frame, enhance)
                
                # Codificar y enviar frame
                q = int(jpeg_quality) if jpeg_quality is not None else 85
                if q < 30:
                    q = 30
                if q > 95:
                    q = 95
                ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, q])
                if not ret:
                    continue
                
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                       
            except Exception as e:
                logger.error(f"Camera {camera_index}: Stream error: {e}")
                consecutive_failures += 1
                if consecutive_failures >= max_failures:
                    break
                continue
                
    except GeneratorExit:
        logger.info(f"Camera {camera_index}: Stream closed by client")
    except Exception as e:
        logger.error(f"Camera {camera_index}: Fatal stream error: {e}")
    finally:
        if cap and cap.isOpened():
            cap.release()
            logger.info(f"Camera {camera_index}: Released")


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
    """Stream de video con detecciones YOLO robusto. camera_index: índice de cámara."""
    cap = _get_capture_by_index(camera_index)
    if not cap:
        return
    detector = get_detector()
    
    if not detector.is_loaded:
        detector.load_model()
    
    retry_count = 0
    max_retries = 3
    consecutive_failures = 0
    max_failures = 10
    
    try:
        while True:
            try:
                success, frame = cap.read()
                
                if not success or frame is None:
                    consecutive_failures += 1
                    logger.warning(f"Camera {camera_index} (detection): Frame read failed #{consecutive_failures}")
                    
                    if consecutive_failures >= max_failures:
                        logger.warning(f"Camera {camera_index} (detection): Too many failures, reopening...")
                        cap.release()
                        cap = _get_capture_by_index(camera_index)
                        if not cap:
                            logger.error(f"Camera {camera_index} (detection): Failed to reopen")
                            break
                        consecutive_failures = 0
                        retry_count += 1
                        if retry_count >= max_retries:
                            logger.error(f"Camera {camera_index} (detection): Max retries exceeded")
                            break
                        continue
                    
                    # Frame negro con mensaje
                    import numpy as np
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(frame, f"Camera {camera_index} - Reconnecting...", (100, 240), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Resetear contadores
                consecutive_failures = 0
                retry_count = 0

                frame = _apply_focus_zoom(frame, focus_x=focus_x, focus_y=focus_y, zoom=zoom)
                frame = _enhance_frame(frame, enhance)
                
                # Detectar objetos
                try:
                    detections = detector.detect(frame, confidence=0.5)
                    frame_with_detections = detector.draw_detections(frame, detections)
                except Exception as e:
                    logger.debug(f"Detection error: {e}")
                    frame_with_detections = frame
                
                # Encodear
                q = int(jpeg_quality) if jpeg_quality is not None else 85
                if q < 30:
                    q = 30
                if q > 95:
                    q = 95
                ret, buffer = cv2.imencode('.jpg', frame_with_detections, [cv2.IMWRITE_JPEG_QUALITY, q])
                if not ret:
                    continue
                
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                       
            except Exception as e:
                logger.error(f"Camera {camera_index} (detection): Stream error: {e}")
                consecutive_failures += 1
                if consecutive_failures >= max_failures:
                    break
                continue
                
    except GeneratorExit:
        logger.info(f"Camera {camera_index} (detection): Stream closed by client")
    except Exception as e:
        logger.error(f"Camera {camera_index} (detection): Fatal stream error: {e}")
    finally:
        if cap and cap.isOpened():
            cap.release()
            logger.info(f"Camera {camera_index} (detection): Released")


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
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            raise HTTPException(500, "Camera not available")
        
        ret, frame = cap.read()
        cap.release()
        
        if not ret:
            raise HTTPException(500, "Cannot read frame")
        
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
        if source == "camera":
            cap = _get_capture_by_index(int(index or 0))
            if not cap or not cap.isOpened():
                raise HTTPException(503, "Camera not available")
            ret, frame = cap.read()
            cap.release()
            if not ret or frame is None:
                raise HTTPException(503, "Cannot read frame")
        else:
            try:
                from PIL import ImageGrab
                import numpy as np
            except ImportError:
                raise HTTPException(503, "PIL not available for screen capture")
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
        ret, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, q])
        if not ret:
            raise HTTPException(500, "Encode failed")
        return Response(content=buffer.tobytes(), media_type="image/jpeg")
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
