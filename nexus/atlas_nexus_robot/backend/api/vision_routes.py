"""
Vision API Routes
"""

import os
from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from typing import List, Dict
import cv2
import logging

from vision.object_detection import get_detector

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/vision", tags=["vision"])


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
    """Obtiene captura por índice de cámara."""
    backend = cv2.CAP_MSMF if hasattr(cv2, "CAP_MSMF") else cv2.CAP_ANY
    cap = cv2.VideoCapture(index, backend)
    return cap if cap.isOpened() else None


def generate_frames(camera_index: int = 0):
    """Stream de video simple. camera_index: 0, 1, 2... para cada cámara instalada."""
    cap = _get_capture_by_index(camera_index)
    if not cap:
        return
    try:
        while True:
            success, frame = cap.read()
            if not success or frame is None:
                break
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                break
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    finally:
        cap.release()


def generate_screen_frames():
    """Stream de captura de pantalla para navegación en PC (cerebro)."""
    try:
        from PIL import ImageGrab
        import numpy as np
    except ImportError:
        logger.warning("PIL/Pillow no disponible para screen capture")
        return

    while True:
        try:
            img = ImageGrab.grab()
            frame = np.array(img)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                break
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        except Exception as e:
            logger.debug("Screen capture error: %s", e)
            break


def generate_frames_with_detection(camera_index: int = 0):
    """Stream de video con detecciones YOLO. camera_index: índice de cámara."""
    cap = _get_capture_by_index(camera_index)
    if not cap:
        return
    detector = get_detector()
    
    if not detector.is_loaded:
        detector.load_model()
    
    try:
        while True:
            success, frame = cap.read()
            if not success:
                break
            
            # Detectar objetos
            detections = detector.detect(frame, confidence=0.5)
            
            # Dibujar detecciones
            frame_with_detections = detector.draw_detections(frame, detections)
            
            # Encodear
            ret, buffer = cv2.imencode('.jpg', frame_with_detections)
            frame_bytes = buffer.tobytes()
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    finally:
        cap.release()


@router.get("/camera/stream")
async def video_stream(index: int = 0):
    """Stream de video sin detección. index: 0, 1, 2... para cada cámara instalada."""
    return StreamingResponse(
        generate_frames(camera_index=index),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )


@router.get("/camera/stream/detection")
async def video_stream_with_detection(index: int = 0):
    """Stream de video CON detección YOLO. index: índice de cámara."""
    return StreamingResponse(
        generate_frames_with_detection(camera_index=index),
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
async def screen_stream():
    """Stream de captura de pantalla para navegación en PC (cerebro)."""
    return StreamingResponse(
        generate_screen_frames(),
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
        "model": props.get("model", "Desconocida"),
        "resolution": props.get("resolution", [640, 480]),
        "capabilities": props.get("capabilities", ["video"]),
        "integration": "ATLAS PUSH: consumir stream_url (reconocimiento) o screen_stream_url (navegación PC).",
    }
