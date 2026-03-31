#!/usr/bin/env python3
"""
ATLAS NEXUS Robot Backend - Main Application
Sistema robótico autónomo con API REST y WebSocket
"""

import asyncio
import json
import logging
import os
import sys
from datetime import datetime
from typing import Any, Dict, List

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# Asegurar imports locales (brain/, api/, vision/, etc.) aunque se ejecute desde otro cwd
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)
REPO_ROOT = os.path.abspath(os.path.join(BASE_DIR, "..", "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

# Import camera service routes
from api.camera_service_routes import router as camera_router
# Import vision routes
from api.vision_routes import router as vision_router
# Import YOLO detector
from yolo_detector import get_detector

# Boot logger used before full logging setup.
boot_logger = logging.getLogger("ATLAS_ROBOT_BOOT")

# Import memory connector routes
try:
    from brain.memory_connector import memory_router
except Exception as e:
    boot_logger.warning("Memory connector disabled: %s", e)
    memory_router = None

# Configurar logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("ATLAS_ROBOT_BACKEND")

# Crear aplicación FastAPI
app = FastAPI(
    title="ATLAS NEXUS Robot Backend API",
    description="Sistema robótico autónomo con IA y visión por computadora",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc",
)

# Configurar CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include vision routes
app.include_router(vision_router)

# Include brain routes (optional: may require heavy deps like langchain)
try:
    from api.brain_routes import router as brain_router  # type: ignore

    app.include_router(brain_router)
except Exception as e:
    logger.warning("Brain routes disabled (missing deps): %s", e)

# Include camera service routes
app.include_router(camera_router)

# Include memory connector routes
if memory_router:
    app.include_router(memory_router)


# Pydantic models for YOLO detection
class DetectionRequest(BaseModel):
    confidence_threshold: float = 0.5
    draw_boxes: bool = True


class DetectionResponse(BaseModel):
    success: bool
    detections: List[Dict[str, Any]]
    total_objects: int
    processing_time: float
    image_shape: List[int]


# Estado del robot
robot_status = {
    "status": "online",
    "uptime": 0,
    "modules": {
        "camera": False,
        "vision": False,
        "ai": False,
        "yolo": False,  # Add YOLO module
        "control": False,
        "sensors": False,
        "actuators": False,
        "communication": False,
    },
    "last_update": datetime.now().isoformat(),
    "version": "1.0.0",
    "websocket_clients": 0,
    "yolo_stats": {
        "total_detections": 0,
        "average_confidence": 0.0,
        "most_common_class": None,
        "detection_rate": 0.0,
    },
}


# ── Persistent Camera Manager ─────────────────────────────────────────────────
import threading
import time as _time

class LiveCameraManager:
    """Mantiene la cámara USB abierta y sirve frames estabilizados."""

    def __init__(self, camera_index: int = 0, target_fps: float = 10.0):
        self._index = camera_index
        self._target_fps = target_fps
        self._cap: Any = None
        self._frame: Any = None
        self._prev_gray: Any = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._frame_count = 0
        self._motion_score = 0.0
        self._motion_detected = False
        self._started_at: float = 0.0

    def start(self) -> bool:
        if self._running:
            return True
        # Retry: DirectShow puede tardar en liberar el device tras un kill
        for attempt in range(3):
            self._cap = cv2.VideoCapture(self._index, cv2.CAP_DSHOW)
            if self._cap.isOpened():
                break
            logger.warning("LiveCamera: attempt %d — cannot open index %d, retrying...",
                           attempt + 1, self._index)
            _time.sleep(2)
        if not self._cap or not self._cap.isOpened():
            logger.error("LiveCamera: cannot open index %d after 3 attempts", self._index)
            return False
        # 720p para balance CPU/calidad
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self._running = True
        self._started_at = _time.time()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        logger.info("LiveCamera STARTED on index %d (target %.1f fps)", self._index, self._target_fps)
        return True

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
        if self._cap:
            self._cap.release()
        self._cap = None
        logger.info("LiveCamera STOPPED")

    def _capture_loop(self):
        # Warmup: descartar primeros frames para auto-exposure (Insta360 necesita ~5 frames)
        for i in range(8):
            ret, _ = self._cap.read()
            if not ret:
                logger.warning("LiveCamera warmup frame %d failed", i)
        logger.info("LiveCamera warmup done — starting capture loop")
        interval = 1.0 / self._target_fps
        consecutive_fails = 0
        while self._running:
            t0 = _time.time()
            ret, frame = self._cap.read()
            if ret and frame is not None:
                consecutive_fails = 0
                with self._lock:
                    self._frame = frame.copy()
                    self._frame_count += 1
                # Detección de movimiento simple
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                gray = cv2.GaussianBlur(gray, (21, 21), 0)
                if self._prev_gray is not None:
                    delta = cv2.absdiff(self._prev_gray, gray)
                    thresh = cv2.threshold(delta, 25, 255, cv2.THRESH_BINARY)[1]
                    motion = (thresh.sum() / 255.0) / (gray.shape[0] * gray.shape[1]) * 100.0
                    self._motion_score = round(motion, 2)
                    self._motion_detected = motion > 1.0  # >1% pixeles cambiaron
                self._prev_gray = gray
            else:
                consecutive_fails += 1
                if consecutive_fails == 1:
                    logger.warning("LiveCamera: frame read failed — reconnecting...")
                if consecutive_fails >= 10:
                    # Reconectar cámara
                    logger.warning("LiveCamera: %d consecutive failures — reopening device", consecutive_fails)
                    try:
                        self._cap.release()
                        _time.sleep(2)
                        self._cap = cv2.VideoCapture(self._index, cv2.CAP_DSHOW)
                        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                        if self._cap.isOpened():
                            logger.info("LiveCamera: reconnected successfully")
                            consecutive_fails = 0
                        else:
                            logger.error("LiveCamera: reconnect failed — waiting 5s")
                            _time.sleep(5)
                    except Exception as e:
                        logger.error("LiveCamera reconnect error: %s", e)
                        _time.sleep(5)
            elapsed = _time.time() - t0
            sleep_time = max(0, interval - elapsed)
            if sleep_time > 0:
                _time.sleep(sleep_time)

    def get_frame(self) -> tuple[bool, Any]:
        with self._lock:
            if self._frame is not None:
                return True, self._frame.copy()
        return False, None

    def get_jpeg(self, quality: int = 85) -> bytes | None:
        ok, frame = self.get_frame()
        if not ok:
            return None
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        h, w = frame.shape[:2]
        cv2.putText(frame, f"ATLAS NEXUS LIVE | {w}x{h} | motion:{self._motion_score:.1f}%",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        cv2.putText(frame, ts, (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        if self._motion_detected:
            cv2.putText(frame, "MOTION", (w - 120, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        _, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        return buf.tobytes()

    @property
    def status(self) -> dict:
        return {
            "running": self._running,
            "camera_index": self._index,
            "frame_count": self._frame_count,
            "target_fps": self._target_fps,
            "motion_score_pct": float(self._motion_score),
            "motion_detected": bool(self._motion_detected),
            "uptime_sec": round(_time.time() - self._started_at, 1) if self._running else 0,
            "has_frame": self._frame is not None,
        }

live_camera = LiveCameraManager(camera_index=int(os.getenv("NEXUS_CAMERA_INDEX", "0")))


# WebSocket connections manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self.camera_streaming = False

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        robot_status["websocket_clients"] = len(self.active_connections)
        logger.info(
            f"✅ WebSocket client connected. Total: {len(self.active_connections)}"
        )

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)
        robot_status["websocket_clients"] = len(self.active_connections)
        logger.info(
            f"❌ WebSocket client disconnected. Total: {len(self.active_connections)}"
        )

    async def send_personal_message(self, message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: Dict[str, Any]):
        for connection in self.active_connections:
            try:
                await connection.send_text(json.dumps(message))
            except:
                # Connection closed, remove it
                self.active_connections.remove(connection)


manager = ConnectionManager()
ai_consultant_instance = None


@app.get("/")
async def root():
    """Endpoint principal — redirige al dashboard 3D"""
    return {
        "message": "🤖 ATLAS NEXUS Robot Backend API",
        "status": "online",
        "version": "1.0.0",
        "timestamp": datetime.now().isoformat(),
        "websocket_clients": robot_status["websocket_clients"],
        "dashboard": "/dashboard",
    }


@app.get("/dashboard")
async def serve_dashboard():
    """Sirve el panel 3D del robot"""
    from pathlib import Path

    from fastapi.responses import FileResponse

    p = Path(__file__).parent / "static" / "robot3d.html"
    if p.exists():
        return FileResponse(p, headers={"Cache-Control": "no-cache, no-store"})
    return {"ok": False, "error": "robot3d.html not found"}


@app.get("/status")
@app.get("/api/status")
async def get_status():
    """Obtener estado completo del robot"""
    return robot_status


@app.get("/health")
@app.get("/api/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "uptime": robot_status["uptime"],
        "modules_active": sum(
            1 for active in robot_status["modules"].values() if active
        ),
    }


@app.get("/modules")
@app.get("/api/modules")
async def get_modules():
    """Obtener estado de todos los módulos"""
    return robot_status["modules"]


@app.post("/modules/{module_name}/enable")
@app.post("/api/modules/{module_name}/enable")
async def enable_module(module_name: str):
    """Activar un módulo específico"""
    if module_name in robot_status["modules"]:
        robot_status["modules"][module_name] = True
        logger.info(f"✅ Module {module_name} enabled")

        # Broadcast status update
        await manager.broadcast(
            {
                "type": "module_status",
                "module": module_name,
                "status": True,
                "timestamp": datetime.now().isoformat(),
            }
        )

        return {"message": f"Module {module_name} enabled successfully"}
    else:
        raise HTTPException(status_code=404, detail=f"Module {module_name} not found")


@app.post("/modules/{module_name}/disable")
@app.post("/api/modules/{module_name}/disable")
async def disable_module(module_name: str):
    """Desactivar un módulo específico"""
    if module_name in robot_status["modules"]:
        robot_status["modules"][module_name] = False
        logger.info(f"❌ Module {module_name} disabled")

        # Broadcast status update
        await manager.broadcast(
            {
                "type": "module_status",
                "module": module_name,
                "status": False,
                "timestamp": datetime.now().isoformat(),
            }
        )

        return {"message": f"Module {module_name} disabled successfully"}
    else:
        raise HTTPException(status_code=404, detail=f"Module {module_name} not found")


class CommandBody(BaseModel):
    actuador: str = ""
    estado: int = 1
    velocidad: int = 255

    class Config:
        extra = "allow"


@app.get("/hardware/status")
@app.get("/api/hardware/status")
async def hardware_status():
    """Detección real de hardware conectado — sin mocks."""
    import serial.tools.list_ports as slp
    com_ports = [{"device": p.device, "description": p.description, "hwid": p.hwid}
                 for p in slp.comports()]
    cameras = []
    for idx in range(6):
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            cap.release()
            cameras.append({"index": idx, "resolution": f"{w}x{h}"})
        else:
            break
    ros2_available = False
    try:
        import rclpy  # noqa: F401
        ros2_available = True
    except ImportError:
        pass
    return {
        "serial_ports": com_ports,
        "serial_count": len(com_ports),
        "cameras": cameras,
        "camera_count": len(cameras),
        "ros2_installed": ros2_available,
        "pyautogui_installed": bool(robot_status["modules"].get("control")),
        "actuators_connected": len(com_ports) > 0,
        "verdict": "HARDWARE PRESENT" if (com_ports or cameras) else "SOFTWARE ONLY",
    }


@app.post("/command")
@app.post("/api/command")
async def receive_command(body: CommandBody):
    """Recibe comandos estructurados desde ATLAS_PUSH (cerebro)."""
    cmd = dict(body.model_dump(exclude_none=True))
    if hasattr(body, "__pydantic_extra__") and body.__pydantic_extra__:
        cmd.update(body.__pydantic_extra__)
    logger.info("Comando recibido: %s", cmd)
    await manager.broadcast(
        {"type": "command", "payload": cmd, "timestamp": datetime.now().isoformat()}
    )
    return {"ok": True, "received": cmd}


@app.get("/camera/stream")
@app.get("/api/camera/stream")
async def camera_stream():
    """Snapshot JPEG del stream persistente de la Insta360."""
    import io
    from fastapi.responses import StreamingResponse

    jpeg = live_camera.get_jpeg(quality=90)
    if jpeg is None:
        return {"status": "error", "message": "Camera not running or no frame available",
                "camera_status": live_camera.status}
    return StreamingResponse(
        io.BytesIO(jpeg), media_type="image/jpeg",
        headers={"Cache-Control": "no-cache"},
    )


@app.get("/camera/mjpeg")
@app.get("/api/camera/mjpeg")
async def camera_mjpeg():
    """MJPEG streaming continuo — abre en navegador para video en vivo."""
    from fastapi.responses import StreamingResponse

    def generate():
        while True:
            jpeg = live_camera.get_jpeg(quality=75)
            if jpeg:
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
            _time.sleep(1.0 / live_camera._target_fps)

    return StreamingResponse(generate(),
                             media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/camera/status")
@app.get("/api/camera/status")
async def camera_live_status():
    """Estado del camera manager persistente."""
    return {"camera": live_camera.status, "mode": "LIVE"}


@app.get("/camera/detect")
@app.get("/api/camera/detect")
async def camera_detect():
    """Detecta cámaras USB reales. Si LiveCamera tiene una activa, la reporta sin reabrir."""
    found = []
    active_idx = live_camera._index if live_camera._running else -1
    for idx in range(6):
        if idx == active_idx:
            # Ya abierta por LiveCamera — no reabrir (DirectShow bloquea)
            found.append({"index": idx, "resolution": "1280x720", "frame_ok": True,
                          "note": "active in LiveCameraManager"})
            continue
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            ret, _ = cap.read()
            cap.release()
            found.append({"index": idx, "resolution": f"{w}x{h}", "frame_ok": bool(ret)})
        else:
            cap.release()
            break
    return {"cameras": found, "count": len(found),
            "active_index": int(os.getenv("NEXUS_CAMERA_INDEX", "0"))}


@app.post("/camera/yolo")
@app.post("/api/camera/yolo")
async def camera_yolo_detect():
    """Ejecuta YOLO sobre el frame actual de la cámara en vivo."""
    ok, frame = live_camera.get_frame()
    if not ok:
        return {"status": "error", "message": "No live frame available"}
    detector = get_detector()
    t0 = _time.time()
    detections = detector.detect_objects(frame)
    elapsed = round(_time.time() - t0, 4)
    return {
        "status": "success",
        "source": "live_camera",
        "detections": detections,
        "total_objects": len(detections),
        "processing_time_sec": elapsed,
        "motion_score_pct": live_camera._motion_score,
        "motion_detected": live_camera._motion_detected,
    }


@app.get("/camera/test")
@app.get("/api/camera/test")
async def test_camera():
    """Test de cámara real — usa LiveCameraManager si ya está corriendo."""
    try:
        # Si el LiveCameraManager ya tiene la cámara abierta, usar ese
        if live_camera._running and live_camera._frame is not None:
            jpeg = live_camera.get_jpeg()
            if jpeg and len(jpeg) > 0:
                logger.info("Camera test OK via LiveCameraManager (frame %d bytes)", len(jpeg))
                return {"status": "success", "hw_detected": True,
                        "resolution": "1280x720", "frame_ok": True,
                        "camera_index": live_camera._index}
        # Fallback: abrir directamente (solo si LiveCamera no está corriendo)
        camera_index = int(os.getenv("NEXUS_CAMERA_INDEX", "0"))
        cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        if not cap.isOpened():
            logger.warning("Camera test: no device at index %d", camera_index)
            return {"status": "error", "hw_detected": False,
                    "message": f"No camera at index {camera_index}"}
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        ret, frame = cap.read()
        cap.release()
        if ret and frame is not None:
            logger.info("Camera test OK: %dx%d at index %d", w, h, camera_index)
            return {"status": "success", "hw_detected": True,
                    "resolution": f"{w}x{h}", "frame_ok": True,
                    "camera_index": camera_index}
        return {"status": "error", "hw_detected": True,
                "message": "Camera opened but frame read failed"}
    except Exception as e:
        logger.error("Camera test error: %s", e)
        return {"status": "error", "message": str(e)}


@app.get("/ai/test")
@app.get("/api/ai/test")
async def test_ai():
    """Testear módulos de IA"""
    try:
        import numpy as np

        logger.info("✅ NumPy working")

        # Test TensorFlow (opcional)
        try:
            import tensorflow as tf

            logger.info("✅ TensorFlow working")
            tf_available = True
        except ImportError:
            logger.warning("⚠️ TensorFlow not available")
            tf_available = False

        # Test PyTorch (opcional)
        try:
            import torch

            logger.info("✅ PyTorch working")
            torch_available = True
        except ImportError:
            logger.warning("⚠️ PyTorch not available")
            torch_available = False

        return {
            "status": "success",
            "numpy": True,
            "tensorflow": tf_available,
            "pytorch": torch_available,
            "message": "AI modules tested",
        }
    except ImportError:
        return {"status": "error", "message": "NumPy not available"}
    except Exception as e:
        return {"status": "error", "message": f"AI test failed: {str(e)}"}


@app.get("/vision/test")
@app.get("/api/vision/test")
async def test_vision():
    """Testear módulo de visión"""
    try:
        import cv2
        import numpy as np

        # Crear imagen de prueba
        test_image = np.zeros((100, 100, 3), dtype=np.uint8)

        # Test operaciones básicas de visión
        gray = cv2.cvtColor(test_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        logger.info("✅ Computer vision modules working")
        return {
            "status": "success",
            "opencv": True,
            "numpy": True,
            "operations": ["cvtColor", "Canny"],
            "test_image_shape": test_image.shape,
            "gray_image_shape": gray.shape,
            "edges_image_shape": edges.shape,
            "message": "Vision modules tested successfully",
        }
    except ImportError as e:
        return {"status": "error", "message": f"Vision module missing: {str(e)}"}
    except Exception as e:
        return {"status": "error", "message": f"Vision test failed: {str(e)}"}


@app.get("/system/info")
@app.get("/api/system/info")
async def get_system_info():
    """Obtener información del sistema"""
    import platform

    import psutil

    mem = psutil.virtual_memory()
    try:
        disk = psutil.disk_usage("/")
    except Exception:
        disk = psutil.disk_usage("C:\\")

    return {
        "platform": platform.system(),
        "platform_version": platform.version(),
        "python_version": platform.python_version(),
        "cpu_count": psutil.cpu_count(),
        "cpu_percent": psutil.cpu_percent(interval=0.5),
        "ram": {
            "percent": mem.percent,
            "used_gb": round(mem.used / (1024**3), 1),
            "total_gb": round(mem.total / (1024**3), 1),
        },
        "disk": {
            "percent": disk.percent,
            "used_gb": round(disk.used / (1024**3), 1),
            "total_gb": round(disk.total / (1024**3), 1),
        },
        "memory_total": f"{mem.total / (1024**3):.2f} GB",
        "memory_available": f"{mem.available / (1024**3):.2f} GB",
        "disk_usage": f"{disk.percent}%",
        "timestamp": datetime.now().isoformat(),
    }


# YOLO Object Detection Endpoints
@app.post("/yolo/detect", response_model=DetectionResponse)
async def detect_objects(request: DetectionRequest):
    """Detect objects in image using YOLO"""
    import base64
    import time

    try:
        # Get detector
        detector = get_detector()

        # Create test image (in real implementation, this would come from camera)
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        test_image[:] = (50, 100, 150)  # Background color

        # Add some test objects (rectangles)
        cv2.rectangle(test_image, (100, 100), (200, 200), (255, 0, 0), -1)
        cv2.rectangle(test_image, (300, 150), (400, 250), (0, 255, 0), -1)
        cv2.rectangle(test_image, (450, 200), (550, 300), (0, 0, 255), -1)

        # Update confidence threshold
        detector.confidence_threshold = request.confidence_threshold

        # Perform detection
        start_time = time.time()
        detections = detector.detect_objects(test_image)
        processing_time = time.time() - start_time

        # Draw boxes if requested
        if request.draw_boxes:
            annotated_image = detector.draw_detections(test_image, detections)
        else:
            annotated_image = test_image

        # Convert image to base64 for response
        _, buffer = cv2.imencode(".jpg", annotated_image)
        img_base64 = base64.b64encode(buffer).decode("utf-8")

        # Update robot status
        robot_status["modules"]["yolo"] = True
        robot_status["yolo_stats"] = detector.get_detection_stats()
        robot_status["last_update"] = datetime.now().isoformat()

        return DetectionResponse(
            success=True,
            detections=detections,
            total_objects=len(detections),
            processing_time=processing_time,
            image_shape=list(test_image.shape),
        )

    except Exception as e:
        logger.error(f"YOLO detection error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/yolo/stats")
async def get_yolo_stats():
    """Get YOLO detection statistics"""
    try:
        detector = get_detector()
        stats = detector.get_detection_stats()
        summary = detector.get_summary()

        return {
            "success": True,
            "stats": stats,
            "summary": summary,
            "timestamp": datetime.now().isoformat(),
        }

    except Exception as e:
        logger.error(f"YOLO stats error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/yolo/detect_from_camera")
async def detect_from_camera(request: DetectionRequest):
    """Detect objects from camera feed"""
    import time

    try:
        # Get detector
        detector = get_detector()

        # Try to capture from camera
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            # Fallback to test image if camera not available
            test_image = np.zeros((480, 640, 3), dtype=np.uint8)
            test_image[:] = (50, 100, 150)
        else:
            ret, frame = cap.read()
            if ret:
                test_image = frame
            else:
                test_image = np.zeros((480, 640, 3), dtype=np.uint8)
                test_image[:] = (50, 100, 150)
            cap.release()

        # Update confidence threshold
        detector.confidence_threshold = request.confidence_threshold

        # Perform detection
        start_time = time.time()
        detections = detector.detect_objects(test_image)
        processing_time = time.time() - start_time

        # Draw boxes if requested
        if request.draw_boxes:
            annotated_image = detector.draw_detections(test_image, detections)
        else:
            annotated_image = test_image

        # Update robot status
        robot_status["modules"]["yolo"] = True
        robot_status["yolo_stats"] = detector.get_detection_stats()
        robot_status["last_update"] = datetime.now().isoformat()

        return DetectionResponse(
            success=True,
            detections=detections,
            total_objects=len(detections),
            processing_time=processing_time,
            image_shape=list(test_image.shape),
        )

    except Exception as e:
        logger.error(f"Camera YOLO detection error: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/yolo/test")
async def test_yolo():
    """Test YOLO system"""
    try:
        detector = get_detector()
        summary = detector.get_summary()

        return {
            "status": "success",
            "message": "YOLO system operational",
            "model_loaded": summary["model_loaded"],
            "device": summary["device"],
            "confidence_threshold": summary["confidence_threshold"],
            "total_classes": summary["total_classes"],
            "timestamp": datetime.now().isoformat(),
        }

    except Exception as e:
        logger.error(f"YOLO test error: {e}")
        return {
            "status": "error",
            "message": f"YOLO system error: {str(e)}",
            "timestamp": datetime.now().isoformat(),
        }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint para comunicación en tiempo real"""
    await manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_text()

            try:
                message = json.loads(data)

                # Procesar diferentes tipos de mensajes
                if message.get("type") == "ping":
                    await manager.send_personal_message(
                        json.dumps(
                            {"type": "pong", "timestamp": datetime.now().isoformat()}
                        ),
                        websocket,
                    )
                elif message.get("type") == "get_status":
                    await manager.send_personal_message(
                        json.dumps(
                            {
                                "type": "status_update",
                                "status": robot_status,
                                "timestamp": datetime.now().isoformat(),
                            }
                        ),
                        websocket,
                    )
                elif message.get("type") == "camera_stream":
                    # Iniciar streaming de cámara
                    if message.get("action") == "start":
                        manager.camera_streaming = True
                        await websocket.send_text(
                            json.dumps(
                                {
                                    "type": "camera_stream_started",
                                    "message": "Camera streaming started",
                                }
                            )
                        )
                    elif message.get("action") == "stop":
                        manager.camera_streaming = False
                        await websocket.send_text(
                            json.dumps(
                                {
                                    "type": "camera_stream_stopped",
                                    "message": "Camera streaming stopped",
                                }
                            )
                        )

            except json.JSONDecodeError:
                await manager.send_personal_message(
                    json.dumps({"type": "error", "message": "Invalid JSON"}), websocket
                )

    except WebSocketDisconnect:
        manager.disconnect(websocket)


async def update_uptime():
    """Actualizar uptime del robot"""
    start_time = getattr(update_uptime, "start_time", datetime.now())
    uptime_seconds = (datetime.now() - start_time).total_seconds()
    robot_status["uptime"] = uptime_seconds
    robot_status["last_update"] = datetime.now().isoformat()


async def broadcast_status():
    """Broadcast status updates periodically"""
    while True:
        await asyncio.sleep(5)  # Cada 5 segundos
        await update_uptime()

        # Broadcast status a todos los clientes
        await manager.broadcast(
            {
                "type": "status_update",
                "status": robot_status,
                "timestamp": datetime.now().isoformat(),
            }
        )


@app.on_event("startup")
async def startup_event():
    """Evento de startup"""
    logger.info("🚀 ATLAS NEXUS Robot Backend starting up...")
    update_uptime.start_time = datetime.now()

    # Iniciar camera manager persistente
    cam_ok = live_camera.start()
    if cam_ok:
        logger.info("LiveCameraManager started — Insta360 capturing at 10fps")
    else:
        logger.warning("LiveCameraManager failed to start — no camera available")

    # Testear módulos básicos
    camera_test = await test_camera()
    robot_status["modules"]["camera"] = camera_test["status"] == "success" or cam_ok

    ai_test = await test_ai()
    robot_status["modules"]["ai"] = ai_test["status"] == "success"

    vision_test = await test_vision()
    robot_status["modules"]["vision"] = vision_test["status"] == "success"

    # Test YOLO
    try:
        yolo_test = await test_yolo()
        robot_status["modules"]["yolo"] = yolo_test["status"] == "success"
    except Exception as e:
        logger.warning(f"YOLO initialization failed: {e}")
        robot_status["modules"]["yolo"] = False

    # Detección real de hardware — no marcar True sin evidencia
    robot_status["modules"]["communication"] = True  # WebSocket siempre disponible
    # Control: solo True si pyautogui disponible (HID)
    try:
        import pyautogui  # noqa: F401
        robot_status["modules"]["control"] = True
    except ImportError:
        robot_status["modules"]["control"] = False
    # Sensors: True si al menos una cámara real detectada
    robot_status["modules"]["sensors"] = robot_status["modules"]["camera"]
    # Actuators: False — no hay motores/serial conectados (sin puertos COM detectados)
    import serial.tools.list_ports as _slp
    _com_ports = list(_slp.comports())
    robot_status["modules"]["actuators"] = len(_com_ports) > 0
    if _com_ports:
        logger.info("Serial ports detected: %s", [p.device for p in _com_ports])
    else:
        logger.info("No serial/COM ports — actuators module disabled (no physical hardware)")

    # Inicializar AI Consultant (Bedrock/Direct) para aprendizaje continuo.
    global ai_consultant_instance
    try:
        from brain.learning.ai_consultant import AIConsultant

        ai_consultant_instance = AIConsultant()
        logger.info(
            "AI Consultant inicializado (modo=%s, region=%s)",
            getattr(ai_consultant_instance, "ai_mode", "unknown"),
            getattr(ai_consultant_instance, "aws_region", "unknown"),
        )
    except Exception as e:
        logger.warning("No se pudo inicializar AI Consultant: %s", e)

    # Iniciar background task para broadcast de status
    asyncio.create_task(broadcast_status())

    logger.info("✅ ATLAS NEXUS Robot Backend ready!")


@app.on_event("shutdown")
async def shutdown_event():
    """Evento de shutdown"""
    live_camera.stop()
    logger.info("ATLAS NEXUS Robot Backend shutting down...")


if __name__ == "__main__":
    # Prints sin emojis para evitar UnicodeEncodeError en Windows (cp1252)
    print("ATLAS NEXUS Robot Backend - Starting API Server...")
    print("=" * 60)
    print("API: http://localhost:8002")
    print("Docs: http://localhost:8002/docs")
    print("WebSocket: ws://localhost:8002/ws")
    print("=" * 60)

    uvicorn.run("main:app", host="0.0.0.0", port=8002, reload=False, log_level="info")
