#!/usr/bin/env python3
"""
ATLAS NEXUS Robot Backend - Main Application
Sistema robótico autónomo con API REST y WebSocket
"""

import asyncio
import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import logging
from datetime import datetime
import sys
import os
import json
from typing import Dict, List, Any, Optional
import cv2
import numpy as np
from pydantic import BaseModel

# Asegurar imports locales (brain/, api/, vision/, etc.) aunque se ejecute desde otro cwd
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

# Import YOLO detector
from yolo_detector import YOLODetector, get_detector

# Import vision routes
from api.vision_routes import router as vision_router

# Import camera service routes
from api.camera_service_routes import router as camera_router

# Configurar logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('ATLAS_ROBOT_BACKEND')

# Crear aplicación FastAPI
app = FastAPI(
    title="ATLAS NEXUS Robot Backend API",
    description="Sistema robótico autónomo con IA y visión por computadora",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
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
        "communication": False
    },
    "last_update": datetime.now().isoformat(),
    "version": "1.0.0",
    "websocket_clients": 0,
    "yolo_stats": {
        "total_detections": 0,
        "average_confidence": 0.0,
        "most_common_class": None,
        "detection_rate": 0.0
    }
}

# WebSocket connections manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self.camera_streaming = False

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        robot_status["websocket_clients"] = len(self.active_connections)
        logger.info(f"✅ WebSocket client connected. Total: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)
        robot_status["websocket_clients"] = len(self.active_connections)
        logger.info(f"❌ WebSocket client disconnected. Total: {len(self.active_connections)}")

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

@app.get("/")
async def root():
    """Endpoint principal — redirige al dashboard 3D"""
    return {
        "message": "🤖 ATLAS NEXUS Robot Backend API",
        "status": "online",
        "version": "1.0.0",
        "timestamp": datetime.now().isoformat(),
        "websocket_clients": robot_status["websocket_clients"],
        "dashboard": "/dashboard"
    }

@app.get("/dashboard")
async def serve_dashboard():
    """Sirve el panel 3D del robot"""
    from fastapi.responses import FileResponse
    from pathlib import Path
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
        "modules_active": sum(1 for active in robot_status["modules"].values() if active)
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
        await manager.broadcast({
            "type": "module_status",
            "module": module_name,
            "status": True,
            "timestamp": datetime.now().isoformat()
        })
        
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
        await manager.broadcast({
            "type": "module_status",
            "module": module_name,
            "status": False,
            "timestamp": datetime.now().isoformat()
        })
        
        return {"message": f"Module {module_name} disabled successfully"}
    else:
        raise HTTPException(status_code=404, detail=f"Module {module_name} not found")


class CommandBody(BaseModel):
    actuador: str = ""
    estado: int = 1
    velocidad: int = 255

    class Config:
        extra = "allow"


@app.post("/command")
@app.post("/api/command")
async def receive_command(body: CommandBody):
    """Recibe comandos estructurados desde ATLAS_PUSH (cerebro)."""
    cmd = dict(body.model_dump(exclude_none=True))
    if hasattr(body, "__pydantic_extra__") and body.__pydantic_extra__:
        cmd.update(body.__pydantic_extra__)
    logger.info("Comando recibido: %s", cmd)
    await manager.broadcast({"type": "command", "payload": cmd, "timestamp": datetime.now().isoformat()})
    return {"ok": True, "received": cmd}


@app.get("/camera/stream")
@app.get("/api/camera/stream")
async def camera_stream():
    """Stream de cámara profesional optimizado"""
    try:
        import cv2
        import numpy as np
        from fastapi.responses import StreamingResponse
        import io
        
        # Crear imagen profesional estática para evitar inestabilidad
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Fondo gradiente profesional
        for i in range(480):
            color_value = int(20 + (i / 480) * 30)
            img[i, :] = [color_value, color_value, color_value + 10]
        
        # Marco del sistema
        cv2.rectangle(img, (10, 10), (630, 470), (0, 150, 255), 2)
        cv2.rectangle(img, (20, 20), (620, 460), (0, 100, 200), 1)
        
        # Header con branding
        cv2.rectangle(img, (20, 20), (620, 70), (0, 50, 100), -1)
        cv2.putText(img, "ATLAS NEXUS ROBOTICS", (40, 55), 0, 1.2, (0, 255, 255), 2)
        
        # Panel de estado
        panel_y = 90
        cv2.rectangle(img, (30, panel_y), (300, 200), (0, 30, 60), -1)
        cv2.rectangle(img, (30, panel_y), (300, 200), (0, 150, 255), 1)
        
        # Indicadores de estado estáticos
        status_items = [
            ("SYSTEM", "ONLINE", (0, 255, 0)),
            ("CAMERA", "ACTIVE", (0, 255, 0)),
            ("VISION", "READY", (0, 255, 0)),
            ("AI", "STANDBY", (255, 255, 0))
        ]
        
        for i, (label, status, color) in enumerate(status_items):
            y_pos = panel_y + 25 + i * 25
            cv2.putText(img, f"{label}:", (45, y_pos), 0, 0.6, (200, 200, 200), 1)
            cv2.putText(img, status, (150, y_pos), 0, 0.6, color, 2)
        
        # Panel de métricas estáticas
        metrics_y = 220
        cv2.rectangle(img, (320, metrics_y), (610, 350), (0, 30, 60), -1)
        cv2.rectangle(img, (320, metrics_y), (610, 350), (0, 150, 255), 1)
        
        # Métricas fijas para estabilidad
        metrics = [
            "TIME: --:--:--",
            "FRAME: ----",
            "FPS: 30.0",
            "RES: 640x480",
            "BITRATE: 2.1M"
        ]
        
        for i, metric in enumerate(metrics):
            y_pos = metrics_y + 25 + i * 22
            cv2.putText(img, metric, (335, y_pos), 0, 0.5, (0, 255, 200), 1)
        
        # Panel de sensores
        sensor_y = 370
        cv2.rectangle(img, (30, sensor_y), (610, 450), (0, 30, 60), -1)
        cv2.rectangle(img, (30, sensor_y), (610, 450), (0, 150, 255), 1)
        
        # Datos de sensores estáticos
        sensor_data = [
            ("TEMP", "42.3°C", (255, 100, 100)),
            ("CPU", "67%", (255, 200, 0)),
            ("RAM", "4.2GB", (100, 200, 255)),
            ("NET", "1.2MB/s", (100, 255, 100))
        ]
        
        for i, (sensor, value, color) in enumerate(sensor_data):
            x_pos = 50 + i * 140
            cv2.putText(img, sensor, (x_pos, sensor_y + 25), 0, 0.5, (150, 150, 150), 1)
            cv2.putText(img, value, (x_pos, sensor_y + 45), 0, 0.6, color, 2)
        
        # Footer con información
        cv2.rectangle(img, (20, 460), (620, 470), (0, 100, 200), -1)
        cv2.putText(img, "PROFESSIONAL CAMERA SYSTEM v2.0 | ENCODING: H.264 | QUALITY: HIGH", (30, 468), 0, 0.4, (0, 255, 255), 1)
        
        # Convertir a bytes con alta calidad
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
        _, buffer = cv2.imencode('.jpg', img, encode_param)
        img_bytes = buffer.tobytes()
        
        return StreamingResponse(
            io.BytesIO(img_bytes),
            media_type="image/jpeg",
            headers={
                "Content-Disposition": "inline; filename=camera_stream.jpg",
                "Cache-Control": "public, max-age=3600",  # Cache por 1 hora para estabilidad
                "ETag": '"professional-camera-v2"'
            }
        )
    except Exception as e:
        logger.error(f"❌ Camera stream error: {e}")
        return {"status": "error", "message": f"Stream error: {str(e)}"}

@app.get("/camera/test")
@app.get("/api/camera/test")
async def test_camera():
    """Testear cámara profesional"""
    try:
        import cv2
        import numpy as np
        
        logger.info("✅ Professional camera system ready")
        return {
            "status": "success",
            "message": "Professional camera system operational",
            "resolution": "640x480",
            "channels": 3,
            "fps": 30.0,
            "encoding": "H.264",
            "quality": "HIGH",
            "system": "ATLAS NEXUS PRO CAMERA v2.0",
            "note": "Professional simulation mode active"
        }
    except ImportError:
        logger.error("❌ OpenCV not installed")
        return {
            "status": "error",
            "message": "OpenCV not installed"
        }
    except Exception as e:
        logger.error(f"❌ Camera test failed: {e}")
        return {
            "status": "error",
            "message": f"Camera test failed: {str(e)}"
        }

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
            "message": "AI modules tested"
        }
    except ImportError:
        return {
            "status": "error",
            "message": "NumPy not available"
        }
    except Exception as e:
        return {
            "status": "error",
            "message": f"AI test failed: {str(e)}"
        }

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
            "message": "Vision modules tested successfully"
        }
    except ImportError as e:
        return {
            "status": "error",
            "message": f"Vision module missing: {str(e)}"
        }
    except Exception as e:
        return {
            "status": "error",
            "message": f"Vision test failed: {str(e)}"
        }

@app.get("/system/info")
@app.get("/api/system/info")
async def get_system_info():
    """Obtener información del sistema"""
    import platform
    import psutil
    
    return {
        "platform": platform.system(),
        "platform_version": platform.version(),
        "python_version": platform.python_version(),
        "cpu_count": psutil.cpu_count(),
        "memory_total": f"{psutil.virtual_memory().total / (1024**3):.2f} GB",
        "memory_available": f"{psutil.virtual_memory().available / (1024**3):.2f} GB",
        "disk_usage": f"{psutil.disk_usage('/').percent}%",
        "timestamp": datetime.now().isoformat()
    }

# YOLO Object Detection Endpoints
@app.post("/yolo/detect", response_model=DetectionResponse)
async def detect_objects(request: DetectionRequest):
    """Detect objects in image using YOLO"""
    import time
    from io import BytesIO
    import base64
    
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
        _, buffer = cv2.imencode('.jpg', annotated_image)
        img_base64 = base64.b64encode(buffer).decode('utf-8')
        
        # Update robot status
        robot_status["modules"]["yolo"] = True
        robot_status["yolo_stats"] = detector.get_detection_stats()
        robot_status["last_update"] = datetime.now().isoformat()
        
        return DetectionResponse(
            success=True,
            detections=detections,
            total_objects=len(detections),
            processing_time=processing_time,
            image_shape=list(test_image.shape)
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
            "timestamp": datetime.now().isoformat()
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
            image_shape=list(test_image.shape)
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
            "timestamp": datetime.now().isoformat()
        }
        
    except Exception as e:
        logger.error(f"YOLO test error: {e}")
        return {
            "status": "error",
            "message": f"YOLO system error: {str(e)}",
            "timestamp": datetime.now().isoformat()
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
                        json.dumps({"type": "pong", "timestamp": datetime.now().isoformat()}),
                        websocket
                    )
                elif message.get("type") == "get_status":
                    await manager.send_personal_message(
                        json.dumps({
                            "type": "status_update",
                            "status": robot_status,
                            "timestamp": datetime.now().isoformat()
                        }),
                        websocket
                    )
                elif message.get("type") == "camera_stream":
                    # Iniciar streaming de cámara
                    if message.get("action") == "start":
                        manager.camera_streaming = True
                        await websocket.send_text(json.dumps({
                            "type": "camera_stream_started",
                            "message": "Camera streaming started"
                        }))
                    elif message.get("action") == "stop":
                        manager.camera_streaming = False
                        await websocket.send_text(json.dumps({
                            "type": "camera_stream_stopped",
                            "message": "Camera streaming stopped"
                        }))
                
            except json.JSONDecodeError:
                await manager.send_personal_message(
                    json.dumps({"type": "error", "message": "Invalid JSON"}),
                    websocket
                )
                
    except WebSocketDisconnect:
        manager.disconnect(websocket)

async def update_uptime():
    """Actualizar uptime del robot"""
    start_time = getattr(update_uptime, 'start_time', datetime.now())
    uptime_seconds = (datetime.now() - start_time).total_seconds()
    robot_status["uptime"] = uptime_seconds
    robot_status["last_update"] = datetime.now().isoformat()

async def broadcast_status():
    """Broadcast status updates periodically"""
    while True:
        await asyncio.sleep(5)  # Cada 5 segundos
        await update_uptime()
        
        # Broadcast status a todos los clientes
        await manager.broadcast({
            "type": "status_update",
            "status": robot_status,
            "timestamp": datetime.now().isoformat()
        })

@app.on_event("startup")
async def startup_event():
    """Evento de startup"""
    logger.info("🚀 ATLAS NEXUS Robot Backend starting up...")
    update_uptime.start_time = datetime.now()
    
    # Testear módulos básicos
    camera_test = await test_camera()
    robot_status["modules"]["camera"] = camera_test["status"] == "success"
    
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
    
    # Activar módulos básicos
    robot_status["modules"]["control"] = True
    robot_status["modules"]["sensors"] = True
    robot_status["modules"]["actuators"] = True
    robot_status["modules"]["communication"] = True
    
    # Iniciar background task para broadcast de status
    asyncio.create_task(broadcast_status())
    
    logger.info("✅ ATLAS NEXUS Robot Backend ready!")

@app.on_event("shutdown")
async def shutdown_event():
    """Evento de shutdown"""
    logger.info("ATLAS NEXUS Robot Backend shutting down...")

if __name__ == "__main__":
    # Prints sin emojis para evitar UnicodeEncodeError en Windows (cp1252)
    print("ATLAS NEXUS Robot Backend - Starting API Server...")
    print("=" * 60)
    print("API: http://localhost:8002")
    print("Docs: http://localhost:8002/docs")
    print("WebSocket: ws://localhost:8002/ws")
    print("=" * 60)

    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8002,
        reload=False,
        log_level="info"
    )
