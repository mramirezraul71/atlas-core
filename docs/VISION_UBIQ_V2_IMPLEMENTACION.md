# VISIÓN UBICUA V2 - GUÍA DE IMPLEMENTACIÓN

**Documento para:** EJECUTOR  
**Fecha:** 2026-02-15  
**Prerequisito:** Leer `VISION_UBIQ_V2_ARQUITECTURA.md`

---

## 🎯 INSTRUCCIONES PARA EJECUTOR

Este documento contiene **pasos exactos** para implementar cada fase. No es necesario tomar decisiones de arquitectura, solo seguir los pasos.

**Formato de cada paso:**
```
✅ [ACCIÓN] - Descripción
   📁 Archivo: ruta/al/archivo.py
   📝 Código: [VER SCAFFOLD en sección correspondiente]
   🧪 Test: comando para verificar
```

---

## 🚀 FASE 1: OPTIMIZACIÓN INMEDIATA

**Objetivo:** Reducir latencia dashboard de 300ms a <80ms  
**Tiempo:** 4-6 horas  
**Riesgo:** MUY BAJO

### PASO 1.1: Crear Frame Broadcaster

✅ **Crear archivo:** `nexus/atlas_nexus_robot/backend/vision/streaming/frame_broadcaster.py`

📝 **Código completo:**

```python
"""
Frame Broadcaster: Patrón Publisher-Subscriber para streaming eficiente.
Un thread captura frames, múltiples clientes se suscriben sin recapturar.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

import cv2
import numpy as np


@dataclass
class FrameData:
    """Frame con metadata."""
    frame: np.ndarray
    timestamp: float
    sequence: int
    camera_index: int


class FrameBroadcaster:
    """
    Broadcaster de frames: 1 captura → N suscriptores.
    
    Uso:
        broadcaster = FrameBroadcaster(camera_index=0, fps=15)
        broadcaster.start()
        
        # En endpoint de streaming:
        for frame_data in broadcaster.subscribe():
            yield encode_mjpeg(frame_data.frame)
        
        broadcaster.stop()
    """
    
    def __init__(
        self,
        camera_index: int = 0,
        fps: int = 15,
        enhance: str = "none",
        jpeg_quality: int = 85,
        focus_x: float = 0.5,
        focus_y: float = 0.5,
        zoom: float = 1.0,
    ):
        self.camera_index = camera_index
        self.fps = fps
        self.enhance = enhance
        self.jpeg_quality = jpeg_quality
        self.focus_x = focus_x
        self.focus_y = focus_y
        self.zoom = zoom
        
        self._lock = threading.Lock()
        self._latest_frame: Optional[FrameData] = None
        self._sequence = 0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._subscribers: list[threading.Event] = []
        self._cap: Optional[cv2.VideoCapture] = None
    
    def start(self) -> bool:
        """Inicia broadcaster en background thread."""
        if self._running:
            return True
        
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        
        # Esperar primer frame (max 2s)
        for _ in range(20):
            if self._latest_frame is not None:
                return True
            time.sleep(0.1)
        return self._latest_frame is not None
    
    def stop(self):
        """Detiene broadcaster y libera cámara."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cap:
            try:
                self._cap.release()
            except Exception:
                pass
    
    def subscribe(self) -> 'FrameIterator':
        """
        Retorna iterador de frames para cliente.
        Cada cliente recibe frames sin recapturar.
        """
        event = threading.Event()
        with self._lock:
            self._subscribers.append(event)
        return FrameIterator(self, event)
    
    def _unsubscribe(self, event: threading.Event):
        """Remueve suscriptor."""
        with self._lock:
            if event in self._subscribers:
                self._subscribers.remove(event)
    
    def _capture_loop(self):
        """Loop principal de captura (background thread)."""
        from vision.cameras.factory import get_camera
        from api.vision_routes import _apply_focus_zoom, _enhance_frame
        
        # Intentar abrir cámara
        try:
            camera = get_camera()
            if camera and hasattr(camera, 'cap'):
                self._cap = camera.cap
            else:
                self._cap = cv2.VideoCapture(self.camera_index)
        except Exception:
            self._cap = cv2.VideoCapture(self.camera_index)
        
        if not self._cap or not self._cap.isOpened():
            self._running = False
            return
        
        frame_time = 1.0 / self.fps
        consecutive_failures = 0
        
        while self._running:
            t0 = time.perf_counter()
            
            try:
                ret, frame = self._cap.read()
                
                if not ret or frame is None:
                    consecutive_failures += 1
                    if consecutive_failures >= 10:
                        # Crear frame de error
                        frame = np.zeros((480, 640, 3), dtype=np.uint8)
                        cv2.putText(
                            frame,
                            f"Camera {self.camera_index} - Reconnecting...",
                            (100, 240),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (255, 255, 255),
                            2,
                        )
                    else:
                        time.sleep(0.1)
                        continue
                else:
                    consecutive_failures = 0
                
                # Aplicar transformaciones
                frame = _apply_focus_zoom(
                    frame,
                    focus_x=self.focus_x,
                    focus_y=self.focus_y,
                    zoom=self.zoom,
                )
                frame = _enhance_frame(frame, self.enhance)
                
                # Publicar frame
                with self._lock:
                    self._sequence += 1
                    self._latest_frame = FrameData(
                        frame=frame.copy(),  # Copy para evitar race conditions
                        timestamp=time.time(),
                        sequence=self._sequence,
                        camera_index=self.camera_index,
                    )
                    
                    # Notificar a todos los suscriptores
                    for event in self._subscribers:
                        event.set()
                
            except Exception as e:
                print(f"Broadcaster error: {e}")
                time.sleep(0.5)
                continue
            
            # Throttle a FPS deseado
            elapsed = time.perf_counter() - t0
            sleep_time = frame_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


class FrameIterator:
    """Iterador de frames para suscriptor."""
    
    def __init__(self, broadcaster: FrameBroadcaster, event: threading.Event):
        self.broadcaster = broadcaster
        self.event = event
        self.last_sequence = -1
    
    def __iter__(self):
        return self
    
    def __next__(self) -> FrameData:
        # Esperar nuevo frame (timeout 5s)
        if not self.event.wait(timeout=5.0):
            self.broadcaster._unsubscribe(self.event)
            raise StopIteration
        
        self.event.clear()
        
        with self.broadcaster._lock:
            frame_data = self.broadcaster._latest_frame
        
        if frame_data is None:
            raise StopIteration
        
        # Skipear frames duplicados
        if frame_data.sequence <= self.last_sequence:
            return self.__next__()
        
        self.last_sequence = frame_data.sequence
        return frame_data
    
    def close(self):
        """Cleanup."""
        self.broadcaster._unsubscribe(self.event)


# Global pool de broadcasters (1 por cámara)
_BROADCASTERS: Dict[int, FrameBroadcaster] = {}
_BROADCASTER_LOCK = threading.Lock()


def get_broadcaster(
    camera_index: int = 0,
    fps: int = 15,
    enhance: str = "none",
    jpeg_quality: int = 85,
    focus_x: float = 0.5,
    focus_y: float = 0.5,
    zoom: float = 1.0,
) -> FrameBroadcaster:
    """
    Obtiene broadcaster existente o crea uno nuevo.
    Broadcaster se comparte entre todos los clientes de la misma cámara.
    """
    with _BROADCASTER_LOCK:
        if camera_index not in _BROADCASTERS:
            broadcaster = FrameBroadcaster(
                camera_index=camera_index,
                fps=fps,
                enhance=enhance,
                jpeg_quality=jpeg_quality,
                focus_x=focus_x,
                focus_y=focus_y,
                zoom=zoom,
            )
            if broadcaster.start():
                _BROADCASTERS[camera_index] = broadcaster
            else:
                raise RuntimeError(f"Failed to start broadcaster for camera {camera_index}")
        
        return _BROADCASTERS[camera_index]


def cleanup_broadcasters():
    """Detiene todos los broadcasters. Llamar al shutdown."""
    with _BROADCASTER_LOCK:
        for broadcaster in _BROADCASTERS.values():
            broadcaster.stop()
        _BROADCASTERS.clear()
```

🧪 **Test básico:**
```bash
cd nexus/atlas_nexus_robot/backend
python -c "
from vision.streaming.frame_broadcaster import get_broadcaster
import time

b = get_broadcaster(camera_index=0, fps=10)
print('Broadcaster started')

# Simular 2 clientes
for i in range(20):
    for frame_data in b.subscribe():
        print(f'Frame {frame_data.sequence}, ts={frame_data.timestamp:.2f}')
        if frame_data.sequence >= 5:
            break
    time.sleep(0.1)

b.stop()
print('OK')
"
```

---

### PASO 1.2: Modificar vision_routes.py

✅ **Backup primero:**
```bash
cp nexus/atlas_nexus_robot/backend/api/vision_routes.py nexus/atlas_nexus_robot/backend/api/vision_routes.py.backup
```

✅ **Modificar:** `nexus/atlas_nexus_robot/backend/api/vision_routes.py`

📝 **Cambios a aplicar:**

**1) Agregar import al inicio del archivo:**
```python
# Después de los imports existentes, agregar:
from vision.streaming.frame_broadcaster import get_broadcaster
```

**2) REEMPLAZAR función `generate_frames()` (línea ~323):**

```python
def generate_frames(
    camera_index: int = 0,
    enhance: str = "auto",
    jpeg_quality: int = 85,
    focus_x: float = 0.5,
    focus_y: float = 0.5,
    zoom: float = 1.0,
):
    """
    Stream de video optimizado con Frame Broadcaster.
    Múltiples clientes comparten 1 captura → latencia <80ms.
    """
    try:
        # Obtener broadcaster compartido (o crear si no existe)
        broadcaster = get_broadcaster(
            camera_index=camera_index,
            fps=15,  # FPS fijo para dashboard
            enhance=enhance,
            jpeg_quality=jpeg_quality,
            focus_x=focus_x,
            focus_y=focus_y,
            zoom=zoom,
        )
        
        # Suscribirse a frames
        for frame_data in broadcaster.subscribe():
            # Frame ya viene procesado (enhance + focus/zoom aplicados)
            frame = frame_data.frame
            
            # Encodear JPEG
            q = int(jpeg_quality) if jpeg_quality is not None else 85
            q = max(30, min(95, q))
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, q])
            
            if not ret:
                continue
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    
    except GeneratorExit:
        logger.info(f"Camera {camera_index}: Stream closed by client")
    except Exception as e:
        logger.error(f"Camera {camera_index}: Stream error: {e}")
```

**3) REEMPLAZAR función `generate_frames_with_detection()` (línea ~461):**

```python
def generate_frames_with_detection(
    camera_index: int = 0,
    enhance: str = "none",
    jpeg_quality: int = 85,
    focus_x: float = 0.5,
    focus_y: float = 0.5,
    zoom: float = 1.0,
):
    """Stream con detecciones YOLO usando broadcaster."""
    detector = get_detector()
    if not detector.is_loaded:
        detector.load_model()
    
    try:
        broadcaster = get_broadcaster(
            camera_index=camera_index,
            fps=10,  # Menor FPS para detección (más CPU)
            enhance=enhance,
            jpeg_quality=jpeg_quality,
            focus_x=focus_x,
            focus_y=focus_y,
            zoom=zoom,
        )
        
        for frame_data in broadcaster.subscribe():
            frame = frame_data.frame
            
            # Detectar objetos
            try:
                detections = detector.detect(frame, confidence=0.5)
                frame_with_detections = detector.draw_detections(frame, detections)
            except Exception as e:
                logger.debug(f"Detection error: {e}")
                frame_with_detections = frame
            
            # Encodear
            q = max(30, min(95, int(jpeg_quality or 85)))
            ret, buffer = cv2.imencode('.jpg', frame_with_detections, [cv2.IMWRITE_JPEG_QUALITY, q])
            
            if not ret:
                continue
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    
    except GeneratorExit:
        logger.info(f"Camera {camera_index} (detection): Stream closed")
    except Exception as e:
        logger.error(f"Camera {camera_index} (detection): Error: {e}")
```

**4) Agregar cleanup al final del archivo:**

```python
# Al final del archivo, antes del último comentario
# Agregar:

import atexit
from vision.streaming.frame_broadcaster import cleanup_broadcasters

# Cleanup al shutdown
atexit.register(cleanup_broadcasters)
```

🧪 **Test:**
```bash
# Reiniciar backend
cd nexus/atlas_nexus_robot/backend
python main.py  # O como esté configurado el startup

# En otro terminal, abrir dashboard
# Abrir 5 tabs del navegador apuntando a dashboard
# Verificar: latencia <100ms, FPS estable 15fps
```

---

### PASO 1.3: Verificación FASE 1

✅ **Checklist:**
- [ ] `frame_broadcaster.py` creado y test pasa
- [ ] `vision_routes.py` modificado (backup guardado)
- [ ] Backend reinicia sin errores
- [ ] Dashboard carga video correctamente
- [ ] Latencia <100ms (medir con DevTools Network)
- [ ] CPU estable con 5 clientes simultáneos

✅ **Si todo pasa → FASE 1 COMPLETA** ✅

---

## 🗄️ FASE 2: UNIFIED REGISTRY

**Objetivo:** Base de datos centralizada para todas las cámaras  
**Tiempo:** 1-2 días  
**Riesgo:** BAJO

### PASO 2.1: Crear Unified Registry

✅ **Crear archivo:** `nexus/atlas_nexus_robot/backend/vision/cameras/unified_registry.py`

📝 **Código completo:** (VER SCAFFOLD AL FINAL DE ESTE DOC - Sección SCAFFOLDS FASE 2)

🧪 **Test:**
```bash
cd nexus/atlas_nexus_robot/backend
python -c "
from vision.cameras.unified_registry import register_camera, list_cameras, health_check

# Registrar cámara de prueba
cam_id = register_camera(
    type='usb',
    url='/dev/video0',
    label='Webcam Test',
    priority=1,
)
print(f'Registered: {cam_id}')

# Listar
cams = list_cameras()
print(f'Total cameras: {len(cams)}')

# Health check
health = health_check(cam_id)
print(f'Health: {health}')
"
```

---

### PASO 2.2: Crear Health Monitor

✅ **Crear archivo:** `nexus/atlas_nexus_robot/backend/vision/cameras/health_monitor.py`

📝 **Código completo:** (VER SCAFFOLD AL FINAL - Sección SCAFFOLDS FASE 2)

---

### PASO 2.3: Integrar con factory.py

✅ **Modificar:** `nexus/atlas_nexus_robot/backend/vision/cameras/factory.py`

📝 **Cambios:**

**1) Agregar import:**
```python
from .unified_registry import register_camera, get_camera_by_id, list_cameras
```

**2) Modificar `get_camera_info()` para consultar registry:**
```python
def get_camera_info(fast: bool = True) -> Dict[str, Any]:
    """Info de cámara activa (prioriza registry)."""
    # Intentar desde registry primero
    try:
        from .unified_registry import list_cameras
        cams = list_cameras(only_online=True)
        if cams:
            # Retornar la de mayor prioridad
            cam = min(cams, key=lambda c: c.get('priority', 999))
            return {
                "active": True,
                "properties": cam.get('metadata', {}),
                "source": "unified_registry",
            }
    except Exception:
        pass
    
    # Fallback a lógica actual
    # ... (mantener código existente)
```

---

### PASO 2.4: Script de Migración

✅ **Crear archivo:** `nexus/atlas_nexus_robot/backend/scripts/migrate_cameras_to_registry.py`

📝 **Código:**
```python
"""
Migra cámaras existentes al unified registry.
Uso: python scripts/migrate_cameras_to_registry.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from vision.cameras.unified_registry import register_camera
from vision.cameras.detector import detect_cameras_list
from vision.cameras.network import get_network_cameras


def migrate():
    print("=== Migrando cámaras a Unified Registry ===\n")
    
    # 1) USB/Local cameras
    print("1) Detectando cámaras USB...")
    usb_cams = detect_cameras_list()
    for cam in usb_cams:
        cam_id = register_camera(
            type="usb",
            url=f"cv2://{cam.get('index', 0)}",
            label=cam.get('model', 'Webcam'),
            priority=1,
            metadata=cam,
        )
        print(f"   ✅ Registrada: {cam_id} - {cam.get('model')}")
    
    # 2) Network cameras
    print("\n2) Migrando cámaras de red...")
    net_cams = get_network_cameras()
    for cam in net_cams:
        cam_id = register_camera(
            type="network",
            url=cam.get('url', ''),
            label=cam.get('model', 'IP Camera'),
            priority=2,
            connection_method=cam.get('source', 'local'),
            metadata=cam,
        )
        print(f"   ✅ Registrada: {cam_id} - {cam.get('model')}")
    
    print("\n=== Migración completada ===")
    print(f"Total: {len(usb_cams) + len(net_cams)} cámaras")


if __name__ == "__main__":
    migrate()
```

🧪 **Test:**
```bash
cd nexus/atlas_nexus_robot/backend
python scripts/migrate_cameras_to_registry.py
# Verificar output
```

---

### PASO 2.5: Verificación FASE 2

✅ **Checklist:**
- [ ] `unified_registry.py` creado y test pasa
- [ ] `health_monitor.py` creado
- [ ] `factory.py` integrado
- [ ] Script migración ejecutado sin errores
- [ ] Registry database creado: `vision/cameras/registry.db`
- [ ] API `/api/camera/service/status` retorna cámaras del registry

✅ **Si todo pasa → FASE 2 COMPLETA** ✅

---

## 🔍 FASE 3: AUTO-DISCOVERY

**Objetivo:** Detectar cámaras automáticamente  
**Tiempo:** 2-3 días  
**Riesgo:** MEDIO

### PASO 3.1: Instalar dependencias

```bash
pip install zeroconf>=0.131.0
# Opcional (para UPnP):
# pip install miniupnpc>=2.2.6
```

### PASO 3.2: Crear Auto-Discovery Engine

✅ **Crear archivo:** `nexus/atlas_nexus_robot/backend/vision/cameras/auto_discovery.py`

📝 **Código completo:** (VER SCAFFOLD AL FINAL - Sección SCAFFOLDS FASE 3)

### PASO 3.3: Crear mDNS Scanner

✅ **Crear archivo:** `nexus/atlas_nexus_robot/backend/vision/cameras/protocols/mdns.py`

📝 **Código completo:** (VER SCAFFOLD AL FINAL - Sección SCAFFOLDS FASE 3)

### PASO 3.4: Script de Discovery

✅ **Crear archivo:** `nexus/atlas_nexus_robot/backend/scripts/discover_cameras.py`

📝 **Código:**
```python
"""
Auto-discovery de todas las cámaras disponibles.
Uso: python scripts/discover_cameras.py
"""
import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))


async def main():
    from vision.cameras.auto_discovery import discover_all_cameras
    
    print("=== Auto-Discovery de Cámaras ===\n")
    print("Escaneando USB, red local (ONVIF, mDNS)...\n")
    
    results = await discover_all_cameras(auto_register=True)
    
    print("\n=== Resultados ===")
    print(f"USB: {results['usb_count']} cámaras")
    print(f"ONVIF: {results['onvif_count']} cámaras")
    print(f"mDNS: {results['mdns_count']} cámaras")
    print(f"Total registradas: {results['registered_count']}")
    
    if results['errors']:
        print("\n⚠️  Errores:")
        for err in results['errors']:
            print(f"   - {err}")


if __name__ == "__main__":
    asyncio.run(main())
```

🧪 **Test:**
```bash
cd nexus/atlas_nexus_robot/backend
python scripts/discover_cameras.py
# Verificar que detecta tus cámaras
```

---

### PASO 3.5: Integrar con API

✅ **Modificar:** `nexus/atlas_nexus_robot/backend/api/camera_service_routes.py`

📝 **Agregar nuevo endpoint:**

```python
@router.post("/discover-all")
async def discover_all_cameras_endpoint():
    """
    Auto-discovery de TODAS las cámaras (USB + red).
    Registra automáticamente en unified registry.
    """
    try:
        from vision.cameras.auto_discovery import discover_all_cameras
        results = await discover_all_cameras(auto_register=True)
        return {
            "ok": True,
            "results": results,
            "message": f"Detectadas {results['registered_count']} cámaras",
        }
    except Exception as e:
        return {"ok": False, "error": str(e)}
```

---

### PASO 3.6: Verificación FASE 3

✅ **Checklist:**
- [ ] Dependencias instaladas
- [ ] `auto_discovery.py` creado
- [ ] `protocols/mdns.py` creado
- [ ] Script discovery ejecuta sin errores
- [ ] API `/api/camera/discover-all` funciona
- [ ] Cámaras USB detectadas: 100%
- [ ] Cámaras IP (si tienes) detectadas

✅ **Si todo pasa → FASE 3 COMPLETA** ✅

---

## 🚗 FASE 4: AUTO DASHCAM (OPCIONAL)

**Nota:** Esta fase es opcional y depende de hardware dashcam.  
**Saltar si no tienes dashcam disponible.**

### PASO 4.1: Crear Dashcam Discovery

✅ **Crear archivo:** `nexus/atlas_nexus_robot/backend/vision/auto_dashcam/discovery.py`

📝 **Código completo:** (VER SCAFFOLD AL FINAL - Sección SCAFFOLDS FASE 4)

### PASO 4.2: VPN Setup (Tailscale)

✅ **Crear archivo:** `nexus/atlas_nexus_robot/backend/vision/auto_dashcam/vpn_setup.py`

📝 **Código completo:** (VER SCAFFOLD AL FINAL - Sección SCAFFOLDS FASE 4)

---

## 📱 FASE 5: ADAPTIVE STREAMING (FUTURO)

**Nota:** Esta fase se implementará después de validar FASES 1-3 en producción.

Scaffold disponible en sección SCAFFOLDS FASE 5.

---

## 📚 SCAFFOLDS (CÓDIGO COMPLETO)

### SCAFFOLDS FASE 2

**(Continúa en siguiente mensaje debido a límite de longitud)**

---

## ✅ VERIFICACIÓN FINAL

Antes de marcar como completo:

```bash
# 1) Todos los tests unitarios
pytest tests/vision/ -v

# 2) Test de integración E2E
python scripts/test_vision_e2e.py

# 3) Dashboard funciona
# Abrir http://localhost:8000/static/dashboard.html
# Verificar: video fluido, <100ms latency

# 4) Cámaras detectadas
python scripts/discover_cameras.py
# Verificar: todas las cámaras encontradas
```

---

## 🆘 TROUBLESHOOTING

### Error: "ModuleNotFoundError: No module named 'vision.streaming'"

**Solución:**
```bash
# Crear __init__.py faltante
touch nexus/atlas_nexus_robot/backend/vision/streaming/__init__.py
```

### Error: "Database is locked"

**Solución:**
```bash
# Cerrar todas las conexiones
pkill -f "atlas_nexus_robot"
rm vision/cameras/registry.db-journal  # Si existe
```

### Error: "Broadcaster not starting"

**Solución:**
```bash
# Verificar cámara disponible
python -c "import cv2; cap = cv2.VideoCapture(0); print(cap.isOpened())"
# Si False, verificar drivers/permisos
```

---

**EJECUTOR: Proceder con FASE 1. Reportar al completar cada fase.**

*Documento generado por Arquitecto Autónomo ATLAS*
