# VISION UBIQ V2 - SCAFFOLDS DE CÓDIGO

**Para:** EJECUTOR  
**Fecha:** 2026-02-15

Este documento contiene los scaffolds (plantillas) de código completo para las FASES 2, 3, 4 y 5.

---

## 📋 SCAFFOLD FASE 2: Unified Registry

### `vision/cameras/unified_registry.py`

```python
"""
Unified Camera Registry - SQLite-based central registry for all camera sources.
"""
from __future__ import annotations

import json
import sqlite3
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2


def _repo_root() -> Path:
    """Retorna raíz del repositorio."""
    return Path(__file__).resolve().parents[5]


def _db_path() -> Path:
    """Path a la base de datos del registry."""
    return Path(__file__).parent / "registry.db"


_DB_LOCK = threading.Lock()


def _get_connection() -> sqlite3.Connection:
    """Obtiene conexión a la DB (thread-safe)."""
    conn = sqlite3.connect(_db_path(), check_same_thread=False)
    conn.row_factory = sqlite3.Row
    return conn


def _init_db():
    """Inicializa schema de la DB si no existe."""
    with _DB_LOCK:
        conn = _get_connection()
        cursor = conn.cursor()
        
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS cameras (
                id TEXT PRIMARY KEY,
                type TEXT NOT NULL,
                url TEXT,
                label TEXT,
                health TEXT DEFAULT 'unknown',
                priority INTEGER DEFAULT 99,
                connection_method TEXT,
                metadata TEXT,
                last_seen REAL,
                created_at REAL NOT NULL
            )
        """)
        
        # Índices para búsquedas rápidas
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_type ON cameras(type)")
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_health ON cameras(health)")
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_priority ON cameras(priority)")
        
        conn.commit()
        conn.close()


# Inicializar DB al importar
_init_db()


def register_camera(
    type: str,
    url: str,
    label: str,
    priority: int = 99,
    connection_method: Optional[str] = None,
    metadata: Optional[Dict[str, Any]] = None,
) -> str:
    """
    Registra una cámara en el registry.
    
    Args:
        type: usb|network|remote|mobile|auto_dashcam
        url: URL o path (e.g., "cv2://0", "rtsp://...", "/dev/video0")
        label: Nombre amigable
        priority: 1=primary, 2=secondary, etc. (menor = mayor prioridad)
        connection_method: direct|tailscale|zerotier|port_forward|cloud|manual
        metadata: Datos adicionales (resolution, fps, model, etc.)
    
    Returns:
        Camera ID (UUID)
    """
    cam_id = str(uuid.uuid4())
    now = time.time()
    
    with _DB_LOCK:
        conn = _get_connection()
        cursor = conn.cursor()
        
        cursor.execute(
            """
            INSERT INTO cameras (id, type, url, label, priority, connection_method, metadata, created_at, last_seen)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                cam_id,
                type,
                url,
                label,
                priority,
                connection_method,
                json.dumps(metadata or {}),
                now,
                now,
            ),
        )
        
        conn.commit()
        conn.close()
    
    return cam_id


def list_cameras(
    filter_type: Optional[str] = None,
    only_online: bool = False,
) -> List[Dict[str, Any]]:
    """
    Lista todas las cámaras registradas.
    
    Args:
        filter_type: Filtrar por tipo (usb, network, etc.)
        only_online: Solo cámaras con health='online'
    
    Returns:
        Lista de cámaras (ordenadas por priority ASC)
    """
    with _DB_LOCK:
        conn = _get_connection()
        cursor = conn.cursor()
        
        query = "SELECT * FROM cameras WHERE 1=1"
        params = []
        
        if filter_type:
            query += " AND type = ?"
            params.append(filter_type)
        
        if only_online:
            query += " AND health = 'online'"
        
        query += " ORDER BY priority ASC, created_at ASC"
        
        cursor.execute(query, params)
        rows = cursor.fetchall()
        conn.close()
    
    cameras = []
    for row in rows:
        cam = dict(row)
        # Parse metadata JSON
        try:
            cam['metadata'] = json.loads(cam['metadata'] or '{}')
        except Exception:
            cam['metadata'] = {}
        cameras.append(cam)
    
    return cameras


def get_camera(cam_id: str) -> Optional[Dict[str, Any]]:
    """Obtiene cámara por ID."""
    with _DB_LOCK:
        conn = _get_connection()
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM cameras WHERE id = ?", (cam_id,))
        row = cursor.fetchone()
        conn.close()
    
    if not row:
        return None
    
    cam = dict(row)
    try:
        cam['metadata'] = json.loads(cam['metadata'] or '{}')
    except Exception:
        cam['metadata'] = {}
    
    return cam


def update_camera(cam_id: str, **updates) -> bool:
    """
    Actualiza campos de una cámara.
    
    Args:
        cam_id: Camera ID
        **updates: Campos a actualizar (label, health, priority, metadata, etc.)
    
    Returns:
        True si actualizó, False si ID no existe
    """
    if not updates:
        return False
    
    # Serializar metadata si está presente
    if 'metadata' in updates and isinstance(updates['metadata'], dict):
        updates['metadata'] = json.dumps(updates['metadata'])
    
    set_clause = ", ".join(f"{k} = ?" for k in updates.keys())
    values = list(updates.values()) + [cam_id]
    
    with _DB_LOCK:
        conn = _get_connection()
        cursor = conn.cursor()
        cursor.execute(f"UPDATE cameras SET {set_clause} WHERE id = ?", values)
        affected = cursor.rowcount
        conn.commit()
        conn.close()
    
    return affected > 0


def delete_camera(cam_id: str) -> bool:
    """Elimina cámara del registry."""
    with _DB_LOCK:
        conn = _get_connection()
        cursor = conn.cursor()
        cursor.execute("DELETE FROM cameras WHERE id = ?", (cam_id,))
        affected = cursor.rowcount
        conn.commit()
        conn.close()
    
    return affected > 0


def health_check(cam_id: str, timeout_s: float = 2.0) -> Dict[str, Any]:
    """
    Realiza health check de una cámara.
    
    Returns:
        {ok: bool, health: "online"|"offline"|"degraded", latency_ms: float, error: str}
    """
    cam = get_camera(cam_id)
    if not cam:
        return {"ok": False, "health": "offline", "error": "camera_not_found"}
    
    url = cam.get('url', '')
    cam_type = cam.get('type', '')
    
    t0 = time.perf_counter()
    
    try:
        # USB/Local camera
        if cam_type == 'usb' or url.startswith('cv2://'):
            index = 0
            if url.startswith('cv2://'):
                try:
                    index = int(url.split('://')[1])
                except Exception:
                    pass
            
            cap = cv2.VideoCapture(index)
            if not cap.isOpened():
                cap.release()
                return {"ok": False, "health": "offline", "error": "cannot_open"}
            
            ret, frame = cap.read()
            cap.release()
            
            latency_ms = (time.perf_counter() - t0) * 1000
            
            if not ret or frame is None:
                return {"ok": False, "health": "degraded", "latency_ms": latency_ms, "error": "no_frame"}
            
            # Actualizar health en DB
            update_camera(cam_id, health='online', last_seen=time.time())
            
            return {"ok": True, "health": "online", "latency_ms": latency_ms}
        
        # Network/Remote camera (RTSP, HTTP)
        elif url.startswith(('rtsp://', 'http://', 'https://')):
            cap = cv2.VideoCapture(url)
            if not cap.isOpened():
                cap.release()
                return {"ok": False, "health": "offline", "error": "cannot_connect"}
            
            ret, frame = cap.read()
            cap.release()
            
            latency_ms = (time.perf_counter() - t0) * 1000
            
            if not ret or frame is None:
                return {"ok": False, "health": "degraded", "latency_ms": latency_ms, "error": "no_frame"}
            
            update_camera(cam_id, health='online', last_seen=time.time())
            return {"ok": True, "health": "online", "latency_ms": latency_ms}
        
        else:
            return {"ok": False, "health": "unknown", "error": "unsupported_url_scheme"}
    
    except Exception as e:
        update_camera(cam_id, health='offline', last_seen=time.time())
        return {"ok": False, "health": "offline", "error": str(e)}


def set_active_eye(cam_id: str, duration_s: Optional[float] = None) -> Dict[str, Any]:
    """
    Marca una cámara como "ojo activo" (prioridad temporal máxima).
    Integración con modules/humanoid/nerve/eyes.py.
    
    Args:
        cam_id: Camera ID
        duration_s: Duración en segundos (None = indefinido)
    
    Returns:
        {ok: bool, message: str}
    """
    cam = get_camera(cam_id)
    if not cam:
        return {"ok": False, "message": "Camera not found"}
    
    # Persistir en settings (compatible con eyes.py)
    try:
        from modules.humanoid.vision.ubiq.registry import set_setting
        set_setting("vision.active_eye", f"ubiq:{cam_id}")
        
        if duration_s:
            until = time.time() + duration_s
            set_setting("vision.active_eye_until", str(until))
        else:
            set_setting("vision.active_eye_until", "")
        
        set_setting("vision.active_eye_failures", "0")
    except Exception as e:
        return {"ok": False, "message": f"Failed to persist setting: {e}"}
    
    return {"ok": True, "message": f"Active eye set to {cam.get('label', cam_id)}"}


def clear_active_eye() -> Dict[str, Any]:
    """Limpia el ojo activo (vuelve a cámara por defecto)."""
    try:
        from modules.humanoid.vision.ubiq.registry import set_setting
        set_setting("vision.active_eye", "")
        set_setting("vision.active_eye_until", "")
        set_setting("vision.active_eye_failures", "0")
        return {"ok": True, "message": "Active eye cleared"}
    except Exception as e:
        return {"ok": False, "message": str(e)}
```

---

### `vision/cameras/health_monitor.py`

```python
"""
Health Monitor - Background service que verifica health de cámaras periódicamente.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Optional

from .unified_registry import list_cameras, health_check

logger = logging.getLogger(__name__)


class HealthMonitor:
    """
    Servicio background que hace ping a cámaras cada N segundos.
    Actualiza health status en registry automáticamente.
    """
    
    def __init__(self, interval_s: int = 30):
        """
        Args:
            interval_s: Intervalo entre health checks (segundos)
        """
        self.interval_s = interval_s
        self._running = False
        self._thread: Optional[threading.Thread] = None
    
    def start(self):
        """Inicia monitor en background."""
        if self._running:
            logger.warning("Health monitor already running")
            return
        
        self._running = True
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._thread.start()
        logger.info(f"Health monitor started (interval={self.interval_s}s)")
    
    def stop(self):
        """Detiene monitor."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=5.0)
        logger.info("Health monitor stopped")
    
    def _monitor_loop(self):
        """Loop principal (background thread)."""
        while self._running:
            try:
                cameras = list_cameras()
                logger.debug(f"Health check: {len(cameras)} cameras")
                
                for cam in cameras:
                    cam_id = cam['id']
                    label = cam.get('label', cam_id)
                    
                    try:
                        result = health_check(cam_id, timeout_s=3.0)
                        health = result.get('health', 'unknown')
                        
                        if not result.get('ok'):
                            logger.warning(f"Camera {label} ({cam_id}): {health} - {result.get('error')}")
                        else:
                            logger.debug(f"Camera {label}: {health} ({result.get('latency_ms', 0):.0f}ms)")
                    
                    except Exception as e:
                        logger.error(f"Health check failed for {label}: {e}")
                
                # Esperar intervalo
                time.sleep(self.interval_s)
            
            except Exception as e:
                logger.error(f"Health monitor error: {e}")
                time.sleep(5.0)  # Retry después de error


# Global singleton
_monitor: Optional[HealthMonitor] = None
_monitor_lock = threading.Lock()


def start_health_monitor(interval_s: int = 30):
    """Inicia health monitor global."""
    global _monitor
    with _monitor_lock:
        if _monitor is None:
            _monitor = HealthMonitor(interval_s=interval_s)
        _monitor.start()


def stop_health_monitor():
    """Detiene health monitor global."""
    global _monitor
    with _monitor_lock:
        if _monitor:
            _monitor.stop()


# Auto-start al importar (opcional, comentar si no deseas auto-start)
# start_health_monitor()
```

---

## 📋 SCAFFOLD FASE 3: Auto-Discovery

### `vision/cameras/auto_discovery.py`

```python
"""
Auto-Discovery Engine - Detecta cámaras USB, red local (ONVIF, mDNS), y remotas.
"""
from __future__ import annotations

import asyncio
import logging
from typing import Any, Dict, List

from .unified_registry import register_camera

logger = logging.getLogger(__name__)


async def discover_usb_cameras() -> List[Dict[str, Any]]:
    """Detecta cámaras USB usando OpenCV probe."""
    try:
        from .detector import detect_cameras_list
        cameras = detect_cameras_list()
        logger.info(f"USB discovery: {len(cameras)} cameras found")
        return cameras
    except Exception as e:
        logger.error(f"USB discovery error: {e}")
        return []


async def discover_onvif_cameras() -> List[Dict[str, Any]]:
    """Detecta cámaras ONVIF usando WS-Discovery."""
    try:
        from modules.humanoid.vision.ubiq.onvif_wsdiscovery import discover_onvif_cameras as discover_sync
        # Ejecutar sync function en executor
        loop = asyncio.get_event_loop()
        cameras = await loop.run_in_executor(None, discover_sync)
        logger.info(f"ONVIF discovery: {len(cameras)} cameras found")
        return cameras
    except Exception as e:
        logger.error(f"ONVIF discovery error: {e}")
        return []


async def discover_mdns_cameras() -> List[Dict[str, Any]]:
    """Detecta cámaras IP usando mDNS/Bonjour."""
    try:
        from .protocols.mdns import scan_mdns_cameras
        cameras = await scan_mdns_cameras(timeout_s=5.0)
        logger.info(f"mDNS discovery: {len(cameras)} cameras found")
        return cameras
    except Exception as e:
        logger.error(f"mDNS discovery error: {e}")
        return []


async def discover_all_cameras(auto_register: bool = True) -> Dict[str, Any]:
    """
    Ejecuta todos los discovery methods en paralelo.
    
    Args:
        auto_register: Si True, registra automáticamente cámaras en registry
    
    Returns:
        {
            usb_cameras: [...],
            onvif_cameras: [...],
            mdns_cameras: [...],
            usb_count: int,
            onvif_count: int,
            mdns_count: int,
            registered_count: int,
            errors: [...]
        }
    """
    logger.info("Starting auto-discovery (USB, ONVIF, mDNS)...")
    
    # Ejecutar en paralelo
    results = await asyncio.gather(
        discover_usb_cameras(),
        discover_onvif_cameras(),
        discover_mdns_cameras(),
        return_exceptions=True,
    )
    
    usb_cameras = results[0] if not isinstance(results[0], Exception) else []
    onvif_cameras = results[1] if not isinstance(results[1], Exception) else []
    mdns_cameras = results[2] if not isinstance(results[2], Exception) else []
    
    errors = []
    for i, r in enumerate(results):
        if isinstance(r, Exception):
            errors.append(str(r))
    
    registered_count = 0
    
    if auto_register:
        # Registrar USB cameras
        for cam in usb_cameras:
            try:
                cam_id = register_camera(
                    type='usb',
                    url=f"cv2://{cam.get('index', 0)}",
                    label=cam.get('model', 'USB Camera'),
                    priority=1,
                    metadata=cam,
                )
                registered_count += 1
                logger.info(f"Registered USB camera: {cam_id}")
            except Exception as e:
                logger.error(f"Failed to register USB camera: {e}")
                errors.append(f"USB register error: {e}")
        
        # Registrar ONVIF cameras
        for cam in onvif_cameras:
            try:
                cam_id = register_camera(
                    type='network',
                    url=cam.get('url', ''),
                    label=cam.get('model', 'ONVIF Camera'),
                    priority=2,
                    connection_method='direct',
                    metadata=cam,
                )
                registered_count += 1
                logger.info(f"Registered ONVIF camera: {cam_id}")
            except Exception as e:
                logger.error(f"Failed to register ONVIF camera: {e}")
                errors.append(f"ONVIF register error: {e}")
        
        # Registrar mDNS cameras
        for cam in mdns_cameras:
            try:
                cam_id = register_camera(
                    type='network',
                    url=cam.get('url', ''),
                    label=cam.get('model', 'IP Camera'),
                    priority=2,
                    connection_method='direct',
                    metadata=cam,
                )
                registered_count += 1
                logger.info(f"Registered mDNS camera: {cam_id}")
            except Exception as e:
                logger.error(f"Failed to register mDNS camera: {e}")
                errors.append(f"mDNS register error: {e}")
    
    return {
        "usb_cameras": usb_cameras,
        "onvif_cameras": onvif_cameras,
        "mdns_cameras": mdns_cameras,
        "usb_count": len(usb_cameras),
        "onvif_count": len(onvif_cameras),
        "mdns_count": len(mdns_cameras),
        "registered_count": registered_count,
        "errors": errors,
    }
```

---

### `vision/cameras/protocols/mdns.py`

```python
"""
mDNS/Bonjour Scanner - Detecta cámaras IP que anuncian servicios mDNS.
"""
from __future__ import annotations

import asyncio
import logging
from typing import Any, Dict, List

logger = logging.getLogger(__name__)


async def scan_mdns_cameras(timeout_s: float = 5.0) -> List[Dict[str, Any]]:
    """
    Escanea red local en busca de cámaras IP usando mDNS/Bonjour.
    
    Servicios típicos:
    - _http._tcp (HTTP MJPEG cameras)
    - _rtsp._tcp (RTSP cameras)
    - _ipp._tcp (algunas webcams)
    
    Returns:
        Lista de cámaras encontradas
    """
    try:
        from zeroconf import ServiceBrowser, ServiceListener, Zeroconf
    except ImportError:
        logger.warning("zeroconf no instalado, skip mDNS discovery")
        return []
    
    cameras = []
    
    class CameraListener(ServiceListener):
        def add_service(self, zc: Zeroconf, type_: str, name: str) -> None:
            try:
                info = zc.get_service_info(type_, name)
                if info:
                    # Parsear info
                    addresses = [addr for addr in info.parsed_addresses()]
                    port = info.port
                    
                    if not addresses or not port:
                        return
                    
                    # Determinar protocolo
                    protocol = 'http'
                    if '_rtsp' in type_:
                        protocol = 'rtsp'
                    
                    url = f"{protocol}://{addresses[0]}:{port}/"
                    
                    # Heurística: si el nombre contiene "camera", "webcam", "ipcam", etc.
                    name_lower = name.lower()
                    is_camera = any(keyword in name_lower for keyword in [
                        'camera', 'webcam', 'ipcam', 'cam', 'video', 'surveillance'
                    ])
                    
                    if is_camera:
                        cameras.append({
                            'url': url,
                            'ip': addresses[0],
                            'port': port,
                            'model': name,
                            'protocol': protocol,
                            'source': 'mdns',
                        })
                        logger.info(f"mDNS: Found camera {name} at {url}")
            
            except Exception as e:
                logger.debug(f"mDNS service parse error: {e}")
        
        def update_service(self, *args, **kwargs):
            pass
        
        def remove_service(self, *args, **kwargs):
            pass
    
    zc = Zeroconf()
    listener = CameraListener()
    
    # Escanear servicios relevantes
    services = [
        "_http._tcp.local.",
        "_rtsp._tcp.local.",
        "_ipp._tcp.local.",
    ]
    
    browsers = [ServiceBrowser(zc, service, listener) for service in services]
    
    # Esperar timeout
    await asyncio.sleep(timeout_s)
    
    # Cleanup
    for browser in browsers:
        try:
            browser.cancel()
        except Exception:
            pass
    
    try:
        zc.close()
    except Exception:
        pass
    
    return cameras
```

---

## 📋 SCAFFOLD FASE 4: Dashcam (OPCIONAL)

### `vision/auto_dashcam/discovery.py`

```python
"""
Dashcam Discovery - Detecta dashcams en WiFi local.
"""
from __future__ import annotations

import logging
import subprocess
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

# SSIDs típicos de dashcams populares
DASHCAM_SSID_PATTERNS = [
    "DDPAI_",
    "VIOFO_",
    "70mai_",
    "Xiaomi_",
    "YI_",
    "THINKWARE_",
    "BlackVue_",
]


def get_available_wifi_networks() -> List[str]:
    """
    Obtiene lista de SSIDs disponibles (Windows/Linux).
    
    Returns:
        Lista de SSIDs detectados
    """
    import platform
    
    try:
        if platform.system() == "Windows":
            result = subprocess.run(
                ["netsh", "wlan", "show", "networks"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if result.returncode == 0:
                lines = result.stdout.split('\n')
                ssids = []
                for line in lines:
                    if "SSID" in line and ":" in line:
                        ssid = line.split(':', 1)[1].strip()
                        if ssid:
                            ssids.append(ssid)
                return ssids
        
        elif platform.system() == "Linux":
            result = subprocess.run(
                ["nmcli", "-t", "-f", "SSID", "device", "wifi", "list"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if result.returncode == 0:
                ssids = [s.strip() for s in result.stdout.split('\n') if s.strip()]
                return ssids
    
    except Exception as e:
        logger.error(f"Failed to scan WiFi networks: {e}")
    
    return []


def detect_dashcam_ssids() -> List[str]:
    """
    Detecta SSIDs que coinciden con patrones de dashcams.
    
    Returns:
        Lista de SSIDs que son probablemente dashcams
    """
    all_ssids = get_available_wifi_networks()
    dashcam_ssids = []
    
    for ssid in all_ssids:
        for pattern in DASHCAM_SSID_PATTERNS:
            if ssid.startswith(pattern):
                dashcam_ssids.append(ssid)
                logger.info(f"Detected dashcam SSID: {ssid}")
                break
    
    return dashcam_ssids


def probe_dashcam_rtsp(ip: str, port: int = 8554, user: str = "admin", password: str = "12345") -> Optional[str]:
    """
    Prueba conexión RTSP a dashcam.
    
    Args:
        ip: IP de la dashcam
        port: Puerto RTSP (8554 típico para DDPAI, 554 para VIOFO)
        user: Usuario RTSP
        password: Password RTSP
    
    Returns:
        URL RTSP si funciona, None si falla
    """
    import cv2
    
    urls_to_try = [
        f"rtsp://{user}:{password}@{ip}:{port}/stream1",
        f"rtsp://{user}:{password}@{ip}:{port}/live",
        f"rtsp://{user}:{password}@{ip}:{port}/",
    ]
    
    for url in urls_to_try:
        try:
            cap = cv2.VideoCapture(url)
            if cap.isOpened():
                ret, frame = cap.read()
                cap.release()
                if ret and frame is not None:
                    logger.info(f"Dashcam RTSP working: {url}")
                    return url
        except Exception as e:
            logger.debug(f"RTSP probe failed for {url}: {e}")
    
    return None


def discover_dashcams() -> List[Dict[str, Any]]:
    """
    Auto-discovery completo de dashcams en WiFi local.
    
    Returns:
        Lista de dashcams detectadas con URLs funcionales
    """
    logger.info("Starting dashcam discovery...")
    
    # 1) Detectar SSIDs
    ssids = detect_dashcam_ssids()
    if not ssids:
        logger.info("No dashcam SSIDs found")
        return []
    
    logger.info(f"Found {len(ssids)} potential dashcam SSIDs: {ssids}")
    
    # 2) Probar IPs típicas
    # La mayoría de dashcams usan 192.168.1.1 o 192.168.0.1 como IP en su hotspot
    typical_ips = ["192.168.1.1", "192.168.0.1", "192.168.43.1"]
    
    dashcams = []
    
    for ip in typical_ips:
        # Probar RTSP en puertos comunes
        for port in [8554, 554]:
            url = probe_dashcam_rtsp(ip, port=port)
            if url:
                dashcams.append({
                    'url': url,
                    'ip': ip,
                    'port': port,
                    'model': 'Dashcam (RTSP)',
                    'type': 'auto_dashcam',
                    'source': 'wifi_direct',
                })
                break  # No probar más puertos para esta IP
    
    logger.info(f"Dashcam discovery complete: {len(dashcams)} cameras")
    return dashcams
```

---

### `vision/auto_dashcam/vpn_setup.py`

```python
"""
VPN Setup - Auto-configuración de Tailscale para dashcam remota.
"""
from __future__ import annotations

import logging
import subprocess
from typing import Dict, Any

logger = logging.getLogger(__name__)


def is_tailscale_installed() -> bool:
    """Verifica si Tailscale está instalado."""
    try:
        result = subprocess.run(
            ["tailscale", "version"],
            capture_output=True,
            timeout=3,
        )
        return result.returncode == 0
    except FileNotFoundError:
        return False
    except Exception:
        return False


def tailscale_status() -> Dict[str, Any]:
    """
    Obtiene estado de Tailscale.
    
    Returns:
        {ok: bool, online: bool, ip: str, hostname: str}
    """
    if not is_tailscale_installed():
        return {"ok": False, "error": "tailscale_not_installed"}
    
    try:
        result = subprocess.run(
            ["tailscale", "status", "--json"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        
        if result.returncode != 0:
            return {"ok": False, "error": "tailscale_status_failed"}
        
        import json
        status = json.loads(result.stdout)
        
        return {
            "ok": True,
            "online": status.get("BackendState") == "Running",
            "ip": status.get("Self", {}).get("TailscaleIPs", [""])[0],
            "hostname": status.get("Self", {}).get("HostName", ""),
        }
    
    except Exception as e:
        return {"ok": False, "error": str(e)}


def tailscale_login() -> Dict[str, Any]:
    """
    Inicia proceso de login en Tailscale (abre browser).
    
    Returns:
        {ok: bool, message: str}
    """
    if not is_tailscale_installed():
        return {"ok": False, "message": "Tailscale no instalado. Instalar desde https://tailscale.com/download"}
    
    try:
        subprocess.run(["tailscale", "up"], timeout=10)
        return {"ok": True, "message": "Tailscale login iniciado. Seguir instrucciones en browser."}
    except Exception as e:
        return {"ok": False, "message": f"Error: {e}"}


def get_tailscale_peers() -> Dict[str, Any]:
    """
    Lista peers conectados a la red Tailscale.
    
    Returns:
        {ok: bool, peers: [{hostname, ip, online}]}
    """
    if not is_tailscale_installed():
        return {"ok": False, "peers": []}
    
    try:
        result = subprocess.run(
            ["tailscale", "status", "--json"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        
        if result.returncode != 0:
            return {"ok": False, "peers": []}
        
        import json
        status = json.loads(result.stdout)
        
        peers = []
        for peer_id, peer_data in status.get("Peer", {}).items():
            peers.append({
                "hostname": peer_data.get("HostName", ""),
                "ip": peer_data.get("TailscaleIPs", [""])[0],
                "online": peer_data.get("Online", False),
            })
        
        return {"ok": True, "peers": peers}
    
    except Exception as e:
        return {"ok": False, "peers": [], "error": str(e)}
```

---

## ✅ INSTRUCCIONES FINALES PARA EJECUTOR

1. **Leer completo** `VISION_UBIQ_V2_ARQUITECTURA.md`
2. **Leer completo** `VISION_UBIQ_V2_IMPLEMENTACION.md`
3. **Ejecutar** `python scripts/verify_vision_setup.py`
4. **Implementar FASE 1** siguiendo pasos exactos
5. **Reportar** al completar cada fase

**Los scaffolds en este documento son para FASES 2, 3, 4. Copiar código literalmente.**

*Fin de scaffolds*
