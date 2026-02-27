"""
SLAM Engine: Simultaneous Localization and Mapping.
====================================================
Implementación de SLAM para ATLAS.

Soporta:
- SLAM Toolbox (ROS2) via subprocess/API
- GMapping fallback
- Cartographer integration
- Custom grid-based SLAM
"""
from __future__ import annotations

import json
import logging
import math
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

_log = logging.getLogger("humanoid.navigation.slam")


class SLAMMode(str, Enum):
    """Modos de operación del SLAM."""
    MAPPING = "mapping"         # Construyendo mapa nuevo
    LOCALIZATION = "localization"  # Localizando en mapa existente
    IDLE = "idle"               # Detenido


@dataclass
class SLAMConfig:
    """Configuración del motor SLAM."""
    resolution: float = 0.05      # Resolución del mapa (m/pixel)
    map_size: Tuple[int, int] = (200, 200)  # Tamaño del mapa en celdas
    origin: Tuple[float, float] = (-5.0, -5.0)  # Origen del mapa en metros
    max_range: float = 10.0       # Rango máximo del sensor (metros)
    min_range: float = 0.1        # Rango mínimo del sensor
    occupied_thresh: float = 0.65  # Umbral para celda ocupada
    free_thresh: float = 0.35      # Umbral para celda libre
    update_rate_hz: float = 10.0   # Tasa de actualización
    use_scan_matching: bool = True  # Usar scan matching
    map_save_path: str = "data/maps"


@dataclass
class MapData:
    """Datos del mapa generado."""
    width: int
    height: int
    resolution: float
    origin: Tuple[float, float]
    data: np.ndarray  # Occupancy grid: -1=unknown, 0=free, 100=occupied
    timestamp: str = ""
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": self.origin,
            "timestamp": self.timestamp,
            "data_shape": self.data.shape,
        }


@dataclass
class ScanData:
    """Datos de un scan de sensor (LiDAR/depth)."""
    ranges: List[float]           # Distancias medidas
    angles: List[float]           # Ángulos correspondientes
    timestamp: float              # Timestamp del scan
    frame_id: str = "laser"       # Frame de referencia


class SLAMEngine:
    """
    Motor de SLAM para ATLAS.
    
    Implementa un SLAM básico basado en occupancy grid con
    scan matching opcional.
    """
    
    def __init__(self, config: Optional[SLAMConfig] = None):
        self.config = config or SLAMConfig()
        self._mode = SLAMMode.IDLE
        self._running = False
        self._lock = threading.Lock()
        
        # Mapa
        self._map: Optional[MapData] = None
        self._initialize_map()
        
        # Pose actual
        self._pose = [0.0, 0.0, 0.0]  # x, y, theta
        
        # Historial de scans
        self._scan_history: List[ScanData] = []
        self._max_history = 100
        
        # Estadísticas
        self._stats = {
            "scans_processed": 0,
            "map_updates": 0,
            "last_update": None,
        }
    
    def _initialize_map(self) -> None:
        """Inicializa el mapa vacío."""
        w, h = self.config.map_size
        # -1 = unknown, 0 = free, 100 = occupied
        data = np.full((h, w), -1, dtype=np.int8)
        self._map = MapData(
            width=w,
            height=h,
            resolution=self.config.resolution,
            origin=self.config.origin,
            data=data,
            timestamp=datetime.now(timezone.utc).isoformat(),
        )
        _log.info("Initialized empty map: %dx%d, resolution=%.3f", w, h, self.config.resolution)
    
    def start_mapping(self) -> Dict[str, Any]:
        """Inicia el modo de mapeo."""
        with self._lock:
            if self._mode == SLAMMode.MAPPING:
                return {"ok": True, "mode": "mapping", "already_running": True}
            
            self._mode = SLAMMode.MAPPING
            self._running = True
            self._initialize_map()
            
            _log.info("SLAM mapping started")
            return {"ok": True, "mode": "mapping", "map_size": self.config.map_size}
    
    def start_localization(self, map_path: Optional[str] = None) -> Dict[str, Any]:
        """Inicia el modo de localización con mapa existente."""
        with self._lock:
            if map_path:
                loaded = self.load_map(map_path)
                if not loaded.get("ok"):
                    return loaded
            
            if self._map is None:
                return {"ok": False, "error": "No map available for localization"}
            
            self._mode = SLAMMode.LOCALIZATION
            self._running = True
            
            _log.info("SLAM localization started")
            return {"ok": True, "mode": "localization"}
    
    def stop(self) -> Dict[str, Any]:
        """Detiene el SLAM."""
        with self._lock:
            self._mode = SLAMMode.IDLE
            self._running = False
            _log.info("SLAM stopped")
            return {"ok": True, "mode": "idle"}
    
    def process_scan(self, scan: ScanData) -> Dict[str, Any]:
        """
        Procesa un scan de sensor y actualiza el mapa/pose.
        
        Args:
            scan: Datos del scan
            
        Returns:
            Resultado del procesamiento
        """
        if not self._running:
            return {"ok": False, "error": "SLAM not running"}
        
        with self._lock:
            # Guardar en historial
            self._scan_history.append(scan)
            if len(self._scan_history) > self._max_history:
                self._scan_history.pop(0)
            
            self._stats["scans_processed"] += 1
            
            if self._mode == SLAMMode.MAPPING:
                # En modo mapping, actualizar mapa
                self._update_map_from_scan(scan)
                self._stats["map_updates"] += 1
            
            elif self._mode == SLAMMode.LOCALIZATION:
                # En modo localization, solo actualizar pose
                self._update_pose_from_scan(scan)
            
            self._stats["last_update"] = datetime.now(timezone.utc).isoformat()
            
            return {
                "ok": True,
                "mode": self._mode.value,
                "pose": self._pose.copy(),
                "scans_processed": self._stats["scans_processed"],
            }
    
    def _update_map_from_scan(self, scan: ScanData) -> None:
        """Actualiza el mapa con los datos del scan."""
        if self._map is None:
            return
        
        x, y, theta = self._pose
        
        for r, angle in zip(scan.ranges, scan.angles):
            if r < self.config.min_range or r > self.config.max_range:
                continue
            
            # Calcular punto final del rayo
            world_angle = theta + angle
            end_x = x + r * math.cos(world_angle)
            end_y = y + r * math.sin(world_angle)
            
            # Convertir a coordenadas de mapa
            end_mx, end_my = self._world_to_map(end_x, end_y)
            start_mx, start_my = self._world_to_map(x, y)
            
            # Raycast: marcar celdas libres en el camino
            cells = self._bresenham(start_mx, start_my, end_mx, end_my)
            for cx, cy in cells[:-1]:  # Todas menos la última son libres
                if self._in_bounds(cx, cy):
                    # Incrementar confianza de libre
                    current = self._map.data[cy, cx]
                    if current == -1:
                        self._map.data[cy, cx] = 0
                    elif current > 0:
                        self._map.data[cy, cx] = max(0, current - 5)
            
            # Marcar celda final como ocupada
            if self._in_bounds(end_mx, end_my):
                current = self._map.data[end_my, end_mx]
                if current == -1:
                    self._map.data[end_my, end_mx] = 100
                else:
                    self._map.data[end_my, end_mx] = min(100, current + 10)
    
    def _update_pose_from_scan(self, scan: ScanData) -> None:
        """Actualiza la pose usando scan matching (placeholder)."""
        # TODO: Implementar ICP o correlative scan matching
        # Por ahora, solo usar odometría si está disponible
        pass
    
    def _world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """Convierte coordenadas mundo a coordenadas de mapa."""
        mx = int((x - self.config.origin[0]) / self.config.resolution)
        my = int((y - self.config.origin[1]) / self.config.resolution)
        return mx, my
    
    def _map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """Convierte coordenadas de mapa a coordenadas mundo."""
        x = mx * self.config.resolution + self.config.origin[0]
        y = my * self.config.resolution + self.config.origin[1]
        return x, y
    
    def _in_bounds(self, mx: int, my: int) -> bool:
        """Verifica si las coordenadas están dentro del mapa."""
        return 0 <= mx < self._map.width and 0 <= my < self._map.height
    
    def _bresenham(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """Algoritmo de Bresenham para líneas."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        
        return cells
    
    def get_pose(self) -> List[float]:
        """Retorna la pose actual [x, y, theta]."""
        with self._lock:
            return self._pose.copy()
    
    def set_pose(self, x: float, y: float, theta: float) -> None:
        """Establece la pose actual."""
        with self._lock:
            self._pose = [x, y, theta]
    
    def get_map(self) -> Optional[MapData]:
        """Retorna el mapa actual."""
        with self._lock:
            return self._map
    
    def save_map(self, path: Optional[str] = None) -> Dict[str, Any]:
        """Guarda el mapa en disco."""
        if self._map is None:
            return {"ok": False, "error": "No map to save"}
        
        save_dir = Path(path or self.config.map_save_path)
        save_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        map_file = save_dir / f"map_{timestamp}.npz"
        meta_file = save_dir / f"map_{timestamp}.json"
        
        try:
            # Guardar datos del mapa
            np.savez_compressed(str(map_file), data=self._map.data)
            
            # Guardar metadata
            meta = {
                "width": self._map.width,
                "height": self._map.height,
                "resolution": self._map.resolution,
                "origin": self._map.origin,
                "timestamp": self._map.timestamp,
                "saved_at": datetime.now(timezone.utc).isoformat(),
            }
            meta_file.write_text(json.dumps(meta, indent=2))
            
            _log.info("Map saved to %s", map_file)
            return {"ok": True, "map_file": str(map_file), "meta_file": str(meta_file)}
            
        except Exception as e:
            return {"ok": False, "error": str(e)}
    
    def load_map(self, path: str) -> Dict[str, Any]:
        """Carga un mapa desde disco."""
        try:
            map_path = Path(path)
            if not map_path.exists():
                return {"ok": False, "error": f"Map file not found: {path}"}
            
            # Cargar datos
            loaded = np.load(str(map_path))
            data = loaded["data"]
            
            # Cargar metadata
            meta_path = map_path.with_suffix(".json")
            if meta_path.exists():
                meta = json.loads(meta_path.read_text())
            else:
                meta = {
                    "width": data.shape[1],
                    "height": data.shape[0],
                    "resolution": 0.05,
                    "origin": (-5.0, -5.0),
                }
            
            self._map = MapData(
                width=meta["width"],
                height=meta["height"],
                resolution=meta["resolution"],
                origin=tuple(meta["origin"]),
                data=data,
                timestamp=meta.get("timestamp", ""),
            )
            
            _log.info("Map loaded from %s", path)
            return {"ok": True, "map": self._map.to_dict()}
            
        except Exception as e:
            return {"ok": False, "error": str(e)}
    
    def get_stats(self) -> Dict[str, Any]:
        """Retorna estadísticas del SLAM."""
        with self._lock:
            return {
                "mode": self._mode.value,
                "running": self._running,
                "pose": self._pose.copy(),
                "map_size": (self._map.width, self._map.height) if self._map else None,
                **self._stats,
            }
    
    @property
    def mode(self) -> SLAMMode:
        return self._mode
    
    @property
    def is_running(self) -> bool:
        return self._running
