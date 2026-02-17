"""
VitalsMonitor: Monitoreo de signos vitales del tronco encefalico.

Analogo biologico: Bulbo raquideo
- Monitoreo continuo de signos vitales
- Deteccion de condiciones criticas
- Alertas automaticas
"""
from __future__ import annotations

import asyncio
import logging
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)


@dataclass
class VitalsReading:
    """Lectura de signos vitales."""
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    
    # Energia
    battery_percent: float = 100.0
    power_draw_watts: float = 0.0
    charging: bool = False
    
    # Temperaturas
    cpu_temp: float = 45.0
    gpu_temp: float = 50.0
    motor_temps: Dict[str, float] = field(default_factory=dict)
    ambient_temp: float = 25.0
    
    # Recursos de sistema
    cpu_usage_percent: float = 0.0
    memory_used_mb: int = 0
    memory_total_mb: int = 8192
    disk_used_percent: float = 0.0
    
    # Red
    network_connected: bool = True
    network_latency_ms: float = 0.0
    
    # Estado de motores
    motors_enabled: bool = True
    motors_in_fault: List[str] = field(default_factory=list)
    
    # Sensores
    sensors_active: int = 0
    sensors_total: int = 0
    
    # Estado general
    emergency_stop: bool = False
    uptime_seconds: float = 0.0
    
    def max_motor_temp(self) -> float:
        """Obtiene temperatura maxima de motores."""
        if not self.motor_temps:
            return 25.0
        return max(self.motor_temps.values())
    
    def memory_usage_percent(self) -> float:
        """Calcula uso de memoria en porcentaje."""
        if self.memory_total_mb == 0:
            return 0.0
        return (self.memory_used_mb / self.memory_total_mb) * 100
    
    def sensor_health(self) -> float:
        """Calcula salud de sensores (0-1)."""
        if self.sensors_total == 0:
            return 1.0
        return self.sensors_active / self.sensors_total
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp_ns": self.timestamp_ns,
            "battery_percent": self.battery_percent,
            "cpu_temp": self.cpu_temp,
            "gpu_temp": self.gpu_temp,
            "max_motor_temp": self.max_motor_temp(),
            "cpu_usage_percent": self.cpu_usage_percent,
            "memory_usage_percent": self.memory_usage_percent(),
            "disk_used_percent": self.disk_used_percent,
            "network_connected": self.network_connected,
            "motors_enabled": self.motors_enabled,
            "motors_in_fault": self.motors_in_fault,
            "emergency_stop": self.emergency_stop,
            "sensor_health": self.sensor_health(),
        }


@dataclass
class VitalThreshold:
    """Umbral para un signo vital."""
    vital_name: str
    warning: Optional[float] = None
    critical: Optional[float] = None
    is_max: bool = True  # True si el umbral es un maximo, False si es minimo


class VitalsMonitor:
    """
    Monitor de signos vitales del tronco encefalico.
    
    Responsabilidades:
    - Monitoreo continuo de signos vitales
    - Deteccion de condiciones criticas
    - Alertas automaticas
    - Historial de lecturas
    """
    
    # Umbrales por defecto
    DEFAULT_THRESHOLDS = {
        "battery_percent": VitalThreshold("battery_percent", warning=20, critical=10, is_max=False),
        "cpu_temp": VitalThreshold("cpu_temp", warning=75, critical=85, is_max=True),
        "gpu_temp": VitalThreshold("gpu_temp", warning=80, critical=90, is_max=True),
        "motor_temp": VitalThreshold("motor_temp", warning=60, critical=70, is_max=True),
        "cpu_usage": VitalThreshold("cpu_usage", warning=85, critical=95, is_max=True),
        "memory_usage": VitalThreshold("memory_usage", warning=85, critical=95, is_max=True),
        "disk_usage": VitalThreshold("disk_usage", warning=85, critical=95, is_max=True),
    }
    
    def __init__(self, 
                 update_interval_s: float = 1.0,
                 history_size: int = 3600):
        """
        Inicializa el monitor.
        
        Args:
            update_interval_s: Intervalo de actualizacion en segundos
            history_size: Tamano del historial de lecturas
        """
        self.update_interval_s = update_interval_s
        self.history_size = history_size
        
        # Umbrales
        self.thresholds = self.DEFAULT_THRESHOLDS.copy()
        
        # Lectura actual
        self._current: VitalsReading = VitalsReading()
        
        # Historial
        self._history: deque = deque(maxlen=history_size)
        
        # Alertas activas
        self._active_alerts: Dict[str, Dict[str, Any]] = {}
        
        # Callbacks
        self._on_warning: List[Callable[[str, float], None]] = []
        self._on_critical: List[Callable[[str, float], None]] = []
        self._on_emergency: List[Callable[[str], None]] = []
        
        # Estado
        self._running = False
        self._monitor_task: Optional[asyncio.Task] = None
        
        # Timestamp de inicio
        self._start_time_ns = time.time_ns()
    
    async def start(self) -> None:
        """Inicia el monitoreo."""
        if self._running:
            return
        
        self._running = True
        self._start_time_ns = time.time_ns()
        self._monitor_task = asyncio.create_task(self._monitor_loop())
        logger.info("VitalsMonitor started")
    
    async def stop(self) -> None:
        """Detiene el monitoreo."""
        self._running = False
        if self._monitor_task:
            self._monitor_task.cancel()
            try:
                await self._monitor_task
            except asyncio.CancelledError:
                pass
        logger.info("VitalsMonitor stopped")
    
    async def _monitor_loop(self) -> None:
        """Loop principal de monitoreo."""
        while self._running:
            try:
                # Leer signos vitales
                await self._read_vitals()
                
                # Verificar umbrales
                self._check_thresholds()
                
                # Guardar en historial
                self._history.append(self._current)
                
                await asyncio.sleep(self.update_interval_s)
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in vitals monitor: {e}")
                await asyncio.sleep(self.update_interval_s)
    
    async def _read_vitals(self) -> None:
        """Lee signos vitales del sistema."""
        try:
            import psutil
            
            # CPU
            self._current.cpu_usage_percent = psutil.cpu_percent()
            
            # Memoria
            mem = psutil.virtual_memory()
            self._current.memory_used_mb = int(mem.used / (1024 * 1024))
            self._current.memory_total_mb = int(mem.total / (1024 * 1024))
            
            # Disco
            disk = psutil.disk_usage('/')
            self._current.disk_used_percent = disk.percent
            
            # Temperaturas (si estan disponibles)
            try:
                temps = psutil.sensors_temperatures()
                if 'coretemp' in temps:
                    cpu_temps = [t.current for t in temps['coretemp']]
                    self._current.cpu_temp = max(cpu_temps) if cpu_temps else 45.0
            except:
                pass
            
            # Bateria (si esta disponible)
            try:
                battery = psutil.sensors_battery()
                if battery:
                    self._current.battery_percent = battery.percent
                    self._current.charging = battery.power_plugged
            except:
                pass
            
            # Red
            try:
                net = psutil.net_if_stats()
                self._current.network_connected = any(s.isup for s in net.values())
            except:
                self._current.network_connected = True
            
        except ImportError:
            # psutil no disponible, usar valores por defecto
            pass
        
        # Uptime
        self._current.uptime_seconds = (time.time_ns() - self._start_time_ns) / 1e9
        self._current.timestamp_ns = time.time_ns()
    
    def _check_thresholds(self) -> None:
        """Verifica umbrales y genera alertas."""
        checks = [
            ("battery_percent", self._current.battery_percent),
            ("cpu_temp", self._current.cpu_temp),
            ("gpu_temp", self._current.gpu_temp),
            ("motor_temp", self._current.max_motor_temp()),
            ("cpu_usage", self._current.cpu_usage_percent),
            ("memory_usage", self._current.memory_usage_percent()),
            ("disk_usage", self._current.disk_used_percent),
        ]
        
        for vital_name, value in checks:
            threshold = self.thresholds.get(vital_name)
            if not threshold:
                continue
            
            is_critical = False
            is_warning = False
            
            if threshold.is_max:
                # Umbral es un maximo
                if threshold.critical and value >= threshold.critical:
                    is_critical = True
                elif threshold.warning and value >= threshold.warning:
                    is_warning = True
            else:
                # Umbral es un minimo
                if threshold.critical and value <= threshold.critical:
                    is_critical = True
                elif threshold.warning and value <= threshold.warning:
                    is_warning = True
            
            # Emitir alertas
            if is_critical:
                self._emit_critical(vital_name, value)
            elif is_warning:
                self._emit_warning(vital_name, value)
            else:
                # Limpiar alerta si existia
                if vital_name in self._active_alerts:
                    del self._active_alerts[vital_name]
        
        # Verificar emergency stop
        if self._current.emergency_stop:
            self._emit_emergency("emergency_stop_active")
        
        # Verificar motores en fallo
        if self._current.motors_in_fault:
            self._emit_warning("motors_in_fault", len(self._current.motors_in_fault))
    
    def _emit_warning(self, vital_name: str, value: float) -> None:
        """Emite alerta de advertencia."""
        if vital_name not in self._active_alerts or self._active_alerts[vital_name].get("level") != "warning":
            self._active_alerts[vital_name] = {
                "level": "warning",
                "value": value,
                "timestamp_ns": time.time_ns(),
            }
            
            logger.warning(f"Vital warning: {vital_name} = {value}")
            
            for callback in self._on_warning:
                try:
                    callback(vital_name, value)
                except Exception as e:
                    logger.error(f"Error in warning callback: {e}")
    
    def _emit_critical(self, vital_name: str, value: float) -> None:
        """Emite alerta critica."""
        if vital_name not in self._active_alerts or self._active_alerts[vital_name].get("level") != "critical":
            self._active_alerts[vital_name] = {
                "level": "critical",
                "value": value,
                "timestamp_ns": time.time_ns(),
            }
            
            logger.error(f"Vital CRITICAL: {vital_name} = {value}")
            
            for callback in self._on_critical:
                try:
                    callback(vital_name, value)
                except Exception as e:
                    logger.error(f"Error in critical callback: {e}")
    
    def _emit_emergency(self, reason: str) -> None:
        """Emite alerta de emergencia."""
        logger.critical(f"EMERGENCY: {reason}")
        
        for callback in self._on_emergency:
            try:
                callback(reason)
            except Exception as e:
                logger.error(f"Error in emergency callback: {e}")
    
    def update_vitals(self, **kwargs) -> None:
        """
        Actualiza signos vitales manualmente.
        
        Args:
            **kwargs: Campos a actualizar
        """
        for key, value in kwargs.items():
            if hasattr(self._current, key):
                setattr(self._current, key, value)
        
        self._current.timestamp_ns = time.time_ns()
        self._check_thresholds()
    
    def get_current(self) -> VitalsReading:
        """Obtiene lectura actual."""
        return self._current
    
    def get_active_alerts(self) -> Dict[str, Dict[str, Any]]:
        """Obtiene alertas activas."""
        return self._active_alerts.copy()
    
    def is_critical(self) -> bool:
        """Verifica si hay alguna condicion critica."""
        return any(a.get("level") == "critical" for a in self._active_alerts.values())
    
    def is_healthy(self) -> bool:
        """Verifica si el sistema esta saludable."""
        return (
            not self._active_alerts and
            not self._current.emergency_stop and
            not self._current.motors_in_fault and
            self._current.battery_percent > 20 and
            self._current.sensor_health() > 0.8
        )
    
    def get_history(self, last_n: int = 60) -> List[VitalsReading]:
        """Obtiene historial de lecturas."""
        return list(self._history)[-last_n:]
    
    def on_warning(self, callback: Callable[[str, float], None]) -> None:
        """Registra callback para advertencias."""
        self._on_warning.append(callback)
    
    def on_critical(self, callback: Callable[[str, float], None]) -> None:
        """Registra callback para criticos."""
        self._on_critical.append(callback)
    
    def on_emergency(self, callback: Callable[[str], None]) -> None:
        """Registra callback para emergencias."""
        self._on_emergency.append(callback)
    
    def set_threshold(self, vital_name: str, warning: float = None, 
                     critical: float = None) -> None:
        """Establece umbral para un signo vital."""
        if vital_name in self.thresholds:
            if warning is not None:
                self.thresholds[vital_name].warning = warning
            if critical is not None:
                self.thresholds[vital_name].critical = critical
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas del monitor."""
        return {
            "running": self._running,
            "current": self._current.to_dict(),
            "active_alerts": self._active_alerts,
            "is_healthy": self.is_healthy(),
            "is_critical": self.is_critical(),
            "history_size": len(self._history),
            "uptime_seconds": self._current.uptime_seconds,
        }
