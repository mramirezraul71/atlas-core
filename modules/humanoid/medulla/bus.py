"""
Médula Atlas: Bus de baja latencia para comunicación sensoriomotora.

Implementación con:
- ZeroMQ PUB/SUB para comunicación asíncrona de alta velocidad
- SharedMemory para estado compartido de latencia ultra-baja
- Fallback a threading si ZeroMQ no está disponible
"""
from __future__ import annotations

import asyncio
import json
import logging
import threading
import time
from collections import defaultdict
from dataclasses import asdict
from typing import Any, Callable, Dict, List, Optional, Set
from queue import Queue, Empty

from .schemas import (
    SensorReading, MotorCommand, WorldState, ActionDecision,
    GoalUpdate, VitalsStatus, SystemAlert, MessageType
)

logger = logging.getLogger(__name__)


def _bitacora(msg: str, ok: bool = True) -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log
        append_evolution_log(msg, ok=ok, source="medulla")
    except Exception:
        pass


# Intentar importar ZeroMQ
try:
    import zmq
    import zmq.asyncio
    HAS_ZMQ = True
except ImportError:
    HAS_ZMQ = False
    logger.warning("ZeroMQ not available, using fallback threading bus")


class SharedState:
    """Estado compartido de baja latencia usando memoria."""
    
    def __init__(self):
        self._lock = threading.RLock()
        self._state: Dict[str, Any] = {}
        self._timestamps: Dict[str, int] = {}
    
    def write(self, key: str, value: Any) -> None:
        """Escribe valor en estado compartido."""
        with self._lock:
            self._state[key] = value
            self._timestamps[key] = time.time_ns()
    
    def read(self, key: str, default: Any = None) -> Any:
        """Lee valor del estado compartido."""
        with self._lock:
            return self._state.get(key, default)
    
    def read_with_timestamp(self, key: str) -> tuple:
        """Lee valor con su timestamp."""
        with self._lock:
            return self._state.get(key), self._timestamps.get(key, 0)
    
    def keys(self) -> List[str]:
        """Lista de claves disponibles."""
        with self._lock:
            return list(self._state.keys())
    
    def clear(self) -> None:
        """Limpia todo el estado."""
        with self._lock:
            self._state.clear()
            self._timestamps.clear()


class ThreadingBus:
    """Bus de mensajes basado en threading (fallback sin ZMQ)."""
    
    def __init__(self):
        self._subscribers: Dict[str, List[Callable]] = defaultdict(list)
        self._queues: Dict[str, Queue] = defaultdict(lambda: Queue(maxsize=100))
        self._lock = threading.RLock()
        self._running = False
        self._workers: List[threading.Thread] = []
    
    def subscribe(self, topic: str, callback: Callable) -> None:
        """Suscribe callback a un topic."""
        with self._lock:
            self._subscribers[topic].append(callback)
    
    def unsubscribe(self, topic: str, callback: Callable) -> None:
        """Desuscribe callback de un topic."""
        with self._lock:
            if callback in self._subscribers[topic]:
                self._subscribers[topic].remove(callback)
    
    def publish(self, topic: str, data: Any) -> None:
        """Publica mensaje a un topic."""
        with self._lock:
            # Buscar suscriptores que coincidan (soporta wildcards)
            for sub_topic, callbacks in self._subscribers.items():
                if self._topic_matches(sub_topic, topic):
                    for callback in callbacks:
                        try:
                            callback(topic, data)
                        except Exception as e:
                            logger.error(f"Error in subscriber callback for {topic}: {e}")
    
    def _topic_matches(self, pattern: str, topic: str) -> bool:
        """Verifica si topic coincide con pattern (soporta *)."""
        if pattern == topic:
            return True
        if '*' in pattern:
            parts = pattern.split('.')
            topic_parts = topic.split('.')
            if len(parts) != len(topic_parts):
                if not pattern.endswith('.*'):
                    return False
            for p, t in zip(parts, topic_parts):
                if p != '*' and p != t:
                    return False
            return True
        return False
    
    def start(self) -> None:
        """Inicia el bus."""
        self._running = True
    
    def stop(self) -> None:
        """Detiene el bus."""
        self._running = False


class ZMQBus:
    """Bus de mensajes basado en ZeroMQ."""
    
    def __init__(self, pub_port: int = 5555, sub_port: int = 5556):
        self.pub_port = pub_port
        self.sub_port = sub_port
        self.context: Optional[zmq.Context] = None
        self.pub_socket: Optional[zmq.Socket] = None
        self.sub_socket: Optional[zmq.Socket] = None
        self._subscribers: Dict[str, List[Callable]] = defaultdict(list)
        self._running = False
        self._recv_thread: Optional[threading.Thread] = None
        self._lock = threading.RLock()
    
    def start(self) -> None:
        """Inicia el bus ZMQ."""
        self.context = zmq.Context()
        
        # Publisher socket
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://*:{self.pub_port}")
        
        # Subscriber socket
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://localhost:{self.pub_port}")
        
        # Suscribir a todos los mensajes por defecto
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        self._running = True
        self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._recv_thread.start()
        
        logger.info(f"ZMQ Bus started on ports {self.pub_port}/{self.sub_port}")
    
    def stop(self) -> None:
        """Detiene el bus ZMQ."""
        self._running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=1.0)
        if self.pub_socket:
            self.pub_socket.close()
        if self.sub_socket:
            self.sub_socket.close()
        if self.context:
            self.context.term()
        logger.info("ZMQ Bus stopped")
    
    def subscribe(self, topic: str, callback: Callable) -> None:
        """Suscribe callback a un topic."""
        with self._lock:
            self._subscribers[topic].append(callback)
    
    def unsubscribe(self, topic: str, callback: Callable) -> None:
        """Desuscribe callback de un topic."""
        with self._lock:
            if callback in self._subscribers[topic]:
                self._subscribers[topic].remove(callback)
    
    def publish(self, topic: str, data: Any) -> None:
        """Publica mensaje a un topic."""
        if not self.pub_socket:
            return
        
        # Serializar datos
        if hasattr(data, 'to_bytes'):
            payload = data.to_bytes()
        elif hasattr(data, '__dict__'):
            payload = json.dumps(asdict(data) if hasattr(data, '__dataclass_fields__') else data.__dict__).encode('utf-8')
        else:
            payload = json.dumps(data, default=str).encode('utf-8')
        
        # Enviar con topic como prefijo
        message = f"{topic}|".encode('utf-8') + payload
        self.pub_socket.send(message, zmq.NOBLOCK)
    
    def _receive_loop(self) -> None:
        """Loop de recepción de mensajes."""
        while self._running:
            try:
                if self.sub_socket.poll(timeout=100):
                    message = self.sub_socket.recv()
                    # Parsear topic y payload
                    try:
                        topic_end = message.index(b'|')
                        topic = message[:topic_end].decode('utf-8')
                        payload = message[topic_end + 1:]
                        data = json.loads(payload.decode('utf-8'))
                        
                        # Notificar suscriptores
                        self._notify_subscribers(topic, data)
                    except Exception as e:
                        logger.error(f"Error parsing message: {e}")
            except Exception as e:
                if self._running:
                    logger.error(f"Error in receive loop: {e}")
    
    def _notify_subscribers(self, topic: str, data: Any) -> None:
        """Notifica a suscriptores de un topic."""
        with self._lock:
            for sub_topic, callbacks in self._subscribers.items():
                if self._topic_matches(sub_topic, topic):
                    for callback in callbacks:
                        try:
                            callback(topic, data)
                        except Exception as e:
                            logger.error(f"Error in subscriber callback: {e}")
    
    def _topic_matches(self, pattern: str, topic: str) -> bool:
        """Verifica si topic coincide con pattern."""
        if pattern == topic:
            return True
        if '*' in pattern:
            parts = pattern.split('.')
            topic_parts = topic.split('.')
            for p, t in zip(parts, topic_parts):
                if p != '*' and p != t:
                    return False
            return True
        return False


class MedullaAtlas:
    """
    Bus principal de la Médula Atlas.
    
    Proporciona comunicación de baja latencia entre:
    - Sensores (visión, audio, IMU, fuerza/torque, encoders)
    - Actuadores (brazos, manos, locomoción, cuello)
    - Módulos cognitivos (percepción, decisión, objetivos)
    
    Uso:
        medulla = MedullaAtlas()
        medulla.start()
        
        # Publicar lectura de sensor
        medulla.publish_sensor("cam_0", vision_frame)
        
        # Suscribirse a comandos
        medulla.subscribe_motor("arm.right.*", handle_arm_command)
    """
    
    def __init__(self, use_zmq: bool = True, zmq_port: int = 5555):
        """
        Inicializa la Médula Atlas.
        
        Args:
            use_zmq: Usar ZeroMQ si está disponible
            zmq_port: Puerto base para ZMQ
        """
        self.use_zmq = use_zmq and HAS_ZMQ
        self.zmq_port = zmq_port
        
        # Bus de mensajes
        if self.use_zmq:
            self._bus = ZMQBus(pub_port=zmq_port, sub_port=zmq_port + 1)
        else:
            self._bus = ThreadingBus()
        
        # Estado compartido de baja latencia
        self.shared_state = SharedState()
        
        # Caché de último estado
        self._last_world_state: Optional[WorldState] = None
        self._last_vitals: Optional[VitalsStatus] = None
        
        # Métricas
        self._message_counts: Dict[str, int] = defaultdict(int)
        self._last_latencies: Dict[str, float] = {}
        
        self._running = False
        logger.info(f"MedullaAtlas initialized (ZMQ: {self.use_zmq})")
    
    def start(self) -> None:
        """Inicia la Médula Atlas."""
        self._bus.start()
        self._running = True
        
        # Suscribirse a mensajes de sistema para caché
        self.subscribe("cognitive.perception.world_state", self._cache_world_state)
        self.subscribe("system.vitals", self._cache_vitals)
        
        logger.info("MedullaAtlas started")
        _bitacora(f"MedullaAtlas started (ZMQ={self.use_zmq})")
    
    def stop(self) -> None:
        """Detiene la Médula Atlas."""
        self._running = False
        self._bus.stop()
        logger.info("MedullaAtlas stopped")
        _bitacora("MedullaAtlas stopped")
    
    # === Publicación de Sensores ===
    
    def publish_sensor(self, sensor_id: str, data: SensorReading) -> None:
        """
        Publica lectura de sensor.
        
        Args:
            sensor_id: ID del sensor (ej: "cam_0", "imu_main", "ft_wrist_right")
            data: Datos del sensor
        """
        topic = f"sensor.{data.sensor_type}.{sensor_id}"
        self._publish(topic, data)
        
        # Actualizar estado compartido para acceso rápido
        self.shared_state.write(f"sensor.{sensor_id}", data)
    
    def publish_vision(self, camera_id: str, frame: Any) -> None:
        """Publica frame de visión."""
        topic = f"sensor.vision.{camera_id}"
        self._publish(topic, frame)
        self.shared_state.write(f"vision.{camera_id}", frame)
    
    def publish_imu(self, data: Any) -> None:
        """Publica lectura de IMU."""
        topic = "sensor.imu"
        self._publish(topic, data)
        self.shared_state.write("imu", data)
    
    def publish_force_torque(self, joint_id: str, data: Any) -> None:
        """Publica lectura de fuerza/torque."""
        topic = f"sensor.ft.{joint_id}"
        self._publish(topic, data)
        self.shared_state.write(f"ft.{joint_id}", data)
    
    def publish_encoder(self, joint_id: str, data: Any) -> None:
        """Publica lectura de encoder."""
        topic = f"sensor.encoder.{joint_id}"
        self._publish(topic, data)
        self.shared_state.write(f"encoder.{joint_id}", data)
    
    # === Comandos de Actuadores ===
    
    def send_motor_command(self, actuator_id: str, command: MotorCommand) -> None:
        """
        Envía comando a actuador.
        
        Args:
            actuator_id: ID del actuador (ej: "arm.right.shoulder", "gripper.left")
            command: Comando a ejecutar
        """
        topic = f"actuator.{actuator_id}"
        self._publish(topic, command)
    
    def send_arm_command(self, arm_id: str, joint_id: str, command: Any) -> None:
        """Envía comando a articulación del brazo."""
        topic = f"actuator.arm.{arm_id}.{joint_id}"
        self._publish(topic, command)
    
    def send_gripper_command(self, gripper_id: str, command: Any) -> None:
        """Envía comando al gripper."""
        topic = f"actuator.gripper.{gripper_id}"
        self._publish(topic, command)
    
    def send_locomotion_command(self, command: Any) -> None:
        """Envía comando de locomoción."""
        topic = "actuator.locomotion"
        self._publish(topic, command)
    
    def send_neck_command(self, command: Any) -> None:
        """Envía comando al cuello (pan-tilt)."""
        topic = "actuator.neck"
        self._publish(topic, command)
    
    # === Mensajes Cognitivos ===
    
    def publish_world_state(self, state: WorldState) -> None:
        """Publica estado del mundo actualizado."""
        topic = "cognitive.perception.world_state"
        self._publish(topic, state)
        self._last_world_state = state
        self.shared_state.write("world_state", state)
    
    def publish_decision(self, decision: ActionDecision) -> None:
        """Publica decisión de acción."""
        topic = "cognitive.decision.action"
        self._publish(topic, decision)
    
    def publish_goal_update(self, update: GoalUpdate) -> None:
        """Publica actualización de objetivo."""
        topic = "cognitive.goal.update"
        self._publish(topic, update)
    
    # === Mensajes de Sistema ===
    
    def publish_vitals(self, vitals: VitalsStatus) -> None:
        """Publica estado de signos vitales."""
        topic = "system.vitals"
        self._publish(topic, vitals)
        self._last_vitals = vitals
        self.shared_state.write("vitals", vitals)
    
    def publish_alert(self, alert: SystemAlert) -> None:
        """Publica alerta del sistema."""
        topic = f"system.alert.{alert.level}"
        self._publish(topic, alert)
        
        # Alertas críticas van al estado compartido
        if alert.level in ("critical", "emergency"):
            self.shared_state.write(f"alert.{alert.alert_id}", alert)
            _bitacora(f"ALERT [{alert.level}] {alert.source}: {alert.message}", ok=False)
    
    # === Suscripciones ===
    
    def subscribe(self, topic_pattern: str, callback: Callable) -> None:
        """
        Suscribe a un patrón de topics.
        
        Args:
            topic_pattern: Patrón de topic (soporta * como wildcard)
            callback: Función callback(topic, data)
        """
        self._bus.subscribe(topic_pattern, callback)
    
    def unsubscribe(self, topic_pattern: str, callback: Callable) -> None:
        """Desuscribe de un topic."""
        self._bus.unsubscribe(topic_pattern, callback)
    
    def subscribe_sensor(self, sensor_pattern: str, callback: Callable) -> None:
        """Suscribe a sensores."""
        self.subscribe(f"sensor.{sensor_pattern}", callback)
    
    def subscribe_motor(self, actuator_pattern: str, callback: Callable) -> None:
        """Suscribe a comandos de actuadores."""
        self.subscribe(f"actuator.{actuator_pattern}", callback)
    
    def subscribe_cognitive(self, callback: Callable) -> None:
        """Suscribe a todos los mensajes cognitivos."""
        self.subscribe("cognitive.*", callback)
    
    def subscribe_alerts(self, callback: Callable, level: str = "*") -> None:
        """Suscribe a alertas del sistema."""
        self.subscribe(f"system.alert.{level}", callback)
    
    # === Acceso Rápido a Estado ===
    
    def get_world_state(self) -> Optional[WorldState]:
        """Obtiene último estado del mundo."""
        return self._last_world_state
    
    def get_vitals(self) -> Optional[VitalsStatus]:
        """Obtiene últimos signos vitales."""
        return self._last_vitals
    
    def get_sensor_value(self, sensor_id: str) -> Any:
        """Obtiene último valor de un sensor."""
        return self.shared_state.read(f"sensor.{sensor_id}")
    
    def get_joint_state(self, joint_id: str) -> Any:
        """Obtiene estado de una articulación."""
        return self.shared_state.read(f"encoder.{joint_id}")
    
    # === Métricas ===
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadísticas del bus."""
        return {
            "running": self._running,
            "use_zmq": self.use_zmq,
            "message_counts": dict(self._message_counts),
            "latencies_ms": {k: v * 1000 for k, v in self._last_latencies.items()},
            "shared_state_keys": self.shared_state.keys(),
        }
    
    # === Internos ===
    
    def _publish(self, topic: str, data: Any) -> None:
        """Publica mensaje interno."""
        start = time.time()
        self._bus.publish(topic, data)
        self._message_counts[topic] += 1
        self._last_latencies[topic] = time.time() - start
    
    def _cache_world_state(self, topic: str, data: Any) -> None:
        """Cachea estado del mundo."""
        if isinstance(data, WorldState):
            self._last_world_state = data
        elif isinstance(data, dict):
            self._last_world_state = WorldState(**data)
    
    def _cache_vitals(self, topic: str, data: Any) -> None:
        """Cachea signos vitales."""
        if isinstance(data, VitalsStatus):
            self._last_vitals = data
        elif isinstance(data, dict):
            self._last_vitals = VitalsStatus(**data)


# === Singleton ===

_medulla_instance: Optional[MedullaAtlas] = None
_medulla_lock = threading.Lock()


def get_medulla() -> MedullaAtlas:
    """
    Obtiene instancia singleton de la Médula Atlas.
    
    Returns:
        MedullaAtlas: Instancia del bus
    """
    global _medulla_instance
    with _medulla_lock:
        if _medulla_instance is None:
            _medulla_instance = MedullaAtlas()
        return _medulla_instance


def init_medulla(use_zmq: bool = True, zmq_port: int = 5555) -> MedullaAtlas:
    """
    Inicializa la Médula Atlas con configuración específica.
    
    Args:
        use_zmq: Usar ZeroMQ
        zmq_port: Puerto para ZMQ
    
    Returns:
        MedullaAtlas: Instancia inicializada
    """
    global _medulla_instance
    with _medulla_lock:
        if _medulla_instance is not None:
            _medulla_instance.stop()
        _medulla_instance = MedullaAtlas(use_zmq=use_zmq, zmq_port=zmq_port)
        _medulla_instance.start()
        return _medulla_instance
