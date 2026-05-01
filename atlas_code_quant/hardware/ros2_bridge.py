"""Módulo 1C — ROS2 Bridge para Atlas Trading Node.

Publica y suscribe mensajes ROS2 Humble para integrar el pipeline de trading
con el sistema robótico físico de ATLAS (chasis diferencial, sensores, etc.).

Tópicos publicados:
    /atlas/trading/signal      → TradeSignal (custom msg)
    /atlas/trading/status      → SystemStatus
    /atlas/camera/ocr_result   → OCRResult codificado como String JSON

Tópicos suscritos:
    /atlas/robot/state         → estado del chasis (batería, posición)
    /atlas/emergency_stop      → señal de parada de emergencia

Nodo standalone: `python -m atlas_code_quant.hardware.ros2_bridge`
"""

from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass, asdict
from typing import Callable, Optional

logger = logging.getLogger("atlas.hardware.ros2")

# ROS2 es opcional — si no está instalado el sistema corre en modo degradado
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Bool, Float32
    _ROS2_OK = True
except ImportError:
    _ROS2_OK = False
    logger.info("rclpy not installed — ROS2 bridge disabled (not required for serial/vision mode)")

    class Node:  # type: ignore[no-redef]
        """Nodo ROS2 inerte — no loguea ni simula nada."""
        def __init__(self, name: str) -> None:
            self._name = name
        def create_publisher(self, *a, **kw): return _NullPublisher()
        def create_subscription(self, *a, **kw): return None
        def create_timer(self, *a, **kw): return None
        def get_logger(self): return logger
        def destroy_node(self): pass

    class _NullPublisher:
        """Descarta mensajes silenciosamente — sin logs, sin stubs."""
        def publish(self, msg: object) -> None:
            pass

    class String:  # type: ignore[no-redef]
        data: str = ""

    class Bool:  # type: ignore[no-redef]
        data: bool = False

    class Float32:  # type: ignore[no-redef]
        data: float = 0.0


@dataclass
class RobotState:
    """Estado del robot físico recibido por ROS2."""
    battery_pct: float = 100.0
    is_mobile: bool = False
    position_x: float = 0.0
    position_y: float = 0.0
    emergency_stop: bool = False
    timestamp: float = 0.0


class ATLASRos2Bridge:
    """Bridge ROS2 ↔ Atlas Code-Quant (thread-safe, sin bloqueo del loop principal).

    Uso::

        bridge = ATLASRos2Bridge()
        bridge.start()
        bridge.publish_signal({"symbol": "AAPL", "side": "BUY", "qty": 10})
        state = bridge.robot_state()
        bridge.stop()
    """

    TOPIC_SIGNAL  = "/atlas/trading/signal"
    TOPIC_STATUS  = "/atlas/trading/status"
    TOPIC_OCR     = "/atlas/camera/ocr_result"
    TOPIC_ROBOT   = "/atlas/robot/state"
    TOPIC_ESTOP   = "/atlas/emergency_stop"

    def __init__(self, node_name: str = "atlas_quant_bridge") -> None:
        self.node_name = node_name
        self._ros_thread: Optional[threading.Thread] = None
        self._running = False
        self._node: Optional[object] = None
        self._robot_state = RobotState(timestamp=time.time())
        self._state_lock = threading.Lock()
        self._emergency_callbacks: list[Callable[[], None]] = []
        self._pub_signal: object = _NullPublisher()
        self._pub_status: object = _NullPublisher()
        self._pub_ocr: object = _NullPublisher()

    def on_emergency_stop(self, callback: Callable[[], None]) -> None:
        self._emergency_callbacks.append(callback)

    def start(self) -> None:
        if not _ROS2_OK:
            logger.info("ROS2 no disponible — bridge en modo simulado")
            return
        self._running = True
        self._ros_thread = threading.Thread(
            target=self._ros_spin, daemon=True, name="atlas-ros2-spin"
        )
        self._ros_thread.start()
        logger.info("ROS2 bridge iniciado — nodo: %s", self.node_name)

    def stop(self) -> None:
        self._running = False
        if _ROS2_OK:
            rclpy.shutdown()

    def robot_state(self) -> RobotState:
        with self._state_lock:
            return RobotState(**asdict(self._robot_state))

    def publish_signal(self, signal_dict: dict) -> None:
        msg = String()
        msg.data = json.dumps(signal_dict)
        self._pub_signal.publish(msg)  # type: ignore[attr-defined]

    def publish_ocr_result(self, ocr_dict: dict) -> None:
        msg = String()
        msg.data = json.dumps(ocr_dict)
        self._pub_ocr.publish(msg)  # type: ignore[attr-defined]

    def publish_status(self, status_dict: dict) -> None:
        msg = String()
        msg.data = json.dumps(status_dict)
        self._pub_status.publish(msg)  # type: ignore[attr-defined]

    def _ros_spin(self) -> None:
        rclpy.init()
        node = Ros2TradingNode(
            self.node_name,
            on_robot_state=self._on_robot_state,
            on_emergency_stop=self._on_estop,
        )
        self._node = node
        self._pub_signal = node.pub_signal
        self._pub_status = node.pub_status
        self._pub_ocr = node.pub_ocr

        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)  # type: ignore[arg-type]
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def _on_robot_state(self, state: RobotState) -> None:
        with self._state_lock:
            self._robot_state = state

    def _on_estop(self) -> None:
        logger.critical("ROS2 Emergency Stop recibido")
        for cb in self._emergency_callbacks:
            try:
                cb()
            except Exception as exc:
                logger.error("Error en emergency callback: %s", exc)


class Ros2TradingNode(Node):
    """Nodo ROS2 real — sólo se instancia si rclpy está disponible."""

    def __init__(
        self,
        name: str,
        on_robot_state: Callable[[RobotState], None] | None = None,
        on_emergency_stop: Callable[[], None] | None = None,
    ) -> None:
        super().__init__(name)

        self._on_robot_state = on_robot_state
        self._on_emergency_stop = on_emergency_stop

        # Publishers
        if _ROS2_OK:
            self.pub_signal = self.create_publisher(String, ATLASRos2Bridge.TOPIC_SIGNAL, 10)
            self.pub_status = self.create_publisher(String, ATLASRos2Bridge.TOPIC_STATUS, 10)
            self.pub_ocr    = self.create_publisher(String, ATLASRos2Bridge.TOPIC_OCR,    10)

            # Subscribers
            self.create_subscription(
                String, ATLASRos2Bridge.TOPIC_ROBOT, self._robot_cb, 10
            )
            self.create_subscription(
                Bool, ATLASRos2Bridge.TOPIC_ESTOP, self._estop_cb, 10
            )
        else:
            self.pub_signal = _NullPublisher()
            self.pub_status = _NullPublisher()
            self.pub_ocr    = _NullPublisher()

        self.get_logger().info("Ros2TradingNode iniciado")

    def _robot_cb(self, msg: object) -> None:
        try:
            data = json.loads(msg.data)  # type: ignore[attr-defined]
            state = RobotState(
                battery_pct=data.get("battery_pct", 100.0),
                is_mobile=data.get("is_mobile", False),
                position_x=data.get("position_x", 0.0),
                position_y=data.get("position_y", 0.0),
                emergency_stop=data.get("emergency_stop", False),
                timestamp=time.time(),
            )
            if self._on_robot_state:
                self._on_robot_state(state)
        except Exception as exc:
            self.get_logger().error(f"Error robot_cb: {exc}")

    def _estop_cb(self, msg: object) -> None:
        if getattr(msg, "data", False) and self._on_emergency_stop:
            self._on_emergency_stop()


# ── Lanzamiento standalone como nodo ROS2 ────────────────────────────────────

def _launch_standalone_node() -> None:
    """Lanzar como: python -m atlas_code_quant.hardware.ros2_bridge"""
    logging.basicConfig(level=logging.INFO)
    if not _ROS2_OK:
        logger.error("rclpy no instalado. Instalar: pip install rclpy (en entorno ROS2)")
        return

    rclpy.init()
    node = Ros2TradingNode("atlas_trading_node_standalone")
    logger.info("Nodo standalone corriendo. Ctrl+C para salir.")
    try:
        rclpy.spin(node)  # type: ignore[arg-type]
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    _launch_standalone_node()
