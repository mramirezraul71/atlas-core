"""Bridge: comunicación bidireccional con ATLAS_NEXUS. Serial/WebSocket/HTTP. Reconexión automática (Self-Healing)."""
from __future__ import annotations

import json
import logging
import os
import threading
import time
import urllib.request
import urllib.error
from queue import Empty, Queue
from typing import Any, Callable, Dict, Optional

logger = logging.getLogger("atlas.bridge")

SERIAL_PORT = os.getenv("ATLAS_NEXUS_SERIAL", "COM3")
SERIAL_BAUD = int(os.getenv("ATLAS_NEXUS_BAUD", "115200"))
ROBOT_API = (os.getenv("NEXUS_ROBOT_API_URL") or "http://127.0.0.1:8002").rstrip("/")
WS_URL = os.getenv("ATLAS_NEXUS_WS_URL", "ws://127.0.0.1:8002/ws")
BRIDGE_MODE = os.getenv("ATLAS_BRIDGE_MODE", "http").lower()
RECONNECT_DELAY = float(os.getenv("ATLAS_BRIDGE_RECONNECT_DELAY", "2.0"))
RX_READ_TIMEOUT = float(os.getenv("ATLAS_BRIDGE_RX_TIMEOUT", "0.1"))


def _normalize_command(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Asegura formato JSON estándar: actuador, estado, velocidad."""
    out = {"actuador": str(payload.get("actuador", "unknown")), "estado": int(payload.get("estado", 1)), "velocidad": int(payload.get("velocidad", 255))}
    for k, v in payload.items():
        if k not in out:
            out[k] = v
    return out


class HardwareBridge:
    def __init__(self, on_message: Optional[Callable[[Dict], None]] = None):
        self._on_message = on_message
        self._connected = False
        self._stop = threading.Event()
        self._tx_queue: Queue = Queue()
        self._serial = None
        self._ws = None
        self._lock = threading.Lock()

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    def send(self, payload: Dict[str, Any]) -> bool:
        try:
            normalized = _normalize_command(payload) if isinstance(payload, dict) else {"actuador": str(payload), "estado": 1, "velocidad": 255}
            self._tx_queue.put(normalized)
            return True
        except Exception as e:
            logger.error("Bridge send error: %s", e)
            return False

    def send_action(self, actuador: str, estado: int = 1, velocidad: int = 255, **kwargs) -> bool:
        return self.send({"actuador": actuador, "estado": estado, "velocidad": velocidad, **kwargs})

    def _set_connected(self, value: bool) -> None:
        with self._lock:
            self._connected = value

    def _connect_serial(self) -> bool:
        try:
            import serial
            ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=RX_READ_TIMEOUT)
            with self._lock:
                self._serial = ser
                self._connected = True
            logger.info("Bridge serial OK: %s", SERIAL_PORT)
            return True
        except ImportError:
            logger.warning("pyserial no instalado: pip install pyserial")
            return False
        except Exception as e:
            logger.debug("Serial fail: %s", e)
            return False

    def _connect_ws(self) -> bool:
        try:
            import websocket
            ws = websocket.create_connection(WS_URL, timeout=5)
            with self._lock:
                self._ws = ws
                self._connected = True
            logger.info("Bridge WebSocket OK: %s", WS_URL)
            return True
        except ImportError:
            logger.warning("websocket-client no instalado: pip install websocket-client")
            return False
        except Exception as e:
            logger.debug("WS fail: %s", e)
            return False

    def _connect_http(self) -> bool:
        try:
            req = urllib.request.Request(ROBOT_API + "/status", method="GET")
            with urllib.request.urlopen(req, timeout=3) as r:
                if r.status == 200:
                    self._set_connected(True)
                    return True
        except Exception as e:
            logger.debug("HTTP probe fail: %s", e)
        return False

    def _connect(self) -> bool:
        if BRIDGE_MODE == "serial":
            return self._connect_serial()
        if BRIDGE_MODE == "ws":
            return self._connect_ws()
        if BRIDGE_MODE == "mock":
            self._set_connected(True)
            return True
        return self._connect_http()

    def _disconnect(self) -> None:
        self._set_connected(False)
        with self._lock:
            try:
                if self._serial:
                    try:
                        self._serial.close()
                    except Exception:
                        pass
                    self._serial = None
            except Exception:
                pass
            try:
                if self._ws:
                    try:
                        self._ws.close()
                    except Exception:
                        pass
                    self._ws = None
            except Exception:
                pass

    def _tx_serial(self, data: str) -> bool:
        try:
            with self._lock:
                ser = self._serial
            if not ser or not getattr(ser, "is_open", True):
                return False
            ser.write((data + "\n").encode("utf-8"))
            return True
        except Exception as e:
            logger.debug("Serial TX fail: %s", e)
            self._set_connected(False)
            return False

    def _tx_ws(self, data: str) -> bool:
        try:
            with self._lock:
                ws = self._ws
            if not ws:
                return False
            ws.send(data)
            return True
        except Exception as e:
            logger.debug("WS TX fail: %s", e)
            self._set_connected(False)
            return False

    def _tx_http(self, payload: Dict) -> bool:
        body = json.dumps(payload).encode("utf-8")
        try:
            req = urllib.request.Request(
                ROBOT_API + "/api/command",
                data=body,
                method="POST",
                headers={"Content-Type": "application/json"},
            )
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status == 200
        except urllib.error.HTTPError as e:
            if e.code == 404:
                try:
                    req2 = urllib.request.Request(ROBOT_API + "/command", data=body, method="POST",
                        headers={"Content-Type": "application/json"})
                    with urllib.request.urlopen(req2, timeout=5):
                        return True
                except Exception:
                    pass
            return False
        except Exception as e:
            logger.debug("HTTP TX fail: %s", e)
            self._set_connected(False)
            return False

    def _rx_serial(self) -> None:
        try:
            with self._lock:
                ser = self._serial
            if not ser or not getattr(ser, "is_open", True):
                return
            line = ser.readline()
            if not line:
                return
            raw = line.decode("utf-8", errors="ignore").strip()
            if not raw:
                return
            msg = json.loads(raw)
            if self._on_message and isinstance(msg, dict):
                self._on_message(msg)
        except json.JSONDecodeError:
            pass
        except Exception as e:
            logger.debug("Serial RX fail: %s", e)
            self._set_connected(False)

    def _rx_ws(self) -> None:
        try:
            with self._lock:
                ws = self._ws
            if not ws:
                return
            raw = ws.recv()
            if not raw:
                return
            msg = json.loads(raw)
            if self._on_message and isinstance(msg, dict):
                self._on_message(msg)
        except Exception as e:
            logger.debug("WS RX fail: %s", e)
            self._set_connected(False)

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            if not self.connected:
                time.sleep(RECONNECT_DELAY)
                continue
            try:
                if BRIDGE_MODE == "serial":
                    self._rx_serial()
                elif BRIDGE_MODE == "ws":
                    self._rx_ws()
            except Exception as e:
                logger.debug("Bridge RX error: %s", e)
                self._set_connected(False)
            time.sleep(0.02)

    def _io_loop(self) -> None:
        while not self._stop.is_set():
            if not self.connected:
                try:
                    self._disconnect()
                except Exception:
                    pass
                if not self._connect():
                    time.sleep(RECONNECT_DELAY)
                    continue
            try:
                payload = self._tx_queue.get(timeout=0.1)
                data = json.dumps(payload)
                if BRIDGE_MODE == "serial":
                    ok = self._tx_serial(data)
                elif BRIDGE_MODE == "ws":
                    ok = self._tx_ws(data)
                elif BRIDGE_MODE == "mock":
                    logger.info("Bridge mock TX: %s", data[:120])
                    ok = True
                else:
                    ok = self._tx_http(payload)
                if not ok and BRIDGE_MODE not in ("mock",):
                    self._set_connected(False)
                    self._tx_queue.put(payload)
            except Empty:
                pass
            except Exception as e:
                logger.debug("Bridge TX error: %s", e)
                self._set_connected(False)
            time.sleep(0.02)

    def start(self) -> None:
        self._stop.clear()
        t_tx = threading.Thread(target=self._io_loop, daemon=True)
        t_tx.start()
        if BRIDGE_MODE in ("serial", "ws"):
            t_rx = threading.Thread(target=self._rx_loop, daemon=True)
            t_rx.start()

    def stop(self) -> None:
        self._stop.set()
        self._disconnect()
