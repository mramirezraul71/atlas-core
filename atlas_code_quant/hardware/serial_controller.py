"""Módulo 1D — Serial Controller para ATLAS.

Escaneo continuo de puertos COM, handshake automático con Arduino/ESP32,
y protocolo de comandos bidireccional para actuadores físicos.

Protocolo Serial ATLAS (115200 baud, 8N1):
    TX (PC → Arduino):  "ATLAS:<cmd>:<payload>\n"
    RX (Arduino → PC):  "ATLAS:ACK:<cmd>\n"  o  "ATLAS:DATA:<json>\n"

Comandos definidos:
    PING         → handshake inicial (respuesta: ACK:PING)
    CMD:1        → acción derecha  (motion quadrant right)
    CMD:2        → acción izquierda (motion quadrant left)
    CMD:3        → acción centro
    ESTOP        → parada de emergencia
    STATUS       → solicitar telemetría (respuesta: DATA:{...})

Uso::

    ctrl = ATLASSerialController()
    ctrl.start()          # escaneo continuo de puertos COM
    ctrl.send_command(1)  # envía CMD:1
    ctrl.stop()
"""

from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

import serial
import serial.tools.list_ports

logger = logging.getLogger("atlas.hardware.serial")

BAUD_RATE = 115200
HANDSHAKE_TIMEOUT = 3.0
SCAN_INTERVAL = 2.0
ATLAS_PROTOCOL_PREFIX = "ATLAS:"


@dataclass
class SerialDevice:
    """Dispositivo serial detectado y sincronizado."""
    port: str
    vid: int = 0
    pid: int = 0
    description: str = ""
    is_atlas: bool = False
    connected_at: float = 0.0
    last_ack: float = 0.0
    firmware_version: str = "unknown"


@dataclass
class ActuatorState:
    """Estado de los actuadores reportado por el dispositivo."""
    connected: bool = False
    device_port: str = ""
    last_command: str = ""
    last_command_at: float = 0.0
    telemetry: dict = field(default_factory=dict)
    error_count: int = 0


class ATLASSerialController:
    """Controlador serial con escaneo continuo y handshake automático.

    Detecta automáticamente dispositivos Arduino/ESP32 en puertos COM,
    realiza handshake ATLAS, y mantiene la conexión viva.
    """

    def __init__(
        self,
        baud_rate: int = BAUD_RATE,
        scan_interval: float = SCAN_INTERVAL,
        on_connect: Callable[[SerialDevice], None] | None = None,
        on_disconnect: Callable[[str], None] | None = None,
        on_data: Callable[[dict], None] | None = None,
    ) -> None:
        self._baud = baud_rate
        self._scan_interval = scan_interval
        self._on_connect = on_connect
        self._on_disconnect = on_disconnect
        self._on_data = on_data

        self._conn: Optional[serial.Serial] = None
        self._device: Optional[SerialDevice] = None
        self._state = ActuatorState()
        self._lock = threading.Lock()

        self._running = False
        self._scan_thread: Optional[threading.Thread] = None
        self._read_thread: Optional[threading.Thread] = None

        # Puertos conocidos que NO son Arduino (filtrar ruido)
        self._ignored_ports: set[str] = set()
        self._known_ports: set[str] = set()

    # ── Ciclo de vida ────────────────────────────────────────────────────────

    def start(self) -> None:
        """Inicia escaneo continuo de puertos COM."""
        if self._running:
            return
        self._running = True
        self._scan_thread = threading.Thread(
            target=self._scan_loop, daemon=True, name="atlas-serial-scan"
        )
        self._scan_thread.start()
        logger.info("Serial scanner STARTED (baud=%d, interval=%.1fs)", self._baud, self._scan_interval)

    def stop(self) -> None:
        """Detiene escaneo y cierra conexión."""
        self._running = False
        if self._scan_thread:
            self._scan_thread.join(timeout=5)
        self._disconnect()
        logger.info("Serial controller STOPPED")

    @property
    def connected(self) -> bool:
        return self._conn is not None and self._conn.is_open

    @property
    def state(self) -> ActuatorState:
        with self._lock:
            return ActuatorState(
                connected=self.connected,
                device_port=self._device.port if self._device else "",
                last_command=self._state.last_command,
                last_command_at=self._state.last_command_at,
                telemetry=dict(self._state.telemetry),
                error_count=self._state.error_count,
            )

    @property
    def device(self) -> Optional[SerialDevice]:
        return self._device

    # ── Envío de comandos ────────────────────────────────────────────────────

    def send_command(self, cmd_id: int, payload: str = "") -> bool:
        """Envía comando numérico al dispositivo ATLAS."""
        return self._send(f"CMD:{cmd_id}" + (f":{payload}" if payload else ""))

    def send_estop(self) -> bool:
        """Parada de emergencia."""
        logger.critical("SERIAL ESTOP sent")
        return self._send("ESTOP")

    def request_status(self) -> bool:
        """Solicita telemetría al dispositivo."""
        return self._send("STATUS")

    def _send(self, message: str) -> bool:
        """Envía mensaje con protocolo ATLAS."""
        if not self.connected:
            logger.warning("Serial SEND failed — no device connected. Message: %s", message)
            return False
        try:
            line = f"{ATLAS_PROTOCOL_PREFIX}{message}\n"
            self._conn.write(line.encode("utf-8"))
            self._conn.flush()
            with self._lock:
                self._state.last_command = message
                self._state.last_command_at = time.time()
            logger.info("Serial TX: %s", message)
            return True
        except (serial.SerialException, OSError) as exc:
            logger.error("Serial SEND error: %s", exc)
            with self._lock:
                self._state.error_count += 1
            self._disconnect()
            return False

    # ── Escaneo de puertos ───────────────────────────────────────────────────

    def _scan_loop(self) -> None:
        """Escanea puertos COM continuamente buscando dispositivos ATLAS."""
        while self._running:
            try:
                if not self.connected:
                    self._scan_and_connect()
                else:
                    # Verificar que la conexión sigue viva
                    if not self._conn or not self._conn.is_open:
                        logger.warning("Serial connection lost — rescanning...")
                        self._disconnect()
            except Exception as exc:
                logger.error("Serial scan error: %s", exc)
            time.sleep(self._scan_interval)

    def _scan_and_connect(self) -> None:
        """Detecta puertos nuevos e intenta handshake."""
        ports = serial.tools.list_ports.comports()
        current_ports = {p.device for p in ports}

        # Detectar desconexiones
        disappeared = self._known_ports - current_ports
        if disappeared:
            logger.info("Serial ports disappeared: %s", disappeared)
            self._ignored_ports -= disappeared

        self._known_ports = current_ports

        for port_info in ports:
            port = port_info.device
            if port in self._ignored_ports:
                continue

            # Filtrar puertos conocidos que no son Arduino
            desc_lower = (port_info.description or "").lower()
            # Arduino/ESP32 típicamente tienen estos VIDs
            is_candidate = (
                port_info.vid in (0x2341, 0x1A86, 0x10C4, 0x0403, 0x239A, 0x303A)  # Arduino, CH340, CP210x, FTDI, Adafruit, ESP32-S3
                or "arduino" in desc_lower
                or "ch340" in desc_lower
                or "cp210" in desc_lower
                or "usb serial" in desc_lower
                or "usb-serial" in desc_lower
                or "esp32" in desc_lower
            )

            if not is_candidate:
                self._ignored_ports.add(port)
                logger.debug("Ignoring non-Arduino port: %s (%s)", port, port_info.description)
                continue

            logger.info("Candidate device on %s: %s (VID:0x%04X PID:0x%04X)",
                        port, port_info.description, port_info.vid or 0, port_info.pid or 0)

            if self._try_handshake(port_info):
                return  # Conectado exitosamente

    def _try_handshake(self, port_info) -> bool:
        """Intenta handshake ATLAS con un puerto específico."""
        port = port_info.device
        try:
            conn = serial.Serial(
                port=port,
                baudrate=self._baud,
                timeout=HANDSHAKE_TIMEOUT,
                write_timeout=HANDSHAKE_TIMEOUT,
            )
            # Arduino hace reset al abrir serial — esperar bootloader
            time.sleep(2.0)

            # Limpiar buffer
            conn.reset_input_buffer()

            # Enviar PING
            conn.write(f"{ATLAS_PROTOCOL_PREFIX}PING\n".encode("utf-8"))
            conn.flush()

            # Esperar ACK
            deadline = time.time() + HANDSHAKE_TIMEOUT
            while time.time() < deadline:
                if conn.in_waiting:
                    line = conn.readline().decode("utf-8", errors="replace").strip()
                    if line.startswith(f"{ATLAS_PROTOCOL_PREFIX}ACK:PING"):
                        # Handshake exitoso
                        device = SerialDevice(
                            port=port,
                            vid=port_info.vid or 0,
                            pid=port_info.pid or 0,
                            description=port_info.description or "",
                            is_atlas=True,
                            connected_at=time.time(),
                            last_ack=time.time(),
                        )
                        # Extraer versión de firmware si viene en el ACK
                        # Formato: ATLAS:ACK:PING:v1.0
                        parts = line.split(":")
                        if len(parts) >= 4:
                            device.firmware_version = parts[3]

                        self._conn = conn
                        self._device = device
                        with self._lock:
                            self._state.connected = True
                            self._state.device_port = port
                            self._state.error_count = 0

                        # Iniciar hilo de lectura
                        self._read_thread = threading.Thread(
                            target=self._read_loop, daemon=True, name="atlas-serial-read"
                        )
                        self._read_thread.start()

                        logger.info("ATLAS HANDSHAKE OK on %s — firmware: %s",
                                    port, device.firmware_version)
                        if self._on_connect:
                            self._on_connect(device)
                        return True
                time.sleep(0.05)

            # No respondió — no es ATLAS
            conn.close()
            self._ignored_ports.add(port)
            logger.info("No ATLAS handshake on %s — ignoring", port)
            return False

        except (serial.SerialException, OSError) as exc:
            logger.warning("Cannot open %s: %s", port, exc)
            self._ignored_ports.add(port)
            return False

    # ── Lectura continua ─────────────────────────────────────────────────────

    def _read_loop(self) -> None:
        """Lee mensajes del dispositivo ATLAS continuamente."""
        logger.info("Serial reader started on %s", self._device.port if self._device else "?")
        while self._running and self.connected:
            try:
                if self._conn.in_waiting:
                    raw = self._conn.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    self._process_message(line)
                else:
                    time.sleep(0.01)
            except (serial.SerialException, OSError) as exc:
                logger.error("Serial read error: %s", exc)
                self._disconnect()
                break
            except Exception as exc:
                logger.error("Serial reader unexpected error: %s", exc)

    def _process_message(self, line: str) -> None:
        """Procesa un mensaje recibido del dispositivo."""
        if not line.startswith(ATLAS_PROTOCOL_PREFIX):
            logger.debug("Serial RX (non-ATLAS): %s", line[:100])
            return

        content = line[len(ATLAS_PROTOCOL_PREFIX):]
        logger.info("Serial RX: %s", content)

        if content.startswith("ACK:"):
            cmd = content[4:]
            if self._device:
                self._device.last_ack = time.time()
            logger.debug("ACK received for: %s", cmd)

        elif content.startswith("DATA:"):
            try:
                data = json.loads(content[5:])
                with self._lock:
                    self._state.telemetry = data
                if self._on_data:
                    self._on_data(data)
            except json.JSONDecodeError as exc:
                logger.warning("Serial DATA parse error: %s — raw: %s", exc, content[:100])

        elif content.startswith("ESTOP"):
            logger.critical("ESTOP received from device!")
            # Propagar emergency stop

        elif content.startswith("EVENT:"):
            event_data = content[6:]
            logger.info("Serial EVENT: %s", event_data)

    # ── Desconexión ──────────────────────────────────────────────────────────

    def _disconnect(self) -> None:
        """Cierra la conexión serial de forma segura."""
        port_name = self._device.port if self._device else "?"
        if self._conn:
            try:
                self._conn.close()
            except Exception:
                pass
        self._conn = None
        old_device = self._device
        self._device = None
        with self._lock:
            self._state.connected = False
            self._state.device_port = ""
        if old_device and old_device.is_atlas:
            logger.warning("Serial device disconnected: %s", port_name)
            # No añadir a ignored — permitir reconexión
            self._ignored_ports.discard(port_name)
            if self._on_disconnect:
                self._on_disconnect(port_name)
