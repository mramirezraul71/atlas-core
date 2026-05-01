"""Módulo 1B — HID Control Interface (Mouse + Teclado).

Fallback físico cuando la API de Tradier falla:
  - pyautogui para mover mouse / hacer click / teclear órdenes
  - pynput para monitorear teclas de emergencia (Esc = emergency stop)
  - TradierWebController encapsula flujo de orden en la web de Tradier

Seguridad: todas las acciones requieren confirmación por flag `armed=True`.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

logger = logging.getLogger("atlas.hardware.control")

try:
    import pyautogui
    pyautogui.FAILSAFE = True       # mover mouse a esquina = abort
    pyautogui.PAUSE = 0.05         # pausa entre acciones (s)
    _PYAUTOGUI_OK = True
except ImportError:
    _PYAUTOGUI_OK = False
    logger.warning("pyautogui no disponible — control HID deshabilitado")

try:
    from pynput import keyboard as _kb
    _PYNPUT_OK = True
except ImportError:
    _PYNPUT_OK = False
    logger.warning("pynput no disponible — hotkeys deshabilitados")


class OrderSide(str, Enum):
    BUY = "buy"
    SELL = "sell"


@dataclass
class HIDOrder:
    """Parámetros de una orden a ejecutar vía control físico."""
    symbol: str
    side: OrderSide
    quantity: int
    order_type: str = "market"      # market | limit
    limit_price: Optional[float] = None
    reason: str = "api_fallback"


class HIDController:
    """Controlador de mouse y teclado para fallback físico.

    Uso::

        ctrl = HIDController(armed=False)   # modo seco por defecto
        ctrl.start_hotkey_listener()
        ctrl.type_text("AAPL")              # no hace nada si armed=False
    """

    def __init__(self, armed: bool = False, emergency_callback: Callable | None = None) -> None:
        self.armed = armed
        self._emergency_callback = emergency_callback
        self._listener: Optional[object] = None
        self._emergency_active = False
        self._action_log: list[dict] = []
        self._lock = threading.Lock()

    # ── Gestión de emergencia ─────────────────────────────────────────────────

    def start_hotkey_listener(self) -> None:
        """Escucha Esc para emergency stop y F12 para arm/disarm."""
        if not _PYNPUT_OK:
            return

        def on_press(key: object) -> None:
            try:
                if key == _kb.Key.esc:  # type: ignore[attr-defined]
                    self._trigger_emergency("tecla Esc pulsada")
                elif key == _kb.Key.f12:  # type: ignore[attr-defined]
                    self.armed = not self.armed
                    logger.warning("HID armado: %s", self.armed)
            except Exception:
                pass

        self._listener = _kb.Listener(on_press=on_press)  # type: ignore[attr-defined]
        self._listener.start()  # type: ignore[attr-defined]
        logger.info("Listener de hotkeys iniciado (Esc=STOP, F12=ARM/DISARM)")

    def _trigger_emergency(self, reason: str = "") -> None:
        if not self._emergency_active:
            self._emergency_active = True
            logger.critical("EMERGENCY STOP HID: %s", reason)
            if self._emergency_callback:
                self._emergency_callback(reason)

    def stop_hotkey_listener(self) -> None:
        if self._listener:
            self._listener.stop()  # type: ignore[attr-defined]

    # ── Primitivas de control ─────────────────────────────────────────────────

    def move_to(self, x: int, y: int, duration: float = 0.3) -> bool:
        if not self._check_armed("move_to"):
            return False
        pyautogui.moveTo(x, y, duration=duration)  # type: ignore[attr-defined]
        self._log("move_to", {"x": x, "y": y})
        return True

    def click(self, x: int | None = None, y: int | None = None,
              button: str = "left", clicks: int = 1) -> bool:
        if not self._check_armed("click"):
            return False
        if x is not None and y is not None:
            pyautogui.click(x, y, clicks=clicks, button=button)  # type: ignore[attr-defined]
        else:
            pyautogui.click(clicks=clicks, button=button)  # type: ignore[attr-defined]
        self._log("click", {"x": x, "y": y, "button": button})
        return True

    def type_text(self, text: str, interval: float = 0.04) -> bool:
        if not self._check_armed("type_text"):
            return False
        pyautogui.typewrite(text, interval=interval)  # type: ignore[attr-defined]
        self._log("type_text", {"text": text[:50]})
        return True

    def hotkey(self, *keys: str) -> bool:
        if not self._check_armed("hotkey"):
            return False
        pyautogui.hotkey(*keys)  # type: ignore[attr-defined]
        self._log("hotkey", {"keys": keys})
        return True

    def screenshot_region(self, x: int, y: int, w: int, h: int) -> Optional[object]:
        """Captura pantalla de una región para verificación OCR post-orden."""
        if not _PYAUTOGUI_OK:
            return None
        return pyautogui.screenshot(region=(x, y, w, h))  # type: ignore[attr-defined]

    # ── Validación ────────────────────────────────────────────────────────────

    def _check_armed(self, action: str) -> bool:
        if not _PYAUTOGUI_OK:
            logger.warning("HID BLOCKED — pyautogui not installed. Action: %s", action)
            return False
        if self._emergency_active:
            logger.error("HID BLOCKED — emergency stop active. Action: %s", action)
            return False
        if not self.armed:
            logger.info("HID DRY-RUN — armed=False, action NOT executed: %s (arm with F12 or armed=True)", action)
            return False
        return True

    def _log(self, action: str, params: dict) -> None:
        entry = {"ts": time.time(), "action": action, **params}
        with self._lock:
            self._action_log.append(entry)
            if len(self._action_log) > 500:
                self._action_log.pop(0)
        logger.info("HID [%s] %s", action, params)

    def action_history(self) -> list[dict]:
        with self._lock:
            return list(self._action_log)


class TradierWebController:
    """Ejecuta órdenes en la interfaz web de Tradier usando el HIDController.

    Coordenadas de pantalla: se deben calibrar según resolución de monitor del Jetson.
    El archivo calibration.json guarda las coordenadas actualizables sin recompilar.
    """

    # Coordenadas default (1080p) — ajustar con calibración
    DEFAULT_COORDS = {
        "search_bar":    (960,  60),
        "symbol_field":  (300, 200),
        "qty_field":     (500, 200),
        "buy_btn":       (700, 400),
        "sell_btn":      (800, 400),
        "confirm_btn":   (960, 550),
        "order_type":    (450, 250),
    }

    def __init__(
        self,
        hid: HIDController,
        calibration_path: str = "config/hid_calibration.json",
    ) -> None:
        self.hid = hid
        self._coords = dict(self.DEFAULT_COORDS)
        self._load_calibration(calibration_path)

    def _load_calibration(self, path: str) -> None:
        import json
        from pathlib import Path
        p = Path(path)
        if p.exists():
            try:
                data = json.loads(p.read_text())
                self._coords.update(data)
                logger.info("Calibración HID cargada: %s", path)
            except Exception as exc:
                logger.warning("Error cargando calibración HID: %s", exc)

    def save_calibration(self, path: str = "config/hid_calibration.json") -> None:
        import json
        from pathlib import Path
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(self._coords, indent=2))
        logger.info("Calibración HID guardada: %s", path)

    def execute_order(self, order: HIDOrder) -> bool:
        """Flujo completo de orden en la web de Tradier vía mouse/teclado."""
        logger.warning(
            "FALLBACK HID: %s %d %s (armed=%s)",
            order.side.value, order.quantity, order.symbol, self.hid.armed
        )

        # 1. Abrir ticket de orden (Ctrl+T = nueva orden en muchas plataformas)
        if not self.hid.hotkey("ctrl", "t"):
            return False
        time.sleep(0.5)

        # 2. Escribir símbolo
        sx, sy = self._coords["symbol_field"]
        self.hid.click(sx, sy)
        self.hid.hotkey("ctrl", "a")
        self.hid.type_text(order.symbol)
        time.sleep(0.3)

        # 3. Escribir cantidad
        qx, qy = self._coords["qty_field"]
        self.hid.click(qx, qy)
        self.hid.hotkey("ctrl", "a")
        self.hid.type_text(str(order.quantity))

        # 4. Precio límite (si aplica)
        if order.order_type == "limit" and order.limit_price:
            # Seleccionar tipo de orden
            ox, oy = self._coords["order_type"]
            self.hid.click(ox, oy)
            time.sleep(0.2)
            self.hid.type_text("LIMIT")
            self.hid.hotkey("enter")
            # Campo de precio
            self.hid.type_text(f"{order.limit_price:.2f}")

        # 5. Click en BUY o SELL
        side_key = "buy_btn" if order.side == OrderSide.BUY else "sell_btn"
        bx, by = self._coords[side_key]
        self.hid.click(bx, by)
        time.sleep(0.4)

        # 6. Confirmar
        cx, cy = self._coords["confirm_btn"]
        self.hid.click(cx, cy)
        time.sleep(1.0)

        logger.info(
            "Orden HID enviada: %s %d %s @ %s",
            order.side.value, order.quantity, order.symbol,
            order.limit_price or "MARKET"
        )
        return True

    def calibrate_interactive(self) -> None:
        """Modo interactivo: el usuario mueve el mouse a cada posición y presiona Enter."""
        if not _PYAUTOGUI_OK:
            print("pyautogui no disponible")
            return

        print("=== Calibración HID TradierWebController ===")
        for key in self._coords:
            input(f"  Mueve el mouse a [{key}] y presiona Enter...")
            pos = pyautogui.position()  # type: ignore[attr-defined]
            self._coords[key] = (pos.x, pos.y)
            print(f"    → ({pos.x}, {pos.y}) guardado")

        self.save_calibration()
        print("Calibración completa ✓")
