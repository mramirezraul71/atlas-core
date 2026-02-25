import base64
import io
import subprocess
import time
from datetime import datetime
from pathlib import Path

import pyautogui
from PIL import ImageGrab

SCREENSHOTS_DIR = Path(r"C:\ATLAS_PUSH\workspace_prime\screenshots")
SCREENSHOTS_DIR.mkdir(parents=True, exist_ok=True)

pyautogui.FAILSAFE = True  # Mueve mouse a esquina superior izquierda para detener
pyautogui.PAUSE = 0.5  # Pausa de seguridad entre acciones


class DesktopHands:
    """
    ATLAS-WORKSPACE-PRIME — Manos internas (escritorio)
    Controla el sistema operativo completo:
    mouse, teclado, ventanas, aplicaciones
    """

    # ── OJOS DE ESCRITORIO ──────────────────────────────────────────────
    def screenshot(self, region=None) -> str:
        """Toma screenshot del escritorio completo o región"""
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = str(SCREENSHOTS_DIR / f"desktop_{ts}.png")
        if region:
            img = ImageGrab.grab(bbox=region)
        else:
            img = ImageGrab.grab()
        img.save(path)
        return path

    def screenshot_bytes(self) -> bytes:
        """Screenshot como bytes para VisionEyes"""
        img = ImageGrab.grab()
        buf = io.BytesIO()
        img.save(buf, format="PNG")
        return buf.getvalue()

    def get_screen_size(self) -> tuple:
        return pyautogui.size()

    # ── MOUSE ───────────────────────────────────────────────────────────
    def move_mouse(self, x: int, y: int, duration: float = 0.3) -> dict:
        """Mueve el mouse a coordenadas x,y"""
        pyautogui.moveTo(x, y, duration=duration)
        return {"success": True, "action": f"mouse moved to ({x},{y})"}

    def click(self, x: int = None, y: int = None, button: str = "left") -> dict:
        """Click izquierdo, derecho o doble"""
        if x and y:
            pyautogui.click(x, y, button=button)
        else:
            pyautogui.click(button=button)
        return {"success": True, "action": f"{button} click at ({x},{y})"}

    def double_click(self, x: int, y: int) -> dict:
        pyautogui.doubleClick(x, y)
        return {"success": True, "action": f"double click at ({x},{y})"}

    def right_click(self, x: int, y: int) -> dict:
        pyautogui.rightClick(x, y)
        return {"success": True, "action": f"right click at ({x},{y})"}

    def drag(self, x1: int, y1: int, x2: int, y2: int, duration: float = 0.5) -> dict:
        """Drag desde (x1,y1) hasta (x2,y2)"""
        pyautogui.dragTo(x2, y2, duration=duration, button="left")
        return {"success": True, "action": f"dragged ({x1},{y1}) → ({x2},{y2})"}

    def scroll(self, clicks: int = 3, x: int = None, y: int = None) -> dict:
        """Scroll: positivo = arriba, negativo = abajo"""
        if x and y:
            pyautogui.scroll(clicks, x=x, y=y)
        else:
            pyautogui.scroll(clicks)
        return {"success": True, "action": f"scrolled {clicks}"}

    # ── TECLADO ─────────────────────────────────────────────────────────
    def type_text(self, text: str, interval: float = 0.05) -> dict:
        """Escribe texto con el teclado"""
        pyautogui.typewrite(text, interval=interval)
        return {"success": True, "action": f"typed: {text[:50]}"}

    def press_key(self, key: str) -> dict:
        """Presiona tecla: enter, tab, esc, ctrl, alt, delete, etc"""
        pyautogui.press(key)
        return {"success": True, "action": f"pressed {key}"}

    def hotkey(self, *keys) -> dict:
        """Combinación de teclas: ctrl+c, ctrl+v, alt+tab, etc"""
        pyautogui.hotkey(*keys)
        return {"success": True, "action": f"hotkey: {'+'.join(keys)}"}

    def copy_to_clipboard(self) -> dict:
        """Ctrl+C"""
        return self.hotkey("ctrl", "c")

    def paste_from_clipboard(self) -> dict:
        """Ctrl+V"""
        return self.hotkey("ctrl", "v")

    # ── APLICACIONES ────────────────────────────────────────────────────
    def open_app(self, app_path: str) -> dict:
        """Abre una aplicación"""
        subprocess.Popen(app_path, shell=True)
        time.sleep(2)
        return {"success": True, "action": f"opened {app_path}"}

    def open_url_in_browser(self, url: str) -> dict:
        """Abre URL en el browser por defecto"""
        import webbrowser

        webbrowser.open(url)
        time.sleep(2)
        return {"success": True, "action": f"opened URL: {url}"}

    def close_window(self) -> dict:
        """Cierra ventana activa"""
        return self.hotkey("alt", "F4")

    def switch_window(self) -> dict:
        """Alt+Tab para cambiar ventana"""
        return self.hotkey("alt", "tab")

    def minimize_window(self) -> dict:
        return self.hotkey("win", "down")

    def maximize_window(self) -> dict:
        return self.hotkey("win", "up")

    # ── ACCIONES COMPUESTAS ─────────────────────────────────────────────
    def click_and_type(self, x: int, y: int, text: str) -> dict:
        """Click en un campo y escribe texto"""
        self.click(x, y)
        time.sleep(0.3)
        self.hotkey("ctrl", "a")  # selecciona todo
        self.type_text(text)
        return {"success": True, "action": f"click_and_type at ({x},{y}): {text[:30]}"}

    def find_and_click(self, image_path: str, confidence: float = 0.8) -> dict:
        """Busca imagen en pantalla y hace click en ella"""
        try:
            location = pyautogui.locateOnScreen(image_path, confidence=confidence)
            if location:
                center = pyautogui.center(location)
                self.click(center.x, center.y)
                return {"success": True, "found": True, "x": center.x, "y": center.y}
            return {"success": False, "found": False}
        except Exception as e:
            return {"success": False, "error": str(e)}


if __name__ == "__main__":
    d = DesktopHands()
    size = d.get_screen_size()
    path = d.screenshot()
    print(f"DesktopHands OK - Screen: {size} - Screenshot: {path}")
