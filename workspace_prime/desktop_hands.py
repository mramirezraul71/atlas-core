import io
import subprocess
import time
from datetime import datetime
from pathlib import Path

import pyautogui
from PIL import ImageGrab

SCREENSHOTS_DIR = Path(r"C:\ATLAS_PUSH\workspace_prime\screenshots")
SCREENSHOTS_DIR.mkdir(parents=True, exist_ok=True)

pyautogui.FAILSAFE = True
pyautogui.PAUSE = 0.5


class DesktopHands:
    """PyAutoGUI — control directo de mouse y teclado en Windows"""

    def screenshot(self, region=None) -> str:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = str(SCREENSHOTS_DIR / f"desktop_{ts}.png")
        img = ImageGrab.grab(bbox=region) if region else ImageGrab.grab()
        img.save(path)
        return path

    def screenshot_bytes(self) -> bytes:
        img = ImageGrab.grab()
        buf = io.BytesIO()
        img.save(buf, format="PNG")
        return buf.getvalue()

    def get_screen_size(self) -> tuple:
        return pyautogui.size()

    def move_mouse(self, x: int, y: int, duration: float = 0.3) -> dict:
        pyautogui.moveTo(x, y, duration=duration)
        return {"success": True, "action": f"mouse moved to ({x},{y})"}

    def click(self, x: int = None, y: int = None, button: str = "left") -> dict:
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

    def scroll(self, clicks: int = 3, x: int = None, y: int = None) -> dict:
        if x and y:
            pyautogui.scroll(clicks, x=x, y=y)
        else:
            pyautogui.scroll(clicks)
        return {"success": True, "action": f"scrolled {clicks}"}

    def type_text(self, text: str, interval: float = 0.05) -> dict:
        pyautogui.typewrite(text, interval=interval)
        return {"success": True, "action": f"typed: {text[:50]}"}

    def press_key(self, key: str) -> dict:
        pyautogui.press(key)
        return {"success": True, "action": f"pressed {key}"}

    def hotkey(self, *keys) -> dict:
        pyautogui.hotkey(*keys)
        return {"success": True, "action": f"hotkey: {'+'.join(keys)}"}

    def click_and_type(self, x: int, y: int, text: str) -> dict:
        self.click(x, y)
        time.sleep(0.3)
        self.hotkey("ctrl", "a")
        self.type_text(text)
        return {"success": True, "action": f"click_and_type at ({x},{y})"}

    def open_app(self, app_path: str) -> dict:
        subprocess.Popen(app_path, shell=True)
        time.sleep(2)
        return {"success": True, "action": f"opened {app_path}"}

    def switch_window(self) -> dict:
        return self.hotkey("alt", "tab")

    def close_window(self) -> dict:
        return self.hotkey("alt", "F4")


if __name__ == "__main__":
    import sys

    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    d = DesktopHands()
    size = d.get_screen_size()
    path = d.screenshot()
    print(f"OK DesktopHands - Screen: {size} - Screenshot: {path}")
