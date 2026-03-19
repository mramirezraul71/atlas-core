"""
ATLAS Master Operator
Núcleo unificado para ejecución E2E: análisis -> visión -> acción -> verificación.

Conecta:
- Ojos: VisionEyes + snapshots de escritorio/cámara
- Manos desktop: PyAutoGUI (DesktopHands)
- Manos web: Playwright (BrowserHands)
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional


ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_PRIME = ROOT / "workspace_prime"
if str(WORKSPACE_PRIME) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_PRIME))
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from browser_hands import BrowserHands  # noqa: E402
from desktop_hands import DesktopHands  # noqa: E402
from vision_eyes import VisionEyes  # noqa: E402

try:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


LOG_DIR = ROOT / "logs" / "atlas_master"
LOG_DIR.mkdir(parents=True, exist_ok=True)


def _ts() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _json_dump(obj: Any) -> str:
    return json.dumps(obj, ensure_ascii=False, indent=2, default=str)


@dataclass
class StepResult:
    ok: bool
    step: str
    data: Dict[str, Any]


class AtlasMasterOperator:
    def __init__(
        self,
        atlas_base: str = "http://127.0.0.1:8791",
        headless_browser: bool = False,
        camera_index: int = 2,
    ):
        self.atlas_base = atlas_base.rstrip("/")
        self.headless_browser = headless_browser
        self.camera_index = int(camera_index)
        self.eyes = VisionEyes()
        self.desktop = DesktopHands()
        self.browser = BrowserHands(headless=headless_browser)
        self.browser_started = False

    async def start(self) -> None:
        if not self.browser_started:
            await self.browser.start()
            self.browser_started = True

    async def stop(self) -> None:
        if self.browser_started:
            await self.browser.stop()
            self.browser_started = False

    def online_banner(self) -> str:
        return "ATLAS en línea. Esperando objetivos de operación."

    def status(self) -> Dict[str, Any]:
        import importlib.util

        deps = {
            "pyautogui": bool(importlib.util.find_spec("pyautogui")),
            "pynput": bool(importlib.util.find_spec("pynput")),
            "cv2": bool(importlib.util.find_spec("cv2")),
            "playwright": bool(importlib.util.find_spec("playwright")),
        }
        return {
            "ok": True,
            "agent": "atlas_master_operator",
            "deps": deps,
            "atlas_base": self.atlas_base,
            "headless_browser": self.headless_browser,
            "camera_index": self.camera_index,
        }

    def _save_bytes(self, name: str, payload: bytes, ext: str = "bin") -> str:
        path = LOG_DIR / f"{name}_{_ts()}.{ext}"
        path.write_bytes(payload)
        return str(path)

    def vision_snapshot(self, source: str = "desktop") -> Dict[str, Any]:
        src = (source or "desktop").strip().lower()
        if src == "desktop":
            data = self.desktop.screenshot_bytes()
            out = self._save_bytes("desktop_snapshot", data, ext="png")
            return {"ok": True, "source": "desktop", "path": out}

        if src == "camera":
            import requests

            url = (
                f"{self.atlas_base}/cuerpo/vision/snapshot"
                f"?enhance=auto&jpeg_quality=80&index={self.camera_index}"
            )
            r = requests.get(url, timeout=25)
            r.raise_for_status()
            if "image" not in (r.headers.get("Content-Type") or "").lower():
                return {"ok": False, "source": "camera", "error": "camera_snapshot_not_image"}
            out = self._save_bytes("camera_snapshot", r.content, ext="jpg")
            return {"ok": True, "source": "camera", "path": out}

        return {"ok": False, "error": f"source_not_supported:{src}"}

    def analyze(self, objective: str, source: str = "desktop") -> Dict[str, Any]:
        snap = self.vision_snapshot(source=source)
        if not snap.get("ok"):
            return {"ok": False, "error": snap.get("error", "snapshot_failed")}

        path = snap.get("path")
        out = self.eyes.analyze_image(
            path,
            f"Objetivo operativo: {objective}. Describe estado actual y próxima acción recomendada.",
        )
        return {
            "ok": bool(out.get("success")),
            "analysis": out.get("analysis", ""),
            "engine": out.get("engine", "unknown"),
            "snapshot": path,
        }

    async def web_operate(self, url: str, objective: str) -> Dict[str, Any]:
        await self.start()
        nav = await self.browser.navigate(url)
        before = self.eyes.analyze_screenshot_bytes(
            await self.browser.screenshot_bytes(),
            f"Analiza la página y ubica el objetivo: {objective}",
        )
        return {
            "ok": bool(nav.get("success")),
            "url": nav.get("url"),
            "title": nav.get("title"),
            "before_analysis": before.get("analysis", ""),
            "before_screenshot": nav.get("screenshot_path"),
        }

    def desktop_operate(self, objective: str) -> Dict[str, Any]:
        analysis = self.analyze(objective=objective, source="desktop")
        coords = self.eyes.find_element_coordinates(
            self.desktop.screenshot_bytes(),
            f"Elemento principal para ejecutar: {objective}",
        )
        return {
            "ok": analysis.get("ok", False),
            "objective": objective,
            "analysis": analysis.get("analysis", ""),
            "suggested_coordinates": coords,
            "evidence": analysis.get("snapshot"),
        }

    def action_mouse(self, x: int, y: int, click: bool = True) -> Dict[str, Any]:
        move = self.desktop.move_mouse(int(x), int(y), duration=0.2)
        if click:
            clk = self.desktop.click(int(x), int(y))
            return {"ok": True, "move": move, "click": clk}
        return {"ok": True, "move": move}

    def action_keyboard(self, text: Optional[str] = None, hotkey: Optional[str] = None) -> Dict[str, Any]:
        if hotkey:
            keys = [k.strip() for k in hotkey.split("+") if k.strip()]
            return {"ok": True, "result": self.desktop.hotkey(*keys)}
        if text is not None:
            return {"ok": True, "result": self.desktop.type_text(text)}
        return {"ok": False, "error": "no_keyboard_action"}

    def verify(self, question: str, source: str = "desktop") -> Dict[str, Any]:
        snap = self.vision_snapshot(source=source)
        if not snap.get("ok"):
            return {"ok": False, "error": snap.get("error", "verify_snapshot_failed")}
        out = self.eyes.analyze_image(snap["path"], question)
        return {
            "ok": bool(out.get("success")),
            "analysis": out.get("analysis", ""),
            "snapshot": snap["path"],
        }


async def _run(args: argparse.Namespace) -> int:
    op = AtlasMasterOperator(
        atlas_base=args.atlas_base,
        headless_browser=args.headless,
        camera_index=args.camera_index,
    )
    try:
        if args.online:
            print(op.online_banner())
            return 0

        if args.status:
            print(_json_dump(op.status()))
            return 0

        if args.objective and args.url:
            out = await op.web_operate(args.url, args.objective)
            print(_json_dump(out))
            return 0 if out.get("ok") else 2

        if args.objective:
            out = op.desktop_operate(args.objective)
            print(_json_dump(out))
            return 0 if out.get("ok") else 2

        if args.snapshot_source:
            out = op.vision_snapshot(source=args.snapshot_source)
            print(_json_dump(out))
            return 0 if out.get("ok") else 2

        if args.mouse is not None:
            x, y = args.mouse
            out = op.action_mouse(x=x, y=y, click=not args.move_only)
            print(_json_dump(out))
            return 0 if out.get("ok") else 2

        if args.type_text is not None or args.hotkey is not None:
            out = op.action_keyboard(text=args.type_text, hotkey=args.hotkey)
            print(_json_dump(out))
            return 0 if out.get("ok") else 2

        if args.verify:
            out = op.verify(args.verify, source=args.verify_source)
            print(_json_dump(out))
            return 0 if out.get("ok") else 2

        print(op.online_banner())
        return 0
    finally:
        await op.stop()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="ATLAS Master Operator - Ojos + Manos + Navegación + Verificación"
    )
    p.add_argument("--atlas-base", default=os.getenv("ATLAS_PUSH_BASE", "http://127.0.0.1:8791"))
    p.add_argument("--camera-index", type=int, default=int(os.getenv("ATLAS_CAMERA_INDEX", "2")))
    p.add_argument("--headless", action="store_true", help="Playwright headless")
    p.add_argument("--online", action="store_true", help="Imprime banner operativo")
    p.add_argument("--status", action="store_true", help="Estado de dependencias")
    p.add_argument("--objective", help="Objetivo operativo en lenguaje natural")
    p.add_argument("--url", help="URL objetivo (si es tarea web)")
    p.add_argument("--snapshot-source", choices=["desktop", "camera"], help="Tomar snapshot")
    p.add_argument("--mouse", nargs=2, type=int, metavar=("X", "Y"), help="Mover/click mouse")
    p.add_argument("--move-only", action="store_true", help="Con --mouse, no hacer click")
    p.add_argument("--type-text", help="Escribir texto con teclado")
    p.add_argument("--hotkey", help="Atajo teclado, ejemplo ctrl+shift+i")
    p.add_argument("--verify", help="Pregunta de verificación visual")
    p.add_argument("--verify-source", default="desktop", choices=["desktop", "camera"])
    return p


if __name__ == "__main__":
    parser = build_parser()
    ns = parser.parse_args()
    raise SystemExit(asyncio.run(_run(ns)))
