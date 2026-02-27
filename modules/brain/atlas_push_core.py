"""AtlasPushCore: Snapshot → Review → Action. Ciclo asíncrono con bridge al hardware."""
from __future__ import annotations

import asyncio
import json
import logging
import os
import threading
import time
from dataclasses import dataclass, field
from queue import Empty, Queue
from typing import Any, Dict, Optional

logger = logging.getLogger("atlas.core")

from .bridge import HardwareBridge

NEXUS_URL = (os.getenv("NEXUS_BASE_URL") or "http://127.0.0.1:8000").rstrip("/")
ROBOT_URL = (os.getenv("NEXUS_ROBOT_API_URL") or "http://127.0.0.1:8002").rstrip("/")
SNAPSHOT_INTERVAL = float(os.getenv("ATLAS_SNAPSHOT_INTERVAL", "2.0"))
ACTION_DEBOUNCE = float(os.getenv("ATLAS_ACTION_DEBOUNCE", "0.5"))


@dataclass
class Snapshot:
    timestamp: float
    sensors: Dict[str, Any] = field(default_factory=dict)
    screen: Optional[Dict] = None
    env: Dict[str, str] = field(default_factory=dict)
    robot_status: Optional[Dict] = None
    nexus_status: Optional[Dict] = None

    def to_dict(self) -> Dict:
        return {
            "timestamp": self.timestamp,
            "sensors": self.sensors,
            "env": self.env,
            "robot_status": self.robot_status,
            "nexus_status": self.nexus_status,
        }


@dataclass
class ActionCommand:
    actuador: str
    estado: int = 1
    velocidad: int = 255
    extra: Dict = field(default_factory=dict)

    def to_dict(self) -> Dict:
        return {"actuador": self.actuador, "estado": self.estado, "velocidad": self.velocidad, **self.extra}


def _fetch_json(url: str, timeout: float = 3) -> Optional[Dict]:
    try:
        import urllib.request
        req = urllib.request.Request(url, method="GET", headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode("utf-8"))
    except Exception:
        return None


def _capture_screen_meta() -> Optional[Dict]:
    try:
        import mss
        with mss.mss() as sct:
            mon = sct.monitors[0]
            return {"width": mon["width"], "height": mon["height"], "capture_ok": True}
    except ImportError:
        return None
    except Exception:
        return None


def capture_snapshot() -> Snapshot:
    ts = time.time()
    env = dict(os.environ) if os.environ else {}
    for k in list(env.keys()):
        if "KEY" in k.upper() or "SECRET" in k.upper() or "PASSWORD" in k.upper():
            env[k] = "***"
    robot = _fetch_json(ROBOT_URL + "/status")
    nexus = _fetch_json(NEXUS_URL + "/status")
    screen = _capture_screen_meta()
    return Snapshot(
        timestamp=ts,
        sensors={},
        screen=screen,
        env={k: str(v)[:80] for k, v in list(env.items())[:20]},
        robot_status=robot,
        nexus_status=nexus,
    )


def review_snapshot(snap: Snapshot) -> Optional[ActionCommand]:
    if snap.robot_status and snap.robot_status.get("status") != "online":
        return ActionCommand(actuador="wake_robot", estado=1, extra={"reason": "robot_offline"})
    if snap.nexus_status is None and NEXUS_URL:
        return ActionCommand(actuador="wake_nexus", estado=1, extra={"reason": "nexus_unreachable"})
    return None


class AtlasPushCore:
    def __init__(self):
        self._bridge = HardwareBridge(on_message=self._on_hardware_message)
        self._action_queue: Queue = Queue()
        self._snapshot_queue: Queue = Queue()
        self._last_action_ts = 0.0
        self._running = False
        self._snapshot_task: Optional[asyncio.Task] = None
        self._action_task: Optional[asyncio.Task] = None

    def _on_hardware_message(self, msg: Dict) -> None:
        try:
            logger.debug("Bridge RX: %s", str(msg)[:100])
        except Exception:
            pass

    def _snapshot_loop(self) -> None:
        while self._running:
            try:
                snap = capture_snapshot()
                self._snapshot_queue.put(snap)
            except Exception as e:
                logger.exception("Snapshot error: %s", e)
            time.sleep(SNAPSHOT_INTERVAL)

    def _review_loop(self) -> None:
        while self._running:
            try:
                snap = self._snapshot_queue.get(timeout=0.5)
                cmd = review_snapshot(snap)
                if cmd:
                    self._action_queue.put(cmd)
            except Empty:
                pass
            except Exception as e:
                logger.debug("Review error: %s", e)

    def _action_loop(self) -> None:
        while self._running:
            try:
                cmd = self._action_queue.get(timeout=0.2)
            except Empty:
                continue
            try:
                if time.time() - self._last_action_ts < ACTION_DEBOUNCE:
                    self._action_queue.put(cmd)
                    continue
                payload = cmd.to_dict() if hasattr(cmd, "to_dict") else cmd
                self._bridge.send(payload if isinstance(payload, dict) else {"actuador": str(cmd)})
                self._last_action_ts = time.time()
            except Exception as e:
                logger.debug("Action error: %s", e)

    def queue_action(self, cmd: ActionCommand) -> None:
        self._action_queue.put(cmd)

    def start(self) -> None:
        self._running = True
        self._bridge.start()
        try:
            snap0 = capture_snapshot()
            self._snapshot_queue.put(snap0)
        except Exception as e:
            logger.debug("Initial snapshot: %s", e)
        self._snap_thread = threading.Thread(target=self._snapshot_loop, daemon=True)
        self._snap_thread.start()
        self._rev_thread = threading.Thread(target=self._review_loop, daemon=True)
        self._rev_thread.start()
        self._act_thread = threading.Thread(target=self._action_loop, daemon=True)
        self._act_thread.start()
        logger.info("AtlasPushCore started: Snapshot(%.1fs) + Review + Action", SNAPSHOT_INTERVAL)

    def stop(self) -> None:
        self._running = False
        self._bridge.stop()
        logger.info("AtlasPushCore stopped")


_core: Optional[AtlasPushCore] = None


def get_core() -> AtlasPushCore:
    global _core
    if _core is None:
        _core = AtlasPushCore()
    return _core
