from __future__ import annotations

import json
import queue
import threading
import time
from datetime import datetime, timezone
from typing import Any

import requests

try:
    from .config import VisionSensorConfig
except ImportError:  # pragma: no cover
    from config import VisionSensorConfig


def _ts() -> str:
    return datetime.now(timezone.utc).isoformat()


class EventPublisher:
    def __init__(self, config: VisionSensorConfig, logger: Any) -> None:
        self.config = config
        self.logger = logger
        self._pending: "queue.Queue[dict[str, Any]]" = queue.Queue(maxsize=config.queue_maxsize)
        self._stop = threading.Event()
        self._worker: threading.Thread | None = None
        self._local_mode = config.local_mode
        self._queue_file = config.queue_dir / "pending_events.jsonl"
        self._last_recovery_attempt = 0.0

    @property
    def local_mode(self) -> bool:
        return self._local_mode

    def start(self) -> None:
        if self._worker and self._worker.is_alive():
            return
        self._load_persisted_events()
        self._worker = threading.Thread(target=self._run, name="vision-event-publisher", daemon=True)
        self._worker.start()

    def stop(self) -> None:
        self._stop.set()
        if self._worker and self._worker.is_alive():
            self._worker.join(timeout=5.0)
        self.flush()

    def probe_bus(self, retries: int = 5) -> bool:
        for attempt in range(1, retries + 1):
            try:
                response = requests.post(
                    self.config.bus_url,
                    json=self._build_event("heartbeat", "INFO", 1.0, {"probe": True}, "ACTIVE"),
                    timeout=self.config.http_timeout_sec,
                )
                if response.ok:
                    self._local_mode = False
                    self._log_delivery("heartbeat", response.status_code, True, probe=True)
                    return True
                self._log_delivery("heartbeat", response.status_code, False, probe=True, body=response.text[:160])
            except Exception:
                pass
            time.sleep(min(2 ** (attempt - 1), 8))
        self._local_mode = True
        return False

    def emit(
        self,
        event_type: str,
        *,
        severity: str,
        confidence: float,
        payload: dict[str, Any],
        sensor_state: str,
    ) -> None:
        event = self._build_event(event_type, severity, confidence, payload, sensor_state)
        try:
            self._pending.put_nowait(event)
        except queue.Full:
            self._persist_event(event)

    def flush(self) -> None:
        while not self._pending.empty():
            try:
                event = self._pending.get_nowait()
            except queue.Empty:
                return
            if not self._send(event):
                self._persist_event(event)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                event = self._pending.get(timeout=0.5)
            except queue.Empty:
                continue
            if not self._send_with_retry(event):
                self._persist_event(event)

    def _build_event(
        self,
        event_type: str,
        severity: str,
        confidence: float,
        payload: dict[str, Any],
        sensor_state: str,
    ) -> dict[str, Any]:
        return {
            "source": "atlas_vision_sensor",
            "type": event_type,
            "timestamp": _ts(),
            "severity": severity,
            "confidence": round(max(0.0, min(1.0, float(confidence))), 4),
            "payload": payload,
            "sensor_state": sensor_state,
        }

    def _send_with_retry(self, event: dict[str, Any]) -> bool:
        for attempt in range(1, 5):
            if self._send(event):
                return True
            time.sleep(min(2 ** (attempt - 1), 8))
        return False

    def _send(self, event: dict[str, Any]) -> bool:
        if self._local_mode:
            if not self._maybe_recover_bus():
                return False
        try:
            response = requests.post(self.config.bus_url, json=event, timeout=self.config.http_timeout_sec)
            ok = bool(response.ok)
            if ok:
                self._local_mode = False
            self._log_delivery(
                str(event.get("type") or "unknown"),
                response.status_code,
                ok,
                body=response.text[:200],
            )
            return ok
        except Exception as exc:
            self._log_delivery(str(event.get("type") or "unknown"), 0, False, error=str(exc))
            return False

    def _maybe_recover_bus(self) -> bool:
        now = time.monotonic()
        if now - self._last_recovery_attempt < max(2.0, self.config.http_timeout_sec):
            return False
        self._last_recovery_attempt = now
        try:
            response = requests.post(
                self.config.bus_url,
                json=self._build_event("heartbeat", "INFO", 1.0, {"probe": True, "recovery": True}, "ACTIVE"),
                timeout=self.config.http_timeout_sec,
            )
            ok = bool(response.ok)
            self._log_delivery("heartbeat", response.status_code, ok, probe=True, body=response.text[:160])
            if ok:
                self._local_mode = False
                return True
        except Exception as exc:
            self._log_delivery("heartbeat", 0, False, probe=True, error=str(exc))
        return False

    def _persist_event(self, event: dict[str, Any]) -> None:
        self.config.queue_dir.mkdir(parents=True, exist_ok=True)
        with self._queue_file.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(event, ensure_ascii=True) + "\n")

    def _load_persisted_events(self) -> None:
        if not self._queue_file.exists():
            return
        try:
            lines = self._queue_file.read_text(encoding="utf-8").splitlines()
        except Exception:
            return
        self._queue_file.unlink(missing_ok=True)
        for line in lines:
            try:
                event = json.loads(line)
            except Exception:
                continue
            try:
                self._pending.put_nowait(event)
            except queue.Full:
                self._persist_event(event)
                break

    def _log_delivery(
        self,
        event_type: str,
        status_code: int,
        ok: bool,
        *,
        probe: bool = False,
        body: str = "",
        error: str = "",
    ) -> None:
        message = "event published" if ok else "event publish failed"
        level = "info" if ok else "warning"
        payload = {
            "event_type": event_type,
            "status_code": status_code,
            "probe": probe,
            "local_mode": self._local_mode,
        }
        if body:
            payload["body"] = body
        if error:
            payload["error"] = error
        log_fn = getattr(self.logger, level, self.logger.info)
        log_fn(message, extra={"extra_data": payload})
