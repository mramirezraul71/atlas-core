from __future__ import annotations

import signal
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any
import json
import os
import ctypes

_SENSOR_DIR = Path(__file__).resolve().parent
_WORKSPACE_ROOT = _SENSOR_DIR.parent.parent
for _path in (str(_SENSOR_DIR), str(_WORKSPACE_ROOT)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

try:
    from .candle_detector import CandleDetector
    from .camera import CameraService
    from .config import VisionSensorConfig
    from .event_publisher import EventPublisher
    from .forensics import ForensicRecorder
    from .gesture_detector import GestureDetector
    from .logger import configure_logging, log_event
    from .multimodal_fusion import MultimodalFusion
    from .pattern_analyzer import PatternAnalyzer
    from .preprocessor import FramePreprocessor
    from .security import SecurityMonitor
    from .watchdog import SensorWatchdog
    from .geometry_detector import GeometryDetector
except ImportError:  # pragma: no cover
    from candle_detector import CandleDetector
    from camera import CameraService
    from config import VisionSensorConfig
    from event_publisher import EventPublisher
    from forensics import ForensicRecorder
    from gesture_detector import GestureDetector
    from logger import configure_logging, log_event
    from multimodal_fusion import MultimodalFusion
    from pattern_analyzer import PatternAnalyzer
    from preprocessor import FramePreprocessor
    from security import SecurityMonitor
    from watchdog import SensorWatchdog
    from geometry_detector import GeometryDetector


class SensorInstanceLock:
    def __init__(self, lock_path: Path) -> None:
        self.lock_path = lock_path
        self._fd: int | None = None

    def acquire(self) -> tuple[bool, int | None]:
        self.lock_path.parent.mkdir(parents=True, exist_ok=True)
        while True:
            try:
                self._fd = os.open(str(self.lock_path), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
                payload = {"pid": os.getpid(), "started_at": time.time()}
                os.write(self._fd, json.dumps(payload, ensure_ascii=True).encode("utf-8"))
                os.fsync(self._fd)
                return True, None
            except FileExistsError:
                existing_pid = self._read_pid()
                if existing_pid and self._pid_is_alive(existing_pid):
                    return False, existing_pid
                try:
                    self.lock_path.unlink(missing_ok=True)
                except Exception:
                    return False, existing_pid

    def release(self) -> None:
        try:
            if self._fd is not None:
                os.close(self._fd)
        except Exception:
            pass
        self._fd = None
        try:
            self.lock_path.unlink(missing_ok=True)
        except Exception:
            pass

    def _read_pid(self) -> int | None:
        try:
            payload = json.loads(self.lock_path.read_text(encoding="utf-8"))
        except Exception:
            return None
        try:
            return int(payload.get("pid"))
        except Exception:
            return None

    @staticmethod
    def _pid_is_alive(pid: int) -> bool:
        if pid <= 0:
            return False
        if os.name == "nt":
            try:
                kernel32 = ctypes.windll.kernel32
                process = kernel32.OpenProcess(0x1000, False, pid)
                if process:
                    kernel32.CloseHandle(process)
                    return True
                return False
            except Exception:
                return False
        try:
            os.kill(pid, 0)
        except OSError:
            return False
        return True


def run() -> int:
    logger = configure_logging(Path(__file__).resolve().parent.parent / "logs" / "vision_sensor")
    config = VisionSensorConfig.load()
    log_event(logger, "INFO", "atlas_vision_sensor booting", sensor_dir=str(config.sensor_dir))
    instance_lock = SensorInstanceLock(config.root_dir / "runtime" / "vision_sensor.lock")
    acquired, owner_pid = instance_lock.acquire()
    if not acquired:
        log_event(
            logger,
            "WARNING",
            "vision sensor instance already running; aborting duplicate start",
            owner_pid=owner_pid,
        )
        return 2

    publisher: EventPublisher | None = None
    camera: CameraService | None = None
    watchdog: SensorWatchdog | None = None
    security: SecurityMonitor | None = None
    gesture: GestureDetector | None = None

    try:
        publisher = EventPublisher(config, logger)
        publisher.start()
        bus_ok = publisher.probe_bus(retries=5)
        if not bus_ok:
            log_event(logger, "WARNING", "bus unavailable, continuing in local mode", bus_url=config.bus_url)

        camera = CameraService(config, publisher, logger)
        camera_status = camera.start()
        if not camera_status.connected:
            log_event(logger, "WARNING", "camera unavailable at startup; entering degraded mode")

        watchdog = SensorWatchdog(config, camera, publisher, logger)
        watchdog.start()
        security = SecurityMonitor(config, publisher, logger)
        security.start()
        gesture = GestureDetector(config, publisher, logger)
        gesture.start()
        preprocessor = FramePreprocessor(config)
        analyzer = PatternAnalyzer()
        candle_detector = CandleDetector()
        geometry_detector = GeometryDetector()
        fusion = MultimodalFusion(config, logger)
        forensics = ForensicRecorder(config, logger)

        stop = {"requested": False}

        def _shutdown(*_: Any) -> None:
            stop["requested"] = True

        signal.signal(signal.SIGINT, _shutdown)
        signal.signal(signal.SIGTERM, _shutdown)

        frame_interval = 1.0 / max(1, config.process_fps)
        last_emit = 0.0

        while not stop["requested"]:
            try:
                raw_frame = camera.get_frame()
                if raw_frame is None:
                    time.sleep(frame_interval)
                    continue
                forensics.observe_frame(raw_frame)
                watchdog.submit(raw_frame)
                security.submit(raw_frame)
                gesture.submit(raw_frame)
                processed = preprocessor.process(raw_frame)
                if processed is None:
                    time.sleep(frame_interval)
                    continue
                _ = security.apply_privacy_mask(raw_frame)
                result = analyzer.analyze(processed, raw_frame=raw_frame)
                telemetry_snapshot = fusion.snapshot()
                candle = candle_detector.detect(
                    telemetry_snapshot,
                    visual_bias=str((result.payload if result else {}).get("chart_bias") or "unknown"),
                )
                geometry = geometry_detector.detect(processed)
                now = time.monotonic()
                if result is not None and now - last_emit >= config.event_emit_interval_sec:
                    fused = fusion.fuse(
                        visual_result={
                            "pattern": result.pattern,
                            "confidence": result.confidence,
                            "payload": result.payload,
                        },
                        candle_result={
                            "confidence": candle.confidence,
                            "payload": candle.payload,
                            "hits": [asdict(hit) for hit in candle.hits],
                        },
                        geometry_result={
                            "confidence": geometry.confidence,
                            "payload": geometry.payload,
                            "hits": [asdict(hit) for hit in geometry.hits],
                        },
                        telemetry_snapshot=telemetry_snapshot,
                    )
                    pattern_payload = {
                        "pattern": result.pattern,
                        "visual_confidence": result.confidence,
                        "visual": result.payload,
                        "candles": {
                            "available": candle.available,
                            "confidence": candle.confidence,
                            "reason": candle.reason,
                            "hits": [asdict(hit) for hit in candle.hits],
                            "payload": candle.payload,
                        },
                        "geometry": {
                            "confidence": geometry.confidence,
                            "hits": [asdict(hit) for hit in geometry.hits],
                            "payload": geometry.payload,
                        },
                    }
                    publisher.emit(
                        "pattern_detected",
                        severity="INFO",
                        confidence=max(result.confidence, candle.confidence, geometry.confidence),
                        payload=pattern_payload,
                        sensor_state=fused.action.upper(),
                    )
                    visual_payload = {
                        "action": fused.action,
                        "confidence_pct": round(fused.confidence * 100.0, 2),
                        "primary_source": fused.primary_source,
                        "agreement_count": fused.agreement_count,
                        "contradictions": fused.contradictions,
                        "reasons": fused.reasons,
                        "visual": result.payload,
                        "candles": candle.payload,
                        "geometry": geometry.payload,
                        "telemetry": telemetry_snapshot,
                        **fused.payload,
                    }
                    artifact = forensics.maybe_capture(
                        event_type="visual_signal",
                        severity="HIGH" if fused.action == "degrade" and fused.contradictions else "INFO",
                        payload=visual_payload,
                        sensor_state=fused.action.upper(),
                        current_frame=raw_frame,
                    )
                    if artifact:
                        visual_payload["forensic_artifact"] = artifact
                    publisher.emit(
                        "visual_signal",
                        severity="WARNING" if fused.action == "degrade" else "INFO",
                        confidence=fused.confidence,
                        payload=visual_payload,
                        sensor_state=fused.action.upper(),
                    )
                    last_emit = now
                time.sleep(frame_interval)
            except KeyboardInterrupt:
                stop["requested"] = True
            except Exception as exc:
                log_event(logger, "ERROR", "main loop exception", error=str(exc))
                time.sleep(0.5)

        log_event(logger, "INFO", "atlas_vision_sensor stopped cleanly")
        return 0
    finally:
        if gesture:
            gesture.stop()
        if security:
            security.stop()
        if watchdog:
            watchdog.stop()
        if camera:
            camera.stop()
        if publisher:
            publisher.flush()
            publisher.stop()
        instance_lock.release()


if __name__ == "__main__":
    raise SystemExit(run())
