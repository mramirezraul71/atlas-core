from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path


def _truthy(value: str | None, default: bool = False) -> bool:
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "y", "on"}


@dataclass(slots=True)
class VisionSensorConfig:
    root_dir: Path
    sensor_dir: Path
    logs_dir: Path
    queue_dir: Path
    forensic_dir: Path
    bus_url: str
    push_health_url: str
    quant_status_urls: tuple[str, ...]
    quant_state_path: Path
    heartbeat_interval_sec: int
    watchdog_interval_sec: int
    freeze_timeout_sec: int
    camera_indexes: tuple[int, ...]
    capture_width: int
    capture_height: int
    process_width: int
    process_height: int
    process_fps: int
    mediapipe_min_confidence: float
    emergency_hold_sec: float
    emergency_min_confidence: float
    people_confidence_threshold: float
    privacy_mask_enabled: bool
    local_mode: bool
    http_timeout_sec: float
    telemetry_cache_ttl_sec: float
    telemetry_timeout_sec: float
    event_emit_interval_sec: float
    forensic_buffer_sec: int
    forensic_post_event_sec: int
    forensic_max_artifacts: int
    forensic_video_fps: int
    queue_maxsize: int

    @classmethod
    def load(cls) -> "VisionSensorConfig":
        sensor_dir = Path(__file__).resolve().parent
        root_dir = sensor_dir.parent
        logs_dir = root_dir / "logs" / "vision_sensor"
        queue_dir = root_dir / "runtime" / "vision_sensor_queue"
        forensic_dir = root_dir / "runtime" / "vision_sensor_forensics"
        quant_state_path = root_dir.parent / "atlas_code_quant" / "data" / "operation" / "operation_center_state.json"
        logs_dir.mkdir(parents=True, exist_ok=True)
        queue_dir.mkdir(parents=True, exist_ok=True)
        forensic_dir.mkdir(parents=True, exist_ok=True)
        quant_urls_raw = os.getenv(
            "ATLAS_VISION_SENSOR_QUANT_STATUS_URLS",
            "http://127.0.0.1:8795/api/v2/quant/operation/status/lite,http://127.0.0.1:8795/operation/status/lite",
        )
        quant_status_urls = tuple(
            url.strip().rstrip("/") for url in quant_urls_raw.split(",") if url.strip()
        )
        return cls(
            root_dir=root_dir,
            sensor_dir=sensor_dir,
            logs_dir=logs_dir,
            queue_dir=queue_dir,
            forensic_dir=forensic_dir,
            bus_url=os.getenv("ATLAS_VISION_SENSOR_BUS_URL", "http://127.0.0.1:8791/events").strip(),
            push_health_url=os.getenv("ATLAS_VISION_SENSOR_PUSH_HEALTH_URL", "http://127.0.0.1:8791/health").strip(),
            quant_status_urls=quant_status_urls,
            quant_state_path=Path(os.getenv("ATLAS_VISION_SENSOR_QUANT_STATE_PATH", str(quant_state_path))),
            heartbeat_interval_sec=max(5, int(os.getenv("ATLAS_VISION_SENSOR_HEARTBEAT_SEC", "30"))),
            watchdog_interval_sec=max(1, int(os.getenv("ATLAS_VISION_SENSOR_WATCHDOG_SEC", "5"))),
            freeze_timeout_sec=max(30, int(os.getenv("ATLAS_VISION_SENSOR_FREEZE_SEC", "120"))),
            camera_indexes=(0, 1, 2),
            capture_width=max(320, int(os.getenv("ATLAS_VISION_SENSOR_CAPTURE_W", "1920"))),
            capture_height=max(240, int(os.getenv("ATLAS_VISION_SENSOR_CAPTURE_H", "1080"))),
            process_width=max(320, int(os.getenv("ATLAS_VISION_SENSOR_PROCESS_W", "1280"))),
            process_height=max(240, int(os.getenv("ATLAS_VISION_SENSOR_PROCESS_H", "720"))),
            process_fps=max(1, int(os.getenv("ATLAS_VISION_SENSOR_PROCESS_FPS", "10"))),
            mediapipe_min_confidence=max(0.1, min(1.0, float(os.getenv("ATLAS_VISION_SENSOR_MP_CONF", "0.85")))),
            emergency_hold_sec=max(0.5, float(os.getenv("ATLAS_VISION_SENSOR_GESTURE_HOLD_SEC", "2.0"))),
            emergency_min_confidence=max(0.1, min(1.0, float(os.getenv("ATLAS_VISION_SENSOR_GESTURE_MIN_CONF", "0.85")))),
            people_confidence_threshold=max(0.0, min(1.0, float(os.getenv("ATLAS_VISION_SENSOR_PERSON_CONF", "0.5")))),
            privacy_mask_enabled=_truthy(os.getenv("ATLAS_VISION_SENSOR_PRIVACY_MASK"), True),
            local_mode=_truthy(os.getenv("ATLAS_VISION_SENSOR_LOCAL_MODE"), False),
            http_timeout_sec=max(1.0, float(os.getenv("ATLAS_VISION_SENSOR_HTTP_TIMEOUT_SEC", "4.0"))),
            telemetry_cache_ttl_sec=max(0.5, float(os.getenv("ATLAS_VISION_SENSOR_TELEMETRY_CACHE_TTL_SEC", "2.0"))),
            telemetry_timeout_sec=max(0.2, float(os.getenv("ATLAS_VISION_SENSOR_TELEMETRY_TIMEOUT_SEC", "1.5"))),
            event_emit_interval_sec=max(0.2, float(os.getenv("ATLAS_VISION_SENSOR_EVENT_EMIT_INTERVAL_SEC", "1.0"))),
            forensic_buffer_sec=max(5, int(os.getenv("ATLAS_VISION_SENSOR_FORENSIC_BUFFER_SEC", "15"))),
            forensic_post_event_sec=max(5, int(os.getenv("ATLAS_VISION_SENSOR_FORENSIC_POST_SEC", "15"))),
            forensic_max_artifacts=max(4, int(os.getenv("ATLAS_VISION_SENSOR_FORENSIC_MAX_ARTIFACTS", "50"))),
            forensic_video_fps=max(1, int(os.getenv("ATLAS_VISION_SENSOR_FORENSIC_VIDEO_FPS", "10"))),
            queue_maxsize=max(8, int(os.getenv("ATLAS_VISION_SENSOR_QUEUE_MAXSIZE", "64"))),
        )
