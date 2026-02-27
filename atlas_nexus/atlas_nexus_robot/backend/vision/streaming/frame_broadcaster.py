from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterator, Optional, Tuple

import cv2
import numpy as np


def _clamp(v: float, lo: float, hi: float) -> float:
    try:
        x = float(v)
    except Exception:
        return lo
    return lo if x < lo else hi if x > hi else x


def _apply_focus_zoom(frame: np.ndarray, focus_x: float = 0.5, focus_y: float = 0.5, zoom: float = 1.0):
    if frame is None:
        return frame
    z = _clamp(zoom, 1.0, 4.0)
    if z <= 1.01:
        return frame
    h, w = frame.shape[:2]
    cx = int(_clamp(focus_x, 0.0, 1.0) * w)
    cy = int(_clamp(focus_y, 0.0, 1.0) * h)
    crop_w = max(16, int(w / z))
    crop_h = max(16, int(h / z))
    x0 = max(0, min(w - crop_w, cx - crop_w // 2))
    y0 = max(0, min(h - crop_h, cy - crop_h // 2))
    crop = frame[y0 : y0 + crop_h, x0 : x0 + crop_w]
    try:
        return cv2.resize(crop, (w, h), interpolation=cv2.INTER_LINEAR)
    except Exception:
        return frame


def _enhance_frame(frame: np.ndarray, profile: str = "none"):
    p = (profile or "none").strip().lower()
    if p in ("0", "false", "off", "no"):
        p = "none"
    if p == "max":
        p = "sharp"
    if p == "none":
        return frame
    try:
        if p == "auto":
            out = cv2.fastNlMeansDenoisingColored(frame, None, 3, 3, 7, 21)
        elif p in ("sharp", "ocr"):
            out = cv2.fastNlMeansDenoisingColored(frame, None, 6, 6, 7, 21)
        else:
            out = frame

        if p == "ocr":
            g = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
            g = cv2.GaussianBlur(g, (3, 3), 0)
            g = cv2.adaptiveThreshold(g, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 5)
            return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)

        if p == "sharp":
            lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            l2 = clahe.apply(l)
            out = cv2.cvtColor(cv2.merge((l2, a, b)), cv2.COLOR_LAB2BGR)

        blur = cv2.GaussianBlur(out, (0, 0), 1.0 if p == "auto" else 1.25)
        amount = 0.7 if p == "auto" else 1.35
        sharp = cv2.addWeighted(out, 1.0 + amount, blur, -amount, 0)
        return np.clip(sharp, 0, 255).astype(out.dtype)
    except Exception:
        return frame


def _open_capture(camera_index: int):
    try:
        from vision.cameras.backend import preferred_backends

        backends = preferred_backends()
    except Exception:
        backends = []
        if hasattr(cv2, "CAP_DSHOW"):
            backends.append(cv2.CAP_DSHOW)
        if hasattr(cv2, "CAP_MSMF"):
            backends.append(cv2.CAP_MSMF)
        backends.append(cv2.CAP_ANY)

    for backend in backends:
        cap = None
        try:
            cap = cv2.VideoCapture(camera_index, backend)
            if not cap.isOpened():
                try:
                    cap.release()
                except Exception:
                    pass
                continue
            ok, frame = cap.read()
            if ok and frame is not None:
                return cap
            try:
                cap.release()
            except Exception:
                pass
        except Exception:
            try:
                if cap is not None:
                    cap.release()
            except Exception:
                pass
    return None


@dataclass
class FrameData:
    frame: np.ndarray
    timestamp: float
    sequence: int


class FrameIterator:
    def __init__(self, broadcaster: "FrameBroadcaster", event: threading.Event):
        self._broadcaster = broadcaster
        self._event = event
        self._last_sequence = -1

    def __iter__(self) -> Iterator[FrameData]:
        return self

    def __next__(self) -> FrameData:
        while True:
            if not self._event.wait(timeout=10.0):
                self.close()
                raise StopIteration
            self._event.clear()
            with self._broadcaster._lock:
                data = self._broadcaster._latest_frame
            if data is None:
                continue
            if data.sequence <= self._last_sequence:
                continue
            self._last_sequence = data.sequence
            return data

    def close(self) -> None:
        self._broadcaster._unsubscribe(self._event)


class FrameBroadcaster:
    def __init__(
        self,
        *,
        camera_index: int,
        fps: int = 15,
        enhance: str = "none",
        focus_x: float = 0.5,
        focus_y: float = 0.5,
        zoom: float = 1.0,
    ):
        self.camera_index = int(camera_index)
        self.fps = max(1, min(30, int(fps)))
        self.enhance = str(enhance or "none")
        self.focus_x = float(focus_x)
        self.focus_y = float(focus_y)
        self.zoom = float(zoom)

        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._cap = None
        self._latest_frame: Optional[FrameData] = None
        self._sequence = 0
        self._subscribers: list[threading.Event] = []

    def start(self) -> bool:
        if self._running:
            return True
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        for _ in range(20):
            with self._lock:
                if self._latest_frame is not None:
                    return True
            time.sleep(0.1)
        return False

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.5)
        if self._cap:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def subscribe(self) -> FrameIterator:
        event = threading.Event()
        with self._lock:
            self._subscribers.append(event)
            if self._latest_frame is not None:
                event.set()
        return FrameIterator(self, event)

    def _unsubscribe(self, event: threading.Event) -> None:
        with self._lock:
            try:
                self._subscribers.remove(event)
            except ValueError:
                pass

    def _capture_loop(self) -> None:
        self._cap = _open_capture(self.camera_index)
        if not self._cap or not self._cap.isOpened():
            self._running = False
            return
        target_dt = 1.0 / float(self.fps)
        failures = 0
        while self._running:
            t0 = time.perf_counter()
            try:
                ok, frame = self._cap.read()
                if not ok or frame is None:
                    failures += 1
                    if failures >= 10:
                        self._cap.release()
                        self._cap = _open_capture(self.camera_index)
                        failures = 0
                    time.sleep(0.05)
                    continue
                failures = 0

                frame = _apply_focus_zoom(frame, focus_x=self.focus_x, focus_y=self.focus_y, zoom=self.zoom)
                frame = _enhance_frame(frame, self.enhance)

                with self._lock:
                    self._sequence += 1
                    self._latest_frame = FrameData(frame=frame.copy(), timestamp=time.time(), sequence=self._sequence)
                    for evt in self._subscribers:
                        evt.set()
            except Exception:
                time.sleep(0.1)
            elapsed = time.perf_counter() - t0
            if elapsed < target_dt:
                time.sleep(target_dt - elapsed)


_BROADCASTERS: Dict[Tuple[int, int, str, float, float, float], FrameBroadcaster] = {}
_BROADCASTER_LOCK = threading.Lock()


def _make_key(
    *,
    camera_index: int,
    fps: int,
    enhance: str,
    focus_x: float,
    focus_y: float,
    zoom: float,
) -> Tuple[int, int, str, float, float, float]:
    return (
        int(camera_index),
        int(max(1, min(30, fps))),
        str(enhance or "none").strip().lower(),
        round(float(focus_x), 4),
        round(float(focus_y), 4),
        round(float(zoom), 4),
    )


def get_broadcaster(
    *,
    camera_index: int = 0,
    fps: int = 15,
    enhance: str = "none",
    focus_x: float = 0.5,
    focus_y: float = 0.5,
    zoom: float = 1.0,
) -> FrameBroadcaster:
    key = _make_key(
        camera_index=camera_index,
        fps=fps,
        enhance=enhance,
        focus_x=focus_x,
        focus_y=focus_y,
        zoom=zoom,
    )
    with _BROADCASTER_LOCK:
        br = _BROADCASTERS.get(key)
        if br is not None:
            return br
        br = FrameBroadcaster(
            camera_index=camera_index,
            fps=fps,
            enhance=enhance,
            focus_x=focus_x,
            focus_y=focus_y,
            zoom=zoom,
        )
        if not br.start():
            raise RuntimeError(f"No se pudo iniciar broadcaster para camara {camera_index}")
        _BROADCASTERS[key] = br
        return br


def cleanup_broadcasters() -> None:
    with _BROADCASTER_LOCK:
        for b in _BROADCASTERS.values():
            b.stop()
        _BROADCASTERS.clear()
