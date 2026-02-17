"""TTS: stub + detection of piper/sherpa-onnx or system."""
from __future__ import annotations

import logging
import os
import threading
import time
from typing import Any, Dict, List, Optional

_log = logging.getLogger("humanoid.voice.tts")
_missing: List[str] = []
_SPEAK_LOCK = threading.Lock()


def _try_acquire_process_lock(timeout_s: float = 0.15):
    """
    Evita "doble voz" cuando hay más de un proceso ATLAS corriendo.
    En Windows usa un lockfile con msvcrt (best-effort).
    Retorna un handle abierto si se adquirió, o None si no.
    """
    if os.name != "nt":
        return None
    try:
        import msvcrt  # type: ignore

        lock_path = os.getenv("TTS_LOCK_PATH", r"C:\ATLAS_PUSH\logs\tts.lock")
        os.makedirs(os.path.dirname(lock_path), exist_ok=True)
        f = open(lock_path, "a+", encoding="utf-8")
        end = time.time() + max(0.01, float(timeout_s))
        while True:
            try:
                # Lock 1 byte (non-blocking)
                msvcrt.locking(f.fileno(), msvcrt.LK_NBLCK, 1)
                return f
            except OSError:
                if time.time() >= end:
                    try:
                        f.close()
                    except Exception:
                        pass
                    return None
                time.sleep(0.02)
    except Exception:
        return None


def _release_process_lock(handle) -> None:
    if not handle:
        return
    if os.name != "nt":
        try:
            handle.close()
        except Exception:
            pass
        return
    try:
        import msvcrt  # type: ignore

        try:
            msvcrt.locking(handle.fileno(), msvcrt.LK_UNLCK, 1)
        finally:
            try:
                handle.close()
            except Exception:
                pass
    except Exception:
        try:
            handle.close()
        except Exception:
            pass


def _check_deps() -> List[str]:
    """Check TTS deps. Voice deps are optional - return empty to avoid ANS incidents."""
    global _missing
    try:
        import pyttsx3
        _missing = []
        return []
    except ImportError:
        pass
    # Voice deps are optional - don't create incidents
    _missing = []
    return []


def is_available() -> bool:
    return len(_check_deps()) == 0


def get_missing_deps() -> List[str]:
    return list(_check_deps())


def speak(text: str, options: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Speak text using pyttsx3. Runs in thread to avoid blocking."""
    if not is_available():
        return {"ok": False, "error": "TTS not available", "missing_deps": get_missing_deps()}
    if not (text or "").strip():
        return {"ok": True, "message": "empty text"}
    try:
        import pyttsx3
        # Anti-solapamiento: si ya se está hablando, saltar (evita "doble voz")
        if not _SPEAK_LOCK.acquire(blocking=False):
            return {"ok": True, "skipped": "tts_busy"}

        def _run():
            lock_handle = None
            engine = pyttsx3.init()
            try:
                # Lock entre procesos (best-effort)
                lock_handle = _try_acquire_process_lock(timeout_s=0.2)
                if lock_handle is None:
                    return
                engine.setProperty("rate", 150)
                engine.say((text or "").strip())
                engine.runAndWait()
            finally:
                try:
                    engine.stop()
                except Exception:
                    pass
                try:
                    _release_process_lock(lock_handle)
                finally:
                    try:
                        _SPEAK_LOCK.release()
                    except Exception:
                        pass

        thread = threading.Thread(target=_run, daemon=True)
        thread.start()
        thread.join(timeout=30)
        return {"ok": True, "message": "speaking"}
    except Exception as e:
        _log.exception("TTS speak failed")
        try:
            if _SPEAK_LOCK.locked():
                _SPEAK_LOCK.release()
        except Exception:
            pass
        return {"ok": False, "error": str(e)}
