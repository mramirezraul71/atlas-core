from __future__ import annotations

import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


@dataclass
class FileLock:
    """PY004: lock simple basado en archivo (offline-first, sin deps).

    Estrategia:
    - crear archivo lock con O_EXCL (atómico) y borrarlo al liberar.
    - si existe, esperar hasta timeout.

    Nota: es un lock cooperativo (los procesos deben respetarlo).
    """

    lock_path: Path
    timeout_s: float = 5.0
    poll_interval_s: float = 0.05
    _acquired: bool = False

    def acquire(self) -> None:
        deadline = time.time() + float(self.timeout_s or 0)
        self.lock_path.parent.mkdir(parents=True, exist_ok=True)
        while True:
            try:
                fd = os.open(str(self.lock_path), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
                try:
                    payload = f"pid={os.getpid()} ts={time.time():.3f}\n"
                    os.write(fd, payload.encode("utf-8", errors="ignore"))
                finally:
                    try:
                        os.close(fd)
                    except Exception:
                        pass
                self._acquired = True
                return
            except FileExistsError:
                if time.time() >= deadline:
                    raise TimeoutError(f"Lock timeout: {self.lock_path}")
                time.sleep(self.poll_interval_s)

    def release(self) -> None:
        if not self._acquired:
            return
        try:
            self.lock_path.unlink(missing_ok=True)
        finally:
            self._acquired = False

    def __enter__(self) -> "FileLock":
        self.acquire()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.release()


def file_lock(target: Path, timeout_s: float = 5.0) -> FileLock:
    """PY004: obtener un lock para un target (crea `<target>.lock`)."""
    p = Path(target)
    lock_path = p.with_suffix(p.suffix + ".lock") if p.suffix else Path(str(p) + ".lock")
    return FileLock(lock_path=lock_path, timeout_s=timeout_s)

