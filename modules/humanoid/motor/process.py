"""Process manager: list/kill by name or PID (Windows-aware)."""
from __future__ import annotations

import os
import signal
from typing import Any, Dict, List, Optional

try:
    import psutil
    _HAS_PSUTIL = True
except ImportError:
    _HAS_PSUTIL = False


class ProcessManager:
    """List processes, kill by PID. Optional psutil for rich info."""

    def list_processes(self, name_filter: Optional[str] = None) -> Dict[str, Any]:
        """Returns {ok, processes: [{pid, name, ...}], error}. If no psutil, returns minimal."""
        if not _HAS_PSUTIL:
            return {"ok": True, "processes": [], "error": None, "message": "psutil not installed"}
        try:
            procs = []
            for p in psutil.process_iter(["pid", "name", "status"]):
                try:
                    info = p.info
                    name = (info.get("name") or "").lower()
                    if name_filter and name_filter.lower() not in name:
                        continue
                    procs.append({"pid": info.get("pid"), "name": info.get("name"), "status": info.get("status")})
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            return {"ok": True, "processes": procs, "error": None}
        except Exception as e:
            return {"ok": False, "processes": [], "error": str(e)}

    def kill(self, pid: int, force: bool = False) -> Dict[str, Any]:
        """Kill process by PID. Returns {ok, error}."""
        try:
            if _HAS_PSUTIL:
                p = psutil.Process(pid)
                p.terminate()
                if force:
                    p.wait(timeout=2)
                    p.kill()
            else:
                sig = signal.SIGTERM if not force else getattr(signal, "SIGKILL", signal.SIGTERM)
                os.kill(pid, sig)
            return {"ok": True, "error": None}
        except ProcessLookupError:
            return {"ok": False, "error": "process not found"}
        except Exception as e:
            return {"ok": False, "error": str(e)}
