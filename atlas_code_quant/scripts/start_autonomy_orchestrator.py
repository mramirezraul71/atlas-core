from __future__ import annotations

import os
import time
import ctypes
from pathlib import Path
from typing import IO

from atlas_code_quant.atlas_core.autonomy.adapters.quant_adapter import QuantAutonomyAdapter
from atlas_code_quant.atlas_core.autonomy.adapters.robot_adapter import RobotAutonomyAdapter
from atlas_code_quant.atlas_core.autonomy.orchestrator import AutonomyOrchestrator


def _acquire_single_instance_lock(lock_path: Path) -> IO[bytes] | None:
    """
    Garantiza single-instance en Windows sin dependencias.
    Mantén el handle abierto durante toda la ejecución.
    """
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    fh = open(lock_path, "a+b")
    try:
        # Windows-only advisory lock
        import msvcrt  # type: ignore

        fh.seek(0)
        fh.truncate()
        fh.write(str(os.getpid()).encode("utf-8"))
        fh.flush()
        fh.seek(0)
        msvcrt.locking(fh.fileno(), msvcrt.LK_NBLCK, 1)
        return fh
    except Exception:
        try:
            fh.close()
        except Exception:
            pass
        return None


def _acquire_single_instance_mutex(name: str) -> int | None:
    """
    Mutex global Windows: evita doble instancia incluso si se lanzan 2 procesos a la vez.
    Devuelve handle (int) o None si ya existe.
    """
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    handle = kernel32.CreateMutexW(None, False, name)
    if not handle:
        return None
    already_exists = ctypes.get_last_error() == 183  # ERROR_ALREADY_EXISTS
    if already_exists:
        kernel32.CloseHandle(handle)
        return None
    return int(handle)


def main() -> None:
    base = Path(r"C:\ATLAS_PUSH\atlas_code_quant\data")
    summary = base / "backtest" / "backtest_results" / "latest" / "iron_butterfly_summary.json"
    operation_state = base / "operation" / "operation_center_state.json"

    mutex = _acquire_single_instance_mutex("Global\\ATLAS_PUSH_AUTONOMY_ORCHESTRATOR")
    if mutex is None:
        print("AutonomyOrchestrator ya está corriendo (mutex activo). Saliendo.")
        return

    lock_file = base / "operation" / "autonomy_orchestrator.lock"
    lock_handle = _acquire_single_instance_lock(lock_file)
    if lock_handle is None:
        print("AutonomyOrchestrator ya está corriendo (lock activo). Saliendo.")
        return

    quant = QuantAutonomyAdapter(summary_path=summary, operation_state_path=operation_state)
    robot = RobotAutonomyAdapter(base_url="http://127.0.0.1:8002")

    orchestrator = AutonomyOrchestrator(
        modules=[quant, robot],
        interval_seconds=30,
        config={"global_mode": "semi", "max_drawdown_pct": -0.08},
    )
    orchestrator.start_background()

    try:
        while True:
            time.sleep(60)
    except KeyboardInterrupt:
        orchestrator.stop()
    finally:
        try:
            lock_handle.close()
        except Exception:
            pass
        try:
            ctypes.WinDLL("kernel32").CloseHandle(mutex)
        except Exception:
            pass


if __name__ == "__main__":
    main()

