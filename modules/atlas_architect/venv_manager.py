from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from .terminal_tools import TerminalTools
from .ans_logger import ANSBitacoraLogger


@dataclass(frozen=True)
class VenvInfo:
    ok: bool
    venv_dir: str
    python_exe: str
    error: Optional[str] = None


class VenvManager:
    """Regla de Oro: 1 venv por app/tarea externa (aislamiento)."""

    def __init__(self, term: TerminalTools, ans: Optional[ANSBitacoraLogger] = None) -> None:
        self.term = term
        self.ans = ans

    def ensure_venv(self, project_dir: Path, venv_name: str = ".venv") -> VenvInfo:
        project_dir = Path(project_dir).resolve()
        venv_dir = (project_dir / venv_name).resolve()
        py = self._python_path(venv_dir)
        if Path(py).exists():
            if self.ans:
                self.ans.log_suggestion(f"[ARCHITECT] VENV ya existe: {venv_dir}")
            return VenvInfo(ok=True, venv_dir=str(venv_dir), python_exe=py, error=None)

        if self.ans:
            self.ans.log_suggestion(f"[ARCHITECT] Creando VENV para proyecto: {venv_dir}")
        r = self.term.run_command(f'python -m venv "{venv_dir}"', timeout_s=120, cwd=project_dir)
        if not r.get("ok"):
            return VenvInfo(ok=False, venv_dir=str(venv_dir), python_exe=py, error=r.get("error") or "venv_create_failed")
        return VenvInfo(ok=True, venv_dir=str(venv_dir), python_exe=py, error=None)

    def install_requirements(self, python_exe: str, requirements_path: Path, *, cwd: Path) -> bool:
        requirements_path = Path(requirements_path).resolve()
        if not requirements_path.exists():
            if self.ans:
                self.ans.log_debug("requirements.txt no existe; omitido.", ok=True)
            return True
        if self.ans:
            self.ans.log_suggestion("[ARCHITECT] Instalando dependencias en VENV...")
        # pip upgrade
        self.term.run_command(f'"{python_exe}" -m pip install -U pip', timeout_s=180, cwd=cwd)
        r = self.term.run_command(f'"{python_exe}" -m pip install -r "{requirements_path}"', timeout_s=300, cwd=cwd)
        return bool(r.get("ok"))

    def _python_path(self, venv_dir: Path) -> str:
        if os.name == "nt":
            return str(venv_dir / "Scripts" / "python.exe")
        return str(venv_dir / "bin" / "python")

