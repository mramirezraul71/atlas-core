from __future__ import annotations

import subprocess
from dataclasses import dataclass
from typing import List, Optional


@dataclass(frozen=True)
class CmdResult:
    cmd: List[str]
    returncode: int
    stdout: str
    stderr: str


def run_cmd(cmd: List[str], timeout_s: float = 10.0, check: bool = True) -> CmdResult:
    """PY008: wrapper seguro de subprocess (sin shell=True).

    - cmd: lista de args
    - timeout_s: timeout
    - check: si True, falla cuando returncode != 0
    """
    if not isinstance(cmd, list) or not cmd or not all(isinstance(x, str) and x for x in cmd):
        raise ValueError("cmd debe ser list[str] no vacía")
    p = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        timeout=timeout_s,
        shell=False,
    )
    res = CmdResult(cmd=cmd, returncode=int(p.returncode), stdout=p.stdout or "", stderr=p.stderr or "")
    if check and res.returncode != 0:
        raise subprocess.CalledProcessError(res.returncode, cmd, output=res.stdout, stderr=res.stderr)
    return res

