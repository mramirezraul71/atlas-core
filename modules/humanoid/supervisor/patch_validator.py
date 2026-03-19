"""Validador mínimo: compilación Python y subset rápido de tests."""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from typing import List, Optional

from .models import (
    REQUIRE_COMPILE_CHECK,
    REQUIRE_TESTS_IF_PRESENT,
    ChangeProposal,
    ValidationResult,
)


def _repo_root() -> Path:
    root = (
        os.getenv("ATLAS_REPO_PATH")
        or os.getenv("ATLAS_PUSH_ROOT")
        or os.getenv("ATLAS_ROOT")
        or ""
    ).strip()
    if root:
        return Path(root).resolve()
    return Path(__file__).resolve().parents[3]


class PatchValidator:
    """Ejecuta validaciones conservadoras posteriores al patch."""

    def __init__(self, repo_root: Optional[Path] = None) -> None:
        self.repo_root = (repo_root or _repo_root()).resolve()

    def _run(self, cmd: List[str], timeout: int = 120) -> subprocess.CompletedProcess:
        return subprocess.run(
            cmd,
            cwd=str(self.repo_root),
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )

    def _discover_tests(
        self,
        changed_files: List[str],
        proposal: Optional[ChangeProposal],
    ) -> List[str]:
        tests: List[str] = []

        if proposal and proposal.test_paths:
            for candidate in proposal.test_paths:
                p = Path(candidate)
                full = p if p.is_absolute() else (self.repo_root / p)
                if full.is_file():
                    tests.append(str(full))

        if tests:
            return tests

        if any("modules\\humanoid\\supervisor" in f or "modules/humanoid/supervisor" in f for f in changed_files):
            default_test = self.repo_root / "tests" / "humanoid" / "test_supervisor_autoprogrammer.py"
            if default_test.is_file():
                tests.append(str(default_test))

        return tests

    def validate(
        self,
        changed_files: List[str],
        proposal: Optional[ChangeProposal] = None,
    ) -> ValidationResult:
        compile_ok = True
        compile_output = "compile check skipped"
        tests_ok: Optional[bool] = None
        tests_output = "tests skipped"
        ran_tests = False
        details = {}

        if REQUIRE_COMPILE_CHECK:
            py_files = [f for f in changed_files if f.lower().endswith(".py")]
            if py_files:
                compile_cmd = [sys.executable, "-m", "py_compile", *py_files]
                c = self._run(compile_cmd, timeout=90)
                compile_ok = c.returncode == 0
                compile_output = ((c.stdout or "") + "\n" + (c.stderr or "")).strip()
                details["compile_cmd"] = compile_cmd
                details["compile_rc"] = c.returncode
            else:
                compile_ok = True
                compile_output = "no python files changed"

        if REQUIRE_TESTS_IF_PRESENT:
            tests = self._discover_tests(changed_files, proposal)
            if tests:
                ran_tests = True
                test_cmd = [sys.executable, "-m", "pytest", "-q", *tests]
                t = self._run(test_cmd, timeout=180)
                tests_ok = t.returncode == 0
                tests_output = ((t.stdout or "") + "\n" + (t.stderr or "")).strip()
                details["test_cmd"] = test_cmd
                details["test_rc"] = t.returncode
            else:
                tests_ok = None
                tests_output = "tests skipped: no fast subset found"

        overall_ok = compile_ok and (tests_ok if tests_ok is not None else True)
        return ValidationResult(
            ok=overall_ok,
            compile_ok=compile_ok,
            tests_ok=tests_ok,
            ran_tests=ran_tests,
            compile_output=compile_output,
            tests_output=tests_output,
            details=details,
        )

