"""Evaluador automático del track Python Mastery (offline-first).

Objetivo:
- Ejecutar checks deterministas (sin red) para una lección PYxxx.
- Producir un resultado tipo "evaluación" con score/passed y evidencia (logs).

Este evaluador está pensado para ser invocado por API o por scripts locales.
"""

from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, List, Optional


def _truthy(v: str | None) -> bool:
    return (v or "").strip().lower() in ("1", "true", "yes", "on")


def _run_pytest(nodeids: List[str], repo_root: Path, timeout_s: int = 180) -> Dict[str, Any]:
    """Corre pytest sobre nodeids concretos y devuelve {ok, exit_code, output}."""
    if not nodeids:
        return {"ok": False, "exit_code": None, "output": "No nodeids provided"}
    env = os.environ.copy()
    # Activa los tests de Python Mastery.
    env["RUN_PYTHON_MASTERY"] = "1"
    # Evita accidentalmente disparar E2E por defecto.
    if not _truthy(env.get("RUN_E2E")):
        env["RUN_E2E"] = "0"

    cmd = ["python", "-m", "pytest", "-q", *nodeids]
    t0 = time.time()
    try:
        p = subprocess.run(
            cmd,
            cwd=str(repo_root),
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
        out = (p.stdout or "") + ("\n" + p.stderr if p.stderr else "")
        return {
            "ok": p.returncode == 0,
            "exit_code": p.returncode,
            "seconds": round(time.time() - t0, 2),
            "output": out[-20000:],  # recorte defensivo
            "cmd": cmd,
        }
    except subprocess.TimeoutExpired as e:
        out = ""
        try:
            out = ((e.stdout or "") + ("\n" + (e.stderr or ""))).strip()
        except Exception:
            out = ""
        return {
            "ok": False,
            "exit_code": None,
            "seconds": round(time.time() - t0, 2),
            "output": ("TIMEOUT\n" + out)[-20000:],
            "cmd": cmd,
        }
    except Exception as e:
        return {"ok": False, "exit_code": None, "output": f"pytest error: {e}", "cmd": cmd}


def _file_exists(repo_root: Path, rel_path: str) -> bool:
    try:
        return (repo_root / rel_path).exists()
    except Exception:
        return False


def _lesson_plan(lesson_id: str) -> Dict[str, Any]:
    """Mapea lección → checks. Extensible por crecimiento."""
    lid = (lesson_id or "").strip().upper()
    if lid == "PY001":
        return {
            "required_files": [
                "training/python_mastery/utils.py",
                "tests/python_mastery/test_py001_utils.py",
            ],
            "pytest_nodeids": ["tests/python_mastery/test_py001_utils.py"],
        }
    if lid == "PY002":
        return {
            "required_files": [
                "training/python_mastery/__init__.py",
                "tests/python_mastery/test_py002_imports.py",
            ],
            "pytest_nodeids": ["tests/python_mastery/test_py002_imports.py"],
        }
    if lid == "PY003":
        return {
            "required_files": [
                "training/python_mastery/cli.py",
                "training/python_mastery/inventory.py",
                "tests/python_mastery/test_py003_inventory.py",
            ],
            "pytest_nodeids": ["tests/python_mastery/test_py003_inventory.py"],
        }
    if lid == "PY004":
        return {
            "required_files": [
                "training/python_mastery/locks.py",
                "tests/python_mastery/test_py004_locks.py",
            ],
            "pytest_nodeids": ["tests/python_mastery/test_py004_locks.py"],
        }
    if lid == "PY005":
        return {
            "required_files": [
                "training/python_mastery/io_utils.py",
                "tests/python_mastery/test_py005_atomic_write.py",
            ],
            "pytest_nodeids": ["tests/python_mastery/test_py005_atomic_write.py"],
        }
    if lid == "PY006":
        return {
            "required_files": [
                "training/python_mastery/cli.py",
                "training/python_mastery/inventory.py",
                "tests/python_mastery/test_py006_cli_inventory_json.py",
            ],
            "pytest_nodeids": ["tests/python_mastery/test_py006_cli_inventory_json.py"],
        }
    if lid == "PY007":
        return {
            "required_files": [
                "training/python_mastery/app_refactor.py",
                "tests/python_mastery/test_py007_refactor.py",
            ],
            "pytest_nodeids": ["tests/python_mastery/test_py007_refactor.py"],
        }
    if lid == "PY008":
        return {
            "required_files": [
                "training/python_mastery/subprocess_utils.py",
                "tests/python_mastery/test_py008_subprocess.py",
            ],
            "pytest_nodeids": ["tests/python_mastery/test_py008_subprocess.py"],
        }
    if lid == "PY009":
        return {
            "required_files": [
                "training/python_mastery/http_client.py",
                "tests/python_mastery/test_py009_http_retries.py",
            ],
            "pytest_nodeids": ["tests/python_mastery/test_py009_http_retries.py"],
        }
    if lid == "PY010":
        return {
            "required_files": [
                "training/python_mastery/sqlite_repo.py",
                "tests/python_mastery/test_py010_sqlite_repo.py",
            ],
            "pytest_nodeids": ["tests/python_mastery/test_py010_sqlite_repo.py"],
        }
    if lid == "PY011":
        return {
            "required_files": [
                "training/python_mastery/async_downloader.py",
                "tests/python_mastery/test_py011_asyncio.py",
            ],
            "pytest_nodeids": ["tests/python_mastery/test_py011_asyncio.py"],
        }
    if lid == "PY012":
        return {
            "required_files": [
                "training/python_mastery/packaging_sample/pyproject.toml",
                "training/python_mastery/packaging_sample/atlas_py_tool/cli.py",
                "tests/python_mastery/test_py012_packaging.py",
            ],
            "pytest_nodeids": ["tests/python_mastery/test_py012_packaging.py"],
        }
    return {"required_files": [], "pytest_nodeids": [], "note": "No evaluator mapping for this lesson yet"}


def get_lesson_plan(lesson_id: str) -> Dict[str, Any]:
    """API pública para obtener mapping de una lección (archivos + nodeids).

    Útil para generar planes de remediación sin depender de LLM.
    """
    return _lesson_plan(lesson_id)


def evaluate_python_mastery_lesson(
    lesson_id: str,
    repo_root: str | Path,
    timeout_s: int = 180,
) -> Dict[str, Any]:
    """Evalúa una lección del track Python Mastery.

    Retorna dict compatible con el estilo de `AITutor.review_robot_performance`:
    - score (0-100)
    - passed (bool)
    - feedback (texto)
    - areas_for_improvement (lista)
    - evidence (salida de pytest / checks)
    """
    root = Path(repo_root).resolve()
    lid = (lesson_id or "").strip().upper()
    plan = _lesson_plan(lid)

    checks: List[Dict[str, Any]] = []

    # Check 1: archivos requeridos
    req = plan.get("required_files") or []
    missing = [p for p in req if not _file_exists(root, p)]
    files_ok = len(missing) == 0
    checks.append(
        {
            "name": "required_files",
            "ok": files_ok,
            "missing": missing,
            "required": req,
        }
    )

    # Check 2: pytest (si hay mapping)
    pytest_nodeids: List[str] = list(plan.get("pytest_nodeids") or [])
    pytest_res: Optional[Dict[str, Any]] = None
    pytest_ok = False
    if pytest_nodeids:
        pytest_res = _run_pytest(pytest_nodeids, root, timeout_s=timeout_s)
        pytest_ok = bool(pytest_res.get("ok"))
    checks.append({"name": "pytest", "ok": pytest_ok, "result": pytest_res, "nodeids": pytest_nodeids})

    # Scoring: simple y determinista (expandible).
    # - Archivos: 20 puntos
    # - Pytest: 80 puntos
    score = 0
    score += 20 if files_ok else 0
    score += 80 if pytest_ok else 0
    passed = score >= 60

    improv: List[str] = []
    if not files_ok:
        improv.append("Crear/ubicar los archivos requeridos de la lección.")
    if pytest_nodeids and not pytest_ok:
        improv.append("Hacer que los tests de la lección pasen (RUN_PYTHON_MASTERY=1).")
    if not pytest_nodeids:
        improv.append("Lección aún sin mapping de evaluador; extender `python_mastery_evaluator.py`.")

    feedback = f"Lección {lid}: score={score}/100. " + ("APROBADA." if passed else "NO APROBADA.")
    if plan.get("note"):
        feedback += f" Nota: {plan['note']}"

    return {
        "lesson_id": lid,
        "score": score,
        "passed": passed,
        "grade": "A" if score >= 90 else "B" if score >= 75 else "C" if score >= 60 else "D",
        "evaluation": feedback,
        "feedback": feedback,
        "areas_for_improvement": improv,
        "strengths_demonstrated": ["offline-first evaluation", "deterministic checks"] if score > 0 else [],
        "knowledge_validated": [],
        "knowledge_corrected": [],
        "next_steps": "Implementar tareas de la lección y re-evaluar.",
        "evidence": {"checks": checks},
    }

