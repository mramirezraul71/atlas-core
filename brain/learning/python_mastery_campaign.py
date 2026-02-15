"""Modo Campaña: ejecutar Python Mastery como campaña (PY001..PY012) con progreso persistente.

Objetivo:
- Seleccionar la siguiente lección pendiente (respetando prerequisitos via AITutor)
- Evaluar (determinista/offline) y registrar progreso (completada/fallida)
- Persistir estado para reanudar
"""

from __future__ import annotations

import json
import os
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional
import re


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _repo_root_fallback() -> Path:
    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT") or ""
    if root:
        return Path(root).resolve()
    # brain/learning/python_mastery_campaign.py -> repo_root
    return Path(__file__).resolve().parents[2]


def _training_tree_mtime(repo_root: Path) -> float:
    """Máximo mtime dentro de training/tests de Python Mastery (para require_change)."""
    roots = [
        repo_root / "training" / "python_mastery",
        repo_root / "tests" / "python_mastery",
    ]
    latest = 0.0
    for r in roots:
        try:
            if not r.exists():
                continue
            for p in r.rglob("*"):
                try:
                    if p.is_file():
                        latest = max(latest, float(p.stat().st_mtime))
                except Exception:
                    continue
        except Exception:
            continue
    return latest


def _extract_checks(evaluation: Dict[str, Any]) -> list[dict]:
    try:
        checks = (evaluation or {}).get("evidence", {}).get("checks", [])
        return checks if isinstance(checks, list) else []
    except Exception:
        return []


def generate_remediation_plan(lesson_id: str, evaluation: Dict[str, Any]) -> Dict[str, Any]:
    """Genera un plan de corrección accionable a partir de la evaluación.

    No aplica cambios por sí mismo. Solo produce instrucciones (offline-first).
    """
    lid = (lesson_id or "").strip().upper()
    checks = _extract_checks(evaluation or {})

    missing_files: list[str] = []
    pytest_output = ""
    pytest_nodeids: list[str] = []
    for c in checks:
        if c.get("name") == "required_files":
            missing_files = list(c.get("missing") or [])
        if c.get("name") == "pytest":
            pytest_nodeids = list(c.get("nodeids") or [])
            r = c.get("result") or {}
            if isinstance(r, dict):
                pytest_output = str(r.get("output") or "")

    # Si la evaluación no trae nodeids, intentar deducirlos desde el evaluador.
    if not pytest_nodeids:
        try:
            from .python_mastery_evaluator import get_lesson_plan

            plan = get_lesson_plan(lid) or {}
            pytest_nodeids = list(plan.get("pytest_nodeids") or [])
            if not missing_files:
                missing_files = [p for p in (plan.get("required_files") or []) if p]
        except Exception:
            pass

    # Heurística de archivos candidatos a editar: training/python_mastery/*.py
    files_hint: list[str] = []
    for m in missing_files:
        if isinstance(m, str) and m:
            files_hint.append(m)
    # Extraer rutas de archivo del output (si aparecen)
    for match in re.findall(r"(training/python_mastery/[A-Za-z0-9_./-]+\.py)", pytest_output.replace("\\", "/")):
        files_hint.append(match)

    # Deducción de causa probable a partir de señales comunes
    probable = []
    if "NotImplementedError" in pytest_output:
        probable.append("Funciones stub (NotImplementedError) aún no implementadas.")
    if "AssertionError" in pytest_output:
        probable.append("Comportamiento no coincide con criterios del test (AssertionError).")
    if "ModuleNotFoundError" in pytest_output or "ImportError" in pytest_output:
        probable.append("Problema de imports/paquete (ModuleNotFoundError/ImportError).")

    # Plan de ejecución mínimo: pasos accionables
    steps: list[str] = []
    if missing_files:
        steps.append(f"Crear archivos requeridos: {', '.join(missing_files)}")
    if pytest_nodeids:
        steps.append(f"Ejecutar verificación local: python -m pytest -q {' '.join(pytest_nodeids)} (RUN_PYTHON_MASTERY=1)")
    if probable:
        steps.append("Causa probable: " + " | ".join(probable))
    steps.append("Implementar cambios en `training/python_mastery/` hasta que los tests pasen.")

    def _edge_case_checklist(lid_: str, pytest_output_: str, missing_: list[str], nodeids_: list[str]) -> list[str]:
        """Checklist offline derivado de señales comunes (sin LLM)."""
        out = (pytest_output_ or "").lower()
        checks: list[str] = []

        # Señales generales
        if missing_:
            checks.append("Archivos requeridos existen en rutas correctas (no en temp_*/).")
        if "notimplementederror" in out:
            checks.append("Eliminar `NotImplementedError`: implementar funciones reales.")
        if "assertionerror" in out:
            checks.append("Alinear comportamiento exactamente con asserts del test (no aproximaciones).")
        if "modulenotfounderror" in out or "importerror" in out:
            checks.append("Arreglar imports/paquete: módulos importables sin hacks de sys.path.")
        if "timeout" in out or "timed out" in out:
            checks.append("Evitar bloqueos: timeouts y bucles con condición de salida.")
        if "typeerror" in out:
            checks.append("Validar tipos/contratos: entradas inválidas deben manejarse (ValueError/TypeError según convenga).")

        # Señales por lección
        lid_ = (lid_ or "").strip().upper()
        if lid_ in ("PY003", "PY004"):
            checks.append("Tolerar errores del sistema de archivos (permisos/archivos desaparecidos) sin romper.")
        if lid_ in ("PY003",):
            checks.append("Extensiones case-insensitive; sin extensión -> '<noext>'.")
        if lid_ in ("PY004",):
            checks.append("Lock: timeout real; cleanup garantizado en __exit__ incluso con excepción.")
        if lid_ in ("PY005",):
            checks.append("Atomic write: tmp en mismo directorio; `os.replace`; cleanup tmp en finally.")
        if lid_ in ("PY006",):
            checks.append("CLI: `--json` produce JSON válido en stdout y return code 0.")
        if lid_ in ("PY008",):
            checks.append("Subprocess: nunca usar shell=True; capturar stdout/stderr; raise si check=True y rc!=0.")
        if lid_ in ("PY009",):
            checks.append("HTTP: sin red en tests; usar session inyectable; retries solo para transitorios (>=500/excepciones).")
        if lid_ in ("PY010",):
            checks.append("SQLite: transacciones; PRAGMAs; upsert correcto; lectura estable.")
        if lid_ in ("PY011",):
            checks.append("Async: respetar limit (Semaphore); cancelación limpia; no dejar tasks colgadas.")
        if lid_ in ("PY012",):
            checks.append("Packaging: pyproject con [project] y [project.scripts]; CLI smoke --ping -> pong.")

        # Señales por nodeids (si vienen)
        nid = " ".join(nodeids_ or []).lower()
        if "cli" in nid:
            checks.append("Tests de CLI: usar argv explícito, no sys.argv global en tests.")
        if "sqlite" in nid:
            checks.append("DB: ubicar en tmp_path y cerrar conexiones.")

        # Dedup
        seen = set()
        uniq: list[str] = []
        for c in checks:
            c = c.strip()
            if not c or c in seen:
                continue
            seen.add(c)
            uniq.append(c)
        return uniq

    def _patch_template_for(lid_: str) -> str:
        """Plantilla de edición (no es un diff aplicable)."""
        lid_ = (lid_ or "").strip().upper()
        if lid_ == "PY001":
            return (
                "@@ training/python_mastery/utils.py\n"
                "Implementa las funciones (sin side-effects) para pasar `tests/python_mastery/test_py001_utils.py`.\n\n"
                "@@ def normalize_text(s: str) -> str\n"
                "- strip espacios extremos\n"
                "- colapsar whitespace interno a un solo espacio\n"
                "- lowercase\n\n"
                "def normalize_text(s: str) -> str:\n"
                "    # 1) validar input\n"
                "    # 2) s.strip()\n"
                "    # 3) split() para colapsar whitespace y ' '.join(...)\n"
                "    # 4) lower()\n"
                "    ...\n\n"
                "@@ def chunk_list(items: Sequence[T], size: int) -> List[List[T]]\n"
                "- size debe ser > 0 (si no, ValueError)\n"
                "- dividir preservando orden\n\n"
                "def chunk_list(items, size):\n"
                "    ...\n\n"
                "@@ def safe_get(d: dict, key: str, default: Any=None) -> Any\n"
                "- key con formato 'a.b.c'\n"
                "- si falta un nivel o no es dict, retornar default\n\n"
                "def safe_get(d, key, default=None):\n"
                "    ...\n"
            )
        if lid_ == "PY003":
            return (
                "@@ training/python_mastery/inventory.py\n"
                "Implementa `scan_inventory` para pasar `tests/python_mastery/test_py003_inventory.py`.\n\n"
                "@@ def scan_inventory(root: Path, min_size_kb: int = 0) -> InventoryResult\n"
                "- contar solo archivos regulares\n"
                "- sumar tamaños en bytes\n"
                "- top extensiones por cantidad (case-insensitive)\n"
                "- sin extensión -> '<noext>'\n"
                "- tolerar permisos/errores sin romper\n\n"
                "def scan_inventory(root, min_size_kb=0):\n"
                "    ...\n"
            )
        if lid_ == "PY004":
            return (
                "@@ training/python_mastery/locks.py\n"
                "Lock cooperativo con archivo .lock (O_EXCL) para pasar `tests/python_mastery/test_py004_locks.py`.\n\n"
                "@@ class FileLock\n"
                "- acquire(): crear archivo lock atómico; si existe, esperar hasta timeout\n"
                "- release(): borrar lock si adquirido\n"
                "- context manager: __enter__/__exit__\n"
            )
        if lid_ == "PY005":
            return (
                "@@ training/python_mastery/io_utils.py\n"
                "atomic_write_json Windows-safe para pasar `tests/python_mastery/test_py005_atomic_write.py`.\n\n"
                "@@ def atomic_write_json(path: Path, payload: dict) -> None\n"
                "- escribir tmp en mismo directorio\n"
                "- os.replace(tmp, path)\n"
                "- cleanup tmp en finally\n"
            )
        if lid_ == "PY006":
            return (
                "@@ training/python_mastery/cli.py\n"
                "Asegurar salida JSON válida con `--json` para pasar `tests/python_mastery/test_py006_cli_inventory_json.py`.\n"
                "- main(argv) retorna 0\n"
                "- print(json) en stdout\n"
            )
        if lid_ == "PY007":
            return (
                "@@ training/python_mastery/app_refactor.py\n"
                "Mantener lógica pura y tipada para pasar `tests/python_mastery/test_py007_refactor.py`.\n"
                "- Config dataclass\n"
                "- parse_args(argv)->Config determinista\n"
                "- build_inventory_payload(...) estable\n"
            )
        if lid_ == "PY008":
            return (
                "@@ training/python_mastery/subprocess_utils.py\n"
                "Wrapper seguro sin shell=True para pasar `tests/python_mastery/test_py008_subprocess.py`.\n"
                "- run_cmd(list[str], timeout, check)\n"
                "- raise CalledProcessError si check y returncode!=0\n"
            )
        if lid_ == "PY009":
            return (
                "@@ training/python_mastery/http_client.py\n"
                "Retries/backoff mockeable para pasar `tests/python_mastery/test_py009_http_retries.py`.\n"
                "- reintentar ante status>=500\n"
                "- no depender de red real (usar session inyectable)\n"
            )
        if lid_ == "PY010":
            return (
                "@@ training/python_mastery/sqlite_repo.py\n"
                "SQLiteRepo CRUD offline-first para pasar `tests/python_mastery/test_py010_sqlite_repo.py`.\n"
                "- WAL + PRAGMAs\n"
                "- upsert/get/list\n"
            )
        if lid_ == "PY011":
            return (
                "@@ training/python_mastery/async_downloader.py\n"
                "Concurrencia con límite + cancelación para pasar `tests/python_mastery/test_py011_asyncio.py`.\n"
                "- Semaphore(limit)\n"
                "- wait_for si timeout_s\n"
                "- CancelledError: cancelar pendientes y devolver cancelled=True\n"
            )
        if lid_ == "PY012":
            return (
                "@@ training/python_mastery/packaging_sample/pyproject.toml\n"
                "Definir [project] y [project.scripts] para pasar `tests/python_mastery/test_py012_packaging.py`.\n\n"
                "@@ training/python_mastery/packaging_sample/atlas_py_tool/cli.py\n"
                "- main(['--ping']) imprime 'pong'\n"
            )
        return "@@ (sin template)\nAgregar plantilla específica para esta lección.\n"

    # Objetivo de diff (detallado por lección, sin LLM)
    lesson_objectives: dict[str, dict] = {
        "PY001": {
            "files": ["training/python_mastery/utils.py"],
            "functions": ["normalize_text", "chunk_list", "safe_get"],
            "objective": "Reemplazar NotImplementedError por implementación pura que cumpla reglas de tests (normalización, chunking, safe_get anidado por 'a.b.c').",
        },
        "PY003": {
            "files": ["training/python_mastery/inventory.py"],
            "functions": ["scan_inventory"],
            "objective": "Implementar scan de filesystem robusto: contar archivos, sumar bytes, top extensiones (case-insensitive), tolerar permisos.",
        },
        "PY004": {
            "files": ["training/python_mastery/locks.py"],
            "functions": ["FileLock.acquire", "FileLock.release", "file_lock"],
            "objective": "Lock cooperativo con archivo .lock (O_EXCL), timeout y cleanup garantizado.",
        },
        "PY005": {
            "files": ["training/python_mastery/io_utils.py"],
            "functions": ["atomic_write_json"],
            "objective": "Escritura atómica Windows-safe: tmp en mismo dir + os.replace + limpieza tmp.",
        },
        "PY006": {
            "files": ["training/python_mastery/cli.py"],
            "functions": ["main"],
            "objective": "Garantizar salida JSON válida con --json y exit code 0 (depende de scan_inventory correcto).",
        },
        "PY007": {
            "files": ["training/python_mastery/app_refactor.py"],
            "functions": ["parse_args", "build_inventory_payload", "Config"],
            "objective": "Mantener lógica pura y tipada: parse_args determinista y payload estable.",
        },
        "PY008": {
            "files": ["training/python_mastery/subprocess_utils.py"],
            "functions": ["run_cmd"],
            "objective": "Wrapper seguro de subprocess sin shell=True, con timeout, stdout/stderr, y raising cuando returncode!=0 si check=True.",
        },
        "PY009": {
            "files": ["training/python_mastery/http_client.py"],
            "functions": ["get_json_with_retries"],
            "objective": "Cliente GET JSON mockeable con retries y backoff/jitter; reintenta ante >=500 o excepciones.",
        },
        "PY010": {
            "files": ["training/python_mastery/sqlite_repo.py"],
            "functions": ["SQLiteRepo.upsert_item", "SQLiteRepo.get_item", "SQLiteRepo.list_items"],
            "objective": "Repositorio SQLite offline-first con WAL/PRAGMA y CRUD estable.",
        },
        "PY011": {
            "files": ["training/python_mastery/async_downloader.py"],
            "functions": ["gather_with_limit"],
            "objective": "Concurrencia con límite (Semaphore), timeout opcional y cancelación limpia.",
        },
        "PY012": {
            "files": ["training/python_mastery/packaging_sample/pyproject.toml", "training/python_mastery/packaging_sample/atlas_py_tool/cli.py"],
            "functions": ["main"],
            "objective": "Packaging sample con pyproject y console_scripts; CLI smoke --ping -> pong.",
        },
    }
    obj = lesson_objectives.get(lid, {})
    diff_objective = {
        "lesson_id": lid,
        "files": obj.get("files", []),
        "symbols": obj.get("functions", []),
        "objective": obj.get("objective", "Extender mapping de objetivos para esta lección."),
        "acceptance_tests": pytest_nodeids,
        "rule": "Modificar primero `training/python_mastery/*`. Solo tocar tests si están mal especificados.",
        "patch_template": _patch_template_for(lid),
        "edge_case_checklist": _edge_case_checklist(lid, pytest_output, missing_files, pytest_nodeids),
    }

    # Deduplicar hints
    seen = set()
    uniq_files = []
    for f in files_hint:
        f = str(f).strip()
        if not f or f in seen:
            continue
        seen.add(f)
        uniq_files.append(f)

    return {
        "lesson_id": lid,
        "generated_ts": _now_iso(),
        "summary": f"Remediación para {lid}: hacer pasar tests y cumplir criterios.",
        "files_to_check": uniq_files[:20],
        "pytest_nodeids": pytest_nodeids,
        "diff_objective": diff_objective,
        "steps": steps,
        "evidence_snippet": pytest_output[-4000:],
    }


def campaign_state_path(repo_root: Optional[Path] = None) -> Path:
    root = (repo_root or _repo_root_fallback()).resolve()
    p = root / "snapshots" / "learning" / "PYTHON_MASTERY_CAMPAIGN.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    return p


def _atomic_write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    content = json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True)
    tmp_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=str(path.parent),
            delete=False,
            suffix=".tmp",
        ) as f:
            tmp_name = f.name
            f.write(content)
        os.replace(tmp_name, str(path))
    finally:
        try:
            if tmp_name and Path(tmp_name).exists():
                Path(tmp_name).unlink(missing_ok=True)
        except Exception:
            pass


def load_campaign_state(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {
            "campaign_id": f"py_campaign_{int(datetime.now(timezone.utc).timestamp())}",
            "created_ts": _now_iso(),
            "updated_ts": _now_iso(),
            "status": "new",
            "current_lesson_id": None,
            "attempts": {},  # lesson_id -> int
            "completed": [],  # list of {lesson_id, score, ts}
            "failed": [],  # list of {lesson_id, score, ts}
        }
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("state not dict")
        data.setdefault("attempts", {})
        data.setdefault("completed", [])
        data.setdefault("failed", [])
        data.setdefault("status", "active")
        data["updated_ts"] = _now_iso()
        return data
    except Exception:
        # Estado corrupto: crear nuevo, pero no borrar el anterior.
        return {
            "campaign_id": f"py_campaign_recover_{int(datetime.now(timezone.utc).timestamp())}",
            "created_ts": _now_iso(),
            "updated_ts": _now_iso(),
            "status": "recovered",
            "current_lesson_id": None,
            "attempts": {},
            "completed": [],
            "failed": [],
        }


def _hydrate_tutor_progress(tutor: Any, state: Dict[str, Any]) -> None:
    """Inyecta progreso persistido en el tutor (in-memory)."""
    tutor.completed_lessons = list(state.get("completed") or [])
    tutor.failed_lessons = list(state.get("failed") or [])


def _persist_from_tutor(state: Dict[str, Any], tutor: Any) -> Dict[str, Any]:
    """Refleja progreso del tutor al estado."""
    state["completed"] = list(getattr(tutor, "completed_lessons", []) or [])
    state["failed"] = list(getattr(tutor, "failed_lessons", []) or [])
    state["updated_ts"] = _now_iso()
    return state


def _ensure_python_curriculum(tutor: Any) -> None:
    # Forzar currículum Python Mastery (offline-first).
    tutor.design_curriculum(
        robot_capabilities=["repo_tools", "cli", "testing", "offline_first"],
        learning_goals=["python_mastery", "python_scripting", "pytest", "sqlite", "tooling"],
        time_horizon_days=int(os.getenv("PYTHON_MASTERY_HORIZON_DAYS", "30") or 30),
        difficulty_level=os.getenv("PYTHON_MASTERY_DIFFICULTY", "progressive"),
    )


def _set_loop_current_lesson(loop: Any, lesson: Optional[Dict[str, Any]]) -> None:
    """Replica la parte relevante de start_daily_routine para campaña."""
    from datetime import datetime as _dt

    loop.current_lesson = lesson
    loop.lesson_start_time = _dt.now()
    if lesson:
        loop.daily_report = {
            "lesson_id": lesson.get("lesson_id"),
            "lesson_name": lesson.get("name"),
            "start_time": loop.lesson_start_time.isoformat(),
            "tasks_completed": [],
            "challenges": [],
            "knowledge_learned": [],
            "uncertainty_episodes": 0,
            "times_asked_for_help": 0,
        }


def start_or_resume_campaign(
    *,
    tutor: Any,
    loop: Any,
    repo_root: Optional[Path] = None,
    reset: bool = False,
) -> Dict[str, Any]:
    """Arranca o reanuda la campaña y deja una lección PY activa."""
    path = campaign_state_path(repo_root)
    if reset and path.exists():
        # Reset seguro: no borra datos ajenos, solo el estado.
        try:
            path.unlink(missing_ok=True)
        except Exception:
            pass
    state = load_campaign_state(path)
    _hydrate_tutor_progress(tutor, state)
    _ensure_python_curriculum(tutor)

    # Seleccionar siguiente lección pendiente respetando prereqs.
    lesson = None
    try:
        lesson = tutor.assign_daily_lesson()
    except Exception:
        lesson = None
    _set_loop_current_lesson(loop, lesson)
    state["current_lesson_id"] = (lesson or {}).get("lesson_id") if isinstance(lesson, dict) else None
    state["status"] = "active" if lesson else "completed"
    state = _persist_from_tutor(state, tutor)
    _atomic_write_json(path, state)
    return {"state_path": str(path), "state": state, "current_lesson": lesson}


def campaign_step(
    *,
    tutor: Any,
    loop: Any,
    repo_root: Optional[Path] = None,
    lesson_id: Optional[str] = None,
    timeout_s: int = 180,
    max_attempts: int = 1,
    backoff_s: float = 0.0,
    backoff_factor: float = 2.0,
    require_change: bool = False,
    auto_remediate: bool = False,
) -> Dict[str, Any]:
    """Ejecuta un paso: evaluar la lección actual y avanzar si aprueba.

    Si `max_attempts` > 1, reintenta (run-until-pass) con backoff.
    Si `require_change`=True, solo reintenta cuando detecta cambios en `training/python_mastery/` o `tests/python_mastery/`.
    """
    from .python_mastery_evaluator import evaluate_python_mastery_lesson

    root = (repo_root or _repo_root_fallback()).resolve()
    path = campaign_state_path(root)
    state = load_campaign_state(path)
    _hydrate_tutor_progress(tutor, state)
    _ensure_python_curriculum(tutor)

    # Determinar lección actual.
    lid = (lesson_id or "").strip().upper()
    if not lid:
        try:
            current = getattr(loop, "current_lesson", None) or getattr(tutor, "current_lesson", None)
            if isinstance(current, dict):
                lid = (current.get("lesson_id") or "").strip().upper()
        except Exception:
            lid = ""
    if not lid:
        # No hay current lesson: intentar asignar una.
        lesson = tutor.assign_daily_lesson()
        _set_loop_current_lesson(loop, lesson)
        lid = (lesson or {}).get("lesson_id", "") if isinstance(lesson, dict) else ""
        lid = (lid or "").strip().upper()

    if not lid:
        state["status"] = "completed"
        state = _persist_from_tutor(state, tutor)
        _atomic_write_json(path, state)
        return {"ok": True, "status": "completed", "state": state, "evaluation": None}

    state["current_lesson_id"] = lid
    max_attempts = max(1, int(max_attempts or 1))
    max_attempts = min(max_attempts, 20)  # guardrail
    backoff_s = max(0.0, float(backoff_s or 0.0))
    backoff_factor = float(backoff_factor or 2.0)
    if backoff_factor < 1.0:
        backoff_factor = 1.0

    evaluations = []
    next_lesson = None
    baseline_mtime = _training_tree_mtime(root) if require_change else 0.0
    last_remediation: Optional[Dict[str, Any]] = None

    for attempt_idx in range(max_attempts):
        # Persist attempt counter per evaluation run
        state["attempts"][lid] = int(state.get("attempts", {}).get(lid, 0)) + 1

        evaluation = evaluate_python_mastery_lesson(
            lesson_id=lid,
            repo_root=root,
            timeout_s=int(timeout_s or 180),
        )

        # Registrar en tutor (offline-first)
        if getattr(tutor, "record_offline_evaluation", None):
            try:
                evaluation = tutor.record_offline_evaluation(
                    lesson_id=lid,
                    evaluation=evaluation,
                    lesson=getattr(loop, "current_lesson", None),
                )
            except Exception:
                pass

        evaluations.append(evaluation)
        passed = bool(evaluation.get("passed")) if isinstance(evaluation, dict) else False
        if passed:
            try:
                next_lesson = tutor.assign_daily_lesson()
            except Exception:
                next_lesson = None
            _set_loop_current_lesson(loop, next_lesson)
            state["current_lesson_id"] = (next_lesson or {}).get("lesson_id") if isinstance(next_lesson, dict) else None
            state["status"] = "active" if next_lesson else "completed"
            break
        if auto_remediate and isinstance(evaluation, dict):
            try:
                last_remediation = generate_remediation_plan(lid, evaluation)
                state["last_remediation"] = last_remediation
                state["last_remediation_ts"] = _now_iso()
                # Best-effort: registrar en memoria (si está disponible)
                try:
                    from modules.humanoid.memory_engine.store import memory_write

                    memory_write(
                        thread_id=None,
                        kind="summary",
                        payload={
                            "title": f"python_mastery_remediation:{lid}",
                            "content": json.dumps(last_remediation, ensure_ascii=False),
                        },
                        task_id=lid,
                    )
                except Exception:
                    pass
            except Exception:
                last_remediation = None

        # Si no pasó y quedan intentos, esperar según estrategia
        if attempt_idx < max_attempts - 1:
            sleep_s = backoff_s * (backoff_factor**attempt_idx)
            sleep_s = min(10.0, max(0.0, float(sleep_s)))
            if require_change:
                deadline = time.time() + max(0.1, sleep_s)
                while time.time() < deadline:
                    cur = _training_tree_mtime(root)
                    if cur > baseline_mtime:
                        baseline_mtime = cur
                        break
                    time.sleep(0.05)
            else:
                if sleep_s > 0:
                    time.sleep(sleep_s)

    # Si nunca avanzó, permanece activo en misma lección
    if state.get("status") != "completed" and not next_lesson:
        state["status"] = "active"

    state = _persist_from_tutor(state, tutor)
    _atomic_write_json(path, state)

    last_eval = evaluations[-1] if evaluations else None
    return {
        "ok": True,
        "status": state.get("status"),
        "lesson_id": lid,
        "attempt": state["attempts"].get(lid),
        "attempts_run": len(evaluations),
        "evaluation": last_eval,
        "evaluations": evaluations[-5:],  # recorte defensivo
        "remediation_plan": last_remediation or state.get("last_remediation"),
        "next_lesson": next_lesson,
        "state": state,
        "state_path": str(path),
    }


def campaign_run(
    *,
    tutor: Any,
    loop: Any,
    repo_root: Optional[Path] = None,
    reset: bool = False,
    max_steps: int = 5,
    max_seconds: float = 60.0,
    step_timeout_s: int = 180,
    step_max_attempts: int = 1,
    step_backoff_s: float = 0.0,
    step_backoff_factor: float = 2.0,
    step_require_change: bool = False,
    step_auto_remediate: bool = False,
) -> Dict[str, Any]:
    """Ejecuta múltiples pasos de campaña en una sola corrida.

    Guarda estado persistido tras cada step (via campaign_step). Termina cuando:
    - status == completed, o
    - steps alcanza max_steps, o
    - tiempo excede max_seconds.
    """
    root = (repo_root or _repo_root_fallback()).resolve()
    started = start_or_resume_campaign(tutor=tutor, loop=loop, repo_root=root, reset=reset)
    t0 = time.time()
    steps = []
    max_steps = max(1, int(max_steps or 1))
    max_steps = min(max_steps, 200)  # guardrail
    max_seconds = max(1.0, float(max_seconds or 60.0))

    status = started["state"].get("status")
    if status == "completed":
        return {
            "ok": True,
            "status": "completed",
            "steps_run": 0,
            "seconds": 0.0,
            "start": started,
            "steps": [],
        }

    for _ in range(max_steps):
        if (time.time() - t0) > max_seconds:
            break
        res = campaign_step(
            tutor=tutor,
            loop=loop,
            repo_root=root,
            lesson_id=None,
            timeout_s=int(step_timeout_s or 180),
            max_attempts=int(step_max_attempts or 1),
            backoff_s=float(step_backoff_s or 0.0),
            backoff_factor=float(step_backoff_factor or 2.0),
            require_change=bool(step_require_change),
            auto_remediate=bool(step_auto_remediate),
        )
        steps.append(
            {
                "lesson_id": res.get("lesson_id"),
                "status": res.get("status"),
                "attempts_run": res.get("attempts_run"),
                "passed": bool((res.get("evaluation") or {}).get("passed")),
                "score": (res.get("evaluation") or {}).get("score"),
                "next_lesson_id": (res.get("next_lesson") or {}).get("lesson_id") if isinstance(res.get("next_lesson"), dict) else None,
                "remediation_plan": res.get("remediation_plan"),
            }
        )
        if res.get("status") == "completed":
            status = "completed"
            break
        status = "active"

    return {
        "ok": True,
        "status": status,
        "steps_run": len(steps),
        "seconds": round(time.time() - t0, 2),
        "start": {"current_lesson": started.get("current_lesson"), "state": started.get("state"), "state_path": started.get("state_path")},
        "steps": steps,
    }

