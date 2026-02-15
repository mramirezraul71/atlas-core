"""Currículum offline: Python Mastery para ATLAS.

Diseñado para que ATLAS aprenda Python de forma práctica (scripts, tooling, calidad, tests).
No requiere LLM externo. Las lecciones están pensadas como "misiones" que el robot puede
ejecutar en el repo, medir con tests y consolidar en memoria.
"""

from __future__ import annotations

from typing import Any, Dict, List


def generate_python_mastery_curriculum(
    time_horizon_days: int = 30,
    difficulty_level: str = "progressive",
) -> List[Dict[str, Any]]:
    """Devuelve una lista de lecciones (dicts) en el formato esperado por `AITutor`.

    - time_horizon_days: sirve para recortar/expandir el set.
    - difficulty_level: "beginner" | "intermediate" | "advanced" | "progressive"
    """

    # Nota: las `success_criteria` están formuladas para que luego puedan evaluarse
    # con checks automáticos (pytest, lint, métricas, etc.).
    lessons: List[Dict[str, Any]] = [
        {
            "lesson_id": "PY001",
            "name": "Python Core: tipos, mutabilidad y control de flujo",
            "difficulty": "beginner",
            "duration_hours": 2,
            "prerequisites": [],
            "description": "Dominar el núcleo del lenguaje (valores, referencias, truthiness, loops, slicing).",
            "learning_objectives": [
                "Distinguir mutabilidad vs inmutabilidad y efectos laterales",
                "Usar comprehensions y unpacking con seguridad",
                "Escribir funciones pequeñas con entradas/salidas claras",
            ],
            "tasks": [
                {
                    "task_id": "PY001-T1",
                    "description": "Implementar utilidades puras en `training/python_mastery/utils.py`: normalize_text, chunk_list, safe_get.",
                    "success_criteria": "pytest pasa: tests/python_mastery/test_py001_utils.py (RUN_PYTHON_MASTERY=1)",
                }
            ],
            "evaluation_criteria": {"excellent": ">= 90", "good": ">= 75", "acceptable": ">= 60"},
            "next_lessons": ["PY002"],
        },
        {
            "lesson_id": "PY002",
            "name": "Entornos, imports y estructura de proyectos",
            "difficulty": "beginner",
            "duration_hours": 2,
            "prerequisites": ["PY001"],
            "description": "Entender venv, pip, dependencias, imports y layout mantenible.",
            "learning_objectives": [
                "Usar venv/pip sin contaminar el sistema",
                "Comprender módulos/paquetes y resolución de imports",
                "Separar código: core, adapters, scripts, tests",
            ],
            "tasks": [
                {
                    "task_id": "PY002-T1",
                    "description": "Crear un paquete interno mínimo con __init__.py y una función expuesta.",
                    "success_criteria": "imports limpios; sin sys.path hacks; tests pasan",
                }
            ],
            "evaluation_criteria": {"excellent": ">= 90", "good": ">= 75", "acceptable": ">= 60"},
            "next_lessons": ["PY003"],
        },
        {
            "lesson_id": "PY003",
            "name": "Scripts Senior: argparse + logging + exit codes",
            "difficulty": "beginner",
            "duration_hours": 3,
            "prerequisites": ["PY001"],
            "description": "Escritura de herramientas CLI robustas para automatización.",
            "learning_objectives": [
                "Construir CLIs con subcomandos y help consistente",
                "Usar logging (no print) y niveles adecuados",
                "Manejar errores con códigos de salida y mensajes útiles",
            ],
            "tasks": [
                {
                    "task_id": "PY003-T1",
                    "description": "Implementar core en `training/python_mastery/inventory.py` y CLI en `training/python_mastery/cli.py`.",
                    "success_criteria": "pytest pasa: tests/python_mastery/test_py003_inventory.py (RUN_PYTHON_MASTERY=1)",
                }
            ],
            "evaluation_criteria": {"excellent": ">= 90", "good": ">= 75", "acceptable": ">= 60"},
            "next_lessons": ["PY004"],
        },
        {
            "lesson_id": "PY004",
            "name": "Errores, excepciones y contexto (with)",
            "difficulty": "beginner",
            "duration_hours": 2,
            "prerequisites": ["PY001"],
            "description": "Diseñar flujos resilientes: fallar bien, registrar bien.",
            "learning_objectives": [
                "Capturar excepciones con contexto sin ocultar fallos",
                "Crear excepciones propias cuando agrega claridad",
                "Usar context managers para recursos (archivos, locks, db)",
            ],
            "tasks": [
                {
                    "task_id": "PY004-T1",
                    "description": "Implementar lock cooperativo en `training/python_mastery/locks.py`.",
                    "success_criteria": "pytest pasa: tests/python_mastery/test_py004_locks.py (RUN_PYTHON_MASTERY=1)",
                }
            ],
            "evaluation_criteria": {"excellent": ">= 90", "good": ">= 75", "acceptable": ">= 60"},
            "next_lessons": ["PY005"],
        },
        {
            "lesson_id": "PY005",
            "name": "E/S y rutas: pathlib, JSON/CSV y atomic writes",
            "difficulty": "intermediate",
            "duration_hours": 3,
            "prerequisites": ["PY003", "PY004"],
            "description": "I/O confiable (Windows-safe), formatos comunes, performance básica.",
            "learning_objectives": [
                "Usar pathlib para rutas y globbing",
                "Escritura atómica de archivos (no corrupción ante cortes)",
                "Serialización segura (JSON) y parsing CSV",
            ],
            "tasks": [
                {
                    "task_id": "PY005-T1",
                    "description": "Implementar `atomic_write_json` en `training/python_mastery/io_utils.py`.",
                    "success_criteria": "pytest pasa: tests/python_mastery/test_py005_atomic_write.py (RUN_PYTHON_MASTERY=1)",
                }
            ],
            "evaluation_criteria": {"excellent": ">= 90", "good": ">= 75", "acceptable": ">= 60"},
            "next_lessons": ["PY006"],
        },
        {
            "lesson_id": "PY006",
            "name": "Testing: pytest, fixtures, mocks y tests de CLI",
            "difficulty": "intermediate",
            "duration_hours": 4,
            "prerequisites": ["PY003", "PY004"],
            "description": "Validación automática: el código se prueba, o no existe.",
            "learning_objectives": [
                "Escribir tests unitarios con fixtures y parametrización",
                "Mock de I/O, tiempo, red y filesystem",
                "Testing de CLIs (captura de stdout/stderr y exit codes)",
            ],
            "tasks": [
                {
                    "task_id": "PY006-T1",
                    "description": "Hacer que el CLI inventory produzca JSON válido y exit code 0.",
                    "success_criteria": "pytest pasa: tests/python_mastery/test_py006_cli_inventory_json.py (RUN_PYTHON_MASTERY=1)",
                }
            ],
            "evaluation_criteria": {"excellent": ">= 90", "good": ">= 75", "acceptable": ">= 60"},
            "next_lessons": ["PY007"],
        },
        {
            "lesson_id": "PY007",
            "name": "Tipado útil: typing, dataclasses y diseño limpio",
            "difficulty": "intermediate",
            "duration_hours": 3,
            "prerequisites": ["PY006"],
            "description": "Interfaces claras y refactors seguros.",
            "learning_objectives": [
                "Usar dataclasses para configuración y modelos simples",
                "Aplicar typing pragmático (sin burocracia)",
                "Separar lógica pura de I/O para testabilidad",
            ],
            "tasks": [
                {
                    "task_id": "PY007-T1",
                    "description": "Implementar `Config`, `parse_args` y lógica pura en `training/python_mastery/app_refactor.py`.",
                    "success_criteria": "pytest pasa: tests/python_mastery/test_py007_refactor.py (RUN_PYTHON_MASTERY=1)",
                }
            ],
            "evaluation_criteria": {"excellent": ">= 90", "good": ">= 75", "acceptable": ">= 60"},
            "next_lessons": ["PY008"],
        },
        {
            "lesson_id": "PY008",
            "name": "Subprocess y automatización de sistema (segura)",
            "difficulty": "intermediate",
            "duration_hours": 3,
            "prerequisites": ["PY004", "PY007"],
            "description": "Ejecutar comandos con control: timeouts, captura de salida, códigos.",
            "learning_objectives": [
                "Usar subprocess sin shell injection",
                "Implementar timeouts y reintentos",
                "Parsear salidas y devolver errores accionables",
            ],
            "tasks": [
                {
                    "task_id": "PY008-T1",
                    "description": "Implementar `run_cmd` en `training/python_mastery/subprocess_utils.py`.",
                    "success_criteria": "pytest pasa: tests/python_mastery/test_py008_subprocess.py (RUN_PYTHON_MASTERY=1)",
                }
            ],
            "evaluation_criteria": {"excellent": ">= 90", "good": ">= 75", "acceptable": ">= 60"},
            "next_lessons": ["PY009"],
        },
        {
            "lesson_id": "PY009",
            "name": "Red y APIs: requests/httpx, timeouts, retries, rate limit",
            "difficulty": "intermediate",
            "duration_hours": 4,
            "prerequisites": ["PY006", "PY008"],
            "description": "Consumir APIs sin colgarse: timeouts, backoff, errores.",
            "learning_objectives": [
                "Configurar timeouts y sesiones",
                "Implementar retries con backoff y jitter",
                "Distinguir errores transitorios vs permanentes",
            ],
            "tasks": [
                {
                    "task_id": "PY009-T1",
                    "description": "Implementar `get_json_with_retries` en `training/python_mastery/http_client.py` (mockeable).",
                    "success_criteria": "pytest pasa: tests/python_mastery/test_py009_http_retries.py (RUN_PYTHON_MASTERY=1)",
                }
            ],
            "evaluation_criteria": {"excellent": ">= 90", "good": ">= 75", "acceptable": ">= 60"},
            "next_lessons": ["PY010"],
        },
        {
            "lesson_id": "PY010",
            "name": "Persistencia local: SQLite, transacciones e índices",
            "difficulty": "intermediate",
            "duration_hours": 4,
            "prerequisites": ["PY005", "PY006"],
            "description": "Offline-first real: escribir/leer sin perder datos.",
            "learning_objectives": [
                "Modelar tablas y consultas con sqlite3",
                "Usar transacciones y PRAGMAs de seguridad",
                "Agregar índices para acelerar consultas",
            ],
            "tasks": [
                {
                    "task_id": "PY010-T1",
                    "description": "Implementar `SQLiteRepo` en `training/python_mastery/sqlite_repo.py`.",
                    "success_criteria": "pytest pasa: tests/python_mastery/test_py010_sqlite_repo.py (RUN_PYTHON_MASTERY=1)",
                }
            ],
            "evaluation_criteria": {"excellent": ">= 90", "good": ">= 75", "acceptable": ">= 60"},
            "next_lessons": ["PY011"],
        },
        {
            "lesson_id": "PY011",
            "name": "Asyncio: concurrencia con propósito",
            "difficulty": "advanced",
            "duration_hours": 4,
            "prerequisites": ["PY009"],
            "description": "Usar async cuando suma (I/O bound), sin crear caos.",
            "learning_objectives": [
                "Entender event loop, tasks, cancellation",
                "Limitar concurrencia (semaphores) y timeouts",
                "Diseñar APIs sync/async sin duplicar lógica",
            ],
            "tasks": [
                {
                    "task_id": "PY011-T1",
                    "description": "Implementar `gather_with_limit` en `training/python_mastery/async_downloader.py`.",
                    "success_criteria": "pytest pasa: tests/python_mastery/test_py011_asyncio.py (RUN_PYTHON_MASTERY=1)",
                }
            ],
            "evaluation_criteria": {"excellent": ">= 90", "good": ">= 75", "acceptable": ">= 60"},
            "next_lessons": ["PY012"],
        },
        {
            "lesson_id": "PY012",
            "name": "Packaging y entrega: pyproject, entrypoints y versionado",
            "difficulty": "advanced",
            "duration_hours": 4,
            "prerequisites": ["PY006", "PY007"],
            "description": "Convertir scripts en producto: instalable, ejecutable, versionado.",
            "learning_objectives": [
                "Definir pyproject y dependencias correctamente",
                "Crear entrypoints de consola",
                "Versionar y generar release notes (mínimo)",
            ],
            "tasks": [
                {
                    "task_id": "PY012-T1",
                    "description": "Crear proyecto ejemplo en `training/python_mastery/packaging_sample/` con `pyproject.toml` y entrypoint.",
                    "success_criteria": "pytest pasa: tests/python_mastery/test_py012_packaging.py (RUN_PYTHON_MASTERY=1)",
                }
            ],
            "evaluation_criteria": {"excellent": ">= 90", "good": ">= 75", "acceptable": ">= 60"},
            "next_lessons": [],
        },
    ]

    # Ajuste simple por horizonte: si es corto, recorta a lo esencial.
    if time_horizon_days <= 7:
        return lessons[:4]
    if time_horizon_days <= 14:
        return lessons[:7]
    if time_horizon_days <= 21:
        return lessons[:10]

    # difficulty_level: si se pide explícito, filtra.
    dl = (difficulty_level or "progressive").strip().lower()
    if dl in ("beginner", "intermediate", "advanced"):
        allowed = {dl}
        # "intermediate" incluye beginner como prereq implícito.
        if dl == "intermediate":
            allowed |= {"beginner"}
        if dl == "advanced":
            allowed |= {"beginner", "intermediate", "advanced"}
        return [l for l in lessons if (l.get("difficulty") or "").lower() in allowed]

    return lessons

