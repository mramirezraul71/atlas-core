# ATLAS — Python Mastery (Entrenamiento Offline-First)

## Objetivo
Convertir a **ATLAS** en un operador **Senior Python**: dominio del lenguaje + confección de scripts + tooling + tests + persistencia offline (SQLite) + prácticas de producción.

Este entrenamiento está diseñado para funcionar **sin internet** y **sin API keys**, usando un currículum local (fallback) y la infraestructura existente de aprendizaje (`brain/learning/*`).

---

## Arquitectura (cómo se enchufa)
- **Tutor**: `brain/learning/ai_tutor.py` (clase `AITutor`)
- **Currículum Python offline**: `brain/learning/python_mastery_curriculum.py`
- **Loop diario**: `brain/learning/continual_learning_loop.py`
- **API**: `atlas_adapter/atlas_http_api.py`

El `AITutor` intenta diseñar currículum con LLM si está habilitado; si no, cae automáticamente al **currículum offline** de Python cuando los objetivos incluyen `python_*`, `pytest`, `cli`, etc.

---

## Operación (API)

### 1) Bootstrap del track Python
Inicializa el currículum de Python Mastery y preasigna una lección elegible.

- `POST /api/learning/python-mastery/bootstrap`

Respuesta: `data.count` y `data.current_lesson`.

Variables opcionales:
- `PYTHON_MASTERY_HORIZON_DAYS` (default 30)
- `PYTHON_MASTERY_DIFFICULTY` (default progressive)

### 2) Iniciar rutina diaria
Asigna la lección del día (según prerequisitos y progreso) y deja preparado el reporte.

- `POST /api/learning/daily-routine/start`

### 2.1) Evaluar una lección (score objetivo)
Corre evaluación determinista: **archivos requeridos + pytest** (sin red).

- `POST /api/learning/python-mastery/evaluate`

Body ejemplo:
```json
{ "lesson_id": "PY001", "timeout_s": 180 }
```

---

## Modo Campaña (recomendado)
El modo campaña hace que ATLAS progrese automáticamente por `PY001..PY012` con estado persistido.

### Iniciar / Reanudar
- `POST /api/learning/python-mastery/campaign/start`

Body:
```json
{ "reset": false }
```

### Paso de campaña (evalúa y avanza)
- `POST /api/learning/python-mastery/campaign/step`

Body:
```json
{
  "lesson_id": null,
  "timeout_s": 180,
  "max_attempts": 1,
  "backoff_s": 0.0,
  "backoff_factor": 2.0,
  "require_change": false,
  "auto_remediate": true
}
```

Si `lesson_id` es `null`, usa la lección activa de la campaña.

Notas:
- `max_attempts>1` activa modo **run-until-pass** (reintenta la MISMA lección hasta aprobar o agotar intentos).
- `require_change=true` hace que los reintentos solo ocurran si detecta cambios en `training/python_mastery/` o `tests/python_mastery/` durante el backoff (evita loops inútiles).
- `auto_remediate=true` adjunta un **plan de corrección** (archivos/steps/comando pytest) en la respuesta y lo persiste en el estado.

### Estado persistido
- `GET /api/learning/python-mastery/campaign/state`

Se guarda en: `snapshots/learning/PYTHON_MASTERY_CAMPAIGN.json`

### Ejecutar campaña en una sola orden (loop)
- `POST /api/learning/python-mastery/campaign/run`

Body ejemplo:
```json
{
  "reset": false,
  "max_steps": 10,
  "max_seconds": 60.0,
  "step_timeout_s": 180,
  "step_max_attempts": 3,
  "step_backoff_s": 1.0,
  "step_backoff_factor": 2.0,
  "step_require_change": true,
  "step_auto_remediate": true
}
```

Esto intenta avanzar varias lecciones en una sola llamada, pero se detiene por **tiempo** o **cantidad de pasos** para no colgar la API.

### 3) Ejecutar “situaciones” (experiencias)
ATLAS procesa experiencias y aprende patrones.

- `POST /api/learning/process-situation`

### 4) Cerrar el día (evaluación)
Genera reporte de desempeño y aplica correcciones del tutor.

- `POST /api/learning/daily-routine/end-report`

---

## Operación (Tutor)

### Diseñar currículum manualmente
Si quieres empujar objetivos específicos:

- `POST /api/learning/tutor/design-curriculum`

Body ejemplo:
```json
{
  "robot_capabilities": ["cli", "repo_tools", "testing"],
  "learning_goals": ["python_mastery", "pytest", "sqlite"],
  "time_horizon_days": 30,
  "difficulty_level": "progressive"
}
```

---

## Reglas del Instructor (ATLAS)
- **No `print`**: usar `logging` con niveles, y errores con contexto.
- **No strings para rutas**: usar `pathlib.Path`.
- **Errores accionables**: timeouts, retries, exit codes.
- **Tests primero para scripts**: `pytest` + fixtures + mocks.
- **Offline-first**: persistir en SQLite donde aplique, evitar depender de red.
- **Separar lógica pura vs I/O**: facilita testing y refactor.

---

## Notas
- Este track no fuerza cambios en producción automáticamente. Es un sistema de entrenamiento: crea misiones y criterios.
- Los tests del track Python se activan con `RUN_PYTHON_MASTERY=1`. Por defecto están **skip** para no romper el repo.
- El endpoint `python-mastery/evaluate` activa esos tests automáticamente para el scoring.

