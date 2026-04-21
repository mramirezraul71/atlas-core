# PLAN_STEP_C — Limpieza del wiring en `atlas_adapter/atlas_http_api.py`

> Versión aprobada por el owner el 2026-04-21. Documenta el Paso C de
> la reestructuración de Atlas Push. Referencias cruzadas:
> `ARCHITECTURE.md`, `CURRENT_STATE.md`, `DEFECTS.md`,
> `PLAN_STEP_B.md`.

## 1. Objetivo

Dejar `atlas_adapter/atlas_http_api.py` con:

- Una sola definición por endpoint (`/status`, `/tools`, `/execute`,
  `/modules`, `/intent`).
- Un único bloque de imports, sin duplicados.
- Una única forma de resolver `command_router.handle`: el import de
  paquete `from modules.command_router import handle`, no
  `importlib.util` ni la ruta absoluta
  `C:\ATLAS\modules\command_router.py`.
- El `IntentRouter` de Paso B como único camino para `/intent`.

Todo con **paridad byte a byte** del JSON público, verificada por los
smokes A2 y B ya existentes más un test nuevo de import/duplicados
(ver §6).

No se cambia ningún otro archivo funcional del repo. No se toca
`atlas_push/`, `modules/command_router.py`, ni los canales Telegram /
GUI.

## 2. Principios (vinculantes)

1. Paridad estricta de la superficie HTTP. El shape de `/status`,
   `/tools`, `/modules`, `/execute` y `/intent` no cambia. Claves,
   orden, tipos: iguales. El contrato B (`ok=True` salvo excepción en
   `/intent`) se mantiene.
2. `atlas_push/` no se modifica en C. C sólo limpia el adaptador HTTP.
3. Ningún import nuevo de `brain_core`, `mission_manager`,
   `safety_kernel`, `state_bus`, `arbitration`, `policy_store`. Regla
   de oro intacta.
4. Portabilidad: ningún literal `C:\ATLAS`, `C:\ATLAS_PUSH`, ninguna
   llamada `importlib.util.spec_from_file_location` dentro de este
   archivo. El módulo debe poder importarse en Linux/macOS/Windows
   sin inventar un filesystem específico.
5. Config por entorno: si en algún punto se necesita una ruta raíz,
   se lee de `ATLAS_ROOT` (mismo patrón ya usado por
   `modules/command_router.py`) con un default razonable, no se
   hardcodea.
6. Retrocompatibilidad con `03_run_atlas_api.ps1`: el arranque sigue
   siendo `uvicorn atlas_adapter.atlas_http_api:app` en
   `127.0.0.1:8791`. No cambian flags ni variables.
7. Defensa: cualquier fallo de import de `command_router` o de
   `atlas_push.intents` se deja fallar claro en import time (igual
   que hoy), no se enmascara.

## 3. Estado previo (diagnóstico sobre `main@f4acd871`)

Sobre `atlas_adapter/atlas_http_api.py`:

- Líneas 6–18: carga de `command_router` vía `importlib.util` desde
  `Path(r"C:\ATLAS\modules\command_router.py")`. Funcionaba en Windows
  del owner; fuera de eso rompe. Es el defecto **D-003**.
- Líneas 53–63 y 65–75: `@app.get("/modules")` declarado dos veces,
  idéntico. FastAPI conservaba la última; la primera era código
  muerto.
- Líneas 85–96: `@app.post("/intent")` v1 eco (sin router). Muerto
  desde B2, lo sobrescribía la segunda definición.
- Líneas 97–137: bloque canónico `/intent` (B2), con reimports de
  `BaseModel`, `Optional`, `Any`, `time` ya declarados arriba, y
  redeclaración de `IntentIn`. Defecto **D-002**.
- Línea 2 (docstring): mencionaba la ruta Windows como "fuente" del
  router, incoherente con el import real.

Runtime antes de C: correcto, porque Python y FastAPI conservan la
última definición. Por eso los smokes B pasaron. C limpia sin cambiar
el comportamiento observable.

## 4. Alcance de C

### Se toca (sólo un archivo de producción y dos de docs)

- `atlas_adapter/atlas_http_api.py` (código, en C1).
- `tests/unit/test_http_api_contract.py` (nuevo, tests, en C2).
- `docs/atlas_push/PLAN_STEP_C.md` (este archivo, en C3).
- `docs/atlas_push/DEFECTS.md` (cierre de D-002 y D-003, en C3).

### No se toca en C

- `atlas_push/` entero.
- `modules/command_router.py`.
- `modules/atlas_remote_api.py` (D-001, fuera de servicio; archivable
  en fase posterior por decisión ya documentada en `DEFECTS.md`).
- `03_run_atlas_api.ps1` y cualquier otro `.ps1`.
- Canales (Telegram, GUI, bridge).
- `docs/atlas_push/ARCHITECTURE.md`, `CURRENT_STATE.md`.
- Tests unitarios de Paso B (`tests/unit/test_intent_router.py`): no
  se tocan, deben seguir en verde tal cual.

## 5. C1 — Limpieza de wiring en `atlas_http_api.py`

### 5.1 Imports (tope del módulo)

Un único bloque ordenado:

```python
from typing import Any, Optional
import time

from fastapi import FastAPI
from pydantic import BaseModel

from modules.command_router import handle
from atlas_push.intents import IntentRouter
```

Se elimina:

- `import importlib.util`
- `from pathlib import Path`
- La segunda aparición de `from pydantic import BaseModel`
- La segunda aparición de `from typing import Any, Optional`
- El segundo `import time`
- Las constantes `ATLAS_ROOT = Path(r"C:\ATLAS")` y `ROUTER_PATH`
- La función `load_handle()` y la línea `handle = load_handle()`

`handle` pasa a venir directamente del paquete
`modules.command_router`, que es la forma en que se arranca el
servicio en el `03_run_atlas_api.ps1` actual (uvicorn ya añade el
repo a `sys.path`). Esto elimina **D-003** sin introducir una nueva
capa de config.

### 5.2 Docstring

Se actualiza la cabecera para describir el contrato real, sin rutas
Windows:

```
"""ATLAS HTTP API adapter.

Expone /status, /tools, /modules, /execute, /intent usando
`modules.command_router.handle` directamente, y envolviendo /intent
con `atlas_push.intents.IntentRouter` (ver
docs/atlas_push/PLAN_STEP_B.md y docs/atlas_push/PLAN_STEP_C.md).
"""
```

### 5.3 Endpoints canónicos

Queda una única definición por endpoint:

1. `GET /status` → `{"ok": True, "atlas": handle("/status")}`.
2. `GET /tools` → `{"ok": True, "tools": [...]}` con los 8 nombres
   idénticos a `main` y en el mismo orden.
3. `POST /execute` → mismo mapeo `tool → cmd`, misma forma de
   respuesta en éxito (`{ok, tool, output}`) y en tool no soportada
   (`{ok, error}`).
4. `GET /modules` → `{"ok": True, "modules": [...]}` con los 4 dicts
   idénticos (vision/voice/agent_router `False`, telegram `True`).
5. `POST /intent` → bloque canónico de B2 intacto: `IntentIn` con
   campos `user="raul"`, `text`, `meta`; try/except; shape
   `{ok, input, output, ms}` / `{ok, input, error, ms}`; `ok=True`
   salvo excepción.

`_intent_router = IntentRouter()` se instancia una sola vez a nivel
de módulo (mismo patrón que en B). `IntentRouter` es stateless.

### 5.4 Forma definitiva del archivo

Resultado esperado: un solo módulo de ~100 líneas (desde 138) sin
bloques repetidos. Cero `importlib`, cero `Path(r"C:\...")`, cero
endpoints duplicados, cero imports duplicados.

El diff completo se presenta al owner antes de cualquier push, igual
que en A y B.

## 6. C2 — Tests para fijar el contrato

Objetivo: que la limpieza no pueda romper el contrato público sin que
un test lo grite. Archivo nuevo:
`tests/unit/test_http_api_contract.py`.

Usa `fastapi.testclient.TestClient` contra
`atlas_adapter.atlas_http_api.app`. No arranca uvicorn. Corre en el
sandbox sin Windows.

### 6.1 Invariantes fijados

1. **Import time no explota fuera de Windows.** El módulo se importa
   en frío con `ATLAS_ROOT` en `tmp_path`; `http_api.handle` es el
   símbolo real de `command_router.handle` (import de paquete, no
   `importlib.util`).
2. **Inventario exacto de rutas.** El conjunto de rutas registradas
   en `app.routes` es exactamente
   `{GET /status, GET /tools, GET /modules, POST /execute,
   POST /intent}`.
3. **Sin duplicados.** Para cada par `(método, path)` hay exactamente
   un handler. Guardrail estructural contra regresión de D-002.
4. **`/status`:** claves `{ok, atlas}`, `ok is True`, `atlas` string
   que arranca con `"ATLAS: OK"`.
5. **`/tools`:** claves `{ok, tools}`, `ok is True`, lista literal de
   8 nombres en orden fijo.
6. **`/modules`:** claves `{ok, modules}`, `ok is True`, lista literal
   de 4 dicts en orden fijo.
7. **`/execute`:** soportada → `{ok, tool, output}`, `ok=True`;
   no soportada → `{ok, error}`, `ok=False`, mensaje contiene
   `"Tool no soportada"` y el nombre del tool.
8. **`/intent`:** happy → `{ok, input, output, ms}`, `ok=True`,
   `ms int ≥ 0`, `input` refleja defaults de `IntentIn`; empty
   (`text=""`) → mismas claves, `ok=True`, `output == "ATLAS: vacío."`
   (matiz B preservado al contrato).
9. **Higiene de imports.** En el fuente del módulo no aparece
   `importlib`, ni `C:\ATLAS`, ni `C:\ATLAS_PUSH`. Guardrail
   estructural contra regresión de D-003.

### 6.2 Estrategia para aislar el filesystem

El `command_router.handle` real escribe en `ATLAS_ROOT` (log, vault).
Para los unit tests:

- `monkeypatch.setenv` de `ATLAS_ROOT`, `ATLAS_VAULT_DIR`,
  `ATLAS_NOTES_DIR`, `ATLAS_LOGS_DIR`, `ATLAS_SNAPS_DIR` a `tmp_path`.
- `sys.modules.pop` de `atlas_adapter.atlas_http_api`,
  `atlas_push.intents[.intent_router]` y `modules.command_router`
  para forzar import en frío que re-evalúe las env vars.

Mismo patrón que `tests/unit/test_intent_router.py`. Hermético en
Linux/macOS/Windows.

### 6.3 Smokes externos (bash)

- `tests/smoke/a2_parity_check.sh` se **reutiliza tal cual**. C no
  añade ni cambia endpoints; su captura `before` (main) y `after`
  (rama C) debe dar paridad en los 9 casos.
- `tests/smoke/b_parity_check.sh` se **reutiliza tal cual**. Las 14
  fixtures de `/intent` deben seguir saliendo idénticas.
- No se añade un `c_parity_check.sh`: no hay superficie HTTP nueva
  que fijar. La cobertura nueva va en pytest unit (§6.1), que sí
  corre en CI/sandbox y no depende de uvicorn. Decisión del owner.

## 7. C3 — PR y commits

### 7.1 Un solo PR

- Rama: `chore/atlas-push-c-cleanup-http-api`.
- Título: `chore(atlas_push): clean atlas_http_api wiring (C)`.
- Base: `main`. Head: la rama anterior.
- PR body: resumen de C1/C2/C3, tabla de cambios, referencia a este
  documento, qué no cambia y checklist de validación. Mismo formato
  que PR #8 (A2) y PR #9 (B).

### 7.2 Tres commits atómicos

**C1 — `chore(atlas_push): consolidate imports and endpoints in atlas_http_api (C1)`**

- Toca sólo `atlas_adapter/atlas_http_api.py`.
- Quita duplicados, consolida imports, elimina `importlib.util` +
  ruta Windows, sustituye por `from modules.command_router import
  handle`.
- No añade tests (eso va en C2).

**C2 — `test(atlas_push): pin http api contract with unit tests (C2)`**

- Añade `tests/unit/test_http_api_contract.py` con los ~11 tests que
  fijan los 9 invariantes de §6.1.
- No modifica código de producción.

**C3 — `docs(atlas_push): add PLAN_STEP_C.md and close D-002/D-003 in DEFECTS.md (C3)`**

- Añade `docs/atlas_push/PLAN_STEP_C.md` (este documento).
- Actualiza `docs/atlas_push/DEFECTS.md`: D-002 y D-003 pasan a
  **Estado: cerrado en Paso C (PR C)**.

D-001 sigue con su estado actual — *no se corrige en la reestructuración;
archivable en fase posterior* — sin cambios. Decisión del owner.

### 7.3 Validación previa al push

Antes de pedir aprobación para el push:

- `git diff main..HEAD` completo (los 3 commits).
- `python -m pytest tests/unit -q` en verde (los 68 de B + los 11
  nuevos de C2).
- `grep -nE "importlib|C:\\\\ATLAS|C:\\\\ATLAS_PUSH"
  atlas_adapter/atlas_http_api.py` sin resultados.
- Inventario de rutas `python -c "from atlas_adapter.atlas_http_api
  import app; print(sorted((r.path for r in app.routes)))"` con la
  lista esperada sin duplicados.
- Opcional, en máquina del owner: `a2_parity_check.sh` y
  `b_parity_check.sh` en modo `before` (main) y `after` (rama C) con
  la API viva en `127.0.0.1:8791`.

### 7.4 Protocolo de push

Como A y B: nada de push hasta que el owner apruebe el diff local.
Secuencia:

1. Crear la rama y hacer los 3 commits en el sandbox.
2. Enseñar diff + salida de tests por cada commit.
3. El owner aprueba cada sub-paso (C1, C2, C3).
4. El owner da OK final al paquete; push a origin.
5. Abrir el PR contra `main` con el body redactado.
6. El owner hace el merge desde `C:\TEMP_ATLAS_CORE\atlas-core` como
   en A1, A2 y B.

## 8. Riesgos y mitigaciones

| Riesgo | Mitigación |
|---|---|
| Que `importlib.util` enmascarase una dependencia no expresada en `sys.path` | No es el caso: `03_run_atlas_api.ps1` arranca uvicorn desde la raíz del repo, `sys.path` incluye `./`, por lo que `from modules.command_router import handle` resuelve igual que en `modules/atlas_remote_api.py`. Test `test_import_works_without_windows_paths` lo fija. |
| Que el orden de registro de rutas en FastAPI dependiese de "la última definición gana" y algún consumidor dependiera sutilmente de la primera | No ocurre: FastAPI conserva la última, que es la única que se ejecuta. Quitar la muerta no tiene efecto observable. Smokes A2 + B lo verifican. |
| Import circular tras sustituir `load_handle()` por el import directo | `atlas_push.intents` solo usa `modules.command_router.handle` vía `self._handle_fn` por defecto; no hay ciclo. `test_import_works_without_windows_paths` fuerza el import en frío. |
| Que algún test de `/status` escriba al disco real | `monkeypatch.setenv("ATLAS_ROOT", tmp_path)` redirige todo a temporales. Patrón ya validado en B. |
| Regresión silenciosa en un shape JSON | Invariantes 4–8 comparan literales; invariantes 2–3 fijan inventario y unicidad; smokes A2/B comparan bytes con normalización de volátiles. Triple red. |

## 9. Out of scope (no llega en C)

- **D-001** (`route_command` fantasma en
  `modules/atlas_remote_api.py`): decisión ya registrada en
  `DEFECTS.md` — archivo fuera de servicio, no se corrige, se
  archivará en fase posterior junto con otras interfaces legadas. C
  **no** lo toca.
- **D-004** (`core/scheduler.py` huérfano): fuera de Atlas Push.
- **D-005** (`snapshots/` versionado) y **D-006** (`.venv/`
  versionado): PR de higiene independiente, como ya documentado.
- **D-007** (paths Windows dispersos fuera del adaptador): se aborda
  parcialmente en Paso E.
- Renombrar endpoints, cambiar shapes, mover autenticación o
  cualquier mejora semántica del API: todo fuera. C es **limpieza,
  no mejora**.
- Refactor de `atlas_push/intents/` o cambio del contrato B: fuera.
  C no toca `atlas_push/`.

## 10. Criterio de aceptación del PR C

- [x] `atlas_adapter/atlas_http_api.py` sin `importlib.util`, sin
  `Path(r"C:\ATLAS")`, sin endpoints duplicados, sin imports
  duplicados. ~100 líneas netas.
- [x] `python -m pytest tests/unit -q` en verde (68 de B + 11 nuevos
  de C2 = 79 tests).
- [x] `tests/unit/test_http_api_contract.py` cubre los 9 invariantes
  de §6.1 con 11 tests.
- [x] `grep -nE "importlib|C:\\\\ATLAS" atlas_adapter/atlas_http_api.py`
  devuelve vacío.
- [x] `app.routes` expone exactamente
  `{/status, /tools, /modules, /execute, /intent}` sin duplicados.
- [ ] Smokes `a2_parity_check.sh` (9/9) y `b_parity_check.sh`
  (14/14) siguen en verde contra la rama C (validación en máquina
  del owner).
- [x] `docs/atlas_push/PLAN_STEP_C.md` añadido.
- [x] `docs/atlas_push/DEFECTS.md` marca D-002 y D-003 como cerrados
  en PR C.
- [x] Diff revisado y aprobado por el owner antes del push.
- [ ] PR abierto contra `main` con título
  `chore(atlas_push): clean atlas_http_api wiring (C)`.
