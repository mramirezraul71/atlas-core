# Atlas Push · Plan del Paso B · `IntentRouter`

> Plan aprobado antes de implementar el Paso B de la reestructuración.
> Referencia obligatoria para el PR `feat(atlas_push): introduce
> IntentRouter as wrapper over command_router (B)`.
>
> Complementa a:
>
> - `docs/atlas_push/ARCHITECTURE.md` (diseño objetivo).
> - `docs/atlas_push/CURRENT_STATE.md` (estado pre‑A2).
> - `docs/atlas_push/DEFECTS.md` (defectos conocidos, no corregidos en B).

## 1. Objetivo

Extraer un `IntentRouter` tipado sobre la función `handle(text: str) -> str`
de `modules/command_router.py`, sin reescribir su lógica. Este wrapper:

1. Devuelve un `IntentResult` inmutable (dataclass `frozen=True`).
2. Clasifica el intent de entrada en uno de 12 `kind`s.
3. Preserva el `output` byte‑a‑byte respecto a `command_router.handle`.
4. No importa nada del brain core mayor de ATLAS.

## 2. Principios (vinculantes)

1. **Regla de oro**: `atlas_push/` no importa símbolos del brain core mayor.
   Los contratos viajan vía Protocols. En B, la única dependencia es
   `modules.command_router` (permitido por `ARCHITECTURE §7`).
2. **Capa nueva, no reescritura**: `command_router.py` sigue siendo la
   verdad operacional actual. B sólo añade un wrapper encima.
3. **No se elimina nada**. Las rutas existentes siguen funcionando.
4. **Paridad HTTP estricta**: el endpoint `/intent` preserva su esquema
   JSON byte‑a‑byte (incluyendo `ok: True` salvo excepción).
5. **No se toca `command_router.py`** en este paso.
6. Todo el plan se presenta y se aprueba antes de ejecutarse.

## 3. Contrato

```python
IntentRouter.handle(text: str) -> IntentResult
```

Con `IntentResult` definido como dataclass inmutable en
`atlas_push/intents/types.py`:

| Campo | Tipo | Semántica |
|---|---|---|
| `kind` | `Kind` (Literal) | Tipo de intent detectado (ver §4) |
| `ok` | `bool` | `True` salvo `kind == "empty"` o excepción |
| `output` | `str` | Texto devuelto por `command_router.handle(text)` |
| `raw_input` | `str` | Texto original recibido |
| `meta` | `dict[str, Any]` | Reservado para uso futuro; en B siempre `{}` |

## 4. Taxonomía de `kind` (12 valores aprobados)

1. `"status"` — texto empieza por `/status`.
2. `"doctor"` — texto empieza por `/doctor`.
3. `"modules"` — texto empieza por `/modules`.
4. `"snapshot"` — texto empieza por `/snapshot` (con o sin label).
5. `"note.create"` — texto empieza por `/note create`.
6. `"note.append"` — texto empieza por `/note append`.
7. `"note.view"` — texto empieza por `/note view`.
8. `"natural.note.create"` — patrón
   `crea\s+una\s+nota\s+(llamada|con\s+el\s+título)\s+...`.
9. `"natural.note.append"` — patrón
   `agrega\s+a\s+la\s+nota\s+...\s+que\s+...`.
10. `"natural.snapshot"` — patrón
    `crea\s+un\s+snapshot\s+llamado\s+...`.
11. `"natural.modules"` — `"módulos"` + (`"activos"` o `"tienes"`).
12. `"inbox.fallback"` — texto no vacío no clasificado en los anteriores.

Caso especial:

- `"empty"` — **estricto**: ``text is None`` o ``text == ""``. `ok = False`.
  `output = "ATLAS: vacío."` (literal que devuelve hoy
  `command_router.handle("")`).

Importante: `command_router.handle` comprueba `if not text` (truthy check),
no `text.strip()`. Por tanto un texto de **sólo whitespace** NO es tratado
como `empty` por el handle original; cae al fallback de inbox. El
`IntentRouter` preserva esa semántica exacta: whitespace → `inbox.fallback`,
no `empty`. Esto se verifica con un caso específico en las pruebas
unitarias.

## 5. Estructura de ficheros (B1)

Todos nuevos; ningún archivo existente se modifica en B1.

```
atlas_push/
├── __init__.py
├── intents/
│   ├── __init__.py
│   ├── types.py
│   └── intent_router.py
tests/
└── unit/
    ├── __init__.py
    └── test_intent_router.py
```

## 6. B2 — Integración en `/intent`

Cambio mínimo en `atlas_adapter/atlas_http_api.py`. Hoy:

```python
from modules.command_router import handle as route_command
...
out = route_command(payload.text)
return {"ok": True, "input": payload.model_dump(), "output": out, "ms": ...}
```

Después:

```python
from atlas_push.intents import IntentRouter
_router = IntentRouter()
...
result = _router.handle(payload.text)
return {"ok": True, "input": payload.model_dump(),
        "output": result.output, "ms": ...}
```

**Paridad estricta**: el campo `ok` de la respuesta HTTP sigue siendo
`True` salvo excepción. `result.ok` queda disponible en `IntentResult`
para consumidores futuros pero **no** se expone en `/intent` en B.

No se tocan en B:

- Endpoints `/modules` y `/intent` duplicados.
- Imports dobles (`BaseModel`, `Optional`, `Any`, `time`).
- Carga de `command_router` vía `importlib.util` con path Windows
  absoluto (`C:\ATLAS\modules\command_router.py`).
- Decorador `@app.post("/intent")` redefinido.

Todo lo anterior está documentado en `DEFECTS.md` y se aborda en
**Step C** (limpieza de `atlas_http_api.py`).

## 7. B3 — Smoke de paridad

Script `tests/smoke/b_parity_check.sh`, análogo a `a2_parity_check.sh`
pero centrado en `/intent`. Captura una batería de 13 textos cubriendo
los 12 `kind`s más el caso empty, normaliza `ms` y rutas volátiles,
y compara antes/después con `diff`.

Modo de uso documentado en `tests/smoke/README.md`.

## 8. Pruebas unitarias (invariantes fijados)

`tests/unit/test_intent_router.py` cubre:

- Los 12 `kind`s más `empty`, con un input representativo por cada uno.
- `IntentResult` es inmutable (`frozen=True`): asignar a un campo lanza
  `dataclasses.FrozenInstanceError`.
- Para cada input, el `output` de `IntentRouter.handle(text)` es
  **byte‑idéntico** al de `command_router.handle(text)`.
- `raw_input` se preserva literal.
- `meta == {}` en todos los casos en B.
- Aislamiento: las pruebas que tocan filesystem (notes, snapshots) usan
  `tmp_path` + `monkeypatch.setenv` sobre `ATLAS_ROOT`, `ATLAS_VAULT_DIR`,
  `ATLAS_NOTES_DIR`, `ATLAS_LOGS_DIR`, `ATLAS_SNAPS_DIR`, sin tocar
  rutas reales de Windows.

## 9. Commits planificados

Rama: `feat/atlas-push-b-intent-router`.

1. `feat(atlas_push): introduce IntentRouter wrapper over command_router.handle (B1)`
2. `feat(atlas_push): route /intent through IntentRouter preserving JSON shape (B2)`
3. `test(atlas_push): add b_parity_check.sh smoke for /intent (B3)`

Título del PR:
`feat(atlas_push): introduce IntentRouter as wrapper over command_router (B)`

## 10. Riesgos y mitigaciones

| Riesgo | Mitigación |
|---|---|
| Clasificación `kind` diverge de la interna de `handle` | Pruebas unitarias comparan `output` byte a byte contra `command_router.handle(text)` directamente. Si divergen, la prueba falla. |
| `output` diverge por transformación accidental | `IntentRouter` pasa el string por valor, sin transformar. |
| Consumidores del `/intent` rompen por cambio de `ok` | Paridad estricta: `ok` de la respuesta HTTP no cambia. |
| Pruebas dependen del filesystem real | Aislamiento vía `tmp_path` + `monkeypatch.setenv`. |
| Import de `modules.command_router` dispara side effects | Verificado: sólo lee env vars en import time; `mkdir` está dentro de funciones. |

## 11. Out‑of‑scope (no llega en B)

| Elemento | Cuándo |
|---|---|
| Corregir `/modules` y `/intent` duplicados | Step C |
| Retirar `importlib.util` absoluto a `C:\ATLAS\...` | Step C |
| Introducir `DecisionEngine`, `MarketState`, strategies | Step D |
| Protocols externos (data/execution/journal) | Step E |
| Arreglar D‑001 (`route_command` fantasma en `atlas_remote_api.py`) | Step C |
| Retirar `legacy/` | Tras E |

## 12. Criterio de aceptación del PR B

- [ ] `python -m pytest tests/unit/test_intent_router.py` pasa.
- [ ] `grep` confirma que `atlas_push/` no importa `brain_core`,
      `mission_manager`, `safety_kernel`, `state_bus`, `arbitration`,
      `policy_store`. La única dependencia externa a `atlas_push/` es
      `modules.command_router`.
- [ ] Diff de `atlas_adapter/atlas_http_api.py` muestra únicamente
      cambios acotados al endpoint `/intent` canónico.
- [ ] `a2_parity_check.sh` sigue saliendo limpio (B no toca `/status`,
      `/tools`, `/modules`, `/execute`).
- [ ] `b_parity_check.sh` sale limpio para los 13 inputs de la batería.
