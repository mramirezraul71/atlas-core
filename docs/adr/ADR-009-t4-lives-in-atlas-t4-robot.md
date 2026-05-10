# ADR-009 — T4 lives in `atlas-t4-robot` (repositorio independiente)

- **Estado:** Aceptada (NO NEGOCIABLE — decisión de producto)
- **Fecha:** 2026-05-10
- **Decisor:** Producto (mramirezraul71)

## Contexto

Durante la Fase 0 se elaboró el diseño asumiendo que el robot T4 vivía como subárbol dentro de `atlas-core` (rutas `atlas-core/atlas_code_quant/execution/brokers/t4/`, `services/t4_robot/`, `dashboard/t4_dashboard/`). Esa asunción se invalida explícitamente por decisión de producto.

## Decisión

El robot T4 vive en un **repositorio independiente y privado**: `mramirezraul71/atlas-t4-robot`.

- **`atlas-core` NO es monorepo.** Cualquier Fase 1 que agregue `atlas_code_quant/execution/brokers/t4/`, `services/t4_robot/` o `dashboard/t4_dashboard/` dentro de `atlas-core` queda **congelada y rechazada**.
- En `atlas-core`, la rama de trabajo `feat/t4-robot-phase0` se mantiene únicamente con este ADR (copia), borrando el resto de docs Fase 0 que se habían añadido temporalmente.
- La integración entre `atlas-core` y `atlas-t4-robot` se hace **exclusivamente** vía contratos externos versionados:
  - **REST** (endpoints documentados en `docs/03_API_TIEMPO_REAL.md` y `docs/07_INTEGRACION_ATLAS_CORE.md`),
  - **Redis pub/sub** (topics `atlas.t4.*` internos del robot, `atlas.bridge.t4.*` para el bridge),
  - **Paquete Python opcional** `atlas-contracts` publicado por `atlas-core` con los modelos compartidos (`RuntimeMode`, `LiveSwitchState`, `OrderRequest`, etc.).

Ningún import cruzado de código no-contractual. Ninguna dependencia de filesystem compartido.

## Razones

1. **Desacoplamiento de releases**: cambios en `atlas-core` (cerebro cognitivo) no requieren recertificar el robot live, y viceversa.
2. **Despliegue independiente**: el robot necesita VPS Chicago por proximidad a CME; `atlas-core` puede vivir en otro hosting/datacenter.
3. **Ciclo de vida 24/7**: el robot opera según calendario CME; `atlas-core` puede ciclar para retraining.
4. **Aislamiento de fallos**: un fallo en `atlas-core` no debe poder tumbar el bucle de trading; y al revés, un crash del robot no debe contaminar el journal cognitivo.
5. **Modelo de seguridad**: el robot mantiene credenciales T4 segregadas. `atlas-core` no debe poder leerlas.
6. **PRs cruzados de baja relación**: en monorepo, cambios al scanner cognitivo y al adapter T4 compiten en CI y revisiones; separados son independientes.
7. **Privacidad**: el robot es privado (estrategia operativa); `atlas-core` puede tener partes públicas.

## Consecuencias

- **Repositorios y trabajo:**
  - Repo nuevo creado: https://github.com/mramirezraul71/atlas-t4-robot (privado).
  - Toda la doc Fase 0 (`00`–`06`, `T4_ROBOT_BUILD`, diagramas) se reescribe en este repo como root.
  - En `atlas-core` rama `feat/t4-robot-phase0`: se borran los docs salvo esta ADR-009 (mismo contenido). La rama queda como puntero histórico.
- **Reuso de código existente en `atlas-core` (`runtime_mode.py`, `live_switch.py`, `circuit_breaker.py`, `mode_switcher.py`, `kill_switch.py`, `paper_broker.py`, `broker_router.py`, `realtime_feed.py`, `brain_bridge.py`):**
  - Opción A (recomendada): publicar paquete `atlas-contracts` desde `atlas-core` con las firmas públicas que el robot necesita.
  - Opción B (fallback): reimplementar localmente en `services/t4_robot/src/contracts/` y `src/safety/` con la misma firma pública y tests de paridad.
- **Cambios al contrato** requieren bump SemVer en `atlas-contracts` antes de mergear el cambio en cualquiera de los dos repos.
- **CI/CD independiente:** `atlas-t4-robot` tiene su propio `.github/workflows/`.
- **Docker compose independiente:** vive en root del nuevo repo.

## No-goals

1. **No** se sincroniza historial git entre repos.
2. **No** se mantiene un mirror automático bidireccional.
3. **No** se permiten dependencias circulares: el contrato fluye `atlas-core` (publica paquete) → `atlas-t4-robot` (consume).
4. **No** se publica este repo como público sin revisión de seguridad.

## Referencias

- Decisión de producto verbatim: ver historial del proyecto (mensaje del 10-May-2026).
- `docs/07_INTEGRACION_ATLAS_CORE.md` — especificación completa de los contratos externos.
- `docs/01_ARQUITECTURA.md` — arquitectura ya alineada con este ADR.
