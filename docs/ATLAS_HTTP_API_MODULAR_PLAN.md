# ATLAS HTTP API Modular Plan

## Objetivo
Reducir el riesgo operativo del monolito `atlas_adapter/atlas_http_api.py` y establecer un camino incremental de modularización sin romper contratos.

## Estado actual
- Gateway único con miles de líneas y múltiples dominios (humanoid, ans, nervous, autonomous, comms, brazos, proxy).
- Existían rutas duplicadas por combinación de endpoints locales + `include_router`.
- Se añadió deduplicación de rutas en tiempo de import para fijar contrato canónico.

## Estrategia incremental
1. Fase de guardas:
- Test automático de colisiones de rutas.
- Inventario de rutas por módulo para priorizar extracción.

2. Fase de extracción por dominio:
- `atlas_adapter/routes/core_status.py`
- `atlas_adapter/routes/ans_legacy.py`
- `atlas_adapter/routes/nervous_legacy.py`
- `atlas_adapter/routes/ops_tools.py`
- `atlas_adapter/routes/arms.py`

3. Fase de registro central:
- Módulo único de `register_routes(app)` para eliminar imports tardíos dispersos.
- Control explícito de precedencia y aliases.

4. Fase de compatibilidad:
- Mantener endpoints legacy críticos con wrappers.
- Publicar deprecaciones con fechas y métricas de uso.

## Criterios de aceptación por extracción
- Sin cambio de contrato (`path`, `method`, shape de respuesta) salvo alias documentados.
- `test_route_collisions` en verde.
- Smoke tests de `04_smoke_tests.ps1` en verde.
- Reporte de rutas actualizado (`scripts/atlas_route_inventory.py`).
