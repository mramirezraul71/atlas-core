# ATLAS Software Hub

Inventario extendido para ATLAS que separa:

- `software_catalog.json`: software versionable (instalado + actualizable).
- `driver_requirements.json`: drivers requeridos para operación estable.
- `development_stack.json`: componentes de plataforma ATLAS (servicios, jobs, ciclos).
- `software_registry.json`: salida generada por `scripts/atlas_software_watchdog.py`.

Objetivo:

1. Consolidar estado real de software y drivers.
2. Detectar candidatos disponibles en red para instalar.
3. Exponer progreso y jobs de actualización desde `/api/software/*`.
4. Ejecutar mantenimiento industrial (triada + makeplay + repo) con reporte.
