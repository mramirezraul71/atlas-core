# ATLAS Central Orchestration

## Objetivo
ATLAS actua como punto unico de decision entre:
- `rauli-panaderia` (gestion: inventario/ventas)
- `RAULI-VISION` (detecciones y analisis)

Ningun ajuste critico de inventario debe ejecutarse sin pasar por el orquestador central.

## Contratos de datos
- Evento entrante de vision:
  - `contracts/rauli_vision_detection_event.schema.json`
- Comando saliente a panaderia:
  - `contracts/panaderia_inventory_adjustment_command.schema.json`

## Flujo centralizado
1. RAULI-VISION genera evento (`source=rauli-vision`).
2. ATLAS valida estructura y consistencia minima.
3. ATLAS decide:
   - `escalate_only`: anomalia alta/critica o baja confianza/calidad.
   - `dry_run_adjustments`: evento valido, sin aplicar cambios.
   - `apply_adjustments`: evento valido y modo aplicacion habilitado.
4. Solo ATLAS envia ajustes a panaderia:
   - `POST /api/inventory/adjustment`

## Comunicacion asincrona (bus local)
- Inbox eventos vision: `state/atlas_bus/vision_inbox/*.json`
- Eventos procesados: `state/atlas_bus/vision_processed/`
- Eventos fallidos: `state/atlas_bus/vision_failed/`
- Decisiones Atlas por evento: `state/atlas_bus/atlas_decisions/*.json`

Worker:
```powershell
python scripts/atlas_cortex_queue_worker.py --once
```

Modo aplicacion real:
```powershell
python scripts/atlas_cortex_queue_worker.py --once --apply
```

## Regla de escalamiento
Se escala (sin escribir en DB) cuando:
- `anomaly.detected=true` y `severity in {high, critical}`
- o `quality_score < 0.40`
- o `confidence < 0.35`

## Logs y trazabilidad
- Decisiones: `logs/atlas_cortex_decisions.log`
- Alertas escaladas: `logs/atlas_cortex_alerts.log`
- Snapshot operativo: `logs/snapshot_safe_diagnostic.log`

## Ejecucion manual
- Dry run con muestra:
```powershell
python scripts/atlas_cortex_orchestrate_event.py --sample
```

- Aplicar en panaderia (requiere backend y token si aplica auth):
```powershell
$env:PANADERIA_API_BASE = "http://127.0.0.1:3001"
$env:PANADERIA_BEARER_TOKEN = "<token-opcional>"
python scripts/atlas_cortex_orchestrate_event.py --event-file C:\ruta\evento.json --apply
```

## Variables de entorno
- `PANADERIA_API_BASE` (default `http://127.0.0.1:3001`)
- `PANADERIA_BEARER_TOKEN` (opcional; recomendado en produccion)

## Tareas VS Code recomendadas
- `ATLAS: Snapshot Safe Diagnostic`
- `ATLAS: Cortex Bridge Dry Run`
- `ATLAS: Cortex Queue Worker (Once)`
