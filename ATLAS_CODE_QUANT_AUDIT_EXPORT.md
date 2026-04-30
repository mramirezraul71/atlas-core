# ATLAS CODE QUANT — AUDIT EXPORT
Timestamp: 2026-04-15T00:00:00Z (local session)
Workspace: `C:\ATLAS_PUSH\atlas_code_quant`
Branch objetivo auditado: rama activa en sesión (referencia solicitada: `journal-forensic-20260411-quant`)

## 1) TREE (estructura del repo)
```text
[Ver archivo completo: TREE.txt]
- api/, backtesting/, config/, context/, data/, execution/, journal/, learning/, monitoring/, notifications/, operations/, scanner/, strategy/, tests/, vision/
- Entrypoints detectados: main.py, api/main.py
```

## 2) CRITICAL FILES (código fuente)
```text
[Ver archivo completo: CRITICAL_FILES.md]
Incluye:
- main.py completo
- journal/__init__.py completo
- extractos críticos de journal/service.py (incluye _upsert_open_entry y sync_scope)
- scanner/opportunity_scanner.py (núcleo scoring)
- selector/strategy_selector.py (proposal/sizing/risk)
- execution/live_loop.py y tradier_execution.py (núcleo ejecución)
- config/settings.py (gates y toggles)
- api/schemas.py y journal/models.py (contratos/persistencia)
```

## 3) FLOW DIAGRAM (mapeo visual)
```text
[Ver archivo completo: OUTPUT_FLOW_DIAGRAM.txt]
Pipeline mapeado:
Ingesta -> Scanner -> Contexto/Régimen -> Selector/Signal ->
Risk/Preflight -> Execution -> Fill/Position Mgmt ->
Journal -> Reconciliation -> Learning/Policy Update
```

## 4) CONFIGURATION (settings actuales)
```text
Archivo fuente: atlas_code_quant/config/settings.py

Bloques relevantes:
- Scanner: QUANT_SCANNER_*
- Entry validation: QUANT_ENTRY_*
- Visual gate: QUANT_VISUAL_GATE_*
- Position management: QUANT_POSITION_MGMT_*
- Exit governance: QUANT_EXIT_GOVERNANCE_*
- Learning: QUANT_ADAPTIVE_LEARNING_*, ATLAS_LEARNING_*
- Notifications: QUANT_NOTIFY_*
- Startup guards: QUANT_STARTUP_*
```

## 5) TEST INVENTORY (tests disponibles)
```text
[Ver archivo completo: TEST_INVENTORY.txt]
Total: 884 tests collected
Cobertura funcional observada:
- scanner / selector / strategy / risk / pdt / options / journal / learning / api / operation center / visual gates
```

## 6) LAST TEST RUN (estado de tests)
```text
[Ver archivo completo: LAST_TEST_RUN.txt]
Resultado: 1 failed, 883 passed, 8 warnings
Failure:
- test_journal_learning_hooks.py::test_upsert_open_entry_normalizes_signed_prices
  ValueError: too many values to unpack (expected 2)
  Causa probable: cambio de firma de retorno en _upsert_open_entry (ahora retorna 3 valores)
```

## 7) RECENT LOGS (actividad reciente)
```text
[Ver archivo completo: RECENT_LOGS.txt]
Hallazgos:
- Múltiples fallos de yfinance (JSONDecodeError) en lotes de símbolos.
- Analytics (DuckDB) sigue operando entre fallos de data source.
```

## 8) FORENSICS (si aplica)
```json
{
  "status": "not_found",
  "requested_path": "atlas_code_quant/output/journal_forensics/",
  "requested_file": "atlas_code_quant/output/journal_forensics/journal_forensics_20260411.json"
}
```

## 9) KNOWN ISSUES (TODOs, FIXMEs, BUG/HACK markers)
```text
Búsqueda TODO/FIXME/XXX/HACK/BUG/BROKEN:
- Evidencia encontrada en módulos puntuales, principalmente comentarios de deuda técnica y compatibilidad.

Issue operativo confirmado por tests:
- Desalineación de contrato en `journal.service._upsert_open_entry` vs test legado.
```

## 10) ENTRY POINTS (APIs públicas)
```text
Entrypoints principales:
- atlas_code_quant/main.py
- atlas_code_quant/api/main.py

Endpoints detectados en api/main.py:
- GET  /health
- GET  /status
- GET  /canonical/snapshot
- POST /signal
- POST /probability/options
- POST /order
- GET  /positions
- GET  /journal/stats
- GET  /journal/entries
- GET  /journal/chart-data
- POST /journal/sync/refresh
- GET/POST suite /operation/* (status, config, test-cycle, loop start/stop/status, vision provider, readiness, emergency)
- V2 mirrors: /api/v2/quant/...
```

## Flujo scanner→signal→risk→execution→journal→learning (audit checklist)
```text
1) scanner/opportunity_scanner.py
   - construye candidatos con selection_score + evidencia + contexto HTF + order_flow

2) selector/strategy_selector.py
   - transforma candidato en proposal operacional (estructura, sizing, risk profile, order_seed)

3) operations/operation_center.py + readiness_eval.py
   - aplica gates operativos, validación visual/contexto, preflight y control auton

4) execution/tradier_execution.py + signal_executor.py + live_loop.py
   - enruta orden, gestiona ciclo en vivo, PDT y controles de emergencia

5) journal/service.py + journal/models.py
   - persiste apertura/cierre, sync con broker, snapshots de calidad y reconstrucción

6) learning/learning_orchestrator.py + adaptive_policy.py + atlas_learning_brain.py
   - reconcilia outcomes, calcula IC, actualiza políticas y readiness
```

## Estado final de artefactos solicitados
```text
Generados:
- TREE.txt
- CRITICAL_FILES.md
- OUTPUT_FLOW_DIAGRAM.txt
- TEST_INVENTORY.txt
- LAST_TEST_RUN.txt
- RECENT_LOGS.txt
- FORENSICS.json
- ATLAS_CODE_QUANT_AUDIT_EXPORT.md
```
