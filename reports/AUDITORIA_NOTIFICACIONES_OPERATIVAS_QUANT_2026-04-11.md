# Auditoría externa — Módulo de notificaciones / briefing operativo (Atlas Code Quant)

**Repositorio:** `C:\ATLAS_PUSH`  
**Fecha:** 2026-04-11  

## 1. Objetivo

Capa de **briefing y control operativo** (no demo) que prioriza información para mejorar disciplina de entrada/salida, claridad, trazabilidad y enlace con scanner, posiciones, journal (vía analytics), learning orchestrator y readiness — con envío **Telegram + WhatsApp** reutilizando `AlertDispatcher`, persistencia auditable y API manual/automática.

## 2. Alcance implementado

| Área | Estado |
|------|--------|
| Pre-market briefing | Sí — síntesis priorizada + plan de sesión + criterios apertura/cierre |
| EoD summary | Sí — PnL día (desde analytics), riesgos, recomendaciones, aprendizaje |
| Intraday | Sí — pulso ligero si candidato scanner supera umbral (con anti-spam) |
| Exit intelligence | Sí — hook en `AlertDispatcher._on_alert` para trades categoría `trade` con “SALIDA” en título |
| Telegram / WhatsApp | Sí — vía `operational_briefing()` con HTML/plano separados |
| Trazabilidad | Sí — `data/notifications/snapshots/*.json` + `notification_events.jsonl` |
| Tests unitarios | Sí — `test_notifications_module.py` |
| Refactor readiness | `operations/readiness_payload_builder.py` compartido con `api/main.py` |

## 3. Diseño (separación de capas)

- **`notifications/models.py`** — tipos y `PrioritizedBriefing`
- **`notifications/payloads.py`** — extracción: readiness, `status_lite`, scanner `report()`, canonical snapshot, orchestrator, adaptive JSON, DuckDB `full_analytics`
- **`notifications/prioritization.py`** — ranking oportunidades, riesgos, posiciones, fase aprendizaje (`ready|warming_up|stalled|degraded`), plan de sesión
- **`notifications/renderers.py`** — HTML Telegram + texto plano WhatsApp
- **`notifications/dispatcher.py`** — dedup por fingerprint + cooldown + subconjunto de canales desde settings
- **`notifications/storage.py`** — snapshots JSON + JSONL
- **`notifications/briefing_service.py`** — orquestación + `configure_operational_briefing` / singleton
- **`notifications/scheduler.py`** — ventanas ET premarket/EoD + intraday; `attach_exit_intelligence_bridge()`

## 4. Archivos tocados / creados

**Nuevos:** `atlas_code_quant/notifications/*`, `atlas_code_quant/operations/readiness_payload_builder.py`, `atlas_code_quant/tests/test_notifications_module.py`, este informe.

**Modificados:** `atlas_code_quant/api/main.py`, `atlas_code_quant/config/settings.py`, `atlas_code_quant/operations/alert_dispatcher.py` (`AlertEvent.body_plain`, `operational_briefing`, `_send_briefing_to_channels`), `config/atlas.env.example`.

## 5. Endpoints

| Método | Ruta |
|--------|------|
| GET | `/notifications/status` |
| POST | `/notifications/premarket/run` |
| POST | `/notifications/eod/run` |
| POST | `/notifications/test` |
| GET | `/notifications/recent` |
| Espejos v2 | `/api/v2/quant/notifications/*` |

Autenticación: mismo `x-api-key` que el resto del API Quant.

## 6. Configuración (`QUANT_NOTIFY_*`)

Documentada en `config/atlas.env.example`: `QUANT_NOTIFY_ENABLED`, premarket/eod/intraday/exit_intel, canales, severidad, cooldown, dedup TTL, máx. oportunidades/posiciones, render, paper-only, TZ, HHMM premarket/EoD, poll scheduler, intervalo mínimo intradía.

## 7. Integración de canales

- **Telegram:** `AlertDispatcher._send_telegram` (HTML en `body`)
- **WhatsApp:** `body_plain` o strip de HTML; envío vía ATLAS API como el resto de alertas
- **Selección de canal:** `notify_channels` + flags internos del dispatcher (tokens/env)

## 8. Riesgos y limitaciones

1. **EoD PnL “día”:** depende de forma de `daily_pnl` en DuckDB (clave `day`/`date`); mercados US vs UTC pueden desalinear el “hoy” — revisar timezone en analytics si se exige precisión contable.
2. **Intraday:** heurística simple (score vs `scanner_min_selection_score+5`); no sustituye reglas de negocio del selector.
3. **Exit intelligence:** dispara solo si el título de `AlertEvent` contiene `SALIDA` — si `trade_executed` cambia redacción, ajustar filtro.
4. **`operational_briefing`:** no pasa por la cola async de alertas genéricas; envío directo tras cooldown/dedup del coordinador de notificaciones.
5. **Singleton `get_operational_briefing_service`:** tests deben llamar `configure_operational_briefing` antes de usar el servicio.

## 9. Pruebas ejecutadas

```powershell
.\venv\Scripts\python.exe -m pytest atlas_code_quant/tests/test_notifications_module.py -q
.\venv\Scripts\python.exe -m pytest atlas_code_quant/tests/test_readiness_eval.py -q
```

Import smoke: `cd atlas_code_quant; python -c "import api.main"`.

## 10. Validación real (operador)

1. Activar `QUANT_NOTIFY_ENABLED=true` y credenciales Telegram/WhatsApp.
2. Reiniciar API Quant; ver log `Operational briefing: scheduler activo`.
3. `POST /notifications/test` con API key.
4. `POST /notifications/premarket/run` y comprobar snapshot en `data/notifications/snapshots/`.
5. Cerrar un trade que dispare `trade_executed` con título de salida y ver mensaje “Exit intelligence”.

## 11. Próximos pasos sugeridos

- Enlazar explícitamente entradas aprobadas/rechazadas desde `OperationCenter` / selector hacia `emit_intraday`.
- Enriquecer EoD con agregados explícitos “cerrados hoy” desde SQL journal si DuckDB daily no alinea TZ.
- Métricas Prometheus para `notifications_sent_total` / `dedup_skipped_total`.

---
*Fin del informe para auditoría externa.*
