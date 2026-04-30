# Auditoria Profesional ATLAS-Quant — 2026-03-31

## Resumen Ejecutivo

Auditoria completa del sistema ATLAS-Quant post-EPOCH RESET (2026-03-30). Se identificaron **10 problemas** en 2 ciclos de auditoria. De estos, **9 fueron corregidos** y **1 es externo** (Brain Bridge — puerto 8791 offline, fuera de scope).

**Estado final:** 742/742 tests passing. Sistema operativo en paper_autonomous con scanner calibrado y learning system funcional.

---

## Ciclo 1 — Bugs que impedian generar trades (commit 07085abec)

| # | Problema | Archivo | Prioridad | Estado |
|---|----------|---------|-----------|--------|
| 1 | Piso scanner hardcodeado a 75.0 rechaza todos los candidatos | `scanner/opportunity_scanner.py` (4 sitios) + `config/settings.py` | CRITICA | CORREGIDO |
| 2 | TIER_SMALL=0.55 mata senales neutrales | `strategy/signal_generator.py:160` | ALTA | CORREGIDO |
| 3 | Default auton_mode "paper_supervised" causa modo preview | `operations/operation_center.py:57` | MEDIA | CORREGIDO |
| 4 | Vision gate bloquea paper trading innecesariamente | `operations/operation_center.py:1260` | MEDIA | CORREGIDO |
| 5 | Logging insuficiente en gate de ordenes | `operations/operation_center.py:1372` | BAJA | CORREGIDO |

### Detalle de cambios Ciclo 1:
- **Scanner:** Removidos pisos `max(75.0, ...)` en 4 ubicaciones. Default de `min_selection_score` bajado a 65.0, piso de validacion a 50.0.
- **Signal Generator:** `TIER_SMALL` de 0.55 a 0.50. Kelly multiplier 0.50 actua como guardrail natural.
- **Operation Center:** Default `auton_mode` cambiado a `paper_autonomous`. Vision gate solo bloquea en scope `live`. Logging agregado para ordenes bloqueadas.
- **Adaptive Policy:** Corregido `TypeError: can't compare offset-naive and offset-aware datetimes` agregando helper `_naive()`.

---

## Ciclo 2 — Bugs del learning system y journal (commit pendiente)

| # | Problema | Archivo | Prioridad | Estado |
|---|----------|---------|-----------|--------|
| 6 | SystemReadinessReport: atributos inexistentes causan AttributeError | `learning/learning_orchestrator.py:432-437` | CRITICA | CORREGIDO |
| 7 | Snapshot adaptivo contaminado con 77K+ entradas pre-reset | `learning/adaptive_policy.py:260` + `data/learning/adaptive_policy_snapshot.json` | CRITICA | CORREGIDO |
| 8 | Metodo duplicado `update_pending_outcome()` en IC tracker | `learning/ic_signal_tracker.py:271-290` | ALTA | CORREGIDO |
| 9 | Deteccion de direccion incompleta para credit spreads | `journal/service.py:1320` | ALTA | CORREGIDO |
| 10 | Brain Bridge desconectado (puerto 8791 offline) | `data/operation/quant_brain_bridge_state.json` | INFO | EXTERNO |

### Detalle de cambios Ciclo 2:

**Fix 6 — SystemReadinessReport (CRITICA):**
El orquestador accedia `readiness.is_ready`, `readiness.readiness_score`, `readiness.criteria_passed`, `readiness.criteria_failed`, `readiness.blockers` — ninguno de estos atributos existe en el dataclass `SystemReadinessReport`. Corregido a los atributos reales: `ready`, `passed`, `failed`, `n_trades_evaluated`, `summary`, `next_step`.

**Fix 7 — Snapshot contaminado (CRITICA):**
- Agregado setting `adaptive_learning_epoch_start` (default "2026-03-30") en `config/settings.py`
- `_compute_snapshot()` ahora usa `max(cutoff, epoch_start)` para filtrar entradas pre-reset
- Reseteado `adaptive_policy_snapshot.json` a estado limpio (sample_count=0)

**Fix 8 — Metodo duplicado (ALTA):**
Eliminada la primera definicion de `update_pending_outcome()` (lineas 271-290) que era una version simplificada con defaults opcionales. Python usaba silenciosamente la segunda definicion (mas completa, con matching por precio y tiempo). Ahora solo existe una definicion clara.

**Fix 9 — Direccion credit spreads (ALTA):**
La deteccion `is_short` solo buscaba keywords bearish (`"short"`, `"bear_put"`, `"bear_call"`, `"long_put"`), pero `bull_put_credit_spread` contiene "put" y se clasificaba incorrectamente. Ahora se verifica primero si la estrategia es bullish (`bull_call`, `bull_put`, `covered_call`, `cash_secured_put`) antes de aplicar la deteccion bearish.

---

## Falsos Positivos Descartados

| Bug reportado | Veredicto | Razon |
|---|---|---|
| PnL formula invertida | FALSO POSITIVO | Ya corregida 2026-03-30. Formula `direction * (exit_px - entry_px/qty) * qty` es correcta |
| Short no recibe proceeds (paper_broker) | FALSO POSITIVO | Modelo PnL-only internamente consistente |
| Trailing stop short invertido | FALSO POSITIVO | Verificado numericamente: trailing BAJA correctamente |
| IC tracker vacio | ESPERADO | EPOCH RESET — se llena con trades nuevos |
| action="preview" en auto-cycle | YA CORREGIDO | `auton_mode: "paper_autonomous"` produce `action = "submit"` |
| Learning loop no arranca | FALSO POSITIVO | SI arranca en `api/main.py` startup (linea 218) |

---

## Lo que funciona bien (no tocar)

1. **Paper Broker:** Modelo PnL-only coherente entre apertura/cierre, `_calc_equity()` compensa via unrealized PnL
2. **Journal ORM:** 57 campos, 11 indices, constraint UNIQUE en `journal_key`, `sync_scope()` thread-safe
3. **Exit Governance:** `exit_governance_snapshot` bien implementado (journal/service.py:646-775)
4. **Kelly Position Sizing:** Circuit breaker funcional con tiers SKIP/SMALL/MEDIUM/LARGE
5. **Scanner Pipeline:** `scanner -> selector -> operation_center -> executor -> broker -> journal` completo
6. **WebSocket Live Updates:** Funcional en `/ws/live-updates`
7. **Tradier Integration:** Paper y live sessions preloaded correctamente
8. **Alert Dispatcher:** Telegram activo, worker corriendo

---

## Metas de Paper Trading — 30 dias

| Metrica | Objetivo | Condicion |
|---------|----------|-----------|
| Trade count | >= 50 trades cerrados | Minimo para significancia estadistica |
| Win rate | >= 52% | Superar aleatorio con margen |
| Profit factor | >= 1.15 | Demostrar edge positivo |
| IC (Spearman) | >= 0.05 | Senales con capacidad predictiva |
| Max drawdown | < 15% | Control de riesgo funcional |
| Expectancy | > 0% | Sistema genera valor neto positivo |

---

## Proximos Pasos (2 semanas)

1. **Semana 1:** Monitorear auto-cycle en paper_autonomous. Verificar que IC tracker acumula senales y adaptive policy genera biases con datos limpios.
2. **Semana 1:** Reconectar Brain Bridge (puerto 8791) cuando ATLAS principal este disponible. 22 eventos en cola pendientes de delivery.
3. **Semana 2:** Evaluar metricas vs metas de 30 dias. Si win rate < 50% a los 14 dias, investigar calidad de senales del scanner.
4. **Semana 2:** Agregar endpoints de learning/IC/knowledge al dashboard frontend (actualmente no consumidos).
5. **Continuo:** Los warnings de OpenCV/DSHOW en logs son cosmeticos (vision intenta detectar camaras). No afectan funcionalidad.

---

## Estado Post-Correccion

| Componente | Estado |
|------------|--------|
| Tests | 742/742 passing |
| Compilacion | Todos los archivos OK |
| Scanner | Piso configurable 65.0 (antes 75.0 hardcoded) |
| Signal Generator | TIER_SMALL=0.50 (antes 0.55) |
| Operation Center | paper_autonomous por defecto, vision gate no bloquea paper |
| Learning Orchestrator | SystemReadinessReport atributos correctos |
| IC Tracker | Sin metodo duplicado, epoch reset limpio |
| Adaptive Policy | Filtro epoch_start, snapshot reseteado |
| Journal | Deteccion de direccion corregida para credit spreads |

---

*Auditoria realizada: 2026-03-30/31*
*Branch: variante/nueva*
*Commits: 07085abec (ciclo 1) + pendiente (ciclo 2)*
