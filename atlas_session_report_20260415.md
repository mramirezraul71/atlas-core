# ATLAS CODE QUANT — INFORME EJECUTIVO DE SESIÓN (15-ABR-2026)

## Resumen

Sesión técnica centrada en **tres bloqueadores operativos**, **corrección del sesgo A–B del scanner**, **migración post-PDT (SEC / regla efectiva 2026-04-14 en stack)**, **relajación del `PDTController` en runtime (opt-in legacy)** y **infraestructura de paper trading / monitoreo**.

**Estado de la suite (verificado hoy en `C:\ATLAS_PUSH`):** **926 tests passed** en ~24 s (**1400 warnings**, mayormente deprecaciones `utcnow` y avisos de dependencias opcionales).

**Rama de referencia remota:** `origin/journal-forensic-20260411-quant` (último push conocido incluye paper-trading + PDT controller + eliminación PDT + scanner + bloqueadores + learning-gate).

---

## PARTE 1 — Auditoría de código: bloqueadores

### Bloqueador #1 — Market Hours Gate

| Ítem | Hallazgo |
|------|----------|
| **Ubicación** | `atlas_code_quant/operations/operation_center.py` línea **234** |
| **Firma** | `def _check_market_hours(self, symbol: str, account_scope: str) -> tuple[bool, str]` |
| **Lógica** | `pytz` zona **America/New_York**; **lun–vie**; ventana **09:30–16:00 ET**; fin de semana `market_closed_weekend`; antes/después de sesión con razones explícitas; **crypto** vía lista corta + `classify_asset` → `crypto_market_24h` |
| **Integración** | `market_hours_readiness` (aprox. L268+); en `evaluate_candidate` uso de `_check_market_hours` y payload de gates (aprox. L1928–1934, L2303–2307) |
| **Tests** | **`tests/test_operation_center_market_hours.py`**: **9** métodos `test_*` (equity intraday / pre-open / post-close / weekend / crypto / preflight blocked-allowed / readiness payload) |

### Bloqueador #2 — Broker Order IDs (stream → polling)

| Ítem | Hallazgo |
|------|----------|
| **Función central** | `_execute_with_fallback` en `execution/tradier_execution.py` (**L321**) |
| **Parámetros por defecto** | `primary_method="stream_confirmation"`, `fallback_method="polling_get_orders"`, **`max_retries=3`**, **`timeout_sec`** en `route_order_to_tradier`: **`max(int(settings.tradier_timeout_sec), 1)`** (no fijo 5 s en ruta principal) |
| **Implementación** | `_stream_order_confirmation` (**L263+**), `_polling_get_orders` (**L286+**) con matching por id enviado y **fallback `_match_order`** (símbolo/lado/cantidad) |
| **Garantía de ID** | Tras fallback, `_extract_broker_order_ids`; si falta **`id`**, **`ValueError`** con mensaje CRITICAL (**L428–434**) |
| **Tests** | **`tests/test_tradier_execution_fallback.py`**: **11** tests (stream ok/timeout, polling match/timeout, match/mismatch, fallback cadena, fallo total, `route_order` con ID garantizado y caso sin ID) |

### Bloqueador #3 — Reconciliación / sync con fallback y phantoms

| Ítem | Hallazgo |
|------|----------|
| **Ubicación** | `journal/service.py` — **`sync_scope`** (**L1997+**) |
| **Fallback** | Matching alternativo; en logs: **`symbol_entry_time_fallback`** cuando falta `broker_order_id` (aprox. L2156–2158) |
| **Phantom** | Log **`PHANTOM DETECTED: Broker position ... not in journal`** (aprox. L2192); estados **`reconciliation_status`**: entre otros **`OK`**, **`PARTIAL_UNMATCHED_BROKER`**, **`PARTIAL_UNMATCHED_JOURNAL`** (aprox. L2198–2202, L2227) |
| **Tests** | **`tests/test_journal_sync_fallback.py`**: **11** tests `test_*` (match por broker id, fallback recency, phantom, journal unmatched, qty mismatch, entrada vieja, escenario complejo, vacío-vacío, logging phantom, `_is_recent_entry`) |

---

## PARTE 2 — Auditoría de tests (resumen numérico)

| Área | Archivo | `# test_*` (aprox.) |
|------|---------|---------------------|
| Market hours | `test_operation_center_market_hours.py` | **9** |
| Broker fallback | `test_tradier_execution_fallback.py` | **11** |
| Journal sync / phantom | `test_journal_sync_fallback.py` | **11** |
| Gates operacionales post-PDT | `test_operational_gates_post_pdt.py` | **7** |
| **Suma bloqueadores** | | **31** |

**Suite completa:** `pytest atlas_code_quant/tests/ -q` → **926 passed** (warnings elevados por entorno; no se auditó aquí cobertura % por línea).

---

## PARTE 3 — PDT elimination + controller legacy

- **`execution/tradier_controls.py`**: `check_pdt_status` **deprecado**, payload con `reason: pdt_rule_removed_2026_04_14` (aprox. L302–337).
- **`operations/operation_center.py`**: constantes **`MAX_DAILY_TRADES_LIVE` (50)**, **`MAX_DAILY_TRADES_PAPER` (100)**, exposición intradía **50%**, drawdown **10%**, racha de pérdidas **3**; métodos **`check_daily_trade_limit`**, **`check_intraday_exposure`**, **`check_intraday_drawdown`**, **`check_consecutive_loss_cooldown`** integrados en **`evaluate_candidate`** (aprox. L345–431, L1864+).
- **`execution/pdt_controller.py`**: por defecto **sin bloqueos** en live salvo **`ATLAS_PDT_LEGACY_BLOCKS=1`**; alertas PDT a **Telegram eliminadas** (solo log).
- **Documentación en repo:** `atlas_code_quant/PDT_MIGRATION.md`, `atlas_code_quant/pdt_gates_found.txt`.
- **Informe de auditoría previo:** `atlas_audit_report_20260415.txt` (raíz del repo).

---

## PARTE 4 — Scanner A–B bug fix

- **Función:** `_stable_symbol_mix` en `scanner/opportunity_scanner.py` (**L221–225**): orden **determinista** por **`hashlib.sha1(...).hexdigest()`** para romper sesgo alfabético con `prefilter_count` / cursor.
- **Uso:** aprox. **L638** al preparar lista de símbolos del catálogo.
- **Test de regresión:** `tests/test_scanner_metric_recalibration.py` — **`test_stable_symbol_mix_breaks_alphabetical_front_bias`** (más otros tests del archivo no ligados al A–B).

---

## PARTE 5 — Learning y XGBoost

- **XGBoost:** módulo `learning/xgboost_signal/` (`model_trainer.py`, `backtest_engine.py`, etc.) con **`xgb.XGBClassifier`** y flags tipo **`QUANT_XGBOOST_ENABLED`**.
- **Learning gate:** commit en historial remoto **`e33a52e53`** — *integrate reconciliation gate into learning orchestrator*.
- **Calidad de journal:** `learning/journal_data_quality.py` y tests asociados en el árbol actual.

---

## PARTE 6 — Comunicaciones e integración (alto nivel)

- **Orden y ejecución:** `route_order_to_tradier` → `_execute_with_fallback` → ledger / respuesta con `broker_order_ids_json`.
- **Centro operativo:** `OperationCenter.evaluate_candidate` con gates de mercado, límites diarios, drawdown, exposición, cooldown, migración PDT en payloads.
- **Journal:** SQLAlchemy en `journal/db.py`; servicio grande en `journal/service.py` (sync, reconciliación, phantoms).
- **Core autónomo:** `atlas_quant_core.py` orquesta loop; no sustituye la API REST en `main.py`.

---

## PARTE 7 — Commits y documentación (remoto `journal-forensic-20260411-quant`)

Orden **reciente** (primeros de la lista = más nuevos):

1. `aa839034a` — feat(paper-trading): 7-day monitoring infrastructure  
2. `66e560afa` — feat(quant): PDT controller default no-block; legacy env; remove Telegram PDT alerts  
3. `c2cce900f` — feat(pdt-elimination): adapt Atlas to SEC PDT rule removal  
4. `bafcaf86b` — docs: audit report (bloqueadores)  
5. `a8118315d` — fix(scanner): alphabetical bias  
6. `213bbb735` — fix(dashboard): realtime / scanner (UI vía API estática; ver reglas de dashboards si se toca UI)  
7. `f4b820865` — fix(tests): conftest + API startup  
8. `37b56b2b5` — fix(runtime): módulos validación / journal quality  
9. `0aad8389e` — chore(tests): pytest markers  
10. `e33a52e53` — feat(learning-gate): reconciliation gate en learning orchestrator  
11. `79ee0e338` — feat(blocker-3): reconciliation + phantom  
12. `196d4560a` — feat(blocker-2): broker_order_ids fallback  
13. `8be98fe30` — feat(blocker-1): market_hours_gate  

*(La rama remota contiene más historia previa; no se listan aquí todos los commits hasta el origen del repo.)*

**Nota:** En **`variante/nueva`** local puede existir el **mismo cambio** paper-trading con hash distinto (`296e309b7`) respecto a `aa839034a` (cherry-pick); el **verdad operativa para GitHub** es **`origin/journal-forensic-20260411-quant`**.

---

## PARTE 8 — Paper trading (infraestructura)

Archivos presentes:

- `atlas_code_quant/config/paper_trading_config.py`
- `atlas_code_quant/start_paper_trading.py` (preflight + opción `--no-core`; arranque `atlas_quant_core` con `PYTHONPATH`)
- `atlas_code_quant/daily_paper_report.py`

**No existe** en el repo **`generate_7day_report.py`** (la plantilla del usuario lo mencionaba; **no fue creado** en esta base de código).

**Uso recomendado desde raíz del repo:**

```text
python -m atlas_code_quant.start_paper_trading --no-core
python -m atlas_code_quant.start_paper_trading
python -m atlas_code_quant.daily_paper_report 1
```

---

## PARTE 9 — Checklist de preparación (paper / validación)

- [x] Tres bloqueadores implementados y cubiertos por tests focalizados (31 tests).  
- [x] Scanner: mezcla estable de símbolos + test de regresión.  
- [x] PDT: capa API/controls deprecada; gates operacionales en `OperationCenter`; tests post-PDT (7).  
- [x] Suite **926** green en entorno local verificado.  
- [x] Rama remota con commits atómicos recientes (lista arriba).  
- [x] Documentación PDT + auditoría texto en repo.  
- [ ] **Validación runtime 7 días:** requiere proceso largo, tokens Tradier paper válidos, y revisión diaria de métricas (ver scripts paper).  
- [ ] **Cobertura “100%” por bloqueador:** no sustituida por mediciones `coverage.py` en este informe.  

---

## Verdict ejecutivo

**Verde para continuar validación paper** a nivel de **código, tests y rama remota documentada**, con la condición operativa de **credenciales / mercado / infraestructura** fuera de este informe.

**Riesgos explícitos no resueltos por código solo:** límites de GitHub por blobs grandes en **`variante/nueva`** (journal sqlite histórico); push directo `HEAD:journal-forensic` desde ramas divergentes sigue pudiendo fallar sin cherry-pick/worktree.

---

*Informe generado por auditoría de repositorio en fecha de sesión 15-abr-2026. Ajustar fechas de “validación 7 días” según calendario real de arranque.*
