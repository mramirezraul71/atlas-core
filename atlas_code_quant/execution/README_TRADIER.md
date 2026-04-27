# Tradier — Implementación canónica (F4)

> **Fase:** F4 (Atlas Code Quant Refactor)
> **Estado:** declaración canónica + marcado legacy del stack paralelo. **No** se cambia routing efectivo ni lógica de ejecución de órdenes.

## TL;DR

* **Stack canónico:** `atlas_code_quant/execution/`
  * `tradier_execution.py` — routing de órdenes (entry point: `route_order_to_tradier`).
  * `tradier_controls.py` — resolución de cuenta + PDT.
  * `tradier_pdt_ledger.py` — ledger local PDT.
  * `atlas_code_quant/pipeline/tradier_stream.py` — streaming/quotes/options chain (no es ruta de envío de órdenes).
* **Stack legacy/PHASE1 (NO usar en producción):**
  * `atlas_options_brain_fase1/atlas_options_brain/broker/tradier_executor.py`
  * `atlas_options_brain_fase1/atlas_options_brain/broker/tradier_live.py`
  * `atlas_options_brain_fase1/atlas_options_brain/providers/tradier_provider.py`

## Por qué `atlas_code_quant/execution` es el canónico

1. Es el **único** stack al que llegan las rutas reales de envío de órdenes en runtime: `BrokerRouter`, `SignalExecutor`, endpoints `/orders`, `OperationCenter`, `AutonExecutor`, `LiveActivation`, `start_paper_trading`.
2. Implementa los **locks de seguridad** del proyecto: `paper_only`, `full_live_globally_locked`, `ATLAS_FORCE_LIVE_PREVIEW`, dry-run defaults, guardrails de `production_guard`.
3. Integra **PDT controls** (`tradier_controls.check_pdt_status`, ledger local en `tradier_pdt_ledger.record_live_order_intent`).
4. Implementa **reconciliación post-timeout** (polling + stream) para evitar duplicados ante errores de red.
5. Está cubierto por `atlas_code_quant/tests/test_tradier_execution_fallback.py` y por las tests de `operation_center` / `production_guard`.

## Por qué el stack PHASE1 es legacy

1. `atlas_options_brain_fase1` es el snapshot de la fase de entrenamiento de Atlas Options Brain. Su carpeta `broker/` se mantiene **sólo** para:
   * Ejecutar tests congelados de phase1 (`tests/test_tradier_executor.py`, `tests/test_tradier_live.py`).
   * Servir como referencia histórica de la API HTTP multileg de Tradier.
2. **No** participa en runtime de Atlas Code Quant. El único consumidor en el repo principal es `atlas/core/options_live.py`, que opera **estrictamente sobre sandbox** (rechaza `sandbox=False` o `allow_production=True`).
3. **No** está integrado con los locks de `atlas_code_quant.config.settings`, ni con el ledger PDT, ni con la reconciliación.
4. `tradier_provider.py` es un cliente de **datos** legacy de phase1, no es ruta de órdenes.

## Reglas duras (F4 y posteriores)

* NO importar `atlas_options_brain_fase1.broker.*` desde código de producción de `atlas_code_quant`.
* NO añadir nuevos consumidores del stack PHASE1.
* NO unificar todavía: la migración real (si llega) pasará por una fase posterior con tests de paridad y plan de rollback.
* NO cambiar firmas ni lógica de `tradier_execution` en F4.
* NO push.

## Migración futura (NO implementada en F4)

Si en una fase posterior se decide consolidar:

1. Inventariar paridad funcional (multileg, locks, PDT, reconciliación).
2. Extraer la API multileg compartida si aplica, dejando phase1 como wrapper deprecated análogo a `lean_simulator.py` (F2).
3. Eliminar consumidores residuales de phase1 en `atlas/core/options_live.py` o sustituirlos por adapter contra el canónico.
4. Test de paridad antes de retirar phase1.

## Referencias

* `docs/ATLAS_CODE_QUANT_F4_TRADIER_CANONICALIZATION.md`
* `atlas_code_quant/config/legacy_flags.py` (`ATLAS_TRADIER_CANONICAL_STACK`, `ATLAS_TRADIER_PHASE1_LEGACY_STACK`)
* `atlas_code_quant/execution/tradier_execution.py` (docstring `.. canonical:: F4`)
* `atlas_options_brain_fase1/atlas_options_brain/broker/tradier_executor.py` (docstring `.. legacy:: F4 PHASE1`)
* `atlas_options_brain_fase1/atlas_options_brain/broker/tradier_live.py` (docstring `.. legacy:: F4 PHASE1`)
