"""Atlas Code Quant — Legacy flags (F1 scaffold).

Marcadores **informativos** introducidos en F1 para anclar el contrato de
componentes congelados. **No alteran el comportamiento de runtime.**

Reglas:
    * NO consultar estas flags desde código productivo en F1.
    * Sólo se usan como referencia documental y para herramientas de auditoría.
    * El cambio de semántica real (deprecation headers, cutover, 410 Gone)
      ocurre en fases posteriores (F3, F10) — no aquí.

Ver:
    - docs/ATLAS_CODE_QUANT_REORG_VERIFICATION.md (F0)
    - docs/ATLAS_CODE_QUANT_F1_REORG.md (F1)
    - docs/ATLAS_CODE_QUANT_F3_SCANNER_DEPRECATION.md (F3)
    - docs/ATLAS_CODE_QUANT_F4_TRADIER_CANONICALIZATION.md (F4)
    - atlas_code_quant/legacy/README_SCANNER_FREEZE.md
    - atlas_code_quant/execution/README_TRADIER.md
"""

from __future__ import annotations

# El scanner heredado (atlas_code_quant/scanner/) queda lógicamente congelado
# desde F1. Esta flag es puramente informativa: no altera handlers, ni rutas,
# ni el comportamiento de operation_center.
SCANNER_IS_LEGACY: bool = True

# El simulador GBM interno NO es LEAN real. Renombrado físicamente en F2 a
# `atlas_code_quant.backtest.internal_gbm_simulator`. El path antiguo
# `atlas_code_quant.backtest.lean_simulator` queda como wrapper deprecated.
LEAN_SIMULATOR_IS_INTERNAL_GBM: bool = True

# ---------------------------------------------------------------------------
# F3 — Deprecación visible de endpoints scanner legacy.
# ---------------------------------------------------------------------------
#
# Habilita la disponibilidad de los endpoints HTTP heredados del scanner
# (rutas `/scanner/*` y `/api/v2/quant/scanner/*` definidos en
# `atlas_code_quant/api/main.py`).
#
# Estado en F3:
#     * Default = ``True``: los endpoints siguen 100% operativos y devuelven
#       el mismo status code, body y schema que antes.
#     * F3 sólo añade headers HTTP de deprecación (`Deprecation`, `Sunset`,
#       `Link`) y un log estructurado WARNING por cada hit.
#     * **Esta flag NO se consulta todavía** en runtime (`api/main.py` no la
#       lee como condicional en F3). Se documenta aquí como contrato
#       público para las fases siguientes.
#
# Roadmap (NO implementado en F3):
#     * Una fase posterior podrá leer esta flag y, cuando sea ``False``,
#       responder ``410 Gone`` o redirigir al sucesor canónico
#       (`/api/radar/*`).
#     * El cutover real depende de migrar a los consumidores documentados
#       en `atlas_code_quant/legacy/README_SCANNER_FREEZE.md`.
#
# Reglas duras:
#     * NO usar esta flag para deshabilitar lógica en F3.
#     * NO cambiar su default sin coordinación explícita con la fase
#       de cutover.
#     * NO interpretar su presencia como permiso para borrar endpoints.
ATLAS_LEGACY_SCANNER_ENABLED: bool = True

# ---------------------------------------------------------------------------
# F4 — Canonicalización del stack Tradier.
# ---------------------------------------------------------------------------
#
# Atlas tiene actualmente DOS implementaciones Tradier coexistiendo:
#
#   1. CANÓNICA (runtime real):
#        ``atlas_code_quant.execution.tradier_execution``
#        (+ ``tradier_controls``, ``tradier_pdt_ledger``).
#      Único stack al que llegan rutas reales de órdenes (BrokerRouter,
#      SignalExecutor, /orders, OperationCenter, AutonExecutor,
#      LiveActivation, start_paper_trading) y que integra los locks
#      ``paper_only`` / ``full_live_globally_locked`` / dry-run defaults,
#      el ledger PDT y la reconciliación post-timeout.
#
#   2. LEGACY/PHASE1 (no es ruta de producción):
#        ``atlas_options_brain_fase1.atlas_options_brain.broker.tradier_executor``
#        ``atlas_options_brain_fase1.atlas_options_brain.broker.tradier_live``
#        ``atlas_options_brain_fase1.atlas_options_brain.providers.tradier_provider``
#      Snapshot de la fase de entrenamiento de Atlas Options Brain. Se
#      mantiene sólo para tests congelados de phase1 y como referencia
#      histórica de la API multileg de Tradier.
#
# Estas dos flags son **puramente documentales** en F4: ningún código de
# runtime las consulta. Sirven como contrato público para herramientas de
# auditoría y para fases posteriores que puedan unificar (con plan explícito
# de paridad y rollback).
#
# Reglas duras:
#     * NO importar el stack PHASE1 desde código de producción de
#       ``atlas_code_quant``.
#     * NO usar estas flags como condicionales de ruta en F4.
#     * NO cambiar sus valores sin coordinación explicita con la fase de
#       unificación.
ATLAS_TRADIER_CANONICAL_STACK: str = "atlas_code_quant"
ATLAS_TRADIER_PHASE1_LEGACY_STACK: str = "atlas_options_brain_fase1"

# ---------------------------------------------------------------------------
# F5 — Radar multi-símbolo (oportunidades).
# ---------------------------------------------------------------------------
#
# Anuncia la existencia de los nuevos endpoints multi-símbolo del Radar:
#
#     GET /api/radar/opportunities
#     GET /api/radar/opportunities/{symbol}
#     GET /api/radar/stream/opportunities  (SSE)
#
# implementados en F5 sobre ``atlas_adapter`` (universe_provider +
# radar_batch_engine + contratos pydantic). Coexisten con el Radar
# single-symbol existente (summary/dealer/diagnostics/camera/stream) sin
# cambiar su contrato.
#
# Estado en F5:
#     * Default = ``False``: los endpoints YA ESTÁN expuestos por el router,
#       pero esta flag se mantiene como **doc-only**: documenta que el
#       “modo multi-símbolo” todavía no está conectado al Code Quant intake
#       real (sin cliente HTTP en ``atlas_code_quant/intake/``) y que los
#       summaries por símbolo dependen de la pipeline existente
#       (``radar_build_dashboard_for_sse``), honestamente degradada cuando
#       Quant no está alcanzable.
#     * **Esta flag NO se consulta todavía** en runtime (radar_public.py
#       NO la lee como condicional en F5).
#
# Reglas duras:
#     * NO usar esta flag para deshabilitar lógica en F5.
#     * NO interpretar su valor como autorización para conectar Code Quant
#       intake (eso queda fuera del alcance F5).
#     * NO tocar los endpoints single-symbol al activar funcionalidades
#       futuras: deben mantenerse retrocompatibles.
ATLAS_RADAR_MULTI_SYMBOL_ENABLED: bool = False

# ---------------------------------------------------------------------------
# F6 — Code Quant Radar intake (shadow only).
# ---------------------------------------------------------------------------
#
# Anuncia la existencia del cliente *intake* interno introducido en F6:
#
#     atlas_code_quant.intake.opportunity
#         RadarOpportunityInternal, RadarOpportunityBatchInternal,
#         RadarIntakeDegradation, from_radar_payload, batch_from_radar_payload.
#     atlas_code_quant.intake.radar_client
#         RadarClient (httpx-based) + fetch_opportunities_snapshot.
#
# El cliente sabe consumir los endpoints multi-símbolo del Radar (F5)
# expuestos por ``atlas_adapter``:
#
#     GET /api/radar/opportunities
#     GET /api/radar/opportunities/{symbol}
#     GET /api/radar/stream/opportunities      (SSE — opcional)
#
# Estado en F6 (shadow only):
#     * Default = ``False``: el cliente y los modelos internos están
#       implementados y testeados, pero **no** están conectados a
#       ``api/main.py``, operations, execution, autonomy, risk, paper loop
#       ni live activation. Code Quant sigue dependiendo del scanner
#       legacy como única fuente efectiva de oportunidades.
#     * **Esta flag NO se consulta todavía** en runtime: ningún módulo de
#       F6 la lee como condicional. Sirve como contrato público para
#       herramientas de auditoría y para la fase posterior que active
#       comparaciones scanner-vs-Radar y, eventualmente, el cutover.
#
# Reglas duras:
#     * NO usar esta flag para enrutar tráfico real a Radar en F6.
#     * NO interpretar su valor como autorización para conectar el cliente
#       intake a operations / execution / autonomy.
#     * NO añadir lógica condicional que haga divergir el comportamiento
#       de runtime en función de esta flag sin un plan explícito de
#       paridad y rollback.
#     * Cualquier cambio de default a ``True`` requiere:
#         - tests de paridad scanner-vs-Radar verdes,
#         - documento de cutover dedicado,
#         - aprobación humana explícita.
ATLAS_RADAR_INTAKE_ENABLED: bool = False

# ---------------------------------------------------------------------------
# F7 — Code Quant scanner→Radar shadow filter.
# ---------------------------------------------------------------------------
#
# Anuncia la existencia del filtro shadow scanner→Radar introducido en F7:
#
#     atlas_code_quant.intake.scanner_radar_filter
#         filter_scanner_candidates_with_radar(
#             scanner_candidates, radar_client=None, *, min_score=70.0
#         ) -> ScannerRadarFilterResult
#
# Objetivo de negocio (referencia):
#     scanner = generador amplio (recall),
#     Radar   = filtro estricto (precision),
#     y eventualmente: nada llega a estrategias sin pasar por Radar.
#
# Estado en F7 (shadow only):
#     * Default = ``False``: el filtro existe, está testeado y es
#       puramente observacional. NO está conectado a:
#         - ``api/main.py`` (endpoints públicos),
#         - scanner (motor / endpoints heredados),
#         - operations (loops live / paper),
#         - execution (Tradier, broker_router, signal_executor),
#         - autonomy / risk / vision / locks.
#     * **Esta flag NO se consulta todavía** en runtime: ningún módulo de
#       F7 la lee como condicional. Sirve como contrato público para
#       herramientas de auditoría y para fases futuras.
#     * Una fase posterior (provisional ``ATLAS_RADAR_FILTER_ENFORCED``)
#       introduciría el gate duro: estrategias dependen de la aprobación
#       del Radar, no sólo del scanner. Eso requiere su propio plan de
#       cutover, paridad y rollback — NO es parte de F7.
#
# Reglas duras:
#     * NO usar esta flag para enrutar tráfico real al filtro en F7.
#     * NO interpretar su valor como autorización para gate duro.
#     * NO conectar el filtro a estrategias / ejecución sin un plan
#       explícito de cutover y aprobación humana.
ATLAS_RADAR_FILTER_SHADOW_ENABLED: bool = False

# ---------------------------------------------------------------------------
# F8 — Code Quant scanner→Radar shadow runtime consumer.
# ---------------------------------------------------------------------------
#
# Anuncia el consumer runtime-safe introducido en F8:
#
#     atlas_code_quant.monitoring.scanner_radar_shadow
#         run_scanner_radar_shadow(scanner_candidates, *, radar_client=None,
#                                  min_score=70.0, emit_logs=True)
#             -> ScannerRadarShadowReport
#
# A diferencia de la flag F7 (puramente doc-only), esta flag está
# pensada para que una fase posterior pueda activar un *hook* de
# observación. F8 NO introduce todavía ese hook en runtime.
#
# Estado en F8:
#     * Default = ``False``: el consumer existe y está testeado pero
#       NO se invoca desde ningún loop de producción. F8 es estrictamente
#       observacional y la herramienta queda disponible para inyección
#       explícita (tests, scripts internos, futuras fases).
#     * Cuando una fase posterior introduzca el hook (probable F9), esta
#       flag puede consultarse en runtime SOLO para habilitarlo. El hook
#       debe seguir siendo observacional: nunca cambia el set efectivo
#       que reciben las estrategias ni introduce decisiones.
#     * El *enforcement* real (Radar como gate duro) requerirá una flag
#       distinta (provisional ``ATLAS_RADAR_FILTER_ENFORCED``) y un
#       documento de cutover dedicado en F10+.
#
# Reglas duras:
#     * F8 NO consulta esta flag en runtime: ningún módulo de F8 la lee
#       como condicional.
#     * Cualquier uso futuro de la flag debe limitarse a habilitar
#       observación / logging / journal interno; nunca a alterar el
#       output del scanner.
#     * Cambiar default a ``True`` requiere el hook ya implementado y
#       tests de no-regresión verdes.
ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED: bool = False

__all__ = [
    "SCANNER_IS_LEGACY",
    "LEAN_SIMULATOR_IS_INTERNAL_GBM",
    "ATLAS_LEGACY_SCANNER_ENABLED",
    "ATLAS_TRADIER_CANONICAL_STACK",
    "ATLAS_TRADIER_PHASE1_LEGACY_STACK",
    "ATLAS_RADAR_MULTI_SYMBOL_ENABLED",
    "ATLAS_RADAR_INTAKE_ENABLED",
    "ATLAS_RADAR_FILTER_SHADOW_ENABLED",
    "ATLAS_RADAR_FILTER_SHADOW_RUNTIME_ENABLED",
]
