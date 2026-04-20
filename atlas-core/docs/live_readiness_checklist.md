> This document is **local, text-only content prepared for commit** in this repository.  
> It does **not** perform any remote action (no pushes, no GitHub edits, no PR changes) by itself.  
> Any decision to commit, push, or forge releases based on this checklist must be made explicitly by a human operator.

# Live Readiness Go / No-Go Checklist

## Summary

This **live go / no-go checklist** sits **on top of** the current umbrella remediation baseline described in PR #3 (`variante/nueva` → `intent-input-rename`): paper safety, conservative equity Kelly sizing in a paper-first posture, a minimal degradable MLSignalRanker runtime with scanner context enrichment, `market_open_config.json` risk/schedule integration with **explicit env > JSON > defaults** precedence, canonical `atlas_core` documentation plus manifest/verification scaffolding, and a **paper-first, read-only** operational self-audit wired through readiness. That work is intentionally scoped to **paper / sandbox** readiness and explicitly **does not** authorize live trading, hard **BLOCK** semantics for self-audit, aggressive live sizing, or strong circuit-breaker-style enforcement—those are expected to land in **separate, explicit phases/PRs** once stakeholders agree the additional risk and controls.

**Nothing in this checklist is assumed satisfied merely because PR #3 exists or is merged.** Each item below requires an **explicit human decision** (yes/no, evidence link, owner, and date), including waivers where policy allows. Until those items are completed to the **MUST** bar (or formally waived with recorded rationale), the system should be treated as **not approved** for any production/live capital exposure.

## Scope and Baseline

- **Paper safety (baseline):** market-hours gating, `max_open_positions`, paper exit governance, and post-timeout `broker_order_id` reconciliation under the paper audit contract.
- **Risk & sizing (paper-first):** conservative equity Kelly sizing with agreed limits **in the paper context only**, coexisting with paper safety via centralized `settings` / configuration.
- **ML & scanner (paper-first):** minimal degradable MLSignalRanker runtime, scanner candidate context enrichment, no new hard vetoes in paper.
- **Operational configuration:** `market_open_config.json` as the operational source for risk/schedule with **explicit env > JSON > defaults** precedence, validation, and fallbacks.
- **Readiness visibility:** `market_open_operational` snapshot in readiness (fast + diagnostic); operational self-audit in paper as **read-only** input to readiness (no new BLOCK semantics).
- **Core integrity (non-trading):** canonical `atlas_core` documentation, manifest, and verification scaffolding (import/manifest checks—not a substitute for live controls).

This document is **for evaluating whether Atlas may move toward live trading** under explicit policies and evidence. It is **not** a line-by-line description of the codebase.

## Checklist Blocks

- **A. Configuración y riesgo (risk & sizing)** — Precedencia y trazabilidad de límites (env / JSON / defaults), sizing en live y alineación con política de capital.
- **B. Seguridad del runtime (gates, self-audit, circuit breakers)** — Comportamiento de gates en live, severidad del self-audit, pausas y límites duros ante pérdidas o anomalías.
- **C. Integridad de ejecución y reconciliación con broker** — Órdenes reales, IDs de broker, timeouts, reintentos y cierre del ciclo operativo vs. paper.
- **D. Observabilidad y alertas** — Logs, métricas, dashboards y rutas de escalado ante degradación.
- **E. Gobernanza y procesos** — Quién autoriza live, registro de decisiones, kill switch y roles.
- **F. Backtesting / forward-testing mínimo aceptable** — Evidencia cuantitativa y período de validación antes de exposición real.

## A. Configuración y riesgo (risk & sizing)

### Covered today (paper / baseline)

- `market_open_config.json` as the operational source for risk/schedule with **explicit env > JSON > defaults** precedence, plus validation and fallbacks.
- Conservative equity Kelly sizing in **paper-first** mode with limits agreed for that context.
- Coexistence with paper safety through centralized `settings` / configuration (per PR #3 umbrella description).

### Missing for live

- An explicit **live risk limits** policy (per symbol, account, intraday) that is distinct from or stricter than paper, with reviewed and versioned values.
- Live behavior for Kelly (and any other sizers): opt-in, caps, and tests under a **live contract** (*to be defined with stakeholders* if product policy differs).
- A single **env + JSON + defaults** matrix for operators in a **live** context (left as a documentation follow-up from the umbrella).

### Checklist questions

- **[MUST]** ¿Existen variables de entorno (o equivalente aprobado) para **riesgo live** con valores acordados, revisados y registrados fuera del solo “default de desarrollo”?
- **[MUST]** ¿Hay una decisión explícita de qué partes de `market_open_config` / env aplican en live y cuáles quedan desactivadas o en modo conservador?
- **[SHOULD]** ¿Está documentada la matriz de precedencia env > JSON > defaults para el entorno live (no solo paper)?
- **[SHOULD]** ¿El sizing en live (Kelly u otros) tiene límites máximos y flags de habilitación explícitos, no inferidos del comportamiento paper?
- **[NICE]** ¿Hay un procedimiento de rotación/revisión periódica de valores de riesgo live (*to be defined with stakeholders*)?

## B. Seguridad del runtime (gates, self-audit, circuit breakers)

### Covered today (paper / baseline)

- Paper safety: market-hours gate, `max_open_positions`, paper exit governance, and post-timeout reconciliation under the **paper** contract.
- Operational self-audit in paper: **read-only**, no hard vetoes; integrated via readiness.
- ML runtime: minimal **degradable** layer; no new hard vetoes in paper (per PR #3).

### Missing for live

- Definition of **live gates** (hours, exposure, kill switch) and whether they mirror or harden paper semantics.
- **BLOCK** semantics (or equivalent) for critical self-audit checks in live, **or** an explicit documented decision to keep self-audit monitoring-only (*stakeholders*).
- **Circuit breakers** / `daily_loss_limit` and other **hard** pauses in live (explicitly out of scope for umbrella PR #3).

### Checklist questions

- **[MUST]** ¿Existe un mecanismo documentado para forzar `live_disabled` (o equivalente) ante incidente, con responsable y pasos de verificación?
- **[MUST]** ¿Está decidido y documentado si el self-audit en live será BLOCK para condiciones críticas, solo alerta, u otra semántica (*to be defined with stakeholders*)?
- **[MUST]** ¿Los gates de mercado y exposición en live están especificados (umbrales, fuentes de verdad, fail-safe)?
- **[SHOULD]** ¿Hay pruebas o simulaciones que demuestren el comportamiento de gates cuando feeds o relojes fallan?
- **[SHOULD]** ¿Está definido el comportamiento ante degradación del ranker ML en live (degradación limpia vs. bloqueo)?
- **[NICE]** ¿Checklist de “pre-open” diario para operadores antes de sesión live (*to be defined with stakeholders*)?

## C. Integridad de ejecución y reconciliación con broker

### Covered today (paper / baseline)

- Paper: exit governance and post-timeout `broker_order_id` reconciliation where the **paper** audit contract applies.

### Missing for live

- A reconciliation contract specific to a **real broker** (partial states, partial fills, cancel/replace, journal–broker desynchronization).
- Retry and idempotency policy for live order submission (*technical detail to agree with broker and stakeholders*).

### Checklist questions

- **[MUST]** ¿Hay un flujo documentado de reconciliación entre journal interno y estados del broker en live tras timeout o error de red?
- **[MUST]** ¿Las órdenes live están acotadas a cuentas/entornos explícitamente aprobados (paper vs live token, cuentas segregadas)?
- **[SHOULD]** ¿Existen pruebas automatizadas o manuales que cubran fallos de broker en el camino live (simulacro o sandbox broker)?
- **[SHOULD]** ¿Está definido qué ocurre si el sistema reinicia a mitad de orden (*to be defined with stakeholders*)?
- **[NICE]** ¿Runbook de “orden colgada” o discrepancia de fills (*to be defined with stakeholders*)?

## D. Observabilidad y alertas

### Covered today (paper / baseline)

- `market_open_operational` snapshot in readiness (fast + diagnostic), per PR #3.
- Self-audit provides a structured readiness summary when enabled (paper).

### Missing for live

- Integration of self-audit (and other signals) into **reporting / monitoring / alerting** for live (explicit PR follow-up).
- Alert thresholds and **ownership** for live incident response.

### Checklist questions

- **[MUST]** ¿Hay canales de alerta acordados para fallos de readiness, errores de ejecución y riesgo en live?
- **[MUST]** ¿Los logs/métricas permiten reconstruir una orden live de extremo a extremo (entrada → broker → journal)?
- **[SHOULD]** ¿Los dashboards o vistas mínimas para live están definidos y accesibles para el operador de guardia?
- **[SHOULD]** ¿El self-audit en live tiene destino claro (log, métrica, ticket) aunque no bloquee?
- **[NICE]** ¿SLOs o alertas de latencia en path crítico live (*to be defined with stakeholders*)?

## E. Gobernanza y procesos

### Covered today (paper / baseline)

- PR #3 explicitly documents posture: **live out of scope**; remediation targets paper/sandbox and holistic review of the bundle.

### Missing for live

- Who approves the first (and subsequent) live activations, with an auditable record.
- Rollback and incident communications (*to be defined with stakeholders* at operational detail level).

### Checklist questions

- **[MUST]** ¿Existe un acta o registro firmado/aprobado (proceso interno) que autorice encender live en una cuenta concreta?
- **[MUST]** ¿Está asignado un responsable on-call durante el piloto live?
- **[SHOULD]** ¿Hay un procedimiento de rollback (código + config + flags) documentado y probado al menos en mesa?
- **[SHOULD]** ¿Hay criterios de cierre de piloto (éxito / fallo) antes de ampliar capital o símbolos?
- **[NICE]** ¿Revisión legal/compliance para trading live (*to be defined with stakeholders*)?

## F. Backtesting / forward-testing mínimo aceptable

### Covered today (paper / baseline)

- PR #3 references automated tests by area (paper safety, Kelly, ML runtime, `market_open_config`, `atlas_core` manifest, self-audit). These support **technical regression** in dev/CI and **do not** replace live validation.

### Missing for live

- Minimum backtest and/or forward-test criteria (duration, markets, stress) before real exposure (*to be defined with stakeholders*).
- An explicit “**stable paper for X time**” criterion before live (*to be defined with stakeholders*).

### Checklist questions

- **[MUST]** ¿Hay criterios cuantitativos mínimos (drawdown, sample size, período) acordados para considerar live, o una decisión explícita de waive con justificación?
- **[SHOULD]** ¿Existe un período de forward test en paper o simulación con parámetros idénticos a los previstos para live?
- **[SHOULD]** ¿Los resultados de back/forward test están archivados y enlazados a la decisión go/no-go?
- **[NICE]** ¿Stress tests de configuración (env incorrecta, JSON corrupto) en entorno que imite prod (*to be defined with stakeholders*)?

## Global Prioritization

### MUST

- Kill switch / `live_disabled` (or equivalent) with a documented incident path.
- Explicit decision: self-audit **BLOCK** vs **monitor-only** semantics for live-critical checks.
- Live gates and live risk limits documented (thresholds, sources of truth, fail-safe behavior).
- Broker–journal reconciliation in live after timeouts/network errors.
- Minimum alerting channels and end-to-end traceability for a live order (intent → broker → journal).
- Formal go-live approval record (human sign-off).
- Minimum evidence criteria for live (or an explicit, recorded waiver).

### SHOULD

- Live matrix documentation for env > JSON > defaults.
- Broker-failure and mid-flight restart tests (manual or automated) on the live path (sandbox/simulation where applicable).
- Documented rollback (code + config + flags), at least table-tested.
- Operator dashboards / minimum live views defined and accessible.
- Archived forward-test results linked to the go/no-go decision.
- Explicit live sizing limits and enablement flags (not inferred from paper defaults).

### NICE

- Periodic risk value review cadence (*to be defined with stakeholders*).
- SLOs / latency alerting on critical live paths (*to be defined with stakeholders*).
- Extended runbooks, compliance review, advanced configuration stress tests (*to be defined with stakeholders*).

## Location and Governance

- **Canonical source:** this file is intended to live at **`docs/live_readiness_checklist.md`** at the repository root of **`atlas-core`** (this path).
- **Optional mirror:** if a copy is ever added at `atlas_code_quant/docs/live_readiness_checklist.md`, that file must begin with a prominent pointer stating that the **source of truth** is **`docs/live_readiness_checklist.md`** in the `atlas-core` repository (avoid divergent edits).
- **Owners:** maintain with **trading operations / risk owners** (roles and names are *to be defined with stakeholders*).
- **Updates:** any material risk-policy change (limits, BLOCK semantics, kill switch rules, evidence bar) should drive a **new commit** revising this document, with a short change note and approver recorded in commit message or linked internal record.

---

> This file is **text-only** and prepared for **local commit**. It does **not** perform remote changes, and it does **not**, by itself, approve live trading or real-capital exposure.  
> Any capital decision requires **explicit human review** against this checklist (including recorded waivers where policy allows).
