# Live go / no-go checklist (Atlas)

> **Sobre este archivo (solo contenido local)**  
> Este markdown es **material listo para commit** en tu copia del repositorio. **No** implica `git push`, apertura/edición de PRs en GitHub, ni ninguna acción remota. Quien integre el cambio decide cuándo hacer commit, push y revisión en la forja.

> **Baseline narrativo (no es aprobación live)**  
> Se alinea con el umbrella **PR #3** (`variante/nueva` → `intent-input-rename`): trabajo **paper-first** y **live explícitamente fuera de alcance** en ese PR. Ningún ítem de este checklist se da por cumplido **solo** porque exista o se mergee PR #3.

---

## Intro (English — paste-friendly for PR or doc header)

This **live go / no-go checklist** sits **on top of** the current umbrella remediation baseline described in PR #3 (`variante/nueva` → `intent-input-rename`): paper safety, conservative equity Kelly sizing in a paper-first posture, a minimal degradable MLSignalRanker runtime with scanner context enrichment, `market_open_config.json` risk/schedule integration with **explicit env > JSON > defaults** precedence, canonical `atlas_core` documentation plus manifest/verification scaffolding, and a **paper-first, read-only** operational self-audit wired through readiness. That work is intentionally scoped to **paper / sandbox** readiness and explicitly **does not** authorize live trading, hard **BLOCK** semantics for self-audit, aggressive live sizing, or strong circuit-breaker-style enforcement—those are expected to land in **separate, explicit phases/PRs** once stakeholders agree the additional risk and controls.

**Nothing in this checklist is assumed satisfied merely because PR #3 exists or is merged.** Each item below requires an **explicit human decision** (yes/no, evidence link, owner, and date), including waivers where policy allows. Until those items are completed to the **MUST** bar (or formally waived with recorded rationale), the system should be treated as **not approved** for any production/live capital exposure.

---

## Leyenda de prioridad (audiencia operaciones / riesgo)

| Etiqueta | Significado operativo |
|----------|------------------------|
| **MUST** | Obligatorio antes de cualquier exposición live; si no aplica, hace falta **waive** firmado con motivo. |
| **SHOULD** | Muy recomendable; si falta, documentar riesgo residual y responsable aceptante. |
| **NICE** | Mejora deseable; no bloquea un piloto **muy** acotado si el comité de riesgo lo acepta por escrito. |

**Campos sugeridos por ítem MUST:** responsable · fecha · enlace a evidencia (acta, ticket, export de config aprobada).

---

## A. Configuración y riesgo (risk & sizing)

Precedencia y trazabilidad de límites (env / JSON / defaults), sizing en live y alineación con política de capital.

### Cubre hoy (baseline PR #3 / `variante/nueva`)

- `market_open_config.json` como fuente operativa de riesgo/schedule con precedencia **env explícita > JSON > defaults** y validación/fallbacks.
- Kelly conservador para equity en **paper-first** con límites acordados en ese contexto.
- Coexistencia con paper safety vía `settings` / configuración centralizada (según descripción del PR).

### Falta para live

- Política explícita de **límites live** (por símbolo, cuenta, intradía) distinta o endurecida respecto a paper, con valores revisados y versionados.
- Comportamiento de Kelly (u otros sizers) en modo live: opt-in, caps y tests bajo contrato live (*to be defined with stakeholders* si el producto fija otra política).
- Matriz única **env + JSON + defaults** para operadores en contexto **live** (follow-up documental respecto al umbrella).

### Preguntas (checklist)

- **[MUST]** ¿Existen variables de entorno (o equivalente aprobado) para **riesgo live** con valores acordados, revisados y registrados fuera del solo “default de desarrollo”?
- **[MUST]** ¿Hay una decisión explícita de qué partes de `market_open_config` / env aplican en live y cuáles quedan desactivadas o en modo conservador?
- **[SHOULD]** ¿Está documentada la matriz de precedencia env > JSON > defaults para el entorno live (no solo paper)?
- **[SHOULD]** ¿El sizing en live (Kelly u otros) tiene límites máximos y flags de habilitación explícitos, no inferidos del comportamiento paper?
- **[NICE]** ¿Hay un procedimiento de rotación/revisión periódica de valores de riesgo live (*to be defined with stakeholders*)?

---

## B. Seguridad del runtime (gates, self-audit, circuit breakers)

Comportamiento de gates en live, severidad del self-audit, pausas y límites duros ante pérdidas o anomalías.

### Cubre hoy

- Paper safety: gate de horario de mercado, `max_open_positions`, gobernanza de salidas en paper, reconciliación post-timeout en el contrato paper.
- Self-audit operacional en paper: **read-only**, sin vetos duros; integrado vía readiness.
- ML runtime: capa mínima **degradable**, sin veto duro nuevo en paper (según PR).

### Falta para live

- Definición de **gates en live** (horario, exposición, kill switch) y si replican o endurecen la semántica paper.
- Semántica **BLOCK** (o equivalente) en self-audit para checks críticos en live, **o** decisión explícita documentada de mantenerlo solo como monitorización (*stakeholders*).
- **Circuit breakers** / `daily_loss_limit` u otras pausas **duras** en live (explícitamente fuera del alcance del umbrella PR #3).

### Preguntas (checklist)

- **[MUST]** ¿Existe un mecanismo documentado para forzar `live_disabled` (o equivalente) ante incidente, con responsable y pasos de verificación?
- **[MUST]** ¿Está decidido y documentado si el self-audit en live será BLOCK para condiciones críticas, solo alerta, u otra semántica (*to be defined with stakeholders*)?
- **[MUST]** ¿Los gates de mercado y exposición en live están especificados (umbrales, fuentes de verdad, fail-safe)?
- **[SHOULD]** ¿Hay pruebas o simulaciones que demuestren el comportamiento de gates cuando feeds o relojes fallan?
- **[SHOULD]** ¿Está definido el comportamiento ante degradación del ranker ML en live (degradación limpia vs. bloqueo)?
- **[NICE]** ¿Checklist de “pre-open” diario para operadores antes de sesión live (*to be defined with stakeholders*)?

---

## C. Integridad de ejecución y reconciliación con broker

Órdenes reales, IDs de broker, timeouts, reintentos y cierre del ciclo operativo vs. paper.

### Cubre hoy

- En paper: gobernanza de salidas, reconciliación post-timeout de `broker_order_id` donde aplica el contrato de auditoría paper.

### Falta para live

- Contrato de reconciliación **específico para broker real** (estados parciales, fills parciales, cancel/replace, desincronización journal–broker).
- Política de **reintentos e idempotencia** en envíos live (*detalle técnico a acordar con broker y stakeholders*).

### Preguntas (checklist)

- **[MUST]** ¿Hay un flujo documentado de reconciliación entre journal interno y estados del broker en live tras timeout o error de red?
- **[MUST]** ¿Las órdenes live están acotadas a cuentas/entornos explícitamente aprobados (paper vs live token, cuentas segregadas)?
- **[SHOULD]** ¿Existen pruebas automatizadas o manuales que cubran fallos de broker en el camino live (simulacro o sandbox broker)?
- **[SHOULD]** ¿Está definido qué ocurre si el sistema reinicia a mitad de orden (*to be defined with stakeholders*)?
- **[NICE]** ¿Runbook de “orden colgada” o discrepancia de fills (*to be defined with stakeholders*)?

---

## D. Observabilidad y alertas

Logs, métricas, dashboards y rutas de escalado ante degradación.

### Cubre hoy

- Snapshot `market_open_operational` en readiness (fast + diagnostic), según PR.
- Self-audit aporta resumen estructurado en readiness cuando está habilitado (paper).

### Falta para live

- Integración de self-audit (y otras señales) en **reporting / monitoring / alertas** para live (follow-up explícito del PR).
- **Umbrales de alerta** y **ownership** de respuesta en incidentes live.

### Preguntas (checklist)

- **[MUST]** ¿Hay canales de alerta acordados para fallos de readiness, errores de ejecución y riesgo en live?
- **[MUST]** ¿Los logs/métricas permiten reconstruir una orden live de extremo a extremo (entrada → broker → journal)?
- **[SHOULD]** ¿Los dashboards o vistas mínimas para live están definidos y accesibles para el operador de guardia?
- **[SHOULD]** ¿El self-audit en live tiene destino claro (log, métrica, ticket) aunque no bloquee?
- **[NICE]** ¿SLOs o alertas de latencia en path crítico live (*to be defined with stakeholders*)?

---

## E. Gobernanza y procesos

Quién autoriza live, registro de decisiones, kill switch y roles.

### Cubre hoy

- Postura documentada en PR #3: live fuera de alcance; remediación orientada a paper/sandbox y revisión del conjunto.

### Falta para live

- **Quién aprueba** el primer live (y siguientes), con registro auditable.
- **Rollback** y comunicación ante incidentes (*to be defined with stakeholders* en detalle operativo).

### Preguntas (checklist)

- **[MUST]** ¿Existe un acta o registro firmado/aprobado (proceso interno) que autorice encender live en una cuenta concreta?
- **[MUST]** ¿Está asignado un responsable on-call durante el piloto live?
- **[SHOULD]** ¿Hay un procedimiento de rollback (código + config + flags) documentado y probado al menos en mesa?
- **[SHOULD]** ¿Hay criterios de cierre de piloto (éxito / fallo) antes de ampliar capital o símbolos?
- **[NICE]** ¿Revisión legal/compliance para trading live (*to be defined with stakeholders*)?

---

## F. Backtesting / forward-testing mínimo aceptable

Evidencia cuantitativa y período de validación antes de exposición real.

### Cubre hoy

- PR #3 menciona suites de tests por área (paper safety, Kelly, ML runtime, `market_open_config`, manifest `atlas_core`, self-audit); cubre **regresión técnica** en desarrollo/CI, **no** sustituye validación live.

### Falta para live

- Criterios mínimos de backtest y/o forward test (duración, mercados, estrés) antes de exposición real (*to be defined with stakeholders*).
- Criterio explícito de “paper estable X tiempo” antes de live (*to be defined with stakeholders*).

### Preguntas (checklist)

- **[MUST]** ¿Hay criterios cuantitativos mínimos (drawdown, sample size, período) acordados para considerar live, o una decisión explícita de waive con justificación?
- **[SHOULD]** ¿Existe un período de forward test en paper o simulación con parámetros idénticos a los previstos para live?
- **[SHOULD]** ¿Los resultados de back/forward test están archivados y enlazados a la decisión go/no-go?
- **[NICE]** ¿Stress tests de configuración (env incorrecta, JSON corrupto) en entorno que imite prod (*to be defined with stakeholders*)?

---

## Priorización global (resumen)

- **MUST:** kill switch / `live_disabled`, decisión self-audit BLOCK vs solo monitorización, gates y límites live documentados, reconciliación broker–journal en live, alertas mínimas y trazabilidad de órdenes, aprobación formal go-live, criterios mínimos de evidencia (o waive explícito).
- **SHOULD:** matriz env/JSON/defaults para live, pruebas de fallo broker/reinicio, rollback documentado, dashboards operador, forward test archivado, sizing live con límites explícitos.
- **NICE:** revisiones periódicas de riesgo, SLOs, runbooks extendidos, compliance, stress de config avanzado.

---

## Ubicación y enlaces sugeridos (referencia)

- **Canónico (este archivo):** `docs/live_readiness_checklist.md` en la raíz del repo donde viva la política operativa.  
  *Alternativa si el repo separa doc quant:* `atlas_code_quant/docs/live_readiness_checklist.md` (una sola copia canónica; la otra solo puntero).
- **Puntero breve** (sin duplicar este contenido): subsección “Live readiness (go/no-go)” en README de operaciones o en `atlas_core/CANONICAL.md`.
- **GitHub (cuando tú lo decidas):** enlace desde el PR umbrella #3 o un issue de tracking al path del archivo en la rama publicada — **fuera del alcance de la generación automática de este documento** (no push, no edición de PR desde aquí).

---

## Control de documento (rellenar al adoptar el checklist)

| Campo | Valor |
|-------|--------|
| Versión del documento | `draft-1.0` *(sugerido)* |
| Fecha de revisión | *(YYYY-MM-DD)* |
| Propietario (riesgo / operaciones) | *to be defined with stakeholders* |
| Próxima revisión programada | *to be defined with stakeholders* |

### Registro de cambios (opcional)

| Versión | Fecha | Cambio | Aprobado por |
|---------|--------|--------|--------------|
| *(ej.)* 1.1 | *(fecha)* | Ajuste límites intradía live | *(rol)* |

---

*Fin del checklist. Recordatorio: **commit local** según tu proceso; **sin** push ni cambios en GitHub salvo que lo ejecutes tú explícitamente.*
