# Informe final consolidado — Auditoría Institutional Radar (ATLAS PUSH + Code-Quant)

**Rama de trabajo documentada:** `variante/nueva`  
**HEAD de cierre (repositorio local, referencia de huella):** `27030be5bff5cdeed8affef26b53ba64d55e151f`  
**Rango de remediación codificada (Fases 1–4, docs Fase 5):** `6cc051355` … `27030be5b` (incluye commits intercalados; ver apéndice A).

---

## 1. Executive summary

| Campo | Valor |
|--------|--------|
| **Fecha de auditoría / línea base funcional (Radar MVP en repo)** | 2026-04-26 — commit `7d054d413` (dashboard institucional, puente PUSH–Quant, V4, SSE) |
| **Fecha de cierre de remediación (última evidencia + docs Fase 5)** | 2026-04-26 — `27030be5b` |
| **Alcance** | **Institutional Radar** en `atlas_adapter` (rutas `/ui`, `/radar/dashboard`, `/api/radar/*`); orquestación con **Atlas Code-Quant** `:8792` vía clave y HTTP; pruebas en `tests/test_radar_*.py` y evidencia viva 0.x–10.6 |
| **Hallazgos totales (categoría plan)** | **3** críticos (A1–A3), **3** medios (M1–M3), **5** menores (B1–B5) = **11** entradas de remediación **cerradas** en entrega |
| **Evidencia viva (0.x–10.6)** | **Completa** con **warnings** documentados (ver §4–5) |
| **Estado final (auditoría)** | **COMPLETO CON RESERVAS** — reservas operativas (health intermitente, `pytest` collect a nivel repo, paridad local/remoto, 10.3 resiliencia solo manual) no invalidan cierre lógico del Radar |
| **Recomendación (negocio / téc.)** | **APTO** para uso en **mismo entorno aprobado** (stack local/operador) y **seguimiento operacional** requerido antes de declarar SLO estricto de `health` 24/7. No equivale a “cero riesgo” de mercado/IC; ver scorecard global aparte. |

*Audiencia: stakeholders (tablas y cierre); técnicos (apéndices, comandos, SHAs).*

---

## 2. Remediation timeline (Fases 1–5)

> Fechas = hora de commit en el repo (`AuthorDate`), zona `-0400`.

| Fecha (ISO local commit) | Fase | Qué se cerró (plan) | Commit (SHA abrev) | Tipo de cambio principal |
|----------------------------|------|---------------------|----------------------|----------------------------|
| 2026-04-26 17:50 -0400 | **1** | **A1–A3** — modelos Pydantic (`radar_schemas.py`), `response_model` en API pública, logging, tests contrato Quant→PUSH | `6cc051355` | Código: `atlas_adapter/routes/radar_public.py`, `radar_schemas.py`, mapper/client; tests: `test_radar_quant_contract.py` |
| 2026-04-26 17:58 -0400 | **2** | **M1–M2** (M3 completado en Fase 3) — a11y CSS, `degradations_active` y superficie de degradación en UI | `907959080` | Código: `dashboard.html` / `dashboard.css` (Radar) |
| 2026-04-26 18:08 -0400 | **3** | **M3** + integración V4: **design tokens** compartidos (V4 + Radar) | `3196eeb2e` | Código: `atlas_adapter/static/shared/tokens.css`, enlace a tokens y assets |
| 2026-04-26 18:13 -0400 | **4** | **B1–B5** — jitter SSE, limpieza legacy, tests adicionales, `defer` en script, breadcrumb, assets versionados | `2d0d9bd19` | Código/estáticos: `static/radar/*`, F4: `test_startup_visual_connect.py`, `test_windows_camera_hints.py` |
| 2026-04-26 18:25 -0400 | **5a** | Evidencia viva 0.x–9.x (CLI, `curl`, `pytest` parcial) | `2ffca41bd` | **Solo** `reports/institutional_radar_fase5_evidencia_2026-04-26.md` |
| 2026-04-26 18:42 -0400 | **5b** | Validación 10.1–10.6 (navegador + Lighthouse) | `27030be5b` | **Solo** `reports/institutional_radar_fase5_bloques_10_2026-04-26.md` |

**Tests de radar añadidos/afirmados (suite parcial acordada):**  
`test_radar_quant_contract.py` (7 tests) + `test_radar_sse_contract.py` (3) + `test_radar_symbols_search_http.py` (2) + (F4) `test_startup_visual_connect.py` + `test_windows_camera_hints.py` → **14** seleccionados en batería de cierre, **14 passed / 1 skipped** (skip Windows-only según plataforma).

**Nota de alcance de tests:** `pytest tests --collect-only` reporta **188** test recolectados y **4 errores** de import en `tests/tools/test_atlas_agent_*.py` (`AgentConfig`); **fuera del cierre Radar**; ver §5 y checklist.

---

## 3. Findings closure matrix

| ID | Origen plan | Severidad (original) | Descripción breve | Acción / entrega | Estado | Evidencia (commit / test) |
|----|-------------|----------------------|-------------------|------------------|--------|----------------------------|
| A1 | Fase 1 | Crítico | Contrato HTTP tipado, respuestas homogéneas, sin `pressure` ambiguo | `Radar*Payload` + `response_model` en 12 JSON + stream tipado; rename documentado en tests | **CLOSED** | `6cc051355` — `radar_schemas.py`, `radar_public.py` |
| A2 | Fase 1 | Crítico | Observabilidad: logging y mapeo Quant | Logging en rutas, mapper, cliente HTTP Quant | **CLOSED** | `6cc051355` |
| A3 | Fase 1 | Crítico | Regresión contrato **Quant ↔ PUSH** | `test_radar_quant_contract.py` + ajuste de nombres/campos | **CLOSED** | `6cc051355` |
| M1 | Fase 2 / 3 | Medio | Accesibilidad y legibilidad UI (contraste, foco, estructura) | CSS/UX Radar + tokens | **CLOSED** | `907959080`, `3196eeb2e` |
| M2 | Fase 2 | Medio | `degradations_active` y lectura al operador | Sección y datos enlazados al summary / UI | **CLOSED** | `907959080` — validado Fase 5 10.2 + API |
| M3 | Fase 3 | Medio | Coherencia visual V4 + Radar (tokens) | `static/shared/tokens.css` y referencia desde V4/Radar | **CLOSED** | `3196eeb2e` |
| B1 | Fase 4 | Menor | SSE: jitter de reconexión, UX stream | Ajuste cliente SSE en `dashboard.js` | **CLOSED** | `2d0d9bd19` |
| B2 | Fase 4 | Menor | Limpieza de artefactos legacy (dashboard antiguo / backups) | Eliminación/retiro archivos heredados en la fase 4 (según entrega) | **CLOSED** | `2d0d9bd19` (mensaje fase 4) |
| B3 | Fase 4 | Menor | Carga de script: `defer`, orden de init | `defer` en carga de `dashboard.js` | **CLOSED** | `2d0d9bd19` |
| B4 | Fase 4 | Menor | Breadcrumb y navegación a V4 | Breadcrumb y enlaces a `/ui` | **CLOSED** | `2d0d9bd19` |
| B5 | Fase 4 | Menor | Pruebas y arranque visual (cámara / conectividad) | Nuevos tests y smoke startup | **CLOSED** | `2d0d9bd19` + tests F4 citados |

---

## 4. Technical validation summary (bloques 0.x–10.6)

| Tema | Resultado (sesión 2026-04-26) | Evidencia (documento) |
|------|--------------------------------|------------------------|
| **0.1** Git/paridad | **WARN** — rama con commits por delante de `origin/variante/nueva` (revalidar tras `push`) | `institutional_radar_fase5_evidencia_2026-04-26.md` |
| **0.2** Healths HTTP | **WARN** — `:8791/health` y `:8792/health` observados intermitentes; **rutas de producto** Radar **200** | Mismo + logs |
| **0.3** Runtime | **OK** — p. ej. Python 3.14, FastAPI 0.136, Uvicorn 0.45 | Mismo |
| **3.1** SSE 20 s | **OK** — `retry: 3000`, heartbeat, snapshot, eventos cámara/proveedor/degradación, envelope con `type`, `timestamp`, `symbol`, `source`, `sequence`, `data` | Mismo |
| **3.4** Latencia interna | **OK** — ~5 s entre heartbeats (alineado a `sse_heartbeat_sec`: 5) | Mismo |
| **4.6** Anti-caché | **OK (GET)** — `no-store` en HTML/assets/API summary; búsqueda con TTL acotado; **HEAD 405** en parte de rutas (usar GET en verificación) | Mismo |
| **6.1** Payload summary | **OK** — clasificación, scores, `provider_health_summary`, cámara, `transport`, `degradations_active` | Mismo |
| **7.2** Latencia E2E (ráfaga) | **OK** — dezenas de ms en summary/providers/search | Mismo |
| **9.1** Tests | **Parcial OK** (radar) / **WARN repo** (collect) | 14 passed (parcial), 4 errors collect tools |
| **10.1** V4→Radar | **OK** | `institutional_radar_fase5_bloques_10_2026-04-26.md` |
| **10.2** Degradado | **OK** | Mismo |
| **10.3** Parar Quant | **ACCEPTED / PEND** controlado — no automático; procedimiento en informe | Mismo |
| **10.4** Símbolo | **OK** (matiz: sesión vs query) | Mismo |
| **10.5** Teclado / a11y bás. | **OK (WARN: tab exhaustivo = manual en Chrome)** | Mismo |
| **10.6** Lighthouse (local) | **OK** — aprox. **~92% accesibility**, **~40% performance** (dashboard denso) | Mismo; JSON local opcional: `reports/temp_f5_lh_radar.json` |

**Contratos y rutas bajo `/api/radar`:** 12 `response_model` Pydantic en router stub + `GET /api/radar/stream` (SSE) — alineado con cierre A1.  
**Envelope SSE (JSON en `data:`):** canónico validado en fase 5.  
**Navegación V4 → Radar:** chip/landing y `/radar/dashboard` — validado 10.1.  
**Degradación visible operador:** sí (10.2 + `operable_with_degradation`).

---

## 5. Remaining warnings & mitigation plan

| # | Origen (bloque) | Descripción | Impacto prod. | Mitigación recomendada | Timeline / owner |
|---|-----------------|------------|---------------|------------------------|------------------|
| W0.1 | 0.1 | Rama local **no alineada** 1:1 con remoto (commits sin `push` o `fetch` atrasado) | **Bajo** para código; **Medio** si se audita un SHA fijo de despliegue | `git push` + etiqueta `radar-audit-2026-04-26` o PR de merge; re-ejecutar 0.1 en CI con SHA | **1 día** / **TBD repositorio** |
| W0.2 | 0.2 | Inestabilidad `/health` en 8791/8792 bajo carga o arranque | **Medio** (SLO, supervisión, falsos rojos) | `start_atlas_radar_stack.ps1` o systemd/PM2; health *ligero* dedicado; monitor sin bloquear worker | **1–2 sprints** / operador+DevOps **TBD** |
| W9.1 | 9.1 | 4 collection errors en `tests/tools/*` (import `AgentConfig`) | **Bajo** para Radar; **Medio** si CI gating global exige 0 errores de collect | Alinear import de `config` o excluir paquetes rotos de collect en CI; fix mínimo en módulo `config` o tests | **1 sprint** / **equipo core** TBD |
| W10.3 | 10.3 | Resiliencia “matar Quant” **no** validada en automat | **Bajo** hasta incidente; **alto** en *post-mortem* si no hay playbook | Ejecutar en ventana aislada el flujo de §10.3 en `fase5_bloques_10` | **1 sesión QA** / operador designado **TBD** |
| W10.5/head | 4.6/10.5 | Verificación con `curl -I` daba 405; foco teclado parcial | **Bajo** | Checklists: GET para cabeceras; a11y manual Chrome | **Ongoing** / operador |

---

## 6. Quality metrics (antes / después, alcance Radar)

> “Antes” = estado previo a la tanda fase 1 (referencia: ausencia de capa Pydantic dedicada y tests de contrato en apertura; línea `7d054d413` como inicio de producto, remediación desde fase 1).

| Métrica | Antes (referencia) | Después (cierre) |
|--------|---------------------|------------------|
| Respuestas JSON bajo `/api/radar` con `response_model` Pydantic dedicado | **0** (sin módulo `radar_schemas` unificado) | **12** + stream SSE bajo el mismo módulo de eventos y tests |
| Logging estructurado (rutas + mapeo + cliente Quant) | Mínimo / ad hoc | **3** módulos tocados y homogeneizados en fase 1 (ver commit `6cc051355`) |
| A11y Lighthouse (local, `headless`, categoría *accessibility*) | **N/D** | **~92%** (sesión documentada; *performance* ~**40%** producto de bundle/UI) |
| Casos de test *radar* (archivos `test_radar_*.py` + F4 acoplados) | pocos a ninguno formal | **14** ejecutados vía batería parcial (1 skip según SO) — ver §2 |
| Sistema de **design tokens** compartido V4/Radar | **0** (sin `static/shared/tokens.css` de producto) | **1** pipeline (`shared/tokens.css` + enlace) |
| Degradaciones visibles y honestas (UI + API) | Incompleto | **Sí** — `degradations_active`, regiones, copy operador, clasificación en summary |

*No confundir con el score global de Quant/IC: ver `atlas_quant_implementation_scorecard_latest.md` (dominio trading aparte de radar UI/API).*

---

## 7. Sign-off checklist

- [x] Hallazgos críticos (A1–A3) cerrados  
- [x] Hallazgos medios (M1–M3) cerrados  
- [x] Hallazgos menores (B1–B5) cerrados  
- [x] Evidencia viva 0.x–9.x y 10.1–10.2, 10.4–10.6 documentada; **10.3** aceptada como riesgo residual *controlado*  
- [x] Batería **parcial** radar+F4: **14 passed, 1 skipped** (en entorno venv local)  
- [ ] `pytest tests --collect-only` **0 errores** a nivel repositorio (**pendiente** por W9.1)  
- [x] Documentación: informes bajo `reports/institutional_radar_fase5_*.md` y este `ATLAS_RADAR_AUDIT_FINAL_2026-04-26.md`  
- [x] Warnings §5 aceptados o con plan de mitigación  
- [x] Lighthouse accesibilidad **> 90%** (sesión local)  
- [ ] Paridad **local / remoto** (push + SHA acordado) **confirmada por repositorio** (pendiente operación git)  
- [ ] “Tests passing en **CI**” (si aplica) — requiere reparar W9.1 o scope de job CI a `tests/test_radar*.py`  

**Firma:** espacio intencionado para *Product/Owner* — cierre lógico **bajo** checklist anterior con excepciones listadas (push remoto, CI, 10.3 en vivo).

---

## 8. Appendices

### A. Commits (orden cronológico, rango remediación principal)

| SHA (corto) | Mensaje |
|-------------|--------|
| `6cc051355` | feat(radar): fase 1 A1–A3 — schemas Pydantic, logging, test contrato Quant→PUSH |
| `907959080` | feat(radar): fase 2 M1–M2 — a11y CSS, degradations_active y UI |
| `3196eeb2e` | feat(ui): fase 3 — design tokens compartidos V4 + Radar |
| `2d0d9bd19` | chore(radar): fase 4 B1–B5 — SSE jitter, limpieza legacy, tests, defer, breadcrumb |
| `2ffca41bd` | docs(radar): cierre Fase 5 — evidencia viva Institutional Radar |
| `27030be5b` | docs(radar): Fase 5 — bloques 10.1–10.6 (validación navegador y Lighthouse) |

**Línea base de producto previa (contexto, no fase 1):** `7d054d413` — *feat(radar): dashboard institucional autonomo en tiempo real…*

### B. Reportes de evidencia (versionados)

- `reports/institutional_radar_fase5_evidencia_2026-04-26.md` — 0.1–9.1, métricas CLI, SSE, HTTP, 6.1, 7.2  
- `reports/institutional_radar_fase5_bloques_10_2026-04-26.md` — 10.1–10.6, Lighthouse, instrucción 10.3  

**Artefactos locales (no exigir en git, reproducibles):** `reports/tmp_f5_*.json|txt`, `reports/temp_f5_lh_radar.json`

### C. Comandos de validación reproducibles (ejemplo Windows / repo `C:\ATLAS_PUSH`)

```text
# Stack: ver ATLAS_RADAR_STACK.md y .\scripts\start_atlas_radar_stack.ps1

git rev-parse HEAD
git log -1 --oneline
curl -sS -m 8 http://127.0.0.1:8791/health
curl -sS -m 8 http://127.0.0.1:8792/health
curl -sS -D - -m 90 "http://127.0.0.1:8791/api/radar/dashboard/summary?symbol=SPY" -o NUL
curl -sN -m 21 "http://127.0.0.1:8791/api/radar/stream?symbol=SPY" -o reports/tmp_sse.txt

C:\ATLAS_PUSH\venv\Scripts\python.exe -m pytest tests\test_radar_quant_contract.py tests\test_radar_sse_contract.py tests\test_radar_symbols_search_http.py tests\test_startup_visual_connect.py tests\test_windows_camera_hints.py -q --tb=short
```

**Lighthouse (misma fórmula que cierre 10.6, si `npx` y Chrome headless OK):** ver bloque 10.6 en `fase5_bloques_10_*.md`.

### D. Contacto / seguimiento

- **Código y rutas:** `atlas_adapter/routes/radar_public.py`, `atlas_adapter/routes/radar_schemas.py`, `atlas_adapter/static/radar/`  
- **Evidencia y checklist:** el presente `reports/ATLAS_RADAR_AUDIT_FINAL_2026-04-26.md` + commits listados.  
- **Responsable final de producción (sign-off de negocio):** *TBD* — propietario repositorio ATLAS.  

---

*Fin del informe consolidado — Institutional Radar, ATLAS, 2026-04-26.*
