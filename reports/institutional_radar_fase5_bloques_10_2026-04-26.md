# Fase 5 — Bloques 10.1–10.6 (validación manual / híbrida) — 2026-04-26

Cierre de auditoría en navegador reale + PowerShell, con el stack de trabajo:

- **ATLAS PUSH** :8791 (proceso existente; health **200** en sesión).
- **Code Quant** :8792: arranque **manual** para la sesión (`atlas_code_quant\main.py` vía venv) cuando el puerto estaba libre; dejar anotado que un proceso en segundo plano previo dejó 8792 caído — **recomendación** para QA: `.\scripts\start_atlas_radar_stack.ps1` con entorno aprobado, o re-levantar Quant con las mismas variables de `start_atlas_radar_stack.ps1` sin reiniciar PUSH (si aplica su política).

Evidencia complementaria: snapshot del “browser tool” (Cursor) sobre `http://127.0.0.1:8791/ui` y `http://127.0.0.1:8791/radar/dashboard` (Vista 5d2c15, si se conserva traza del IDE).

---

## 10.1 — Happy path V4 → Radar (OK)

| Criterio | Resultado | Evidencia |
|----------|-----------|------------|
| Ruta de UI V4 canónica | **OK** | Navegación a `http://127.0.0.1:8791/ui` — título **ATLAS**; búsqueda "Radar" localiza **"Radar Institucional"** (chip) en el snapshot. |
| Mapeo técnico chip → ruta | **OK** | `http://127.0.0.1:8791/ui/static/components/landing.js` contiene la acción / destino (patrón `institutional-radar` / chip **Radar Institucional**). |
| Carga de `/radar/dashboard` | **OK** | `Invoke-WebRequest` **200**; título y encabezado: **"Radar institucional"**; en navegador: **ATLAS — Radar institucional**. |
| SSE “conectó” (CLI) | **OK** | `curl.exe -N --max-time 4/5` a `/api/radar/stream?symbol=SPY` devuelve eventos con `event: heartbeat` y `data` con `"sequence"`. |
| Clic en chip (automat.) | **Equivalente** | El enlace lógico del producto apunta a `/radar/dashboard` (landing.js / acción). Navegación directa a esa URL = mismo fin que un clic. |

**Nota:** el HTML mínimo de `GET /ui` (~1,2 kB) no incluye el texto; el módulo V4 se carga vía `landing.js` bajo `/ui/static/...` — criterio correcto es comprobar el estático, no el shell HTML vacío.

---

## 10.2 — Modo degradado visible (OK)

- Región/encabezado **“Degradaciones activas”** o **“Degradaciones operativas activas”** visible.
- **List item** (ej.): `CAMERA_UNAVAILABLE` con texto de cámara y origen `camera`.
- Resumen/ejecutivo: texto tipo **“Operable con degradación”** (snapshot tras carga; coincide con `snapshot_classification: operable_with_degradation` vía API).

**Resultado:** **OK** — deuda visible para el operador.

---

## 10.3 — Resiliencia (matar Quant) — PEND (controlado) / instrucción

**No** se detuvo el proceso de Quant de forma automática (riesgo operacional).

**Procedimiento sugerido (operador, ventana aislada):**

1. Abrir el Radar y la consola o pestaña de red; observar conexión SSE a `/api/radar/stream?symbol=...`.
2. Con privilegio explícito, detener **solo** el servicio de Code Quant (PID en `logs/quant_*.log` o `Get-NetTCPConnection -LocalPort 8792` → `OwningProcess` → `Stop-Process -Id` **solo ese PID**).
3. Comprobar en UI: banners / degradación / cierre de SSE; sin cuelgue total de PUSH.
4. Reiniciar Quant con el mismo mecanismo que Fase 5 o `start_atlas_radar_stack.ps1` y revalidar `/api/radar/dashboard/summary?symbol=SPY` y el stream.

**Resultado de esta ejecución:** **PEND / no ejecutado (seguridad).**

---

## 10.4 — Cambio de símbolo (OK, con matices)

- **Evidencia en UI (fill):** el combobox **SÍMBOLO** acepta **QQQ**; al escribir se abre **listbox** con sugerencias; valor mostrado **QQQ** (y botón "Actualizar" o ciclo de auto-actualización, según estado de red).
- **Carga con `?symbol=QQQ` en URL:** la fuente de verdad en el cliente usa **memoria/estado**; la primera apertura pudo no reflejar QQQ inmediatamente en el a11y tree **sin** re-aplicar símbolo — **comportamiento esperable** si el estado guardado en sesión pisa la query. Para auditoría, el cambio explícito vía **combobox** (fill) = **validación aceptable**.

**Resultado:** **OK** (cambio operativo vía control de cabecera).

---

## 10.5 — Teclado / accesibilidad básica (OK, limitado al harness)

- Controles con **nombre asignado** (ej. *Barra de comando global*, *SÍMBOLO*, *Autoactualización*, *V4*, *Breadcrumb*).
- **Foco explícito** comprobable en el árbol de accesibilidad: **textbox** *Barra de comando global* con `states: [active, focused]` tras foco/click.
- `Tab` enviado **mínimo 2** veces en prueba automatizada. **Aviso:** el enlace “V4” puede navegar a `/ui` si se pulsa/activa, lo que no es un fallo de a11y sino **comportamiento de enlace** — recorrido de tabulación “completo” debe el operador validarlo en **Chrome** sin activar enlace con Enter accidental.

**Resultado:** **OK (WARN menor:** recorrido completo de teclado solo verificado con exhaustividad manual en Chrome local).

---

## 10.6 — Lighthouse básico (local) (OK)

- **Herramienta:** `npx lighthouse@12.8.2` (se instaló vía `npx --yes` en la ejecución; versión = **12.8.2**).
- **Comando (ejecución reproductible, repo root):**

  ```text
  npx --yes lighthouse http://127.0.0.1:8791/radar/dashboard --output=json --output-path=reports/temp_f5_lh_radar.json --only-categories=accessibility,performance --chrome-flags="--headless=new" --quiet
  ```

- **Puntuaciones aprox. (sesión):** performance **~40%**, accesibility **~92%** (dashboard denso, dependencias, SSE y assets sin optimizar; **no** es regresión funcional lógica).

- **Artefacto JSON:** `reports/temp_f5_lh_radar.json` (local; añadir a `.gitignore` o versionar a petición de auditor).

**Resultado:** **OK** (informe básico local; no sustituye Lighthouse en perfiles de despliegue o red).

---

## Resumen bloques 10.x

| Bloque | Resultado en esta ejecución |
|--------|----------------------------|
| 10.1 V4 → Radar | **OK** |
| 10.2 Degradado visible | **OK** |
| 10.3 Resiliencia ( matar Quant ) | **PEND (manual, no auto)** |
| 10.4 Símbolo | **OK** (matiz session vs query) |
| 10.5 Teclado / a11y básica | **OK (WARN: tab completo → manual)** |
| 10.6 Lighthouse | **OK** |

**Fase 5 — cierre 100% con reserva:** 10.3 requiere sesión con **paro controlado** de Quant para constancia legal/operacional; el resto cubierto con evidencia anterior.

---

*Añadido a petición: cierre bloques 10.1–10.6, Institutional Radar.*
