# Auditoría técnica reproducible — Atlas Code Quant + evaluación Insta360

- Repositorio: `C:\ATLAS_PUSH`
- Fecha de auditoría: `2026-04-11`
- Entorno auditado: API Quant local en `http://127.0.0.1:8795`
- Objetivo:
  - explicar con evidencia por qué Atlas Code Quant no está ejecutando trades de forma consistente,
  - separar bloqueos de señal, selector, ejecución, estado y visión,
  - validar si una cámara Insta360 aporta valor real para observación de mercado,
  - dejar un informe que otra IA pueda reproducir y auditar.

## 1. Método usado

### 1.1 Pruebas runtime

Se ejecutaron llamadas reales contra la API local:

```powershell
$h=@{'x-api-key'='atlas-quant-local'}
Invoke-RestMethod -Headers $h -Uri 'http://127.0.0.1:8795/api/v2/quant/operation/loop/status'
Invoke-RestMethod -Headers $h -Uri 'http://127.0.0.1:8795/api/v2/quant/scanner/status'
Invoke-RestMethod -Headers $h -Uri 'http://127.0.0.1:8795/api/v2/quant/scanner/report?activity_limit=5'
Invoke-RestMethod -Method Post -Headers (@{'x-api-key'='atlas-quant-local';'Content-Type'='application/json'}) -Body '{"action":"run_once"}' -Uri 'http://127.0.0.1:8795/api/v2/quant/scanner/control'
Invoke-RestMethod -Method Post -Headers (@{'x-api-key'='atlas-quant-local';'Content-Type'='application/json'}) -Body $selectorBody -Uri 'http://127.0.0.1:8795/api/v2/quant/selector/proposal'
Invoke-RestMethod -Method Post -Headers (@{'x-api-key'='atlas-quant-local';'Content-Type'='application/json'}) -Body $previewBody -Uri 'http://127.0.0.1:8795/api/v2/quant/operation/test-cycle'
Invoke-RestMethod -Headers $h -Uri 'http://127.0.0.1:8795/api/v2/quant/dashboard/overview?account_scope=paper'
Invoke-RestMethod -Headers $h -Uri 'http://127.0.0.1:8795/api/v2/quant/canonical/snapshot?account_scope=paper'
```

### 1.2 Inspección de código

Se inspeccionaron, entre otros, estos puntos críticos:

- `atlas_code_quant/api/main.py`
- `atlas_code_quant/operations/operation_center.py`
- `atlas_code_quant/selector/strategy_selector.py`
- `atlas_code_quant/operations/sensor_vision.py`
- `atlas_code_quant/vision/insta360_capture.py`
- `atlas_code_quant/vision/visual_pipeline.py`
- `atlas_code_quant/vision/chart_ocr.py`
- `atlas_code_quant/hardware/camera_interface.py`
- `atlas_code_quant/journal/service.py`

### 1.3 Evidencia de persistencia y aprendizaje

Se consultó SQLite real:

```powershell
& '.\venv\Scripts\python.exe' -c "import sqlite3; ..."
```

Base auditada:

- `atlas_code_quant/data/journal/trading_journal.sqlite3`

No se usó `atlas_code_quant/data/trading_journal.db` porque estaba vacío en auditorías previas.

### 1.4 Fuentes externas para Insta360

Se usaron solo fuentes oficiales de Insta360 y una fuente técnica adicional para el problema de filmar pantallas:

- X4 Live Stream Tutorial:
  - https://onlinemanual.insta360.com/x4/en-us/camera/appuse/live
- X4 Webcam Live Streaming Tutorial Using OBS:
  - https://onlinemanual.insta360.com/x4/en-us/camera/appuse/obs
- X4 File Transfer:
  - https://onlinemanual.insta360.com/x4/en-us/camera/appuse/filetransfer
- Insta360 Studio download/install:
  - https://onlinemanual.insta360.com/studio/en-us/operation-guide/installation-and-update/download-and-installation-operations
- X4 User Manual PDF:
  - https://res.insta360.com/static/70093bf5f9bb8eb319b3b60e054ffb91/X4_UserManual_EN.pdf
- Artículo técnico sobre moiré/aliasing al filmar pantallas:
  - https://www.provideocoalition.com/tip-stop-down-to-erase-moire/

## 2. Resumen ejecutivo

### Diagnóstico principal verificado

Atlas Code Quant no está “sin operar” por una sola causa. Se verificaron al menos cinco bloqueos reales:

1. El loop autónomo está apagado por configuración de arranque, no por fallo accidental.
2. El selector estaba proponiendo estrategias de opciones aun con `option_buying_power = 0`, causando bloqueo determinista aguas abajo. Esto fue corregido.
3. Una oportunidad real del escáner (`AAOI`) siguió el flujo completo y quedó bloqueada por tres gates concretos: símbolo ya abierto, `adverse drift` excesivo y `spread` excesivo.
4. La reconciliación entre fuente canónica Tradier y simuladores locales está fallando de forma severa, lo que degrada o bloquea partes del ciclo.
5. El journal histórico usado para aprendizaje y varias métricas presenta evidencia fuerte de contaminación/sesgo estructural, por lo que no debe tratarse hoy como “aprendizaje fiable”.

### Conclusión operativa

El problema dominante no es falta de ideas del escáner. El escáner sí produce candidatos. El problema es una combinación de:

- loop autónomo no arrancado,
- reglas de entrada estrictas bloqueando el momento de ejecución,
- desalineación grave entre estado canónico y estado interno,
- calidad dudosa del histórico usado para aprendizaje.

## 3. Hallazgos verificados

### 3.1 El loop autónomo no estaba ejecutándose

Prueba runtime:

```json
{
  "running": false,
  "task_alive": false,
  "configured_auton_mode": "paper_autonomous",
  "inactive_reasons": [
    "lightweight_startup_enabled",
    "scanner_auto_start_disabled"
  ]
}
```

Rutas/código relacionados:

- `atlas_code_quant/api/main.py:693`
- `atlas_code_quant/api/main.py:799`
- `atlas_code_quant/api/main.py:2264`
- `atlas_code_quant/config/settings.py:210`
- `atlas_code_quant/config/settings.py:278`
- `atlas_code_quant/config/settings.py:279`

Interpretación:

- `paper_autonomous` no implica por sí mismo que el loop esté corriendo.
- En este entorno, el arranque lightweight deja el sistema listo para servir UI y endpoints, pero no inicia el loop autónomo.

### 3.2 El escáner sí genera oportunidades reales

Prueba runtime:

- `scanner/run_once` completó un ciclo.
- Resultado observado:
  - `accepted = 8`
  - `rejected = 92`
  - `last_cycle_ms = 62932.66`

Mejor candidato observado:

```json
{
  "symbol": "AAOI",
  "timeframe": "15m",
  "direction": "alcista",
  "selection_score": 83.17,
  "signal_strength_pct": 93.0,
  "local_win_rate_pct": 56.41,
  "local_profit_factor": 2.77,
  "local_sample": 117,
  "predicted_move_pct": 2.31,
  "has_options": true,
  "higher_tf": "1h",
  "higher_dir": "alcista"
}
```

Conclusión:

- No está fallando el sistema por ausencia de escaneo o por falta de candidatos.

### 3.3 Bug real corregido: probabilidad llamaba con un argumento no soportado

Se reprodujo una advertencia runtime al pedir propuesta del selector:

- `get_winning_probability() got an unexpected keyword argument 'account_id'`

Causa raíz en código:

- `atlas_code_quant/selector/strategy_selector.py` llamaba `get_winning_probability(..., account_id=account_id)`.
- `atlas_code_quant/backtesting/winning_probability.py` no acepta ese argumento.

Corrección aplicada:

- se eliminó el argumento no soportado.

Archivos:

- `atlas_code_quant/selector/strategy_selector.py`
- `atlas_code_quant/tests/test_strategy_selector_options_governance.py`

### 3.4 Bug real corregido: selector proponía opciones con buying power de opciones en cero

Prueba runtime previa:

- con `AAOI`, el selector proponía una estructura de opciones definida.
- el preview posterior caía por:
  - `Option buying power es 0. La apertura de opciones queda bloqueada.`

Corrección aplicada:

- si la mejor alternativa no-equity requiere buying power de opciones y `option_buying_power <= 0`, el selector cae a una alternativa `equity_*` si existe.

Prueba runtime posterior:

```json
{
  "selected_strategy": "equity_long",
  "selector_warnings": [
    "Option buying power es 0; el selector prioriza una alternativa equity para evitar bloqueo operativo."
  ]
}
```

Archivos:

- `atlas_code_quant/selector/strategy_selector.py:1087`
- `atlas_code_quant/tests/test_strategy_selector_options_governance.py`

### 3.5 Flujo end-to-end real: `AAOI` llegó a preview y fue bloqueado por tres gates concretos

Prueba runtime final tras correcciones:

```json
{
  "decision": "blocked",
  "allowed": false,
  "reasons": [
    "Open-symbol guard blocked re-entry for AAOI.",
    "Entry validation blocked preview because adverse drift is 0.93% (> 0.25%).",
    "Entry validation blocked preview because spread is 0.33% (> 0.18%)."
  ]
}
```

Código relacionado:

- `atlas_code_quant/operations/operation_center.py:1328`
- `atlas_code_quant/operations/operation_center.py:1382`
- `atlas_code_quant/operations/operation_center.py:1392`
- `atlas_code_quant/operations/operation_center.py:1485`
- `atlas_code_quant/operations/operation_center.py:1560`

Conclusión:

- Para esta oportunidad concreta, la no-ejecución es explicable y reproducible.
- No fue un fallo silencioso.

### 3.6 El símbolo `AAOI` ya aparece abierto en journal

Consulta SQLite real:

```json
[["AAOI", "closed", 238], ["AAOI", "open", 1]]
```

Esto es consistente con el bloqueo por `Open-symbol guard`.

### 3.7 La reconciliación está degradada de forma severa

Prueba runtime repetida:

```json
{
  "state": "failed",
  "max_equity_gap_usd": 86331.0343,
  "max_positions_gap": 493
}
```

Diferencias observadas:

- Canonical Tradier:
  - `total_equity = 96331.0343`
  - `positions = 493`
- Simuladores internos:
  - `paper_local.open_positions = 0`
  - `atlas_internal.open_positions = 0`

Conclusión:

- La fuente de verdad del estado operativo no está unificada.
- Esto degrada decisiones, dashboards y gates.

### 3.8 Las salidas automáticas de opciones no están cubiertas de forma homogénea

Evidencia en código/eventos:

- `atlas_code_quant/api/main.py:1892`
- eventos `exit.manual_review_required`

Conclusión:

- el motor sí tiene pass de salida, pero parte de las estructuras de opciones terminan en revisión manual en vez de cierre automático homogéneo.

### 3.9 El dashboard histórico y el aprendizaje están contaminados por un journal dudoso

Hallazgo cuantificado:

```json
{
  "closed_stats": {
    "n_all": 77484,
    "n_le_5s": 1372,
    "n_negative_entry": 14873,
    "n_negative_exit": 14770
  },
  "top_days": [
    {"day": "2026-03-30", "n": 59093},
    {"day": "2026-03-29", "n": 18035}
  ]
}
```

Métricas por estrategia observadas:

```json
[
  {"strategy_type": "equity_short", "n": 13256, "win_rate_pct": 100.0, "avg_pnl": 370.12},
  {"strategy_type": "equity_long",  "n": 55463, "win_rate_pct": 0.0,   "avg_pnl": -617.3}
]
```

Muestra real de `equity_short`:

```json
{
  "symbol": "ACHC",
  "strategy_type": "equity_short",
  "realized_pnl": 91.73,
  "entry_price": -45.91,
  "exit_price": -45.82
}
```

Conclusión:

- Hay evidencia fuerte de registros anómalos o sintéticos en la base.
- No es aceptable usar este histórico tal cual para afirmar aprendizaje robusto o edge.
- Cualquier learning/ranking basado en este journal debe considerarse contaminado hasta limpieza o reetiquetado.

## 4. Lifecycle detallado de una operación

### 4.1 Flujo operativo observado

1. Escáner genera candidatos.
2. Selector propone estructura, sizing, chart plan y camera plan.
3. `OperationCenter.evaluate_candidate(...)` aplica gates:
   - scope/paper-live,
   - kill switch y failsafe,
   - presencia de operador,
   - integridad de pantallas,
   - PDT,
   - open-symbol guard,
   - reconciliación,
   - validación de entrada (drift/spread),
   - capital guard,
   - market context gate,
   - visual entry gate,
   - execution preflight,
   - executor.
4. Si pasa, el ejecutor emite preview o submit.
5. El auto-cycle luego hace exit pass y journal sync.

### 4.2 Referencias principales

- auto-cycle:
  - `atlas_code_quant/api/main.py:1809`
  - `atlas_code_quant/api/main.py:1996`
- status del loop:
  - `atlas_code_quant/api/main.py:2264`
- selector:
  - `atlas_code_quant/selector/strategy_selector.py:1087`
- centro operacional:
  - `atlas_code_quant/operations/operation_center.py:1328`

## 5. Causas raíz priorizadas

### Prioridad 1 — El loop autónomo no está corriendo

Probabilidad: muy alta  
Impacto: crítico

Evidencia:

- `running = false`
- `task_alive = false`
- `inactive_reasons = ["lightweight_startup_enabled","scanner_auto_start_disabled"]`

### Prioridad 2 — El estado canónico y el estado interno están reconciliando mal

Probabilidad: muy alta  
Impacto: crítico

Evidencia:

- `reconciliation.state = failed`
- `positions canonical = 493` vs simuladores `0`

### Prioridad 3 — Las reglas de entrada están bloqueando el timing real

Probabilidad: alta  
Impacto: alto

Evidencia en `AAOI`:

- `Open-symbol guard`
- `adverse drift 0.93% > 0.25%`
- `spread 0.33% > 0.18%`

### Prioridad 4 — El aprendizaje está sesgado por journal contaminado

Probabilidad: alta  
Impacto: alto

Evidencia:

- `equity_short` con `100%` de win rate
- `equity_long` con `0%`
- miles de precios negativos y cierres en ráfaga

### Prioridad 5 — Las salidas de opciones no están plenamente automatizadas

Probabilidad: media-alta  
Impacto: alto

Evidencia:

- `exit.manual_review_required`

## 6. Hipótesis descartadas

### 6.1 “El escáner no encuentra oportunidades”

Descartada.

Evidencia:

- un ciclo real aceptó `8` candidatos.

### 6.2 “La causa principal es falta de capital”

Descartada como causa principal general.

Evidencia en `AAOI` preview:

- `available_buying_power = 13317.04`
- `estimated_open_notional = 1064.0`
- `capital_guard.blocked = false`

### 6.3 “Insta360 ya está operativa en este entorno”

Descartada.

Evidencia local:

```json
{
  "provider": "off",
  "checks": [
    {
      "name": "insta360",
      "source": "none",
      "reachable": false
    }
  ]
}
```

## 7. Cambios de código aplicados en esta auditoría

### 7.1 Selector: fallback a equity cuando `option_buying_power = 0`

Archivo:

- `atlas_code_quant/selector/strategy_selector.py`

Objetivo:

- evitar que el selector emita una estructura de opciones que el propio motor bloqueará inmediatamente por buying power.

### 7.2 Selector: eliminación de parámetro inválido en `get_winning_probability`

Archivo:

- `atlas_code_quant/selector/strategy_selector.py`

Objetivo:

- corregir error runtime reproducible en la construcción de la probabilidad.

### 7.3 Dashboard overview: rebase de equity curve al equity canónico actual

Archivo:

- `atlas_code_quant/api/main.py`

Objetivo:

- evitar curvas absurdas ancladas a un baseline histórico de 10k que no corresponde al equity real actual.

### 7.4 Status del loop: exposición de `inactive_reasons`

Archivo:

- `atlas_code_quant/api/main.py`

Objetivo:

- explicar por API por qué el loop no está corriendo.

### 7.5 Contrato HTTP: `operation/test-cycle` acepta `order_seed` como alias de `order`

Archivo:

- `atlas_code_quant/api/schemas.py:303`

Objetivo:

- cerrar el desajuste entre lo que entrega el selector (`order_seed`) y lo que espera el endpoint (`order`).

Estado:

- verificado con prueba unitaria
- verificado en runtime tras reiniciar el servidor

## 8. Tests creados/actualizados

### Ejecutados

```powershell
.\venv\Scripts\python.exe -m pytest `
  atlas_code_quant/tests/test_api_schemas.py `
  atlas_code_quant/tests/test_strategy_selector_options_governance.py `
  atlas_code_quant/tests/test_dashboard_overview_helpers.py `
  atlas_code_quant/tests/test_auto_cycle_startup.py `
  atlas_code_quant/tests/test_journal_chart_data.py -q
```

Resultado observado:

- `23 passed`

### Nuevos o modificados

- `atlas_code_quant/tests/test_api_schemas.py`
- `atlas_code_quant/tests/test_dashboard_overview_helpers.py`
- `atlas_code_quant/tests/test_auto_cycle_startup.py`
- `atlas_code_quant/tests/test_journal_chart_data.py`
- `atlas_code_quant/tests/test_strategy_selector_options_governance.py`

## 9. Evaluación realista de Insta360

## 9.1 Capacidades confirmadas por documentación oficial

### Confirmado

- X4 soporta live streaming.
- X4 puede usarse como webcam.
- El modo webcam para ordenador está documentado con OBS.
- El tutorial oficial indica USB + `Webcam Mode` y configuración en OBS.
- El tutorial oficial especifica resolución `2880x1440` y `30 fps` para la captura en OBS.
- La transferencia de archivos al PC está soportada por USB Drive Mode.
- Insta360 Studio es la herramienta oficial de edición/exportación.
- El manual oficial lista modos útiles para observación/registro:
  - video,
  - timelapse,
  - loop recording,
  - photo,
  - single-lens video hasta `4K60`.

Base documental:

- Live stream app:
  - https://onlinemanual.insta360.com/x4/en-us/camera/appuse/live
- Webcam/OBS:
  - https://onlinemanual.insta360.com/x4/en-us/camera/appuse/obs
- File transfer:
  - https://onlinemanual.insta360.com/x4/en-us/camera/appuse/filetransfer
- Studio:
  - https://onlinemanual.insta360.com/studio/en-us/operation-guide/installation-and-update/download-and-installation-operations
- User manual PDF:
  - https://res.insta360.com/static/70093bf5f9bb8eb319b3b60e054ffb91/X4_UserManual_EN.pdf

## 9.2 Estado real en este entorno

### Confirmado localmente

- `SensorVisionService.diagnose()` reporta:
  - `provider = off`
  - `insta360.source = none`
  - `insta360.reachable = false`
- No hay evidencia de cámara física conectada ni stream RTMP configurado.

### Conclusión

- En este entorno, Insta360 no está hoy integrada de forma operativa.
- No se puede afirmar que Atlas “vea” el mercado mediante Insta360 en esta máquina.

## 9.3 Valor real para trading visual

### Confirmado

El repo sí tiene plumbing para usar una fuente de video externa:

- `atlas_code_quant/operations/sensor_vision.py`
- `atlas_code_quant/vision/insta360_capture.py`
- `atlas_code_quant/vision/visual_pipeline.py`
- `atlas_code_quant/hardware/camera_interface.py`

### Hipótesis razonable, no confirmada localmente

Una Insta360 puede servir como instrumento auxiliar para:

- registrar el entorno físico del setup,
- grabar sesiones para post-mortem,
- documentar contexto visual del operador,
- capturar una visión panorámica del puesto de trabajo.

### Limitación importante

No hay evidencia suficiente en este entorno para defenderla como fuente primaria de lectura de velas mejor que la captura nativa de pantalla.

Razones:

1. La propia capa visual del repo hoy no hace detección de velas robusta; hace OCR + color + heurísticas.
2. Filmar monitores introduce riesgo de aliasing/moiré y artefactos ópticos al capturar pixeles de pantalla con sensor de cámara.
3. Incluso si la cámara funcionara, el pipeline actual no demuestra edge estadístico basado en imagen.

Fuente técnica sobre filmar pantallas:

- ProVideo Coalition explica que grabar pantallas con cámaras digitales puede producir aliasing y moiré por la interacción entre pixeles de pantalla y fotositos del sensor:
  - https://www.provideocoalition.com/tip-stop-down-to-erase-moire/

### Recomendación

- Para análisis cuantitativo reproducible de velas, la fuente primaria debería ser captura nativa de pantalla o datos OHLC directos.
- Insta360 puede evaluarse como fuente secundaria de documentación/contexto, no como fuente principal de edge, hasta demostrar lo contrario.

## 10. Análisis de patrones de velas y ciclos repetitivos

## 10.1 Qué se buscó

Se intentó evaluar si el sistema tiene hoy evidencia verificable de edge en:

- estructura de velas,
- impulso/retroceso,
- repetición por horario,
- sesgo por estrategia,
- gating por contexto.

## 10.2 Qué se pudo verificar

### Verificable hoy

- El escáner usa criterios reproducibles y cuantificados:
  - `selection_score`
  - `local_win_rate_pct`
  - `local_profit_factor`
  - confirmación temporalidad superior
- El selector y el market context generan una tesis operativa reproducible.

### No verificable hoy como edge visual robusto

- No existe en esta auditoría un dataset limpio y etiquetado de patrones de velas + imagen + outcome.
- `ChartOCR` no detecta patrones de velas de forma estadística; usa:
  - OCR de textos,
  - color dominante,
  - keywords heurísticas.

Referencias de código:

- `atlas_code_quant/vision/chart_ocr.py:57`
- `atlas_code_quant/vision/chart_ocr.py:105`
- `atlas_code_quant/vision/visual_pipeline.py:57`

### Conclusión

- No hay base para afirmar una ventaja verificable por “intuición visual” o por reconocimiento de velas via cámara en el sistema actual.
- Esa hipótesis queda no confirmada.

## 10.3 Qué sesgos/ciclos sí aparecen hoy

Hay un patrón fuerte en el journal, pero no es interpretable como edge porque la base está contaminada:

- `equity_short` siempre gana
- `equity_long` siempre pierde

Eso no se interpreta como ventaja; se interpreta como posible problema de datos o de registro.

## 10.4 Línea base y descarte

Línea base razonable:

- usar solo OHLC limpio + timestamps + costes + slippage + tamaño + contexto,
- comparar reglas simples por sesión/hora/estructura,
- exigir muestra limpia y OOS.

Estado actual:

- el repositorio no demuestra todavía esa validación end-to-end para patrones visuales.

## 11. Limitaciones abiertas

1. No se conectó una Insta360 física en este entorno.
2. No se probó un flujo real de video RTMP desde Insta360 hacia el pipeline.
3. No se ejecutó live trading; solo paper/runtime local.
4. El journal histórico presenta contaminación suficiente como para invalidar conclusiones fuertes de aprendizaje.
5. No se corrigió aún la causa raíz de la contaminación del journal; solo se documentó y se evitó sobreafirmar resultados.
6. La automatización de salidas en opciones sigue requiriendo revisión adicional.

## 12. Riesgos antes de pasar de paper a real

1. Reconciliación no saludable entre canónico y simuladores.
2. Aprendizaje posiblemente entrenado sobre datos defectuosos.
3. Visual pipeline sin validación estadística de edge.
4. Contractos entre módulos todavía frágiles si no se despliega siempre el mismo código que el repo.
5. Loop autónomo no arrancado por defecto en este perfil de startup.
6. Cierres de opciones no homogéneamente automatizados.

## 13. Recomendaciones siguientes

### Corto plazo

1. Corregir la causa raíz que generó los registros anómalos del journal de `2026-03-29` y `2026-03-30`.
2. Añadir una capa de “data quality quarantine” para excluir registros evidentemente defectuosos de:
   - learning,
   - scorecards,
   - equity curve histórica,
   - pattern mining.
3. Mantener el selector en fallback equity cuando `option_buying_power = 0`.
4. Decidir explícitamente si el entorno lightweight debe o no iniciar scanner/loop.

### Medio plazo

1. Unificar la fuente de verdad de posiciones abiertas.
2. Instrumentar por qué una oportunidad se bloquea en cada gate con contadores persistentes.
3. Crear dataset limpio OHLC + screenshot nativo + decisión + outcome.
4. Evaluar Insta360 solo como capa auxiliar comparándola contra screen capture nativo.

### Experimento controlado para Insta360

1. Conectar la X4 por USB y verificar `Webcam Mode`.
2. Configurar OBS a `2880x1440 @ 30fps` según tutorial oficial.
3. En paralelo, capturar la misma escena con screen capture nativo.
4. Extraer frames sincronizados.
5. Medir:
   - nitidez,
   - OCR confidence,
   - latencia,
   - estabilidad del patrón,
   - error de lectura de precios,
   - consistencia del sesgo visual.
6. Aceptar Insta360 solo si supera un umbral predefinido respecto a screen capture.

## 14. Estado final de esta auditoría

### Confirmado

- El sistema sí escanea.
- El selector ya no se rompe por `account_id`.
- El selector ya no prioriza opciones inviables cuando `option_buying_power = 0`.
- La oportunidad `AAOI` se sigue hasta preview y se bloquea por razones concretas, no silenciosas.
- `operation/test-cycle` ya puede aceptar `order_seed` en el código auditado.
- Insta360 no está operativa en este entorno.
- El aprendizaje no puede considerarse fiable con el journal actual.

### No confirmado

- Edge real basado en patrones de velas via cámara.
- Utilidad superior de Insta360 frente a captura nativa.
- Calidad estadística del aprendizaje hasta limpiar la base histórica.
