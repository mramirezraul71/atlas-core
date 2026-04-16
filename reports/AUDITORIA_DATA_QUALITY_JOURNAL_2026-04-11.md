# Auditoría Data Quality — Journal Atlas Code Quant

**Fecha:** 2026-04-11  
**Repo:** `C:\ATLAS_PUSH`  
**Objetivo:** cortar la propagación de datos contaminados del journal hacia aprendizaje adaptativo y analítica, dejando un gate reproducible y un scorecard de calidad verificable.

## 1. Hallazgos verificados

### 1.1 Journal completo (`trading_journal.sqlite3`, cierres)

- `total_closed_count`: `77,484`
- `negative_entry_price_count`: `14,873`
- `negative_exit_price_count`: `14,770`
- `short_duration_count` (`<= 5s`): `1,372`
- `row_level_clean_count`: `61,475`
- `trusted_closed_count`: `1`
- `score_pct`: `2.24`
- `status`: `critical`
- `learning_allowed`: `false`

### 1.2 Concentración anómala por día

- `2026-03-30`: `47,632` cierres válidos a nivel fila tras excluir negativos/duración imposible, `77.48%` del subconjunto saneado
- `2026-03-29`: `13,842` cierres, `22.52%`

Estas dos fechas quedan marcadas como outliers de concentración y explican por qué el dataset no es fiable para aprendizaje.

### 1.3 Ventana real usada por el aprendizaje adaptativo

`AdaptiveLearningService` opera sobre los `latest 2000 closed trades` tras el corte temporal del propio servicio.

Verificación real tras el cambio:

- `raw_sample_count`: `2000`
- `row_level_clean_count`: `1198`
- `trusted_closed_count`: `1`
- `score_pct`: `0.0`
- `learning_allowed`: `false`
- `blocked_reasons`:
  - `negative_entry_price_ratio`
  - `negative_exit_price_ratio`
  - `outlier_close_day_concentration`
  - `quality_score_below_threshold`

Conclusión verificada: el aprendizaje adaptativo estaba expuesto a datos no fiables y ahora queda bloqueado automáticamente.

## 2. Causa técnica observada

El journal contaminado estaba entrando directamente en:

- `atlas_code_quant/learning/adaptive_policy.py`
  - `_compute_snapshot()` leía cierres cerrados sin gate de calidad.
- `atlas_code_quant/journal/service.py`
  - `_process_closed_entry_payloads()` refrescaba aprendizaje tras cada sincronización de cierres.
- `atlas_code_quant/journal/service.py`
  - `chart_data()` construía la curva con cierres crudos, sin excluir filas objetivamente inválidas.

Eso permitía que precios negativos, duraciones imposibles y ráfagas masivas de cierres sesgaran tanto la capa de aprendizaje como la analítica del dashboard.

## 3. Cambios aplicados

### 3.1 Nuevo módulo de calidad del journal

Archivo nuevo: `atlas_code_quant/learning/journal_data_quality.py`

Implementa:

- `row_quality_flags(row)`
  - detecta `negative_entry_price`, `negative_exit_price`, `invalid_realized_pnl`, `too_short_duration`, `negative_duration`
- `build_journal_quality_scorecard(rows)`
  - calcula score, ratios, días outlier y razones de bloqueo
- `filter_closed_entries(...)`
  - reutilizable para cuarentena de filas inválidas

### 3.2 Gate del aprendizaje adaptativo

Archivo modificado: `atlas_code_quant/learning/adaptive_policy.py`

Cambios:

- calcula `quality` antes de derivar sesgos
- expone:
  - `learning_allowed`
  - `raw_sample_count`
  - `quality`
- si `quality.learning_allowed == false`, el snapshot efectivo queda con:
  - `sample_count = 0`
  - sin sesgos activos
  - `risk_multiplier` neutro

### 3.3 Analítica del chart

Archivo modificado: `atlas_code_quant/journal/service.py`

Cambios:

- `chart_data()` excluye filas objetivamente inválidas antes de construir trades
- devuelve:
  - `quality`
  - `source_closed_trades`
  - `excluded_trades`

Nota importante: aquí solo se filtran filas objetivamente inválidas; el bloqueo completo del aprendizaje usa además la concentración por día.

### 3.4 Configuración

Archivo modificado: `atlas_code_quant/config/settings.py`

Nuevos umbrales:

- `QUANT_JOURNAL_QUALITY_MIN_SCORE_PCT`
- `QUANT_JOURNAL_QUALITY_MIN_DURATION_SEC`
- `QUANT_JOURNAL_QUALITY_OUTLIER_DAY_SHARE_PCT`
- `QUANT_JOURNAL_QUALITY_MAX_NEGATIVE_PRICE_RATIO_PCT`
- `QUANT_JOURNAL_QUALITY_STRATEGY_ANOMALY_MIN_SAMPLES`

## 4. Pruebas ejecutadas

Comando ejecutado:

```powershell
.\venv\Scripts\python.exe -m pytest `
  atlas_code_quant\tests\test_journal_data_quality.py `
  atlas_code_quant\tests\test_adaptive_policy_quality_gate.py `
  atlas_code_quant\tests\test_adaptive_policy_status.py `
  atlas_code_quant\tests\test_journal_chart_data.py -q
```

Resultado:

- `13 passed`

## 5. Archivos tocados

- `atlas_code_quant/learning/journal_data_quality.py`
- `atlas_code_quant/learning/adaptive_policy.py`
- `atlas_code_quant/journal/service.py`
- `atlas_code_quant/config/settings.py`
- `atlas_code_quant/tests/test_journal_data_quality.py`
- `atlas_code_quant/tests/test_adaptive_policy_quality_gate.py`
- `atlas_code_quant/tests/test_journal_chart_data.py`

## 6. Hipótesis descartadas

- **“El problema era solo visual del dashboard”**  
  Descartado. Hay contaminación real en el journal y esa contaminación llegaba al aprendizaje.

- **“Con quitar precios negativos basta”**  
  Descartado. Incluso tras excluir filas inválidas, la concentración por día deja `trusted_closed_count = 1` sobre el dataset completo.

## 7. Limitaciones abiertas

- Esta intervención **bloquea el aprendizaje contaminado**, pero **no repara todavía el origen de la corrupción histórica**.
- La raíz exacta que generó los bursts de `2026-03-29/30` sigue pendiente de trazado fino en el flujo de cierre/sync.
- El chart del dashboard puede seguir mostrando poca historia útil mientras la base permanezca contaminada, porque el filtrado actual prioriza no mentir antes que rellenar.

## 8. Siguiente fase recomendada

1. Trazar el origen de los cierres corruptos en `_close_entry()` y en el pipeline de sync.
2. Añadir marca persistente de cuarentena por fila o un job de saneamiento offline.
3. Bloquear también consumo de journal contaminado en scorecards derivados si aún leen cierres crudos.
4. Preparar un script de remediación controlada para:
   - exportar
   - clasificar
   - archivar outliers
   - reconstruir métricas limpias

## 9. Reproducción rápida

### Scorecard completo desde SQLite

Usado para verificar el journal completo:

```powershell
$env:PYTHONPATH='c:\ATLAS_PUSH;c:\ATLAS_PUSH\atlas_code_quant'
```

Luego ejecutar un script que cargue cierres desde `atlas_code_quant\data\journal\trading_journal.sqlite3` y pase las filas a `build_journal_quality_scorecard(...)`.

### Estado del aprendizaje

```powershell
$env:PYTHONPATH='c:\ATLAS_PUSH;c:\ATLAS_PUSH\atlas_code_quant'
.\venv\Scripts\python.exe - <<'PY'
from atlas_code_quant.learning.adaptive_policy import AdaptiveLearningService
payload = AdaptiveLearningService().refresh(force=True)
print(payload["learning_allowed"])
print(payload["quality"])
PY
```

Resultado esperado en el estado actual del journal:

- `learning_allowed == False`
- `quality.status == "critical"`
