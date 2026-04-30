# ATLAS Quant - Informe Maestro Pre-Live y Plan Go/No-Go

**Fecha de auditoria:** 2026-03-28
**Repositorio:** `C:\ATLAS_PUSH`
**Branch auditada:** `variante/nueva`
**Modo observado:** `paper_autonomous`
**Alcance:** auditoria operativa, trazabilidad de posiciones, sincronizacion con dashboards, correlacion con estrategias, analisis de resultados y plan pre-live
**Restricciones respetadas:** sin tocar archivos de runtime ni UIs protegidas

---

## 1. Resumen ejecutivo

ATLAS Quant ya no esta en estado de prototipo roto. El sistema consigue arrancar, monitorear, exponer metricas y mantener un loop autonomo funcional en paper. Sin embargo, la auditoria confirma que **todavia no esta listo para live controlado**.

La causa principal del resultado negativo no es un fallo visual de Grafana ni una sola averia puntual. El deterioro viene de una cadena de control incompleta:

1. el selector y el ejecutor no preservan correctamente la identidad estrategica de muchas ordenes de equity;
2. el loop autonomo permite submits repetidos con guardias insuficientes;
3. el journal y la monitorizacion terminan clasificando el libro como `untracked`;
4. la capa adaptativa no recibe feedback util porque no se estan cerrando operaciones con trazabilidad suficiente;
5. el control de riesgo queda degradado por inconsistencias de reconciliacion y por una vista parcial de equity.

El resultado es un libro con hit rate aceptable pero con **asimetria negativa severa en las perdidas del lado long**, escasa atribucion por estrategia y una gobernanza insuficiente para pasar a live.

**Dictamen profesional:** `NO-GO` para live en el estado actual. `GO` para una fase pre-live controlada en paper con checklist duro, observacion guiada y correcciones de trazabilidad/riesgo antes de reabrir la discusion de live.

---

## 2. Base objetiva verificada

### 2.1 Estado de libro y monitorizacion

- Prometheus reflejo durante la auditoria:
  - `atlas_broker_open_pnl_usd ~= -2157.8468`
  - `atlas_broker_open_positions ~= 350`
  - `atlas_broker_equity_usd ~= 93435.179194`
  - `atlas_broker_cash_usd ~= 21720.14`
  - `atlas_broker_market_value_usd ~= 87038.5732`
- La sincronizacion entre el libro canonico y la cuenta local de paper estaba rota:
  - `atlas_sync_status = 0`
  - `atlas_reconcile_gap_positions = -350`
  - `atlas_reconcile_gap_usd ~= -68435.1792`
- La base local [paper_account.db](C:/ATLAS_PUSH/atlas_code_quant/data/paper_account.db) no representa el libro operativo actual. Esta practicamente vacia frente al estado real observado en la capa canonica.

### 2.2 Journal de trading

- En [trading_journal.sqlite3](C:/ATLAS_PUSH/atlas_code_quant/data/journal/trading_journal.sqlite3) se verificaron:
  - `351` filas abiertas
  - `0` filas cerradas
  - PnL total abierto cercano a `-2152.708 USD`
  - `351` filas clasificadas como `strategy_type = untracked`
  - `351` filas sin `win_rate_at_entry` y sin `current_win_rate_pct`
- Implicacion: el sistema esta operando, pero no esta atribuyendo correctamente las operaciones a una estrategia exploitable para control o aprendizaje.

### 2.3 Patrones de exposicion

- Se detectaron `829` unidades agregadas sobre `351` posiciones, con cantidades maximas de hasta `7` unidades en un mismo nombre.
- El loop externo produjo una secuencia muy agresiva de envio:
  - `1087` eventos `submit`
  - `0` eventos `blocked`
  - `0` eventos `skip`
- En el bridge aparecio repetidamente la advertencia:
  - `strategy_type is missing; autonomous gate is operating without Monte Carlo validation.`

### 2.4 Patron de resultado

- El problema no es una tasa de acierto nula. El libro abierto mostraba aproximadamente:
  - `56.13%` de posiciones positivas
  - ganancia media de `+7.11 USD`
  - perdida media de `-23.07 USD`
- Por lado:
  - libro `long`: `291` posiciones, PnL cercano a `-2277 USD`
  - libro `short`: `60` posiciones, PnL cercano a `+125 USD`
- Conclusion: el dano principal esta en el control del lado long, no en una destruccion uniforme del sistema.

---

## 3. Hallazgos principales

### Hallazgo 1 - Los dashboards dicen la verdad del libro, pero la reconciliacion esta rota

Grafana y Prometheus no son la causa raiz del problema. Estan mostrando una fotografia coherente del libro canonico y, al mismo tiempo, estan avisando que la reconciliacion con la capa local de paper no cierra.

Interpretacion operativa:

- el dashboard es util como panel de alerta;
- la capa de reconciliacion no puede considerarse confiable mientras haya gap de posiciones y gap economico tan alto;
- cualquier decision de risk gating que dependa de la cuenta local queda degradada.

### Hallazgo 2 - La identidad estrategica se pierde en la cadena de ejecucion

Se verifico una fuga estructural de metadata:

- [strategy_selector.py](C:/ATLAS_PUSH/atlas_code_quant/selector/strategy_selector.py) vacia o no consolida `strategy_type` para equities;
- [main.py](C:/ATLAS_PUSH/atlas_code_quant/api/main.py) reconstruye `OrderRequest` sin propagar metadata estrategica relevante;
- [operation_center.py](C:/ATLAS_PUSH/atlas_code_quant/operations/operation_center.py) permite submit aunque falte `strategy_type`;
- [advanced_monitor.py](C:/ATLAS_PUSH/atlas_code_quant/monitoring/advanced_monitor.py) reagrupa muchas equities como `untracked`.

Consecuencia:

- no hay correlacion confiable entre posicion, setup y resultado;
- el gate autonomo pierde la validacion probabilistica por estrategia;
- el journal deja de servir como instrumento real de aprendizaje y auditoria.

### Hallazgo 3 - El loop autonomo permite sobre-ejecucion

En [autonomous_loop.py](C:/ATLAS_PUSH/scripts/autonomous_loop.py) se observo una combinacion peligrosa:

- `size = 1` fijo;
- cooldown corto por simbolo;
- ausencia de control explicito contra reentrada sobre posicion ya abierta;
- falta de `strategy_type` y de contexto de sizing/risk al enviar orden.

El sistema no parecia "volverse loco" por un error visual. Lo que ocurrio es mas serio: el loop seguia encontrando candidatos y enviando entradas sin la gobernanza suficiente para impedir stacking de exposicion.

### Hallazgo 4 - El libro pierde por asimetria de perdida, no por falta absoluta de acierto

El hit rate del libro abierto no es desastroso. Lo desastroso es la relacion entre ganancia media y perdida media. En el lado long, las perdidas abiertas grandes borran con facilidad multiples aciertos pequenos.

Esto sugiere tres fallos combinados:

- las entradas long no estan siendo filtradas con suficiente calidad;
- el sistema no esta limitando reentradas ni concentracion con suficiente dureza;
- la gestion de salida o la contencion de deterioro no esta cerrando suficientemente rapido a los peores nombres.

### Hallazgo 5 - La capa adaptativa esta ciega para el contexto actual

En [adaptive_policy_snapshot.json](C:/ATLAS_PUSH/atlas_code_quant/data/learning/adaptive_policy_snapshot.json) se observo `sample_count = 0`, mientras que la tabla historica de aprendizaje contiene registros viejos y poco representativos del universo actual.

Ademas, al tener `0` operaciones cerradas en el journal, el sistema no esta generando feedback util para reajustar el comportamiento. En la practica:

- el loop aprende poco o nada del regimen vigente;
- la politica adaptativa no puede corregir el sesgo del lado long;
- la decision autonoma queda desacoplada del resultado reciente real.

### Hallazgo 6 - Hay una inconsistencia de equity que degrada el control de riesgo

En [operation_center.py](C:/ATLAS_PUSH/atlas_code_quant/operations/operation_center.py) se identifico una lectura de `balances["equity"]`, mientras que en monitorizacion/canonico aparece `total_equity`.

Esto provoca escenarios en los que la vista `what-if` queda con `current_equity = 0.0` pese a existir equity real. No explica por si solo el PnL negativo, pero si debilita la calidad del control y del diagnostico interno.

---

## 4. Causa raiz consolidada

La perdida no viene de una unica "mala estrategia". Viene de una cadena rota de gobernanza:

1. el motor autonomo genera o selecciona candidatos;
2. al pasar a la capa de submit, se pierde metadata esencial de estrategia;
3. el gate permite ejecutar aun sin dicha metadata;
4. el loop no bloquea con suficiente dureza duplicados, stacking o reentrada sobre libro ya abierto;
5. el monitor/journal ya no pueden atribuir bien el rendimiento;
6. la capa adaptativa no recibe cierres ni senales limpias para corregir;
7. el libro se llena de longs con perfil de perdida asimetrica y sin disciplina suficiente de contencion.

En otras palabras: **el error ha estado menos en "falta de features" y mas en una arquitectura de ejecucion que todavia no protege bien el capital cuando el contexto se degrada**.

---

## 5. Por que los resultados no han sido positivos

### 5.1 Seleccion y ejecucion no estan alineadas

No puede demostrarse hoy, con la trazabilidad existente, que las posiciones negativas correspondan a una estrategia concreta con edge validado. La mayor parte del libro esta etiquetada como `untracked`. Eso invalida cualquier lectura seria de "esta estrategia falla" frente a "el pipeline de ejecucion esta perdiendo identidad".

### 5.2 El lado long concentra el dano

La evidencia apunta a que el sesgo long ha sido el principal generador de perdida abierta. El libro short no solo no explica el deterioro, sino que muestra un comportamiento mejor.

### 5.3 La sobre-ejecucion multiplica el impacto del error

Un mal candidato aislado es manejable. Decenas o cientos de entradas repetidas sobre un universo sin trazabilidad ni freno fuerte convierten un fallo moderado en una perdida estructural.

### 5.4 El sistema no esta aprendiendo de sus errores recientes

Sin cierres utiles y sin metadata consistente, el aprendizaje adaptativo no puede recalibrar el comportamiento. El sistema sigue empujando decisiones con memoria debil del dano reciente.

---

## 6. Checklist Go/No-Go pre-live

Todo lo siguiente debe quedar en verde de forma sostenida antes de considerar live controlado.

| Categoria | Criterio | Estado actual | Criterio Go |
|-----------|----------|---------------|-------------|
| Reconciliacion | `atlas_sync_status` | Rojo | `1` sostenido durante sesion completa |
| Atribucion | Operaciones `untracked` | Critico | `0` nuevas operaciones `untracked` |
| Metadata | `strategy_type`, setup, risk budget, origen | Incompleto | Obligatorio en cada submit |
| Exposicion | Reentrada y stacking no autorizados | Debil | Bloqueo duro salvo regla explicita |
| Riesgo | Equity y balances coherentes | Inconsistente | Campo canonico unico y validado |
| Loop autonomo | Submit/blocked/skip razonables | Desbalanceado | Evidencia de guardias activas |
| Journal | Aperturas y cierres trazables | Insuficiente | Cierre y feedback completos |
| Learning | Politica adaptativa con muestras utiles | Ciega | Muestras representativas por scope |
| Monitorizacion | Dashboard y libro canonico coherentes | Parcial | Coherencia con reconciliacion verde |
| Estabilidad | Backend Quant accesible toda la sesion | Fragil | Sin caidas en ventana vigilada |

**Regla de veto:** si cualquiera de los cuatro puntos criticos siguientes falla, la respuesta debe ser `NO-GO` sin debate:

- reconciliacion en rojo;
- nuevas operaciones `untracked`;
- reentradas no autorizadas sobre simbolos abiertos;
- backend inestable durante la sesion de validacion.

---

## 7. Plan de remediacion priorizado

### P0 - Bloqueadores inmediatos de capital

Objetivo: impedir que el sistema siga ampliando un problema de gobernanza.

1. Hacer `strategy_type` obligatorio en toda orden autonoma de equity.
2. Bloquear submit si falta metadata minima: `strategy_type`, setup, score, account scope y risk budget.
3. Introducir guardia dura de reentrada:
   - no abrir una nueva posicion sobre simbolo ya abierto salvo pyramiding declarado;
   - no incrementar cantidad si no existe una razon explicita y registrada.
4. Vetar operacion autonoma cuando `atlas_sync_status != 1`.
5. Corregir la fuente de equity para que el control de riesgo use un unico campo canonico.

### P1 - Recuperar trazabilidad y capacidad de diagnostico

Objetivo: saber exactamente que estrategia hace dinero, cual pierde y por que.

1. Persistir en journal y monitor:
   - `strategy_type`
   - setup
   - score del selector
   - motivo de entrada
   - origen del loop o del componente que envio la orden
2. Exponer en dashboards paneles por estrategia:
   - PnL abierto
   - PnL realizado
   - hit rate
   - avg win
   - avg loss
   - profit factor
3. Marcar toda operacion sin metadata como error operativo y no como caso tolerado.
4. Anadir un reporte diario automatico de peores nombres, repetidos, concentracion y drift de exposicion.

### P2 - Endurecer decision y aprendizaje

Objetivo: evitar que el sistema repita sesgos negativos en el lado long.

1. Revisar la logica de seleccion long y su filtro de contexto.
2. Separar claramente familias:
   - equity momentum long
   - equity mean reversion
   - short defensivo
   - opciones si aplica
3. Reforzar kill criteria por nombre y por libro:
   - max perdida abierta por simbolo
   - max exposicion por sector
   - max deterioro por ventana temporal
4. Cerrar el loop adaptativo solo con operaciones validas y trazables.
5. Exigir un minimo de cierres representativos antes de recalibrar politicas.

---

## 8. Plan operativo de validacion pre-live

### Fase A - Contencion y saneamiento

Duracion sugerida: `24-48h`

- detener o limitar el loop autonomo actual hasta activar las guardias P0;
- confirmar que no nacen nuevas operaciones `untracked`;
- verificar que dashboard, journal y libro canonico describen el mismo estado.

### Fase B - Paper controlado y vigilado

Duracion sugerida: `1 sesion completa`

- un solo loop activo;
- universo reducido;
- limites duros por simbolo, sector y exposicion total;
- observacion cada `15 min` de:
  - reconciliacion
  - numero de submits
  - numero de bloqueos
  - posiciones nuevas
  - peores nombres
  - latencia y salud del backend

### Fase C - Evaluacion de calidad

Duracion sugerida: `3-5 sesiones`

- comparar rendimiento por estrategia real, no por libro agregado;
- confirmar que las operaciones long no vuelven a concentrar el deterioro;
- verificar que la capa adaptativa ya recibe cierres y muestras representativas;
- comprobar consistencia sostenida entre journal, snapshot canonico y dashboards.

### Fase D - Decision Go/No-Go

Solo considerar live controlado si:

- no aparecen operaciones `untracked`;
- reconciliacion se mantiene verde;
- el libro deja de depender de stacking para sostener actividad;
- el lado long muestra expectativa al menos neutra bajo guardias;
- el backend y la observabilidad se sostienen sin caidas durante la ventana completa.

---

## 9. Recomendacion final

ATLAS Quant esta **fuerte para paper autonomous**, pero todavia no en un nivel de madurez profesional para `live`.

La prioridad correcta no es anadir mas features ni maquillar dashboards. La prioridad es cerrar disciplina operativa:

- identidad estrategica obligatoria;
- reconciliacion como precondicion de ejecucion;
- guardias anti-duplicado y anti-stacking;
- trazabilidad total para auditoria y aprendizaje;
- validacion larga y vigilada antes de abrir live.

**Veredicto final:** `NO-GO` a live hoy. `GO` a una fase pre-live seria, acotada y medible, con este checklist como contrato tecnico-operativo.

---

## 10. Archivos de referencia auditados

- [operation_center.py](C:/ATLAS_PUSH/atlas_code_quant/operations/operation_center.py)
- [advanced_monitor.py](C:/ATLAS_PUSH/atlas_code_quant/monitoring/advanced_monitor.py)
- [strategy_selector.py](C:/ATLAS_PUSH/atlas_code_quant/selector/strategy_selector.py)
- [main.py](C:/ATLAS_PUSH/atlas_code_quant/api/main.py)
- [autonomous_loop.py](C:/ATLAS_PUSH/scripts/autonomous_loop.py)
- [trading_journal.sqlite3](C:/ATLAS_PUSH/atlas_code_quant/data/journal/trading_journal.sqlite3)
- [paper_account.db](C:/ATLAS_PUSH/atlas_code_quant/data/paper_account.db)
- [operation_center_state.json](C:/ATLAS_PUSH/atlas_code_quant/data/operation/operation_center_state.json)
- [adaptive_policy_snapshot.json](C:/ATLAS_PUSH/atlas_code_quant/data/learning/adaptive_policy_snapshot.json)
- [quant_brain_bridge.jsonl](C:/ATLAS_PUSH/atlas_code_quant/logs/quant_brain_bridge.jsonl)
- [atlas_live_loop.log](C:/ATLAS_PUSH/logs/atlas_live_loop.log)
