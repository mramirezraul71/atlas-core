# ATLAS Quant - Marco Maestro de Auditoria del Proceso de Trading

**Fecha:** 2026-03-28
**Repositorio:** `C:\ATLAS_PUSH`
**Foco activo:** `execution_quality`
**Alcance del algoritmo:** abierto para `entry_validation`, `execution_quality`, `position_management`, `exit_governance` y `post_trade_learning`

---

## 1. Proposito

ATLAS ya no debe corregirse por intuicion aislada ni por reaccion a sintomas sueltos.
Desde esta version, el sistema debe analizar cada etapa del trading con el mismo esquema logico que ya se uso primero en `scanner_selection`, luego en `entry_validation` y ahora se aplica a `execution_quality`:

1. detectar degradacion interna;
2. recoger evidencia local suficiente;
3. buscar benchmark serio en red;
4. comparar ATLAS vs benchmark;
5. aislar causa raiz;
6. diseñar correccion medible;
7. seguir el before/after;
8. persistir aprendizaje;
9. promover o revertir politica segun evidencia.

Este marco evita tres errores comunes:

- corregir una capa sin demostrar que ahi nace el daño;
- confundir un indicador puntual con una regla estructural;
- saltar a la siguiente etapa sin haber dejado memoria y criterio de seguimiento.

---

## 2. Regla maestra

Toda etapa del proceso de trading debe pasar por la misma cadena:

`detectar -> evidenciar -> investigar -> comparar -> diagnosticar -> corregir -> seguir -> persistir -> promover o revertir`

Eso convierte el aprendizaje de ATLAS en un proceso acumulativo y reusable.

---

## 3. Aplicacion por etapa

### 3.1 scanner_selection

Pregunta central:
`Que criterios de seleccion anticipan mejor trayectoria futura con payoff sano y muestra defendible?`

Estado:
`baseline_hardened`

Ya aprendido:

- no basta con `local_win_rate`;
- hay que priorizar `expectancy`, `profit_factor`, `sample_confidence` y `payoff_ratio`;
- hay que separar ruido intradia de trayectoria swing;
- hay que introducir contexto de regimen, sector y crash risk.

### 3.2 entry_validation

Pregunta central:
`La tesis sigue viva en el momento exacto del ticket o ya se degrado?`

Estado:
`baseline_hardened`

ATLAS debera estudiar:

- precio de decision vs precio de entrada;
- spread, liquidez y slippage esperado;
- confirmacion final del setup;
- deterioro entre scan y submit.

Ya incorporado en `P0`:

- `entry_reference_price`;
- deriva adversa maxima;
- spread maximo para entradas de equity;
- payload de entry validation en `OperationCenter`.

### 3.3 execution_quality

Pregunta central:
`La ejecucion preserva la tesis o la degrada con errores operativos?`

Estado:
`active_focus`

ATLAS debera estudiar:

- orden planificada vs orden emitida;
- duplicados, reentradas y stacking;
- slippage real, latencia y rechazos;
- salud de reconciliacion.

Ya incorporado en `P0`:

- lectura estructurada `planned_order -> request_payload -> tradier_response`;
- clasificacion `warning/degraded`;
- checks de coincidencia de simbolo, lado, cantidad, modo y estado del broker.

### 3.4 position_management

Pregunta central:
`La posicion abierta sigue sana o ya esta violando el plan de riesgo?`

ATLAS debera estudiar:

- `MAE`, `MFE` y hold time;
- exposicion por simbolo, sector y libro;
- deterioro de volatilidad o correlacion;
- invalidacion progresiva de la tesis.

### 3.5 exit_governance

Pregunta central:
`La salida se hace por criterio o por ruido?`

ATLAS debera estudiar:

- motivo real de salida;
- diferencia entre target, invalidez y time stop;
- si corta ganadores demasiado pronto;
- si deja correr perdedores demasiado tiempo.

### 3.6 post_trade_learning

Pregunta central:
`El sistema esta aprendiendo de forma acumulativa o solo archivando eventos?`

ATLAS debera estudiar:

- expectativa prometida vs resultado real;
- memoria/bitacora con contexto suficiente;
- politicas promovidas, retenidas o revertidas;
- reuso del aprendizaje entre etapas.

---

## 4. Guardrails del algoritmo

- No promover cambios a `live` por una prueba corta.
- No dar por buena una mejora sin trazabilidad `before/after`.
- No usar benchmark externo como receta ciega; primero hay que comprobar el problema local.
- No cerrar una etapa si no queda huella en informe, memoria y bitacora.
- No abrir la siguiente etapa si la anterior sigue ciega o mal atribuida.

---

## 5. Estado actual del programa

Dictamen operativo a esta fecha:

- `scanner_selection`: baseline endurecida y ya recalibrada en el ranking local.
- `entry_validation`: baseline endurecida con guardrails de drift y spread.
- `execution_quality`: en auditoria activa y con primera capa estructurada de quality checks.
- `position_management`: pendiente.
- `exit_governance`: pendiente.
- `post_trade_learning`: marco creado y memoria activa, falta enriquecer atribucion continua.

---

## 6. Salida esperada

Si ATLAS sigue este marco de forma disciplinada, el crecimiento no vendra de "mas features", sino de:

- mejor atribucion causal;
- mejores correcciones por prioridad;
- menos cambios impulsivos;
- aprendizaje acumulativo entre etapas del proceso;
- mayor capacidad de decidir `GO / NO-GO` con evidencia real.

---

## 7. Implementacion en repo

Soporte principal:

- `atlas_code_quant/learning/trading_self_audit_protocol.py`
- `atlas_code_quant/scripts/record_trading_self_audit_note.py`
- `reports/trading_self_audit_protocol.json`
- `reports/atlas_quant_external_benchmark_confrontation_20260328.md`

El protocolo ya contiene:

- esquema comun de analisis;
- mapa de etapas del proceso;
- guardrails del algoritmo;
- flujo de confrontacion web con benchmark externo;
- registro versionado de fuentes externas ya consultadas;
- foco activo en `scanner_selection`;
- extension abierta a todas las fases siguientes.
