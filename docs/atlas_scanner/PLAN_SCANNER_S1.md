# PLAN SCANNER S1 (Propuesto)

Este plan define la ejecucion de S1 para `atlas_scanner` sobre la rama
`feat/atlas-scanner-s1-offline`, creada desde el estado S0 actual del clon
(`f47d134d`), con `main` congelada (sin cambios).

## 1) Objetivo de S1

Construir un runner offline minimo pero real sobre los contratos S0 ya
existentes, capaz de ejecutar `run_scan` de punta a punta con datos
deterministas, sin red y sin dependencias de providers reales.

Resultado esperado:
- Flujo offline completo: universo -> filtros -> features -> scoring/ranking
  -> resultado final.
- Ejecucion repetible y determinista con fixtures pequenas.
- Base preparada para evolucion a S2/S3 sin romper contratos de dominio.

## 2) Que añade S1 por encima de S0

S0 ya aporta:
- Contratos y modelos de dominio.
- Configuracion de scoring rica (`ScoringConfig`).
- Excepciones base.
- Scaffolding de `universe/`, `filters/`, `features/`, `providers/`.

S1 añade:
- Implementacion funcional offline (no stub) de universo, filtros, features y
  scoring/ranking/dedup.
- Orquestacion real por `run_scan` en modo offline.
- Capa de carga de configuracion/fixtures offline.
- Guardrails explicitos de determinismo.

## 3) Fuera de alcance en S1

- Integracion con providers reales (`Tradier`, `Polygon`, `OpenBB`, etc.).
- I/O de red, llamadas HTTP o sockets.
- I/O de archivos en runtime del scanner.
- Dependencia del reloj del sistema para decisiones de negocio.
- Aleatoriedad (`random`) o fuentes no deterministas.
- Integracion productiva con scheduler, broker o decision engine.

## 4) Alcance funcional S1

### 4.1 Loader/config offline
- Cargar configuracion y parametros desde estructuras internas/fixtures offline.
- Mantener `SCORING_CONFIG` como base canonica.
- Sin lectura de `.env` ni configuracion remota.

### 4.2 Fixtures offline
- Fixtures pequenas y estables para universo, features y expected outputs.
- Formatos simples de test para facilitar mantenimiento.
- Sin dependencia de snapshots dinamicos externos.

### 4.3 Universe selector real
- `select_universe(...)` pasa de stub a seleccion funcional offline.
- Resultado determinista para un `name` conocido.

### 4.4 Filtros reales
- `apply_liquidity_filter(...)`, `apply_tradability_filter(...)` y
  `apply_event_risk_filter(...)` con reglas minimas reales para S1.
- Salida canonica `(kept_symbols, rejected_symbols)`.

### 4.5 Features reales
- `compute_volatility`, `compute_gamma`, `compute_flow`, `compute_price`,
  `compute_macro`, `compute_visual` dejan de retornar constante.
- Calculo deterministico basado solo en fixture input.

### 4.6 Scoring/ranking/dedup reales
- Construccion de `ScoreBreakdown` y `CandidateOpportunity`.
- Uso de pesos/reglas en `SCORING_CONFIG` para score final.
- Ranking y deduplicacion por `strategy_family` usando reglas S1.

### 4.7 `run_scan` offline end-to-end
- Orquesta el pipeline offline completo y produce `ScannerRunResult`.
- Llena `filtered_symbols`, `rejected_symbols`, `error_symbols`,
  `metrics`, `warnings` y `data_source_path` coherentemente.

### 4.8 Guardrails de determinismo
- Mismo input offline -> mismo output byte a byte (o estructura equivalente).
- Tests que bloquean no determinismo accidental.

## 5) Arquitectura S1 (flujo)

Pipeline S1 propuesto:

1. Input: `ScanSnapshot` + contexto offline.
2. Universe: seleccion de simbolos (`universe/selector.py`).
3. Filters: liquidez -> tradability -> event risk (`filters/*`).
4. Features: calculo por simbolo (`features/*`).
5. Scoring: agregacion de factores y `ScoreBreakdown`.
6. Ranking/dedup: orden final y limites por familia.
7. Output: `ScannerRunResult` completo y determinista.

Regla de diseno:
- Cada etapa devuelve estructuras puras tipadas, sin side effects.
- Providers quedan como fachada futura, no como fuente de datos en S1.

## 6) Datos y determinismo

Principios obligatorios en S1:
- Sin red.
- Sin I/O (runtime del pipeline).
- Sin reloj (sin `datetime.now()` para decisiones de negocio).
- Sin random.
- Fixtures pequenas y deterministas como fuente unica de verdad.

Medidas practicas:
- Ordenamiento explicito antes de agregar/rankear.
- Eliminacion de dependencias de iteraciones no ordenadas.
- Inputs de prueba minimos pero representativos.

## 7) Tests planeados

Cobertura esperada:
- Unit tests de `universe` (seleccion conocida y fallback seguro).
- Unit tests de `filters` (kept/rejected segun fixture controlado).
- Unit tests de `features` (valores esperados deterministas).
- Unit tests de scoring/ranking/dedup (orden y top-k reproducibles).
- Unit tests de `run_scan` offline end-to-end.
- Tests de determinismo: doble corrida, mismo input, mismo output.
- Regression tests para contratos S0 (compatibilidad hacia adelante).

## 8) Plan de commits S1.0-S1.8

Secuencia propuesta y obligatoria:
1. **S1.0** docs (`PLAN_SCANNER_S1.md`).
2. **S1.1** `config_loader` offline.
3. **S1.2** fixtures offline.
4. **S1.3** `universe` + `filters` reales.
5. **S1.4** `features` reales.
6. **S1.5** scoring/ranking/dedup reales.
7. **S1.6** `run_scan` + offline runner.
8. **S1.7** guardrails de determinismo.
9. **S1.8** cierre documental.

Regla de trabajo:
- Commit atomico por bloque.
- No mezclar implementacion de varios bloques en un solo commit.
- Verificar tests en cada bloque antes de continuar.

## 9) Criterios de aceptacion S1 (borrador)

- [ ] `run_scan` offline ejecuta end-to-end sin red ni I/O.
- [ ] Resultado determinista con fixtures fijas.
- [ ] Cobertura unitaria para universe/filters/features/scoring/pipeline.
- [ ] Contratos S0 siguen validos.
- [ ] Rama `main` permanece intacta.

## 10) Protocolo operativo

1. Aprobacion de plan S1.
2. Implementacion incremental S1.1-S1.8.
3. Verificacion de tests por bloque.
4. Ajustes documentales de cierre.
5. Preparacion de PR con evidencia de determinismo.

Estado de este documento:
- Version: `S1-plan-v1`
- Rama: `feat/atlas-scanner-s1-offline`
- Base: `f47d134d`
- Fecha: `2026-04-22`

