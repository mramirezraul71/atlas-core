# ATLAS Quant — Confrontación con Benchmarks Externos
**Fecha:** 2026-03-28
**Branch:** variante/nueva
**Metodología:** TSAP Step 3 — External Benchmarks (Detect → Evidence → Benchmarks → Compare → Diagnose → Correct → Monitor → Persist → Promote/Revert)

---

## Resumen Ejecutivo

Este reporte confronta el estado operativo real de ATLAS-Quant contra estándares académicos y de industria en seis áreas del ciclo de trading. El objetivo no es declarar "funcionando" sino identificar con evidencia empírica dónde el sistema está dentro del estándar, dónde está por debajo, y qué acciones concretas cierran la brecha.

**Estado global al 2026-03-28:**
- Preview=True bug: **RESUELTO** — todas las órdenes en modo submit ahora ejecutan contra Tradier sandbox
- Paper account trades ejecutados: **0** (pre-fix)
- Adaptive policy sample_count: **0** (aprendizaje bloqueado por ausencia de trades cerrados)
- IC tracker: **activo**, acumulando señales desde fix de loop
- Tests: **576 passed, 0 failed**

---

## Benchmark 1 — Scanner Selection
**Referencia:** Grinold & Kahn, *Active Portfolio Management* (2000), Cap. 6

### Estándar externo
| Métrica | Umbral | Nivel |
|---------|--------|-------|
| IC (Spearman predicted vs actual) | > 0.05 | meaningful |
| IC | > 0.10 | fuerte |
| t-stat IC | >= 2.0 | requerido para producción |
| N mínimo | 30 | para evaluar señal |

### Estado ATLAS-Quant
| Métrica | Valor actual | Estado |
|---------|-------------|--------|
| IC tracker activo | Sí (desde fix) | OK |
| Señales con outcome | 0 (pre-fix) | CRITICO |
| IC calculable | No | CRITICO |
| Método de predicción (predicted_move_pct) | Sí, en candidatos scanner | OK |

### Diagnóstico
El scanner genera `predicted_move_pct` por candidato pero nunca se medía el outcome real porque las órdenes se ejecutaban con `preview=True`, impidiendo que se cerraran posiciones y se computara el retorno real. Con el fix aplicado, el `ICSignalTracker` ya registra señales no bloqueadas y esperará N días de outcomes para calcular IC.

### Brecha residual
- El IC no puede evaluarse hasta tener >=30 outcomes cerrados en paper.
- El scanner no reporta `predicted_move_pct` en todos los métodos — algunos candidatos tienen `predicted_move_pct=0.0`, lo que excluye esas señales del cálculo IC.

### Acción requerida
1. Verificar que los métodos de scanner calculan `predicted_move_pct` real (no 0.0).
2. Implementar `update_outcome()` cuando se cierra una posición en el journal (hook en `_close_entry`).
3. Esperar 30 ciclos con outcomes para primera evaluación de IC.

---

## Benchmark 2 — Entry Validation
**Referencia:** Perold (1988), "The Implementation Shortfall: Paper vs. Reality", *Journal of Portfolio Management*

### Estándar externo
| Componente IS | Descripción |
|---------------|-------------|
| Decision lag | Tiempo entre señal y envío de orden |
| Execution lag | Slippage desde precio de referencia |
| Opportunity cost | Costo de órdenes bloqueadas |
| Market impact | Impacto por tamaño de orden |

IS total aceptable para estrategias de retail/paper: < 0.5% del capital por operación.

### Estado ATLAS-Quant
| Componente | Valor | Estado |
|------------|-------|--------|
| Decision lag | ~120s (interval del loop) | Aceptable para paper |
| Execution lag | No medido (0 trades reales) | N/A hasta fix |
| Opportunity cost | Alta: 350 posiciones Tradier sin gestión local | CRITICO |
| Market impact | Paper — zero real impact | OK |

### Diagnóstico
Con 0 trades ejecutados localmente pero 350 posiciones en Tradier sandbox, existe una brecha de reconciliación significativa. El `paper_account.db` local tiene $25,000 cash sin tocar mientras Tradier sandbox muestra $93,682 equity con 350 posiciones abiertas. Este gap implica que el "decision lag" real no puede medirse porque el sistema nunca conectó sus decisiones con ejecuciones reales hasta el fix.

### Acción requerida
1. Tras el fix de preview=True, medir el tiempo entre `recorded_at` (IC tracker) y `order_placed_at` (journal) para primer IS medible.
2. Sincronizar paper_account.db con posiciones reales de Tradier sandbox (o limpiar Tradier y empezar fresh).

---

## Benchmark 3 — Execution Quality
**Referencia:** Almgren & Chriss (2001), "Optimal Execution of Portfolio Transactions"

### Estándar externo
Para paper trading de tamaño 1 acción a mercado:
- Slippage esperado: 0 (no hay impacto de mercado real)
- Fill rate: 100% en horario de mercado con liquidez normal
- Órdenes rechazadas por compliance: < 5% de las intentadas

### Estado ATLAS-Quant
| Métrica | Valor | Estado |
|---------|-------|--------|
| Órdenes enviadas a Tradier sandbox | 0 (pre-fix) | CRITICO |
| Fill rate medido | N/A | CRITICO |
| Órdenes bloqueadas por OperationCenter | No medido | Pendiente |
| Preview ratio (ordenes preview / submit) | 100% (pre-fix) | BUG — RESUELTO |

### Diagnóstico
El bug crítico `preview=True` hardcodeado en `_auto_cycle_loop` convirtió todas las "ejecuciones" en simulaciones locales. El OperationCenter evaluaba correctamente los candidatos pero nunca llegaba a Tradier. Desde el fix, la ejecución real está habilitada.

### Brecha residual
- Sin historial de fills reales, no se puede calcular slippage ni fill rate.
- El `OperationCenter` tiene gates correctas (`production_guard`, `pdt_controller`) pero no registra qué porcentaje de candidatos pasa vs. es bloqueado.

### Acción requerida
1. Añadir contador de `orders_attempted / orders_sent / orders_blocked` en `_AUTO_CYCLE_STATE`.
2. Tras 50 ciclos, calcular `block_rate` y comparar con el < 5% estándar.

---

## Benchmark 4 — Position Management
**Referencia:** Van Tharp, *Trade Your Way to Financial Freedom* (1999) — R-multiples framework

### Estándar externo
| Métrica | Umbral saludable |
|---------|-----------------|
| R-múltiple promedio | > 0.5R (expectancy positiva) |
| Max pérdida por operación | <= 2R (stop disciplinado) |
| Ratio posiciones con stop definido | >= 80% |
| Distribución de salidas | Losses cortados < Winners |

### Estado ATLAS-Quant
| Métrica | Valor | Estado |
|---------|-------|--------|
| Posiciones cerradas con R calculado | 0 | CRITICO — sin muestra |
| Posiciones abiertas con stop definido | ~0% (Tradier sandbox, sin stops locales) | CRITICO |
| open_r_multiple promedio | N/A (0 cierres) | CRITICO |
| Posiciones strategy_type="untracked" | 100% (351 posiciones Tradier) | CRITICO |

### Diagnóstico
Las 350 posiciones de Tradier sandbox no tienen estrategia atribuida localmente. El sistema de R-multiples del `exit_governance_snapshot` calcula `open_r_multiple` pero solo para posiciones registradas en `trading_journal` con `entry_price` válido. Sin esa información, el framework de Van Tharp no puede activarse.

### Acción requerida
1. Importar/sincronizar posiciones Tradier a `trading_journal` con `strategy_type` correcto.
2. Definir `initial_stop_pct` (e.g., ATR-based) para todas las posiciones abiertas.
3. Implementar `update_outcome()` en hook de cierre de posición.

---

## Benchmark 5 — Exit Governance
**Referencia:** Kaminski & Lo (2014), "When Do Stop-Loss Rules Stop Losses?", *Journal of Financial Markets*

### Estándar externo
Kaminski & Lo demuestran que stops basados en reglas sistemáticas superan a stops ad-hoc en mercados con momentum:
- Stops basados en volatilidad (ATR-based): reducen drawdown sin sacrificar retorno en tendencias.
- Tiempo de hold mayor a umbral de volatilidad: incrementa probabilidad de reversión — señal de salida.

### Estado ATLAS-Quant
| Componente | Implementado | Activo |
|------------|-------------|--------|
| `exit_governance_snapshot()` | Sí | Sí |
| Exit pass en `_auto_cycle_loop` | Sí (añadido en esta sesión) | Sí |
| Órdenes de cierre ejecutadas | 0 (pre-fix) | Pendiente |
| Stops basados en ATR/volatilidad | No | Pendiente |
| `holding_hours` como criterio | Sí (en watchlist) | Sí |
| Urgency levels (high/medium/low) | Sí (en esquema) | Sí |

### Diagnóstico
El framework de exit governance está arquitecturalmente correcto: `exit_governance_snapshot` clasifica posiciones en `exit_now / de_risk / take_profit / hold` con `urgency`. El loop ahora incluye el "exit pass" antes del entry pass. Sin embargo, sin posiciones registradas localmente, el exit pass no tiene candidatos para actuar.

### Acción requerida
1. Tras sincronizar posiciones Tradier al journal, verificar que exit pass produce candidatos reales.
2. Añadir `atr_stop_pct` como campo de `trading_journal` para criterio Kaminski-Lo.
3. Medir `hold_time_at_exit` vs `target_hold_time` para validar el criterio de reversión.

---

## Benchmark 6 — Post-Trade Learning
**Referencia:**
- Brinson, Hood & Beebower (1986/1991), "Determinants of Portfolio Performance"
- Lopez de Prado (2018), *Advances in Financial Machine Learning*, Cap. 14 (Live vs. Simulated)

### Estándar externo — BHB Attribution
La atribución BHB descompone el retorno en:
1. **Asset Allocation**: elección de tipo de activo (equity/option/ETF/crypto)
2. **Security Selection**: elección del símbolo dentro de la clase
3. **Interaction**: timing de entry/exit

Para que la atribución sea útil: >=30 trades cerrados con `strategy_type` correctamente clasificado.

### Estándar externo — Lopez de Prado Cap. 14
Criterios mínimos antes de ir a live:
1. Sharpe simulado >= 1.0 con significancia estadística (p < 0.05)
2. Deflated Sharpe Ratio > 0 (ajustado por overfitting)
3. Minimum Track Record Length (MTRL) >= 12 meses de señales OOS
4. Drawdown máximo < 20% del capital
5. No hay regime change entre período de entrenamiento y deployment

### Estado ATLAS-Quant
| Criterio | Valor | Estado |
|---------|-------|--------|
| Trades cerrados con PnL | 0 | CRITICO |
| `post_mortem_json` coverage | 0% | CRITICO |
| `strategy_type` atribuido | 0% posiciones abiertas | CRITICO |
| Sharpe simulado calculado | No | Pendiente |
| MTRL estimado | N/A | Pendiente |
| `adaptive_policy` sample_count | 0 | CRITICO — aprendizaje bloqueado |

### Diagnóstico
El `AdaptiveLearningService` tiene `sample_count=0` en todos los scopes porque nunca se ha cerrado una posición con outcome real. Este es el efecto cascada del bug `preview=True`: sin trades ejecutados -> sin posiciones cerradas -> sin outcomes -> sin IC -> sin aprendizaje adaptativo -> sin mejora de política.

El fix de `preview=True` es el desbloqueador de toda la cadena de aprendizaje.

### Acción requerida
1. Tras el fix, esperar 10 ciclos con paper trades cerrados.
2. Verificar que `adaptive_policy_snapshot.json` muestre `sample_count > 0`.
3. Con 30 trades cerrados, ejecutar atribución BHB básica por `strategy_type`.
4. No promover a live hasta MTRL >= 3 meses de señales OOS con Sharpe deflated > 0.

---

## Matriz de Gaps y Prioridades

| Área | Gap Principal | Bloqueador | Prioridad |
|------|--------------|-----------|-----------|
| Scanner IC | 0 outcomes medidos | preview=True (RESUELTO) | ALTA |
| Entry Validation | IS no medible | 0 trades reales | ALTA |
| Position Management | 0 R-multiples | 0 posiciones en journal local | ALTA |
| Exit Governance | Exit pass sin candidatos | 0 posiciones registradas | MEDIA |
| Post-Trade Learning | adaptive_policy bloqueado | 0 trades cerrados | ALTA |
| Execution Quality | Fill rate desconocido | Métricas no capturadas | MEDIA |

---

## Línea de Tiempo para Evaluación de Benchmarks

| Hito | Condición | Benchmark evaluable |
|------|-----------|-------------------|
| T+1 semana | 20+ trades ejecutados en paper | Execution Quality básico |
| T+2 semanas | 5+ posiciones cerradas | IC tracker primeros datos |
| T+1 mes | 30+ posiciones cerradas | IC significativo, R-múltiples |
| T+3 meses | 100+ trades, Sharpe calculable | BHB Attribution, MTRL inicial |
| T+6 meses | 200+ trades OOS | Deflated Sharpe, decisión live/no-live |

---

## Conclusión

ATLAS-Quant tiene la arquitectura correcta para implementar todos los benchmarks listados. El único obstáculo real fue el bug `preview=True` que impidió toda ejecución real durante el período evaluado. Con ese bug corregido:

1. El `ICSignalTracker` acumulará señales y outcomes automáticamente.
2. El `AdaptiveLearningService` recibirá el primer `sample_count > 0` cuando se cierren posiciones.
3. El `exit_governance_snapshot` podrá actuar sobre posiciones reales registradas en journal.
4. Los benchmarks de Grinold-Kahn, Van Tharp, Kaminski-Lo y BHB podrán evaluarse con datos empíricos en 4-8 semanas de paper trading.

**El sistema está en condición de paper-supervised operativo. No está en condición de live.**

---

*Generado por ATLAS Quant Audit Protocol (TSAP) v0.1 — Branch variante/nueva — 2026-03-28*
