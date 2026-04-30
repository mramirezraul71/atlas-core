# ATLAS Quant - Auditoria de Entry Validation

**Fecha:** 2026-03-28
**Repositorio:** `C:\ATLAS_PUSH`
**Fase activa del proceso:** `entry_validation`
**Fase previa endurecida:** `scanner_selection`

---

## 1. Dictamen ejecutivo

ATLAS ya tenia una capa de `entry_plan`, `camera_plan` y `playbook`, pero esa capa describia como deberia verse una entrada sana sin verificar con dureza si todavia lo era en el instante del ticket.

El hueco principal era este:

- el sistema no comparaba el precio de decision con el precio realmente ejecutable justo antes del submit;
- no convertia spread y deriva adversa en gates duros;
- la validacion visual ayudaba a contexto, pero no sustituia microestructura ni costo de ejecucion.

Ese hueco puede explicar parte del daño observado en paper:
ATLAS podia elegir un candidato razonable en el scanner y aun asi entrar tarde, con spread malo o con edge ya erosionado.

---

## 2. Hallazgos en codigo

### 2.1 Lo que si existia

- `strategy_selector.py` generaba `entry_plan`, `camera_plan` y `playbook`.
- `operation_center.py` ya validaba reconciliacion, reentrada, `strategy_type`, PDT y kill switch.
- `sensor_vision.py` permitia contexto visual y captura operativa.

### 2.2 Lo que faltaba

- `OrderRequest` no cargaba `entry_reference_price`.
- `OperationCenter` no comparaba `decision price` vs `basis price` de mercado.
- No habia veto por `spread_pct` ni por `adverse_drift_pct`.
- El auto-cycle no transportaba al submit el precio base del scanner.

---

## 3. Benchmark externo resumido

La referencia seria fuera de ATLAS no trata la entrada solo como “setup correcto”.
La trata tambien como problema de ejecucion y costo:

- el benchmark de `implementation shortfall` usa el precio de decision como referencia previa a la ejecucion;
- la calidad de ejecucion mira spread, probabilidad de mejora de precio y probabilidad de ejecucion;
- en la practica, una entrada puede seguir siendo mala aunque la tesis siga siendo buena, si el costo de entrar ya se comio demasiado del movimiento esperado.

---

## 4. Causa raiz

La causa raiz de esta etapa no era falta de una tesis de entrada.
Era falta de una compuerta entre:

- la tesis elegida por scanner/selector,
- y la realidad del precio ejecutable al momento del ticket.

En otras palabras:
ATLAS sabia que queria comprar o vender, pero no verificaba con suficiente rigor si todavia debia hacerlo a ese precio.

---

## 5. Correccion P0 aplicada

Se implemento una primera capa dura de `entry_validation` para entradas de equity:

- `OrderRequest` ahora puede transportar:
  - `entry_reference_price`
  - `entry_expected_move_pct`
  - `entry_confidence_reference_pct`
  - `max_entry_drift_pct`
  - `max_entry_spread_pct`
- `strategy_selector.py` ahora mete esos datos en `entry_plan` y `order_seed`.
- `api/main.py` hace que el auto-cycle herede `price`, `predicted_move_pct` y `local_win_rate_pct` del scanner.
- `operation_center.py` ahora consulta quote y mide:
  - `basis_price`
  - `spread_pct`
  - `signed_drift_pct`
  - `adverse_drift_pct`
  - `drift_vs_expected_move_pct`
- si la deriva adversa o el spread superan umbrales, el sistema bloquea `preview/submit`.

---

## 6. Lo que sigue

La fase `entry_validation` no termina con este `P0`.
Los siguientes pasos serios son:

- medir slippage realizado por familia y timeframe;
- comparar `entry_reference_price` vs fill real en journal;
- añadir validacion de liquidez minima y volumen reciente;
- extender el mismo criterio a estructuras de opciones y spreads;
- correlacionar entradas tardias con perdidas posteriores por setup.

---

## 7. Estado dentro del algoritmo maestro

El algoritmo abierto de ATLAS queda asi:

- `scanner_selection`: baseline endurecida
- `entry_validation`: baseline endurecida
- `execution_quality`: foco activo
- `position_management`: pendiente
- `exit_governance`: pendiente
- `post_trade_learning`: pendiente de enriquecimiento continuo
