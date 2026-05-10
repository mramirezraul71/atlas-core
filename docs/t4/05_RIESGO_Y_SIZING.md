# Riesgo y sizing

## 1. Tres capas (anexo §3.5)

### Capa 1 — por trade
- `risk_per_trade ≤ 1% equity` (`PAPER_AGGRESSIVE`), `≤ 0.5%` (`SUPERVISED_LIVE`), `≤ 0.75%` (`GUARDED_LIVE`).
- Stop ATR(14) × multiplicador por régimen (HMM).
- Take-profit por trailing o señal contraria.

### Capa 2 — portfolio
- Exposure cap por correlación (matriz pre-computada con histórico 60 d, refresco diario).
- VaR 1d 95 %: alerta al 80 % del budget, bloqueo al 100 %.
- Margen utilizado ≤ 70 % del available_margin (deja colchón anti-margin-call).

### Capa 3 — sesión (circuit breakers)
- Daily loss lock: −3 % en `PAPER_AGGRESSIVE`, −1 % en `SUPERVISED_LIVE`.
- Auto-flatten 15 min antes de cierre CME por contrato.
- Fat-finger: rechazo si `size > 10 × avg_recent_size`.
- Price-band: ±2 % del last; **además** banda dinámica = ±2 × ATR(20).
- Throttle: ≤ 10 órdenes/s por cuenta.

## 2. Reglas duras

Implementadas en `t4_risk.py` y verificadas en pre-trade:

```python
@dataclass(frozen=True)
class HardLimits:
    max_position_per_instrument: int       # 10 default
    daily_loss_pct: float                  # 0.005 in SUPERVISED_LIVE
    margin_usage_max_pct: float            # 0.70
    order_throttle_per_s: int              # 10
    price_band_pct: float                  # 0.02
    price_band_atr_mult: float             # 2.0
    require_stop_loss: bool                # True
    require_take_profit: bool              # True
    block_on_circuit_breaker: bool         # True
    block_on_kill_switch: bool             # True
    block_on_data_lag_ms: int              # 250
    block_on_runtime_modes: list[RuntimeMode]
```

`HardLimits` se carga por perfil desde Doppler (`atlas-t4/configs/<env>/hard_limits.yaml`).

## 3. Sizing — fórmula canónica

Ver `04_PARIDAD_PAPER_LIVE.md §4`. Vive en `t4_risk.size_from_equity()` y es **idéntica en paper y live**. La única diferencia: los inputs (`equity`, `available_margin`) vienen de `t4.account.snapshot` de la cuenta activa.

## 4. Trazabilidad

Cada decisión de riesgo genera un evento `t4.risk.decision` con:
- inputs (equity, margin, ATR, régimen),
- fórmula desplegada,
- resultado (size, max_loss),
- bloqueos (lista vacía si aprobada).

El componente UI `RiskHUD` renderiza la última decisión en directo (sin caja negra).

## 5. Kill switch

Reutiliza `atlas_code_quant/operations/kill_switch.py` + extensión T4:

1. Cancela **todas** las órdenes vivas en T4 (mass-cancel WS).
2. Aplana todas las posiciones (market orders contrarias).
3. Bloquea `RuntimeMode` ↓ `PAPER_BASELINE`.
4. Pública `t4.audit` con `action=kill_switch_triggered`, `trace_id` y stack del trigger.
5. Solo `operator` con 2FA puede `kill_switch.reset` tras revisión.

Triggers automáticos:
- desconexión `trade_ws > 5 s`,
- `daily_drawdown_pct ≥ daily_loss_pct`,
- `data_lag_ms > 1000` durante > 3 s,
- `hash_chain_audit_invalid` (manipulación detectada),
- comando `POST /kill_switch`.
