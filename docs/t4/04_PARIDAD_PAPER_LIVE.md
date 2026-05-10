# Paridad Paper/Live — diseño y enforcement

## 1. Principio rector

> El **único** elemento del dashboard que distingue paper de live es el `ConnectionToggle`. Las pantallas, módulos, métricas, atajos, formularios, validaciones, telemetría y journal son **idénticos en estructura**.

La diferenciación visual (badge `LIVE`, color de borde rojo, banner sticky) es **decoración de seguridad**, no estructura.

## 2. Implementación: cómo se enforce

### 2.1 Una sola jerarquía de rutas en el dashboard

```
app/
  layout.tsx              ← header global + ConnectionToggle + LIVE badge si scope=live
  page.tsx                ← redirect /trade
  trade/
    page.tsx              ← cockpit principal (book, ticket, posiciones, riesgo)
  audit/page.tsx
  account/page.tsx
  monitor/page.tsx        ← scanner + señales activas
  settings/page.tsx
  api/                    ← opcional Next.js API; preferimos llamar al t4_robot directo
```

> No existe `/paper/*` ni `/live/*`. El `scope` es **estado del cliente**, alimentado por `t4.connection` + `t4.runtime.mode`.

### 2.2 Una sola fuente de payloads

Todos los componentes consumen el mismo schema (`02_CONTRATOS_DE_DATOS.md`). En paper, el origen del payload es el `T4Adapter` apuntando al T4 Simulator. En live, apunta a T4 Live. **Mismos campos, mismas unidades, misma cadencia.**

### 2.3 Snapshot tests de paridad

Para cada componente UI, mantenemos:
- `__snapshots__/<Component>.paper.json`
- `__snapshots__/<Component>.live.json`

Y test:
```ts
expect(stripDecorations(paperSnapshot)).toEqual(stripDecorations(liveSnapshot));
```
donde `stripDecorations` elimina campos puramente visuales (`badge_text`, `border_color`, `account_scope` literal). Si el shape difiere, falla el test y bloquea merge.

### 2.4 Aislamiento por `account_scope`

| Recurso | Aislamiento |
|---|---|
| Postgres `audit`, `orders`, `fills`, `positions` | columna `account_scope` + índice; vistas filtradas |
| Redis topics | mismos canales; cada mensaje lleva `account_scope`; consumidores filtran |
| Journal JSONL | un archivo por scope (`logs/t4_audit.paper.jsonl`, `logs/t4_audit.live.jsonl`) |
| Métricas Prometheus | label `account_scope` siempre presente |
| Telegram alertas | mismo bot; prefijo `[PAPER]`/`[LIVE]` automático |
| State bus → atlas-core | mismo bus; eventos con `account_scope` |
| Modelos ML | mismos artefactos; el contexto de inferencia recibe `account_scope` como feature solo si el modelo lo admite (default: NO se usa para no contaminar) |

### 2.5 Sin "modo demo" de UI

El dashboard **no tiene** valores hardcoded de cuenta. Si no hay conexión: estado `Disconnected`, todas las pantallas presentes con shape vacío, controles inhabilitados con tooltip explicativo. Esto garantiza que paper y live se comporten igual en arranque.

## 3. El `ConnectionToggle`

```tsx
type Target = "simulator" | "live";

function ConnectionToggle() {
  const [target, setTarget] = useState<Target>("simulator");
  const [switchState, setSwitchState] = useLiveSwitchState();   // hooked to t4.runtime.mode

  const onToggle = async (next: Target) => {
    if (next === "live" && !switchState.effective_live_enabled) {
      showBlockingReasons(switchState.blocking_reasons);
      return;
    }
    await api.post("/connection/connect", { target: next });
  };
  // …
}
```

- En `simulator`: conecta directo, sin confirmación adicional.
- En `live`: 4 gates en orden:
  1. `LiveSwitchState.effective_live_enabled === true` (servidor).
  2. Modal de revisión que muestra equity, margen, posiciones abiertas (debe ser 0).
  3. Captcha de teclado (escribir `CONFIRMO TRADING REAL EN <símbolo>`).
  4. Re-validación servidor con header `X-Live-Confirm` (60 s, one-shot).
- Una vez en `live`: el banner sticky aparece, el borde global pasa a rojo (#C0212F), todas las acciones destructivas piden segunda confirmación.

## 4. Adaptación al capital existente

El sizing **siempre** se calcula así (ambos modos):

```python
def size_from_equity(
    *,
    equity: float,
    available_margin: float,
    instrument: InstrumentSpec,
    risk_budget_pct: float,         # ej. 0.005 (0.5%) en SUPERVISED_LIVE
    stop_ticks: int,
    kelly_fraction_cap: float = 0.02,
) -> int:
    max_loss_usd = equity * risk_budget_pct
    loss_per_contract = stop_ticks * instrument.tick_value_usd
    by_risk = math.floor(max_loss_usd / loss_per_contract)
    by_margin = math.floor(available_margin / instrument.initial_margin_intraday)
    by_kelly = math.floor((equity * kelly_fraction_cap) / instrument.initial_margin_intraday)
    return max(0, min(by_risk, by_margin, by_kelly, MAX_CONTRACTS_PER_INSTRUMENT))
```

`equity`, `available_margin` y `initial_margin_intraday` se actualizan en tiempo real desde `t4.account.snapshot`. El user puede ver la fórmula resuelta en `RiskHUD` (anti-caja-negra).

## 5. Tests obligatorios para sellar paridad

Ver `06_TESTS_Y_QA.md` §3.
