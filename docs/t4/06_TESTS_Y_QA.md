# Plan de tests y QA

## 1. Pirámide de tests

```
       ┌─────────────────────┐
       │   E2E (Playwright)  │   ← UI + servicio + simulator   (≤ 30)
       ├─────────────────────┤
       │   Integration       │   ← adapter ↔ simulator T4      (≈ 150)
       ├─────────────────────┤
       │   Component (UI)    │   ← React Testing + snapshots   (≈ 400)
       ├─────────────────────┤
       │   Unit              │   ← lógica pura (Python + TS)   (≈ 1500)
       └─────────────────────┘
```

## 2. Cobertura mínima por capa

| Capa | Cobertura mínima | Herramientas |
|---|---|---|
| codec.py (Protobuf) | 100% líneas | pytest + golden fixtures |
| ws_client.py (reconnect, backoff, heartbeat) | 95% | pytest + asyncio + ws server fake |
| order_manager.py | 100% sobre paths de éxito y rechazo | pytest |
| risk_hooks.py / t4_risk.py | 100% | pytest + property-based (hypothesis) |
| api_server.py | 90% | pytest + httpx |
| Frontend componentes críticos (Toggle, KillSwitch, OrderTicket, RiskHUD) | 100% | Vitest + RTL |
| Snapshots paridad | exhaustivo (todos los componentes que pintan datos) | Vitest snapshot |

## 3. Tests **obligatorios** de paridad Paper/Live

### 3.1 Estructural (snapshot)
Para cada componente listado:
```ts
test("paridad UI Paper vs Live: <Component>", async () => {
  const paper = renderWith({ accountScope: "paper", payload: fixturePaper });
  const live  = renderWith({ accountScope: "live",  payload: fixtureLive });
  expect(stripDecorations(paper.toJSON())).toEqual(stripDecorations(live.toJSON()));
});
```
Componentes: `DepthLadder`, `MBOPanel`, `OrderTicket`, `PositionGrid`, `RiskHUD`, `EquityCurve`, `AuditTimeline`, `AccountSummary`, `OrderBlotter`, `KillSwitchButton`.

### 3.2 Funcional (E2E)
- Conectar a simulator → emitir orden MES → ver fill → cerrar → verificar journal.
- Conectar a live (dry-run: el adapter envía a `T4_LIVE_HOST=simulator` también) → mismo flujo → comparar payloads bit-a-bit con scope paper.

### 3.3 Aislamiento (no efectos cruzados)
```python
def test_isolation_no_cross_scope(t4_robot):
    t4_robot.connect("paper")
    t4_robot.place_order(symbol="MESM6", side="buy", qty=1)
    paper_journal = read_jsonl("logs/t4_audit.paper.jsonl")
    live_journal  = read_jsonl("logs/t4_audit.live.jsonl")
    assert len(live_journal) == 0
    assert any(e["action"]=="order_intent" for e in paper_journal)

    t4_robot.disconnect(); t4_robot.connect("live", dry_run=True)
    t4_robot.place_order(symbol="MESM6", side="buy", qty=1)
    live_journal  = read_jsonl("logs/t4_audit.live.jsonl")
    paper_journal_after = read_jsonl("logs/t4_audit.paper.jsonl")
    assert len(live_journal) == 1
    assert len(paper_journal) == len(paper_journal_after)
```

### 3.4 Sizing por capital
```python
@pytest.mark.parametrize("equity,available_margin,expected_size", [
    (4500, 4350, 1),     # MES margen 50, stop 8 ticks * 1.25 = $10 loss, budget 0.5% = $22.5 → 2 contratos por riesgo, 87 por margen → cap kelly
    (1000, 950, 0),      # equity insuficiente para margen
    (50000, 48000, 10),  # techo MAX_CONTRACTS_PER_INSTRUMENT
])
def test_sizing_from_equity(...):
    ...
```

### 3.5 Audit hash chain
```python
def test_audit_chain_integrity():
    events = generate_random_events(1000)
    chain = build_audit_chain(events)
    assert verify_chain(chain) is True
    # tampering
    chain[500]["details"]["size"] += 1
    assert verify_chain(chain) is False
```

## 4. Fixtures de Protobuf (golden)

Por cada `.proto` message:
- `tests/fixtures/proto/<MessageName>/01.bin` — payload binario real capturado del simulator
- `tests/fixtures/proto/<MessageName>/01.expected.json` — modelo interno esperado tras `codec.decode`
- Test: `decode(bin) == expected` y `encode(expected) == bin` (round-trip).

## 5. Pipeline CI

GitHub Actions (`.github/workflows/t4_robot.yml`):
1. Lint Python (`ruff`, `mypy --strict`).
2. Lint TS (`eslint`, `tsc --noEmit`).
3. Unit tests Python + TS.
4. Integration tests contra **simulator stub** (no red real).
5. Build Docker image `t4_robot`.
6. Build dashboard estático.
7. **Job nocturno** (cron): integration real contra T4 Simulator con credenciales de demo (separadas en secret repo, no commit).
8. Bloqueo de merge: cobertura paridad 100 % en componentes listados §3.1.

## 6. Smoke en producción

Tras cada deploy:
- `GET /t4/v1/ready` → 200 en < 5 s.
- WS abierto a `/t4/v1/ws`, recepción de `_sub_ack` en < 2 s.
- Suscripción a `t4.feed.quote` para MES → al menos 1 quote en < 10 s.
- `t4.account.snapshot` recibido en < 5 s.
- Resultado publicado en `t4.audit` con `action=deploy_smoke`.
