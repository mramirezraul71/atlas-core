# Contrato Radar Institucional → Code Quant

**Versión:** 2026-05-01  
**Contexto:** Tubería objetivo Radar → Quant → selector → OperationCenter → Tradier PAPER. Este documento fija el payload mínimo de “activo que cumple criterios”.

## Rutas (dos modos compatibles)

| Modo | Origen | Destino / uso |
|------|--------|----------------|
| A — Legacy bridge | PUSH 8791 vía `radar_quant_client` | Quant `GET /scanner/report` |
| B — Arquitectura objetivo | PUSH `GET /api/radar/opportunities` | Quant intake (`RadarOpportunityClient` → `RadarOpportunity`) |

Recomendación: **B** como destino final; **A** se mantiene para dashboards que aún lean `/scanner/report`.

## Payload JSON (oportunidad normalizada)

Campos obligatorios / esperados por el intake y el mapper hacia candidatos Quant:

```json
{
  "symbol": "SPY",
  "asset_class": "optionable_equity",
  "direction": "long",
  "selection_score": 82.4,
  "signal_strength_pct": 77.1,
  "timeframe": "intraday",
  "price": 512.32,
  "has_options": true,
  "criteria_passed": [
    "liquidity",
    "trend",
    "volume",
    "volatility",
    "options_available",
    "risk_allowed"
  ]
}
```

Extensiones frecuentes (van en `payload` al parsear con `RadarOpportunity.from_dict`, luego se exponen vía `to_dict()` / `serialize_opportunity`):

- `volume`, `order_flow`, `confirmation`, `iv_rank`, `liquidity_score`, `options_thesis`
- Campos técnicos: `macd_hist`, `bb_pct`, `rsi`, `vix_context`, `strategy_label`, `strategy_key`

## Serialización (slots / dataclass / Pydantic)

`RadarOpportunity` usa `@dataclass(slots=True)`: **no** usar `.__dict__`. Usar:

- `opportunity.to_dict()`, o
- `atlas_code_quant.intake.serialization.serialize_opportunity(obj)`

## Variables de entorno (cámara fuera del camino crítico)

Si la cámara está deshabilitada a propósito, no debe generar `CAMERA_UNAVAILABLE` crítico en el radar institucional:

- `ENABLE_CAMERA=false` | `ATLAS_CAMERA_ENABLED=false` | `ATLAS_VISION_CAMERA_ENABLED=false`

## Modos de ejecución paper (recordatorio)

- `paper_supervised`: preview / sin submit automático.
- `paper_autonomous`: submit paper (con `ATLAS_TRADIER_DRY_RUN=true` y órdenes virtuales según política).

## Implementación reciente (worktree `\_wt_feature_atlas_codequant_f3`)

- `intake/opportunity.py`: `RadarOpportunity.to_dict()`
- `intake/serialization.py`: `serialize_opportunity()`
- `api/main.py`: `RadarScannerAdapter.report()` usa serialización segura; `_to_candidate` reenvía campos de mercado opcionales.
- `atlas_adapter/routes/radar_quant_mapper.py`: omite degradación de cámara si está intencionalmente apagada.

Fusionar este worktree a la rama principal cuando valides en runtime (8791 / 8795).

## Implementación motor unificado (2026-05-01)

- `atlas_code_quant.backtest.unified_engine`: LEAN CLI (`ATLAS_LEAN_USE_CLI`) → adapter F13 → degradación GBM (`ATLAS_LEAN_DEGRADE_TO_GBM`).
- Tests: `tests/atlas_code_quant/test_plan_radar_lean_pipeline.py`.
