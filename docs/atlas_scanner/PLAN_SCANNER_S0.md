# PLAN SCANNER S0 (Aprobado)

Este plan define la ejecucion de S0 para `atlas_scanner` en la rama
`feat/atlas-scanner-s0-scaffold`, con `main` congelada (sin cambios).

## 1) Objetivo de S0

Construir la base del modulo `atlas_scanner` con contratos estables y
arquitectura clara, sin feeds reales ni ejecucion de ordenes.

Resultado esperado:
- Modulo `atlas_scanner` creado como hermano de `atlas_push`.
- Contratos de datos S0 implementados en codigo y tipados.
- Configuracion de scoring disponible via `SCORING_CONFIG`.
- Jerarquia de excepciones base lista para S1.
- Tests unitarios S0 verdes.
- Documentacion S0 consistente con contratos implementados.

## 2) Naming final unico (definitivo)

No se permiten alternativas de nombre en S0. Los nombres canonicos son:
- `ScoreComponent`
- `ScoreBreakdown`
- `ScannerRunMetrics`
- `ScannerRunResult`

## 3) Alcance S0 (IN / OUT)

Incluido:
- Estructura de paquete `atlas_scanner`.
- `config.py` con dataclasses de thresholds/scoring y `SCORING_CONFIG`.
- `exceptions.py` con jerarquia base.
- `models/*.py` con contratos S0 en dataclasses `frozen=True`.
- Exportes en `atlas_scanner/__init__.py` y `atlas_scanner/models/__init__.py`.
- Tests unitarios de contratos/config/excepciones.

Excluido:
- Providers en tiempo real.
- Integracion con broker/ordenes.
- Persistencia productiva o scheduler.
- Integracion profunda con DecisionEngine (solo contratos listos).

## 4) Contratos canonicos minimos S0

Base documental:
- `ATLAS_SCANNER_CONTRACTS.md`
- `ATLAS_SCANNER_OVERVIEW.md`
- `ATLAS_SCANNER_ARCHITECTURE.md`
- `ATLAS_SCANNER_PHASES.md`

### 4.1 `SymbolSnapshot` (minimo)
- `symbol: str`
- `asset_type: str`
- `base_currency: str`
- `ref_price: float`
- `volatility_lookback: float`
- `liquidity_score: float`
- `meta: dict[str, object]`

### 4.2 `ScanSnapshot` (minimo)
- `snapshot_id: str`
- `created_at: str` (ISO8601 UTC)
- `universe_name: str`
- `symbols: tuple[SymbolSnapshot, ...]`
- `config_version: str`
- `meta: dict[str, object]`

### 4.3 `CandidateOpportunity` (minimo)
- `candidate_id: str`
- `snapshot_id: str`
- `symbol: str`
- `direction: str` (`long` / `short`)
- `thesis: str`
- `score: float`
- `score_breakdown: ScoreBreakdown`
- `time_horizon_minutes: int`
- `max_risk_pct: float`
- `expected_rr: float`
- `tags: tuple[str, ...]`
- `meta: dict[str, object]`

### 4.4 `ScannerRunMetrics` (minimo)
- `total_symbols: int`
- `evaluated_symbols: int`
- `generated_candidates: int`
- `avg_score: float`
- `p95_score: float`
- `error_count: int`
- `warnings: tuple[str, ...]`

### 4.5 `ScannerRunResult` (minimo)
- `snapshot_id: str`
- `started_at: str` (ISO8601 UTC)
- `finished_at: str` (ISO8601 UTC)
- `candidates: tuple[CandidateOpportunity, ...]`
- `metrics: ScannerRunMetrics`
- `meta: dict[str, object]`

Nota de contrato S0 sobre `meta`:
- Aunque los modelos sean `@dataclass(frozen=True)`, en S0 se acepta
  `meta: dict[str, object]` por compatibilidad con JSON y simplicidad de
  integracion. Si se requiere endurecer la inmutabilidad interna del payload
  (`mappingproxy`, estructuras inmutables profundas, etc.), se evaluara en S1/S2.

## 5) Estructura objetivo en repo

Ruta base:
- `atlas_scanner/`

Archivos S0:
- `atlas_scanner/__init__.py`
- `atlas_scanner/config.py`
- `atlas_scanner/exceptions.py`
- `atlas_scanner/models/__init__.py`
- `atlas_scanner/models/symbol_snapshot.py`
- `atlas_scanner/models/scan_snapshot.py`
- `atlas_scanner/models/score_breakdown.py`
- `atlas_scanner/models/candidate.py`
- `atlas_scanner/models/scanner_metrics.py`
- `atlas_scanner/models/scanner_result.py`

Tests S0:
- `tests/unit/atlas_scanner/test_models_contracts.py`
- `tests/unit/atlas_scanner/test_scoring_config.py`
- `tests/unit/atlas_scanner/test_exceptions.py`

## 6) Reglas tecnicas S0

- `from __future__ import annotations` en modulos de contratos.
- Dataclasses inmutables: `@dataclass(frozen=True)`.
- Sin side effects en import.
- Sin dependencias pesadas en S0.
- Defaults explicitos y validaciones ligeras en `__post_init__` si aplica.
- Compatibilidad S1-S3 sin romper contratos base.

## 7) Estructura de commits S0 (fija)

Secuencia estable y obligatoria:
1. **S0.1** docs/plan en disco.
2. **S0.2** config + exceptions.
3. **S0.3** models/contracts.
4. **S0.4** tests S0.
5. **S0.5** ajustes finales de docs (si hacen falta).

Regla:
- Commits atomicos, uno por bloque.
- No mezclar varios bloques S0 en un mismo commit.

## 8) Criterios de aceptacion S0 (checklist verificable)

- [x] Import limpio de `atlas_scanner`.
- [x] Dataclasses de contratos con `frozen=True`.
- [x] `SCORING_CONFIG` accesible desde `atlas_scanner.config`.
- [x] Tests S0 en verde.
- [x] `main` sin tocar; todo en `feat/atlas-scanner-s0-scaffold`.

## 9) Protocolo operativo acordado

1. Plan en disco.
2. Revision/aprobacion del plan.
3. Implementacion por commits atomicos S0.2-S0.5.
4. Verificacion de tests por bloque.
5. Cero cambios en `main`.

Estado de este documento:
- Version: `S0-plan-v2`
- Rama: `feat/atlas-scanner-s0-scaffold`
- Fecha: `2026-04-22`

