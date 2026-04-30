# Local Models (Ollama) for ATLAS Code Quant

This module integrates local Ollama models as **assistive components** without changing deterministic trading core behavior.

## Role mapping

- `embeddings` -> `nomic-embed-text:latest`
  - Journal embeddings, semantic retrieval, incident/note search.
- `classifier` -> `llama3.1:8b`
  - Lightweight event/journal classification and short summaries.
- `coder_primary` -> `deepseek-coder-v2:16b`
- `coder_fallback` -> `qwen2.5-coder:7b`
- `coder_premium` -> `qwen3-coder:30b`
  - Exposed as profile only (not executing code automatically).
- `vision` -> `llama3.2-vision:11b`
- `vision_premium` -> `qwen3-vl:30b`
  - Screenshot analysis.

## Configuration

Default config is in `config.py` under `LOCAL_MODEL_CONFIG`.

Env overrides:

- `ATLAS_LOCAL_MODELS_ENABLED=true|false`
- `ATLAS_OLLAMA_BASE_URL=http://localhost:11434`
- `ATLAS_OLLAMA_TIMEOUT_SECONDS=20`
- `ATLAS_LOCAL_MODEL_EMBEDDINGS=...`
- `ATLAS_LOCAL_MODEL_CLASSIFIER=...`
- `ATLAS_LOCAL_MODEL_CODER_PRIMARY=...`
- `ATLAS_LOCAL_MODEL_CODER_FALLBACK=...`
- `ATLAS_LOCAL_MODEL_CODER_PREMIUM=...`
- `ATLAS_LOCAL_MODEL_VISION=...`
- `ATLAS_LOCAL_MODEL_VISION_PREMIUM=...`

## Safe degradation behavior

- If Ollama is down/unreachable, providers use graceful fallback behavior.
- System emits operational events:
  - `local_model_call`
  - `local_model_failure`
  - `local_model_fallback`
- Prometheus metrics (aggregated):
  - `atlas_local_model_requests_total{provider_role,requested_model,used_fallback,fallback_reason,outcome}`
  - `atlas_local_model_request_duration_ms{provider_role,requested_model,used_fallback}`

## Operational interpretation (fallback/degradation)

- **Fallback rate (per role/model):**
  - `sum(rate(atlas_local_model_requests_total{used_fallback="true"}[5m])) / sum(rate(atlas_local_model_requests_total[5m]))`
- **Healthy baseline:** fallback near 0 during normal Ollama availability.
- **Degraded mode:** sustained fallback > 0 indicates provider is operating with graceful degradation.
- **Hard failure signal:** high `outcome="failure"` (with low or no fallback) indicates upstream client/transport issue before provider-level fallback path.
- `fallback_reason` is normalized to low-cardinality categories (example: `provider_disabled`, `ollama_unavailable`, `file_not_found`, `invalid_json_response`).

## Quick health test

```bash
curl http://localhost:11434/api/tags
```

## Minimal usage examples

```python
from atlas_code_quant.local_models.router import get_embedding_provider
vectors = get_embedding_provider().embed_text(["journal text A", "journal text B"])
```

```python
from atlas_code_quant.local_models.integrations import classify_journal_event
review = classify_journal_event({"event_type": "entry_blocked", "reason": "insufficient_capital"})
```

```python
from atlas_code_quant.local_models.integrations import analyze_dashboard_screenshot
result = analyze_dashboard_screenshot("snapshots/dash.png")
```

## Production paper activation recommendation

1. Activate `embeddings` first (lowest operational risk).
2. Activate `classifier` for low-cost observability enrichment.
3. Activate `vision` after screenshot pipeline is stable.
4. Keep trading decisions deterministic (no model-gated entry/exit decisions).
