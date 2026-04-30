from __future__ import annotations

from prometheus_client import REGISTRY

from atlas_code_quant.local_models.metrics import record_local_model_result


def _sample_lines(metric_name: str) -> list[str]:
    lines: list[str] = []
    for family in REGISTRY.collect():
        for sample in family.samples:
            if sample.name == metric_name:
                labels = ",".join(f'{k}="{v}"' for k, v in sorted(sample.labels.items()))
                lines.append(f"{sample.name}{{{labels}}} {sample.value}")
    return lines


def test_local_model_metrics_include_requested_dimensions():
    record_local_model_result(
        provider_role="classifier",
        requested_model="llama3.1:8b",
        used_fallback=False,
        fallback_reason=None,
        outcome="success",
        duration_ms=12.3,
    )
    record_local_model_result(
        provider_role="classifier",
        requested_model="llama3.1:8b",
        used_fallback=True,
        fallback_reason="ollama_unavailable:connection refused",
        outcome="fallback",
        duration_ms=None,
    )
    rows = _sample_lines("atlas_local_model_requests_total")
    assert any('provider_role="classifier"' in row for row in rows)
    assert any('requested_model="llama3.1:8b"' in row for row in rows)
    assert any('used_fallback="true"' in row for row in rows)
    # normalized category, not full exception payload
    assert any('fallback_reason="ollama_unavailable"' in row for row in rows)
