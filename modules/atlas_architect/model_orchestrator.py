from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class ModelOutput:
    ok: bool
    text: str
    model_routing: Dict[str, Any]


class MultiModelOrchestrator:
    """Orquestación multi-modelo para tareas de edición pesada.

    Implementación:
    - Reutiliza `modules.humanoid.ai.router.route_and_run` (free-first).
    - Si el Owner habilita pago y hay API key, puede usar modelos externos via override.
    """

    def __init__(self) -> None:
        pass

    def run(self, prompt: str, intent_hint: str = "code", prefer_free: bool = True, timeout_s: int = 60) -> ModelOutput:
        try:
            from modules.humanoid.ai.router import route_and_run

            out, decision, meta = route_and_run(
                prompt,
                intent_hint=intent_hint,
                prefer_free=prefer_free,
                timeout_s=int(timeout_s or 60),
            )
            mr = {
                "provider_id": decision.provider_id,
                "model_key": decision.model_key,
                "route": decision.route,
                "reason": decision.reason,
                "latency_ms": meta.get("latency_ms", 0),
                "used_fallback": meta.get("used_fallback", False),
            }
            return ModelOutput(ok=bool(out), text=(out or "").strip(), model_routing=mr)
        except Exception as e:
            return ModelOutput(ok=False, text=str(e), model_routing={"provider_id": "error", "error": str(e)})

