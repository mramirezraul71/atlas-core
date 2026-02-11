from __future__ import annotations
import time
import math
from typing import Dict, Any
from .config import LLMSettings
from .router import HybridRouter
from .ollama_client import OllamaClient
from .schemas import LLMRequest, LLMResponse

class LLMService:
    def __init__(self) -> None:
        self.settings = LLMSettings()
        self.router = HybridRouter(
            fast_max_chars=self.settings.FAST_MAX_CHARS,
            chat_max_chars=self.settings.CHAT_MAX_CHARS,
            reason_min_chars=self.settings.REASON_MIN_CHARS,
        )
        self.client = OllamaClient(
            base_url=self.settings.OLLAMA_BASE_URL,
            timeout_connect=self.settings.TIMEOUT_CONNECT,
            timeout_read=self.settings.TIMEOUT_READ,
        )

    def run(self, req: LLMRequest) -> LLMResponse:
        t0 = time.perf_counter()
        try:
            decision = self.router.decide(req.prompt, forced=req.route)
            model = req.model or self.settings.model_for_route(decision.route)

            data = self.client.generate(
                model=model,
                prompt=req.prompt,
                system=req.system,
                temperature=req.temperature or 0.2,
                top_p=req.top_p or 0.9,
                stream=bool(req.stream),
            )

            output = data.get("response", "") or ""
            ms_ollama = int(data.get("_ms", 0))
            ms_total = int((time.perf_counter() - t0) * 1000)
            tokens_est = max(1, math.ceil(len(req.prompt) / 4))

            meta: Dict[str, Any] = {"decision_reason": decision.reason, "ollama_ms": ms_ollama}

            return LLMResponse(
                ok=True,
                output=output,
                route=decision.route,
                model_used=model,
                ms=ms_total,
                tokens_est=tokens_est,
                error=None,
                meta=meta,
            )
        except Exception as e:
            ms_total = int((time.perf_counter() - t0) * 1000)
            return LLMResponse(
                ok=False,
                output="",
                route=req.route,
                model_used=req.model,
                ms=ms_total,
                tokens_est=None,
                error=str(e),
                meta={},
            )
