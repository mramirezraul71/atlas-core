from __future__ import annotations
import time
import httpx
from typing import Optional, Dict, Any

class OllamaClient:
    def __init__(self, base_url: str, timeout_connect: int = 5, timeout_read: int = 120) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout = httpx.Timeout(timeout_read, connect=timeout_connect)

    def generate(
        self,
        model: str,
        prompt: str,
        system: Optional[str] = None,
        temperature: float = 0.2,
        top_p: float = 0.9,
        stream: bool = False,
        max_tokens: Optional[int] = None,
        timeout_override: Optional[int] = None,
    ) -> Dict[str, Any]:
        url = f"{self.base_url}/api/generate"
        options: Dict[str, Any] = {"temperature": temperature, "top_p": top_p}
        if max_tokens is not None:
            options["num_predict"] = max_tokens
        payload: Dict[str, Any] = {
            "model": model,
            "prompt": prompt,
            "stream": stream,
            "options": options,
        }
        if system:
            payload["system"] = system

        t0 = time.perf_counter()
        timeout = httpx.Timeout(timeout_override) if timeout_override is not None else self.timeout
        with httpx.Client(timeout=timeout) as client:
            r = client.post(url, json=payload)
            r.raise_for_status()
            data = r.json()

        data["_ms"] = int((time.perf_counter() - t0) * 1000)
        return data
