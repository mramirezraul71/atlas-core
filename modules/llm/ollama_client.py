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
        stream: bool = False
    ) -> Dict[str, Any]:
        url = f"{self.base_url}/api/generate"
        payload: Dict[str, Any] = {
            "model": model,
            "prompt": prompt,
            "stream": stream,
            "options": {"temperature": temperature, "top_p": top_p},
        }
        if system:
            payload["system"] = system

        t0 = time.perf_counter()
        with httpx.Client(timeout=self.timeout) as client:
            r = client.post(url, json=payload)
            r.raise_for_status()
            data = r.json()

        data["_ms"] = int((time.perf_counter() - t0) * 1000)
        return data
