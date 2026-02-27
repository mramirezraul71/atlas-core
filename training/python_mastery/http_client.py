from __future__ import annotations

import random
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

try:
    import requests
except Exception:  # pragma: no cover
    requests = None


@dataclass(frozen=True)
class HttpResponse:
    status_code: int
    json: Dict[str, Any]


def get_json_with_retries(
    url: str,
    *,
    timeout_s: float = 5.0,
    retries: int = 3,
    backoff_s: float = 0.2,
    jitter_s: float = 0.1,
    session: Optional[Any] = None,
) -> HttpResponse:
    """PY009: GET JSON robusto con retries (sin red en tests: se mockea `session`).

    Reintenta ante:
    - excepciones de transporte
    - status >= 500
    Respeta Retry-After (segundos) si existe.
    """
    if not url or not isinstance(url, str):
        raise ValueError("url requerida")
    if requests is None and session is None:
        raise RuntimeError("requests no disponible y session no provista")

    sess = session or requests
    last_err: Optional[Exception] = None
    attempts = max(1, int(retries) + 1)
    for i in range(attempts):
        try:
            resp = sess.get(url, timeout=timeout_s)
            code = int(getattr(resp, "status_code", 0) or 0)
            if code >= 500:
                raise RuntimeError(f"server_error:{code}")
            data = resp.json() if hasattr(resp, "json") else {}
            if not isinstance(data, dict):
                data = {"_raw": data}
            return HttpResponse(status_code=code, json=data)
        except Exception as e:
            last_err = e
            if i >= attempts - 1:
                break
            sleep_s = backoff_s * (2**i)
            # Retry-After si resp existe y lo expone
            try:
                ra = None
                if "resp" in locals() and hasattr(resp, "headers"):
                    ra = resp.headers.get("Retry-After")
                if ra is not None:
                    sleep_s = max(sleep_s, float(ra))
            except Exception:
                pass
            sleep_s += random.random() * float(jitter_s)
            time.sleep(sleep_s)
    raise RuntimeError(f"request_failed: {last_err}")

