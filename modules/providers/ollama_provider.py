"""Ollama API client. Base URL and timeouts via env."""
import os
import httpx

OLLAMA_BASE_URL = os.getenv("OLLAMA_BASE_URL", "http://127.0.0.1:11434").rstrip("/")


def ollama_chat(model: str, messages: list[dict], timeout_sec: int = 60) -> str:
    """
    POST {base}/api/chat with stream=False.
    Returns message["content"] from response. Raises RuntimeError on failure.
    """
    url = f"{OLLAMA_BASE_URL}/api/chat"
    payload = {"model": model, "messages": messages, "stream": False}
    try:
        with httpx.Client(timeout=float(timeout_sec)) as client:
            r = client.post(url, json=payload)
            r.raise_for_status()
    except httpx.ConnectError as e:
        raise RuntimeError(f"Ollama: no se pudo conectar a {OLLAMA_BASE_URL}: {e}") from e
    except httpx.TimeoutException as e:
        raise RuntimeError(f"Ollama: timeout ({timeout_sec}s) en {url}: {e}") from e
    except httpx.HTTPStatusError as e:
        raise RuntimeError(f"Ollama: HTTP {e.response.status_code} - {e.response.text}") from e

    try:
        data = r.json()
        msg = data.get("message")
        if not msg:
            raise RuntimeError(f"Ollama: respuesta sin 'message': {data}")
        content = msg.get("content")
        if content is None:
            raise RuntimeError(f"Ollama: respuesta sin 'message.content': {data}")
        return content if isinstance(content, str) else str(content)
    except (KeyError, TypeError, ValueError) as e:
        raise RuntimeError(f"Ollama: respuesta inválida - {e}") from e
