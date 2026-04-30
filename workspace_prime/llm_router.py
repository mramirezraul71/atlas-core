# Router LLM multi-IA para workspace_prime.
# Todas las llamadas a IA pasan por aquí: Bedrock → Anthropic → OpenAI → Gemini → Groq → DeepSeek.
# Uso: from llm_router import text_completion, get_langchain_llm

import json
import os
from typing import Any, Dict, List, Optional

# Cargar credenciales al importar
try:
    import load_credentials  # noqa: F401
except Exception:
    pass


def text_completion(
    user_message: str,
    system: Optional[str] = None,
    max_tokens: int = 2048,
) -> Dict[str, Any]:
    """
    Completar texto con la primera IA disponible.
    Orden: Bedrock → Anthropic → OpenAI → Gemini → Groq → DeepSeek.
    Retorna {"success": bool, "text": str, "engine": str} o {"success": False, "error": str}.
    """
    for name, fn in [
        ("bedrock", _completion_bedrock),
        ("anthropic", _completion_anthropic),
        ("openai", _completion_openai),
        ("gemini", _completion_gemini),
        ("groq", _completion_groq),
        ("deepseek", _completion_deepseek),
    ]:
        out = fn(user_message, system or "", max_tokens)
        if out.get("success"):
            return out
    return {
        "success": False,
        "error": "Ningún proveedor IA disponible (Bedrock/Anthropic/OpenAI/Gemini/Groq/DeepSeek)",
    }


def _completion_bedrock(
    user_message: str, system: str, max_tokens: int
) -> Dict[str, Any]:
    try:
        import boto3

        region = (
            os.getenv("AWS_DEFAULT_REGION") or os.getenv("AWS_REGION") or "us-east-1"
        ).strip()
        client = boto3.client("bedrock-runtime", region_name=region)
        body = {
            "anthropic_version": "bedrock-2023-05-31",
            "max_tokens": max_tokens,
            "messages": [{"role": "user", "content": user_message}],
        }
        if system:
            body["system"] = system
        response = client.invoke_model(
            modelId="anthropic.claude-3-5-sonnet-20241022-v2:0",
            body=json.dumps(body),
        )
        result = json.loads(response["body"].read())
        text = result["content"][0]["text"]
        return {"success": True, "text": text, "engine": "bedrock"}
    except Exception as e:
        return {"success": False, "error": str(e)}


def _completion_anthropic(
    user_message: str, system: str, max_tokens: int
) -> Dict[str, Any]:
    key = (os.getenv("ANTHROPIC_API_KEY") or "").strip()
    if not key:
        return {"success": False, "error": "ANTHROPIC_API_KEY no configurada"}
    try:
        from anthropic import Anthropic

        client = Anthropic(api_key=key)
        msg = client.messages.create(
            model="claude-3-5-sonnet-20241022",
            max_tokens=max_tokens,
            system=system or "Eres un asistente útil.",
            messages=[{"role": "user", "content": user_message}],
        )
        text = msg.content[0].text if msg.content else ""
        return {"success": True, "text": text, "engine": "anthropic"}
    except Exception as e:
        return {"success": False, "error": str(e)}


def _completion_openai(
    user_message: str, system: str, max_tokens: int
) -> Dict[str, Any]:
    key = (os.getenv("OPENAI_API_KEY") or "").strip()
    if not key:
        return {"success": False, "error": "OPENAI_API_KEY no configurada"}
    try:
        import httpx

        msgs = [{"role": "user", "content": user_message}]
        if system:
            msgs.insert(0, {"role": "system", "content": system})
        r = httpx.post(
            "https://api.openai.com/v1/chat/completions",
            headers={
                "Authorization": f"Bearer {key}",
                "Content-Type": "application/json",
            },
            json={"model": "gpt-4o", "max_tokens": max_tokens, "messages": msgs},
            timeout=60,
        )
        r.raise_for_status()
        data = r.json()
        text = data["choices"][0]["message"]["content"] or ""
        return {"success": True, "text": text, "engine": "openai"}
    except Exception as e:
        return {"success": False, "error": str(e)}


def _completion_gemini(
    user_message: str, system: str, max_tokens: int
) -> Dict[str, Any]:
    key = (os.getenv("GEMINI_API_KEY") or "").strip()
    if not key:
        return {"success": False, "error": "GEMINI_API_KEY no configurada"}
    try:
        import httpx

        url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash-exp:generateContent?key={key}"
        parts = (
            [{"text": system + "\n\n" + user_message}]
            if system
            else [{"text": user_message}]
        )
        r = httpx.post(
            url,
            json={
                "contents": [{"parts": parts}],
                "generationConfig": {"maxOutputTokens": max_tokens},
            },
            timeout=60,
        )
        r.raise_for_status()
        data = r.json()
        parts_out = data.get("candidates", [{}])[0].get("content", {}).get("parts", [])
        text = " ".join(p.get("text", "") for p in parts_out)
        return {"success": True, "text": text, "engine": "gemini"}
    except Exception as e:
        return {"success": False, "error": str(e)}


def _completion_groq(user_message: str, system: str, max_tokens: int) -> Dict[str, Any]:
    key = (os.getenv("GROQ_API_KEY") or "").strip()
    if not key:
        return {"success": False, "error": "GROQ_API_KEY no configurada"}
    try:
        import httpx

        msgs = [{"role": "user", "content": user_message}]
        if system:
            msgs.insert(0, {"role": "system", "content": system})
        r = httpx.post(
            "https://api.groq.com/openai/v1/chat/completions",
            headers={
                "Authorization": f"Bearer {key}",
                "Content-Type": "application/json",
            },
            json={
                "model": "llama-3.3-70b-versatile",
                "max_tokens": max_tokens,
                "messages": msgs,
            },
            timeout=60,
        )
        r.raise_for_status()
        data = r.json()
        text = data["choices"][0]["message"]["content"] or ""
        return {"success": True, "text": text, "engine": "groq"}
    except Exception as e:
        return {"success": False, "error": str(e)}


def _completion_deepseek(
    user_message: str, system: str, max_tokens: int
) -> Dict[str, Any]:
    key = (os.getenv("DEEPSEEK_API_KEY") or "").strip()
    if not key:
        return {"success": False, "error": "DEEPSEEK_API_KEY no configurada"}
    try:
        import httpx

        msgs = [{"role": "user", "content": user_message}]
        if system:
            msgs.insert(0, {"role": "system", "content": system})
        r = httpx.post(
            "https://api.deepseek.com/v1/chat/completions",
            headers={
                "Authorization": f"Bearer {key}",
                "Content-Type": "application/json",
            },
            json={"model": "deepseek-chat", "max_tokens": max_tokens, "messages": msgs},
            timeout=60,
        )
        r.raise_for_status()
        data = r.json()
        text = data["choices"][0]["message"]["content"] or ""
        return {"success": True, "text": text, "engine": "deepseek"}
    except Exception as e:
        return {"success": False, "error": str(e)}


def get_langchain_llm():
    """
    Devuelve el primer LLM LangChain disponible (para browser-use, etc.).
    Orden: Anthropic → OpenAI → Gemini. Retorna None si ninguno está configurado.
    """
    try:
        key = (os.getenv("ANTHROPIC_API_KEY") or "").strip()
        if key:
            from langchain_anthropic import ChatAnthropic

            return (
                ChatAnthropic(model="claude-3-5-sonnet-20241022", max_tokens=4096),
                "anthropic",
            )
    except Exception:
        pass
    try:
        key = (os.getenv("OPENAI_API_KEY") or "").strip()
        if key:
            from langchain_openai import ChatOpenAI

            return ChatOpenAI(model="gpt-4o", max_tokens=4096), "openai"
    except Exception:
        pass
    try:
        key = (os.getenv("GEMINI_API_KEY") or "").strip()
        if key:
            from langchain_google_genai import ChatGoogleGenerativeAI

            return (
                ChatGoogleGenerativeAI(
                    model="gemini-2.0-flash-exp", max_output_tokens=4096
                ),
                "gemini",
            )
    except Exception:
        pass
    return None, None


def get_available_engines() -> List[str]:
    """Lista de motores con credenciales configuradas (solo nombres)."""
    out = []
    if (os.getenv("AWS_ACCESS_KEY_ID") or os.getenv("AWS_PROFILE")) and os.getenv(
        "AWS_SECRET_ACCESS_KEY"
    ):
        try:
            import boto3

            boto3.client(
                "bedrock-runtime",
                region_name=os.getenv("AWS_DEFAULT_REGION", "us-east-1"),
            )
            out.append("bedrock")
        except Exception:
            pass
    if (os.getenv("ANTHROPIC_API_KEY") or "").strip():
        out.append("anthropic")
    if (os.getenv("OPENAI_API_KEY") or "").strip():
        out.append("openai")
    if (os.getenv("GEMINI_API_KEY") or "").strip():
        out.append("gemini")
    if (os.getenv("GROQ_API_KEY") or "").strip():
        out.append("groq")
    if (os.getenv("DEEPSEEK_API_KEY") or "").strip():
        out.append("deepseek")
    return out
