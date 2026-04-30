import base64
import json
import os
import re
from pathlib import Path

import load_credentials  # noqa: F401 — carga credenciales antes de boto3


class VisionEyes:
    """IA visual con fallback multi-modelo: Bedrock → OpenAI → Gemini."""

    def __init__(self):
        self._bedrock_client = None
        self.model_id = "anthropic.claude-3-5-sonnet-20241022-v2:0"
        try:
            import boto3

            self._bedrock_client = boto3.client(
                service_name="bedrock-runtime",
                region_name=os.getenv("AWS_DEFAULT_REGION", "us-east-1"),
            )
        except Exception:
            pass

    def image_to_b64(self, image_path: str) -> str:
        with open(image_path, "rb") as f:
            return base64.b64encode(f.read()).decode("utf-8")

    def screenshot_to_b64(self, screenshot_bytes: bytes) -> str:
        return base64.b64encode(screenshot_bytes).decode("utf-8")

    def _analyze_b64_bedrock(
        self, b64: str, question: str, media_type: str = "image/png"
    ) -> dict:
        if not self._bedrock_client:
            return {"success": False, "error": "Bedrock no configurado"}
        try:
            response = self._bedrock_client.invoke_model(
                modelId=self.model_id,
                body=json.dumps(
                    {
                        "anthropic_version": "bedrock-2023-05-31",
                        "max_tokens": 2048,
                        "messages": [
                            {
                                "role": "user",
                                "content": [
                                    {
                                        "type": "image",
                                        "source": {
                                            "type": "base64",
                                            "media_type": media_type,
                                            "data": b64,
                                        },
                                    },
                                    {"type": "text", "text": question},
                                ],
                            }
                        ],
                    }
                ),
            )
            result = json.loads(response["body"].read())
            return {
                "success": True,
                "analysis": result["content"][0]["text"],
                "engine": "bedrock",
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _analyze_b64_openai(self, b64: str, question: str) -> dict:
        key = (os.getenv("OPENAI_API_KEY") or "").strip()
        if not key:
            return {"success": False, "error": "OPENAI_API_KEY no configurada"}
        try:
            import httpx

            resp = httpx.post(
                "https://api.openai.com/v1/chat/completions",
                headers={
                    "Authorization": f"Bearer {key}",
                    "Content-Type": "application/json",
                },
                json={
                    "model": "gpt-4o",
                    "max_tokens": 2048,
                    "messages": [
                        {
                            "role": "user",
                            "content": [
                                {
                                    "type": "image_url",
                                    "image_url": {
                                        "url": f"data:image/png;base64,{b64}"
                                    },
                                },
                                {"type": "text", "text": question},
                            ],
                        }
                    ],
                },
                timeout=60,
            )
            resp.raise_for_status()
            data = resp.json()
            text = data["choices"][0]["message"]["content"]
            return {"success": True, "analysis": text, "engine": "openai"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _analyze_b64_gemini(self, b64: str, question: str) -> dict:
        key = (os.getenv("GEMINI_API_KEY") or "").strip()
        if not key:
            return {"success": False, "error": "GEMINI_API_KEY no configurada"}
        try:
            import httpx

            url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash-exp:generateContent?key={key}"
            payload = {
                "contents": [
                    {
                        "parts": [
                            {"inline_data": {"mime_type": "image/png", "data": b64}},
                            {"text": question},
                        ]
                    }
                ],
                "generationConfig": {"maxOutputTokens": 2048},
            }
            r = httpx.post(url, json=payload, timeout=60)
            r.raise_for_status()
            data = r.json()
            parts = data.get("candidates", [{}])[0].get("content", {}).get("parts", [])
            text = " ".join(p.get("text", "") for p in parts)
            return {"success": True, "analysis": text, "engine": "gemini"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _analyze_b64_multi(
        self, b64: str, question: str, media_type: str = "image/png"
    ) -> dict:
        for name, fn in [
            ("openai", lambda: self._analyze_b64_openai(b64, question)),
            ("gemini", lambda: self._analyze_b64_gemini(b64, question)),
        ]:
            out = fn()
            if out.get("success"):
                return out
        return {
            "success": False,
            "error": "Ningun backend de vision disponible (OpenAI/Gemini)",
        }

    def analyze_screenshot_bytes(self, screenshot_bytes: bytes, question: str) -> dict:
        b64 = self.screenshot_to_b64(screenshot_bytes)
        result = self._analyze_b64_bedrock(b64, question, "image/png")
        if not result.get("success"):
            result = self._analyze_b64_multi(b64, question, "image/png")
        return result

    def analyze_image(
        self, image_path: str, question: str = "Describe what you see in detail"
    ) -> dict:
        try:
            b64 = self.image_to_b64(image_path)
            ext = Path(image_path).suffix.lower().replace(".", "")
            media_type = f"image/{ext}" if ext != "jpg" else "image/jpeg"
            result = self._analyze_b64_bedrock(b64, question, media_type)
            if not result.get("success"):
                result = self._analyze_b64_multi(b64, question, media_type)
            if result.get("success"):
                result["image_path"] = image_path
            return result
        except Exception as e:
            return {"success": False, "error": str(e)}

    def find_element_coordinates(
        self, screenshot_bytes: bytes, element_description: str
    ) -> dict:
        question = f"""
Mira esta screenshot. Encuentra las coordenadas exactas (x, y en píxeles) de:
{element_description}

Responde SOLO con este JSON exacto:
{{"found": true, "x": 123, "y": 456, "description": "lo que encontré"}}
o si no lo encuentras:
{{"found": false, "x": 0, "y": 0, "description": "no encontrado"}}
"""
        result = self.analyze_screenshot_bytes(screenshot_bytes, question)
        if result["success"]:
            try:
                json_match = re.search(r"\{.*\}", result["analysis"], re.DOTALL)
                if json_match:
                    return json.loads(json_match.group())
            except:
                pass
        return {"found": False, "x": 0, "y": 0, "description": "parse error"}


if __name__ == "__main__":
    import sys

    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    print("OK VisionEyes - Bedrock Claude Vision listo")
