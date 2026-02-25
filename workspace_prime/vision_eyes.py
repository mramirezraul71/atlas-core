import base64
import io
import json
from pathlib import Path

import boto3
from PIL import Image


class VisionEyes:
    """
    ATLAS-WORKSPACE-PRIME — Ojos internos
    Analiza imágenes, screenshots y pantallas via Claude Vision (Bedrock)
    """

    def __init__(self):
        self.client = boto3.client(
            service_name="bedrock-runtime", region_name="us-east-1"
        )
        self.model_id = "anthropic.claude-3-5-sonnet-20241022-v2:0"

    def image_to_b64(self, image_path: str) -> str:
        with open(image_path, "rb") as f:
            return base64.b64encode(f.read()).decode("utf-8")

    def screenshot_to_b64(self, screenshot_bytes: bytes) -> str:
        return base64.b64encode(screenshot_bytes).decode("utf-8")

    def analyze_image(
        self, image_path: str, question: str = "Describe what you see in detail"
    ) -> dict:
        """Analiza una imagen desde archivo"""
        try:
            b64 = self.image_to_b64(image_path)
            ext = Path(image_path).suffix.lower().replace(".", "")
            media_type = f"image/{ext}" if ext != "jpg" else "image/jpeg"

            response = self.client.invoke_model(
                modelId=self.model_id,
                body=json.dumps(
                    {
                        "anthropic_version": "bedrock-2023-05-31",
                        "max_tokens": 1024,
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
                "image_path": image_path,
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    def analyze_screenshot_bytes(self, screenshot_bytes: bytes, question: str) -> dict:
        """Analiza screenshot directo desde bytes (para browser agent)"""
        try:
            b64 = self.screenshot_to_b64(screenshot_bytes)
            response = self.client.invoke_model(
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
                                            "media_type": "image/png",
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
            return {"success": True, "analysis": result["content"][0]["text"]}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def find_element_coordinates(
        self, screenshot_bytes: bytes, element_description: str
    ) -> dict:
        """
        Mira la pantalla y encuentra coordenadas de un elemento visual.
        Retorna x,y para que las manos puedan hacer click.
        """
        question = f"""
        Mira esta screenshot de pantalla.
        Necesito las coordenadas exactas (x, y en píxeles) de: {element_description}

        Responde SOLO en este formato JSON exacto:
        {{"found": true, "x": 123, "y": 456, "description": "lo que encontré"}}
        o si no lo encuentras:
        {{"found": false, "x": 0, "y": 0, "description": "no encontrado"}}
        """
        result = self.analyze_screenshot_bytes(screenshot_bytes, question)
        if result["success"]:
            try:
                import re

                json_match = re.search(r"\{.*\}", result["analysis"], re.DOTALL)
                if json_match:
                    return json.loads(json_match.group())
            except:
                pass
        return {"found": False, "x": 0, "y": 0, "description": "parse error"}


if __name__ == "__main__":
    eyes = VisionEyes()
    print("VisionEyes OK - Bedrock Claude Vision conectado")
