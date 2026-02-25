import json, base64, os
from pathlib import Path
from datetime import datetime
from typing import Any

class MultimodalPipeline:
    """ATLAS-WORKSPACE-PRIME - Pipeline unificado multimodal"""

    IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".gif", ".webp", ".bmp"}
    CODE_EXTS  = {".py", ".ts", ".js", ".json", ".yaml", ".yml", ".md"}

    def __init__(self):
        self.root = Path(r"C:\ATLAS_PUSH")

    def detect_type(self, data: Any) -> str:
        if not isinstance(data, str):
            return "object"
        if data.startswith(("http://", "https://")):
            return "url"
        p = Path(data)
        if p.exists():
            if p.suffix.lower() in self.IMAGE_EXTS:
                return "image"
            return "file"
        return "text"

    def process(self, input_data: Any, task: str = "") -> dict:
        itype = self.detect_type(input_data)
        result = {
            "timestamp": datetime.now().isoformat(),
            "input_type": itype,
            "task": task,
            "primary_response": "",
            "artifacts": [],
            "actions_taken": [],
            "next_suggested_actions": []
        }

        if itype == "url":
            result["primary_response"] = f"URL detectada: {input_data}"
            result["actions_taken"].append("url_detected")
            result["next_suggested_actions"].append("Usar web_tools.WebTools().fetch(url)")

        elif itype == "image":
            with open(input_data, "rb") as f:
                b64 = base64.b64encode(f.read()).decode()
            result["primary_response"] = f"Imagen cargada: {Path(input_data).name}"
            result["artifacts"].append({"type": "image_b64", "path": input_data, "preview": b64[:50]})
            result["actions_taken"].append("image_loaded")
            result["next_suggested_actions"].append("Enviar b64 a Claude Vision via AWS Bedrock")

        elif itype == "file":
            content = Path(input_data).read_text(encoding="utf-8", errors="ignore")
            result["primary_response"] = f"Archivo leído: {len(content)} chars"
            result["artifacts"].append({"type": "file", "path": input_data, "content": content[:2000]})
            result["actions_taken"].append("file_read")

        else:
            result["primary_response"] = f"Texto procesado: {str(input_data)[:100]}"
            result["actions_taken"].append("text_processed")

        return result

    def status(self) -> dict:
        return {
            "agent": "ATLAS-WORKSPACE-PRIME v3.0",
            "capabilities": {
                "text":           "✅ ACTIVE",
                "file_rw":        "✅ ACTIVE",
                "code_exec":      "✅ ACTIVE",
                "persistent_mem": "✅ ACTIVE",
                "web_fetch":      "✅ ACTIVE",
                "image_load":     "✅ ACTIVE",
                "vision_ai":      "🔄 PENDING - Bedrock endpoint",
                "audio":          "🔄 PENDING - Whisper endpoint",
                "web_search":     "✅ ACTIVE - DuckDuckGo"
            }
        }

if __name__ == "__main__":
    p = MultimodalPipeline()
    print(json.dumps(p.status(), indent=2, ensure_ascii=False))
    test = p.process("Hola ATLAS, pipeline activo", task="self_test")
    print("Pipeline test:", test["actions_taken"])
