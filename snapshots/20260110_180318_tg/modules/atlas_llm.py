import os
from dotenv import load_dotenv

def _load():
    load_dotenv(r"C:\ATLAS\config\.env")

class AtlasLLM:
    def __init__(self, model: str = "gpt-4.1-mini"):
        _load()
        self.model = model
        self.api_key = os.getenv("OPENAI_API_KEY","").strip()

    def ready(self) -> bool:
        return bool(self.api_key)

    def think(self, prompt: str, system: str | None = None) -> str:
        # OpenAI SDK nuevo
        from openai import OpenAI
        if not self.ready():
            return " OPENAI_API_KEY no está configurada en C:\\ATLAS\\config\\.env"

        client = OpenAI(api_key=self.api_key)
        msgs = []
        if system:
            msgs.append({"role":"system","content":system})
        msgs.append({"role":"user","content":prompt})

        r = client.chat.completions.create(
            model=self.model,
            messages=msgs,
            temperature=0.2
        )
        return (r.choices[0].message.content or "").strip()

atlas = AtlasLLM()
