import os
from pathlib import Path
import contextlib
import io


def _load() -> None:
    """Carga credenciales priorizando la Bóveda local."""
    try:
        from dotenv import load_dotenv
    except Exception:
        return

    candidates = [
        (os.getenv("ATLAS_VAULT_PATH") or "").strip(),
        r"C:\dev\credenciales.txt",
        r"C:\Users\Raul\OneDrive\RAUL - Personal\Escritorio\credenciales.txt",
        str(Path(__file__).resolve().parents[1] / ".env"),
    ]
    for candidate in candidates:
        if not candidate:
            continue
        try:
            if Path(candidate).is_file():
                sink = io.StringIO()
                with contextlib.redirect_stderr(sink), contextlib.redirect_stdout(sink):
                    load_dotenv(candidate, override=True)
                break
        except Exception:
            continue


class AtlasLLM:
    def __init__(self, model: str = "gpt-4.1-mini"):
        _load()
        self.model = model
        self.api_key = os.getenv("OPENAI_API_KEY", "").strip()

    def ready(self) -> bool:
        return bool(self.api_key)

    def think(self, prompt: str, system: str | None = None) -> str:
        # OpenAI SDK nuevo
        from openai import OpenAI

        if not self.ready():
            return (
                "OPENAI_API_KEY no está configurada en la Bóveda "
                "(ATLAS_VAULT_PATH o C:\\dev\\credenciales.txt)."
            )

        client = OpenAI(api_key=self.api_key)
        msgs = []
        if system:
            msgs.append({"role": "system", "content": system})
        msgs.append({"role": "user", "content": prompt})

        r = client.chat.completions.create(
            model=self.model, messages=msgs, temperature=0.2
        )
        return (r.choices[0].message.content or "").strip()


atlas = AtlasLLM()
