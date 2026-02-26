# Carga credenciales desde .env y opcionalmente desde un archivo .txt
# Uso: importar al inicio antes de boto3 (ej: import load_credentials)
# Nunca imprime ni registra valores de claves.

import os
from pathlib import Path


def _load_dotenv():
    try:
        from dotenv import load_dotenv

        env_path = Path(__file__).resolve().parent / ".env"
        load_dotenv(env_path)
    except Exception:
        pass


def load_credentials_from_file(filepath: str) -> None:
    """Lee archivo KEY=value y establece os.environ. No imprime valores."""
    filepath = filepath.strip().strip('"').strip("'")
    path = Path(filepath)
    if not path.is_file():
        return
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if "=" in line:
                    k, _, v = line.partition("=")
                    k, v = k.strip(), v.strip()
                    if k:
                        v = v.strip('"').strip("'")
                        os.environ[k] = v
    except Exception:
        pass


# Al importar: cargar .env y luego archivo indicado por CREDENTIALS_FILE
_load_dotenv()
_cred_file = (os.getenv("CREDENTIALS_FILE") or "").strip().strip('"').strip("'")
if _cred_file:
    load_credentials_from_file(_cred_file)
