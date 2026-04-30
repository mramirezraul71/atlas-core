#!/usr/bin/env python3
"""
Carga y registra la batería de modelos ATLAS (locales + externos) para que el agente
elija el modelo más barato o eficiente según la tarea.

- Lee credenciales desde ATLAS_VAULT_PATH o C:\\dev\\credenciales.txt.
- Descubre modelos Ollama (GET /api/tags).
- Registra proveedores externos con API key (OpenAI, Anthropic, Gemini, Groq, etc.).
- Expone get_best_model(route, prefer_cheap=True) y list_all_models().

Uso:
  python scripts/load_atlas_models.py          # listar modelos cargados
  python scripts/load_atlas_models.py --sync   # sincronizar .env desde bóveda y listar
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

REPO_ROOT = Path(__file__).resolve().parent.parent

# Rutas por tarea (prioridad: barato/eficiente primero)
ROUTE_MODELS = [
    (
        "FAST",
        ["ollama:llama3.2:3b", "ollama:qwen2.5:7b", "groq:llama-3.3-70b-versatile"],
    ),
    (
        "CHAT",
        [
            "ollama:llama3.1:latest",
            "ollama:deepseek-r1:14b",
            "anthropic:claude-3-5-sonnet",
            "openai:gpt-4o",
        ],
    ),
    (
        "CODE",
        ["ollama:deepseek-coder:6.7b", "openai:gpt-4o", "anthropic:claude-3-5-sonnet"],
    ),
    (
        "REASON",
        ["ollama:deepseek-r1:14b", "anthropic:claude-3-5-sonnet", "openai:gpt-4o"],
    ),
    ("TOOLS", ["ollama:qwen2.5:7b", "anthropic:claude-3-5-sonnet"]),
    (
        "VISION",
        ["ollama:llama3.2-vision:11b", "openai:gpt-4o", "anthropic:claude-3-5-sonnet"],
    ),
    (
        "ARCHITECT",
        ["ollama:deepseek-r1:14b", "anthropic:claude-3-5-sonnet", "openai:gpt-4o"],
    ),
    ("OPTIMIZER", ["ollama:deepseek-coder:6.7b", "openai:gpt-4o"]),
]

ENV_TO_PROVIDER = {
    "OPENAI_API_KEY": "openai",
    "ANTHROPIC_API_KEY": "anthropic",
    "GEMINI_API_KEY": "gemini",
    "GROQ_API_KEY": "groq",
    "DEEPSEEK_API_KEY": "deepseek",
    "XAI_API_KEY": "xai",
    "MISTRAL_API_KEY": "mistral",
    "PERPLEXITY_API_KEY": "perplexity",
}


def _load_vault() -> None:
    """Carga Bóveda en os.environ sin imprimir claves."""
    vault = (os.getenv("ATLAS_VAULT_PATH") or r"C:\dev\credenciales.txt").strip()
    for p in [Path(vault), REPO_ROOT / ".env", REPO_ROOT / "config" / "atlas.env"]:
        if not p.is_file():
            continue
        try:
            with open(p, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#") or "=" not in line:
                        continue
                    k, _, v = line.partition("=")
                    k = k.strip()
                    v = v.strip().strip('"').strip("'")
                    if k and k not in os.environ:
                        os.environ[k] = v
            break
        except Exception:
            continue


def _ollama_models(base_url: str) -> List[str]:
    """Lista modelos disponibles en Ollama."""
    try:
        import urllib.request

        req = urllib.request.Request(f"{base_url.rstrip('/')}/api/tags", method="GET")
        with urllib.request.urlopen(req, timeout=5) as r:
            data = json.loads(r.read().decode())
        return [
            m.get("name", "").strip() for m in data.get("models", []) if m.get("name")
        ]
    except Exception:
        return []


def _provider_available(provider: str) -> bool:
    """True si el proveedor tiene API key en env."""
    for env_key, pid in ENV_TO_PROVIDER.items():
        if pid == provider and (os.getenv(env_key) or "").strip():
            return True
    return False


def list_all_models() -> Dict[str, Any]:
    """
    Lista todos los modelos disponibles (Ollama + externos con key).
    Retorna dict con keys: ollama, openai, anthropic, gemini, groq, deepseek, etc.
    """
    _load_vault()
    base = os.getenv("OLLAMA_BASE_URL", "http://127.0.0.1:11434")
    ollama_list = _ollama_models(base)
    out = {
        "ollama": {"available": True, "base_url": base, "models": ollama_list},
    }
    for env_key, provider in ENV_TO_PROVIDER.items():
        out[provider] = {
            "available": _provider_available(provider),
            "models": [],
        }
    return out


def get_best_model(route: str, prefer_cheap: bool = True) -> Optional[Tuple[str, str]]:
    """
    Devuelve (provider_id, model_name) para la ruta dada.
    prefer_cheap=True: prioriza Ollama/Groq; False: prioriza mejor calidad.
    """
    _load_vault()
    route = (route or "CHAT").strip().upper()
    candidates = []
    for r, model_keys in ROUTE_MODELS:
        if r != route:
            continue
        for full_key in model_keys:
            if ":" in full_key:
                prov, name = full_key.split(":", 1)
            else:
                prov, name = "ollama", full_key
            if prov == "ollama":
                base = os.getenv("OLLAMA_BASE_URL", "http://127.0.0.1:11434")
                if full_key in _ollama_models(base) or name in _ollama_models(base):
                    candidates.append((prov, name, True))
                else:
                    candidates.append((prov, name, False))
            else:
                candidates.append((prov, name, _provider_available(prov)))
        break
    if not candidates:
        return None
    available = [(p, n) for p, n, ok in candidates if ok]
    if not available:
        available = [(p, n) for p, n, _ in candidates]
    return available[0] if available else None


def main() -> int:
    parser = argparse.ArgumentParser(description="Carga batería de modelos ATLAS")
    parser.add_argument(
        "--sync", action="store_true", help="Ejecutar sync_env_from_vault antes"
    )
    parser.add_argument(
        "--route",
        type=str,
        default="",
        help="Ruta para probar get_best_model (ej: ARCHITECT)",
    )
    parser.add_argument("--json", action="store_true", help="Salida JSON")
    args = parser.parse_args()

    if args.sync:
        sync_script = REPO_ROOT / "scripts" / "sync_env_from_vault.py"
        if sync_script.is_file():
            with open(sync_script, encoding="utf-8") as f:
                code = f.read()
            exec(compile(code, str(sync_script), "exec"), {"__name__": "__main__"})

    _load_vault()
    all_models = list_all_models()
    total = len(all_models.get("ollama", {}).get("models", []))
    for pid, data in all_models.items():
        if pid == "ollama":
            continue
        if isinstance(data, dict) and data.get("available"):
            total += 1

    if args.json:
        print(
            json.dumps(
                {"models": all_models, "total_providers_with_keys": total}, indent=2
            )
        )
        return 0

    print("ATLAS — Batería de modelos cargada")
    print("=" * 50)
    print(f"Ollama: {len(all_models.get('ollama', {}).get('models', []))} modelos")
    for pid, data in all_models.items():
        if pid == "ollama":
            continue
        if isinstance(data, dict) and data.get("available"):
            print(f"  {pid}: disponible")
    print("=" * 50)
    if args.route:
        best = get_best_model(args.route)
        print(f"Mejor modelo para ruta {args.route or 'CHAT'}: {best}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
