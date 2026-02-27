"""Agente Codificador: AtlasCoder. Genera, ejecuta y autocorrige código en atlas_workspace/ (sandbox)."""
from __future__ import annotations

import logging
import os
import re
import subprocess
import sys
import uuid
from pathlib import Path
from typing import Any, Dict, Optional

logger = logging.getLogger("atlas.coder")

BASE = Path(__file__).resolve().parent.parent
ATLAS_WORKSPACE = BASE / "atlas_workspace"
TEMP_SCRIPTS = BASE / "temp_scripts"
EXECUTE_TIMEOUT = 10
ATLAS_WORKSPACE.mkdir(parents=True, exist_ok=True)
TEMP_SCRIPTS.mkdir(parents=True, exist_ok=True)


class AtlasCoder:
    """Genera código vía LLM, lo guarda en atlas_workspace/, ejecuta con subprocess y autocorrige si hay Traceback."""

    def __init__(self, workspace: Optional[Path] = None, timeout_sec: int = EXECUTE_TIMEOUT) -> None:
        self._workspace = Path(workspace) if workspace else ATLAS_WORKSPACE
        self._timeout_sec = timeout_sec
        self._workspace.mkdir(parents=True, exist_ok=True)

    def _extract_code(self, texto: str) -> str:
        """Extrae código puro de la respuesta del LLM (Markdown ```python ... ``` o ``` ... ```)."""
        if not texto or not isinstance(texto, str):
            return ""
        texto = texto.strip()
        pattern = r"```(?:python)?\s*\n(.*?)```"
        match = re.search(pattern, texto, re.DOTALL)
        if match:
            return match.group(1).strip()
        if "```" in texto:
            parts = texto.split("```")
            for i, p in enumerate(parts):
                if "def " in p or "import " in p or "print(" in p:
                    return p.strip()
            return parts[1].strip() if len(parts) > 1 else texto
        return texto

    def write_script(self, filename: str, code: str) -> Path:
        """Guarda el código limpio en atlas_workspace/."""
        path = self._workspace / filename
        if not filename.endswith(".py"):
            path = self._workspace / f"{filename}.py"
        path.write_text(code, encoding="utf-8")
        return path

    def execute_script(self, filename: str) -> Dict[str, Any]:
        """Ejecuta el script en el workspace. Captura stdout/stderr, timeout 10s."""
        path = self._workspace / filename
        if not path.suffix:
            path = self._workspace / f"{filename}.py"
        if not path.exists():
            return {"ok": False, "stdout": "", "stderr": f"File not found: {path}", "returncode": -1}
        try:
            py = os.environ.get("PYTHON", sys.executable)
            r = subprocess.run(
                [py, str(path)],
                cwd=str(self._workspace),
                capture_output=True,
                timeout=self._timeout_sec,
                text=True,
            )
            return {
                "ok": r.returncode == 0,
                "stdout": (r.stdout or "").strip(),
                "stderr": (r.stderr or "").strip(),
                "returncode": r.returncode,
            }
        except subprocess.TimeoutExpired:
            return {"ok": False, "stdout": "", "stderr": f"Timeout ({self._timeout_sec}s)", "returncode": -1}
        except Exception as e:
            return {"ok": False, "stdout": "", "stderr": str(e), "returncode": -1}

    def _ask_llm(self, prompt: str, error_context: Optional[str] = None) -> str:
        """Pide código al LLM (OpenAI si hay key, si no mock)."""
        api_key = os.environ.get("OPENAI_API_KEY", "").strip()
        if error_context:
            prompt = f"{prompt}\n\nEl código falló con este error:\n{error_context}\n\nReescribe el código corrigiendo esto."
        if not api_key:
            return _mock_generate(prompt)
        try:
            import openai
            client = openai.OpenAI(api_key=api_key)
            r = client.chat.completions.create(
                model=os.environ.get("CODER_AGENT_MODEL", "gpt-4o-mini"),
                messages=[
                    {"role": "system", "content": "You are a Python 3 code generator. Reply with valid code only; use markdown code block if needed."},
                    {"role": "user", "content": prompt},
                ],
                max_tokens=2000,
            )
            text = (r.choices[0].message.content or "").strip()
            return self._extract_code(text) or text
        except Exception as e:
            logger.warning("LLM falló: %s", e)
            return _mock_generate(prompt)

    def auto_program_loop(self, prompt_tarea: str, max_intentos: int = 3, filename: str = "auto_script.py") -> Dict[str, Any]:
        """
        Pide código al LLM, extrae y guarda, ejecuta. Si stderr tiene Traceback/error,
        envía el error al LLM y repite hasta stderr limpio o max_intentos.
        """
        if not filename.endswith(".py"):
            filename = f"{filename}.py"
        last_stderr = ""
        for intento in range(max_intentos):
            try:
                raw = self._ask_llm(prompt_tarea, last_stderr if intento > 0 else None)
                code = self._extract_code(raw)
                if not code:
                    code = raw
                path = self.write_script(filename, code)
                result = self.execute_script(path.name)
                if result["ok"]:
                    return {
                        "ok": True,
                        "stdout": result["stdout"],
                        "stderr": result["stderr"],
                        "path": str(path),
                        "intentos": intento + 1,
                    }
                last_stderr = result["stderr"] or f"returncode {result['returncode']}"
                if "traceback" in last_stderr.lower() or "error" in last_stderr.lower():
                    continue
                return {
                    "ok": False,
                    "stdout": result["stdout"],
                    "stderr": result["stderr"],
                    "path": str(path),
                    "intentos": intento + 1,
                }
            except Exception as e:
                logger.exception("auto_program_loop intento %s: %s", intento + 1, e)
                last_stderr = str(e)
        return {
            "ok": False,
            "stdout": "",
            "stderr": last_stderr or "max_intentos alcanzado",
            "path": "",
            "intentos": max_intentos,
        }


def _mock_generate(prompt: str) -> str:
    """Mock: genera código según el prompt (Fibonacci, etc.)."""
    lower = prompt.lower()
    if "fibonacci" in lower or "fib" in lower:
        return '''def fib(n):
    a, b = 0, 1
    out = []
    for _ in range(n):
        out.append(a)
        a, b = b, a + b
    return out
print(fib(20))
'''
    return f'''# Mock: {prompt[:60]}
print("Script generado (mock). Tarea:", {repr(prompt[:80])})
'''


def execute_order(order: str) -> Dict[str, Any]:
    """Compatibilidad con Orquestador: genera código, guarda en temp_scripts, ejecuta."""
    try:
        coder = AtlasCoder(workspace=TEMP_SCRIPTS, timeout_sec=30)
        raw = coder._ask_llm(order)
        code = coder._extract_code(raw) or raw
        name = f"agent_script_{uuid.uuid4().hex[:8]}.py"
        path = coder.write_script(name, code)
        path = TEMP_SCRIPTS / name
        path.write_text(code, encoding="utf-8")
        r = coder.execute_script(name)
        return {
            "ok": r["ok"],
            "script_path": str(path),
            "returncode": r.get("returncode", -1),
            "output": r.get("stdout") or r.get("stderr") or "(vacío)",
            "error": None if r["ok"] else r.get("stderr"),
        }
    except Exception as e:
        logger.exception("execute_order: %s", e)
        return {"ok": False, "error": str(e), "output": None}


if __name__ == "__main__":
    coder = AtlasCoder(timeout_sec=EXECUTE_TIMEOUT)
    prompt = "Escribe un script que calcule los primeros 20 números de la secuencia de Fibonacci y los imprima (solo código Python)."
    print("AtlasCoder: auto_program_loop (test_math.py - Fibonacci, max 3 intentos)")
    print("-" * 50)
    result = coder.auto_program_loop(prompt, max_intentos=3, filename="test_math.py")
    print("-" * 50)
    if result.get("ok"):
        print("stdout:")
        print(result.get("stdout", ""))
        if result.get("stderr"):
            print("stderr:", result.get("stderr"))
        print("path:", result.get("path", ""), "| intentos:", result.get("intentos", 0))
    else:
        print("error:", result.get("stderr", ""))
        print("intentos:", result.get("intentos", 0))
