"""
Agentic execution engine for Atlas Workspace.

Implements a Cursor-like tool-calling loop with intelligent model routing:
  - Task complexity classification (simple/medium/complex)
  - Model selection: Haiku (fast) → Sonnet (balanced) → Opus (complex)
  - Interpreter integrated as a tool, not a separate path
  - Bedrock Converse API (boto3) + Anthropic direct + Ollama fallback
"""
from __future__ import annotations

import json
import logging
import os
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Generator, List, Optional

_log = logging.getLogger("atlas.agent_engine")

ATLAS_ROOT = Path(os.getenv("ATLAS_ROOT", r"C:\ATLAS_PUSH"))
MAX_ITERATIONS = 20
MAX_TOOL_OUTPUT = 8000
COMMAND_TIMEOUT = 60

# ──────────────────────────────────────────────
#  Model tiers for intelligent routing
# ──────────────────────────────────────────────
MODEL_TIERS = {
    "bedrock": {
        "fast":    "us.anthropic.claude-haiku-4-5-20251001-v1:0",
        "balanced": "us.anthropic.claude-sonnet-4-5-20250929-v1:0",
        "complex": "us.anthropic.claude-opus-4-6-v1",
    },
    "anthropic": {
        "fast":    "claude-haiku-4-5-20251001",
        "balanced": "claude-sonnet-4-5-20250929",
        "complex": "claude-opus-4-6-20250918",
    },
}

COMPLEXITY_INDICATORS = {
    "complex": [
        r"\b(refactor|redesign|architect|implement|migrat|rewrit|reestructur)",
        r"\b(debug|investig|diagnos|audit)",
        r"\b(crea|implement|build|construy)\w*\b.*\b(sistema|modul|servicio|api|engine|feature)",
        r"\b(optimiz|performanc|rendimiento|seguridad|security)",
        r"\b(complej|complex|profund|deep)\b",
        r"\b(analiz|analys|evalua|review|revis)\w*\b.*\b(codigo|code|sistema|system|arquitect|bug|error)",
        r"\b(configur|deploy|desplieg|integr)\w*\b.*\b(complet|todo|entire|full)",
    ],
    "balanced": [
        r"\b(lee|leer|read|busca|buscar|search|encuentra|find)\w*\b.*\b(archivo|file|codigo|code|log)",
        r"\b(ejecuta|ejecutar|run|corre|correr)\b",
        r"\b(edita|editar|modifica|modificar|cambia|cambiar|edit|fix|arregl)",
        r"\b(instala|instalar|pip|npm|git)\b",
        r"\b(cuenta|contar|lista|listar|muestra|mostrar)\w*\b.*\b(archivo|carpeta|proceso|servicio)",
        r"\b(crea|crear|genera|generar|escrib)\w*\b.*\b(archivo|script|fichero|file)",
        r"\b(verifica|verificar|comprueba|comprobar|check|test|prueba)",
        r"\b(estado|status|health|memoria|disco|cpu)\b",
    ],
    "simple": [
        r"^\s*(hola|hi|hello|hey|que tal|como estas|buenos?\s+dias?|buenas?\s+tardes?|buenas?\s+noches?)\s*[?!.,]?\s*$",
        r"^\s*(gracias|thanks|ok|si|no|bien|perfecto|entendido|claro|dale)\s*[?!.,]?\s*$",
        r"^\s*(que|what|cual|which)\s+(es|is|son|are)\s",
        r"^\s*(dime|tell me)\s+(que|what|quien|who)\s+(eres|are you)\b",
        r"^\s*(ping|version)\s*$",
    ],
}


def classify_complexity(text: str) -> str:
    """Classify task complexity: fast (Haiku), balanced (Sonnet), or complex (Opus)."""
    import re
    lower = text.lower().strip()

    for pattern in COMPLEXITY_INDICATORS["simple"]:
        if re.search(pattern, lower, re.IGNORECASE):
            return "fast"

    complex_score = 0
    for pattern in COMPLEXITY_INDICATORS["complex"]:
        if re.search(pattern, lower, re.IGNORECASE):
            complex_score += 1

    if len(lower) > 300:
        complex_score += 1
    if lower.count("\n") > 3:
        complex_score += 1

    if complex_score >= 1:
        return "complex"

    balanced_score = 0
    for pattern in COMPLEXITY_INDICATORS["balanced"]:
        if re.search(pattern, lower, re.IGNORECASE):
            balanced_score += 1

    if balanced_score >= 1:
        return "balanced"
    if len(lower) > 100:
        return "balanced"

    return "fast"


def select_model(backend: str, complexity: str, model_override: Optional[str] = None) -> str:
    """Select the best model for the given complexity tier."""
    if model_override:
        return model_override
    tiers = MODEL_TIERS.get(backend, MODEL_TIERS.get("bedrock", {}))
    return tiers.get(complexity, tiers.get("balanced", "us.anthropic.claude-sonnet-4-5-20250929-v1:0"))


# ──────────────────────────────────────────────
#  Tool definitions (Claude/Anthropic format)
# ──────────────────────────────────────────────
TOOLS = [
    {
        "name": "read_file",
        "description": "Read a file and return its contents with line numbers. Use for inspecting source code, configs, logs.",
        "input_schema": {
            "type": "object",
            "properties": {
                "path": {"type": "string", "description": "Absolute path to the file"},
            },
            "required": ["path"],
        },
    },
    {
        "name": "write_file",
        "description": "Create or overwrite a file with the given content.",
        "input_schema": {
            "type": "object",
            "properties": {
                "path": {"type": "string"},
                "content": {"type": "string", "description": "Full file content to write"},
            },
            "required": ["path", "content"],
        },
    },
    {
        "name": "edit_file",
        "description": "Replace exact text in a file. old_text must match exactly (including whitespace and indentation).",
        "input_schema": {
            "type": "object",
            "properties": {
                "path": {"type": "string"},
                "old_text": {"type": "string", "description": "Exact text to find"},
                "new_text": {"type": "string", "description": "Replacement text"},
            },
            "required": ["path", "old_text", "new_text"],
        },
    },
    {
        "name": "execute_command",
        "description": "Execute a shell command (PowerShell on Windows). Returns stdout + stderr. Use for: running scripts, installing packages, git operations, system diagnostics, file operations, compiling, testing.",
        "input_schema": {
            "type": "object",
            "properties": {
                "command": {"type": "string", "description": "The command to execute"},
                "working_directory": {"type": "string", "description": "Working directory (default: ATLAS_ROOT)"},
                "timeout": {"type": "integer", "description": "Timeout in seconds (default: 60, max: 300)"},
            },
            "required": ["command"],
        },
    },
    {
        "name": "search_text",
        "description": "Search for text pattern across files using ripgrep. Returns matching lines with file paths and line numbers.",
        "input_schema": {
            "type": "object",
            "properties": {
                "pattern": {"type": "string", "description": "Regex pattern to search for"},
                "directory": {"type": "string", "description": "Directory to search in (default: ATLAS_ROOT)"},
                "file_glob": {"type": "string", "description": "File filter, e.g. '*.py', '*.js'"},
            },
            "required": ["pattern"],
        },
    },
    {
        "name": "list_directory",
        "description": "List files and subdirectories at a path with sizes.",
        "input_schema": {
            "type": "object",
            "properties": {
                "path": {"type": "string", "description": "Directory path"},
            },
            "required": ["path"],
        },
    },
    {
        "name": "atlas_api",
        "description": "Call an internal ATLAS REST endpoint for diagnostics and control. Endpoints: /health, /audit/tail, /status, /api/autonomy/status, /watchdog/status, /api/libro-vida/status, /agent/models, /api/monitor/snapshot.",
        "input_schema": {
            "type": "object",
            "properties": {
                "method": {"type": "string", "enum": ["GET", "POST"]},
                "endpoint": {"type": "string", "description": "API path, e.g. /health"},
                "body": {"type": "object", "description": "JSON body for POST requests"},
            },
            "required": ["method", "endpoint"],
        },
    },
    {
        "name": "interpreter",
        "description": "Execute a complex autonomous task using Open Interpreter. Use for tasks that require multiple steps of code execution, web browsing, data processing, or interactive programming. The interpreter runs Python, shell commands, and can install packages autonomously. Prefer this over execute_command for multi-step programming tasks, data analysis, web scraping, or when the task needs iterative code execution with state.",
        "input_schema": {
            "type": "object",
            "properties": {
                "task": {"type": "string", "description": "Natural language description of the task to execute"},
                "language": {"type": "string", "description": "Preferred language: python, shell, javascript (default: auto)"},
            },
            "required": ["task"],
        },
    },
]


# ──────────────────────────────────────────────
#  Tool execution
# ──────────────────────────────────────────────
def _truncate(text: str, limit: int = MAX_TOOL_OUTPUT) -> str:
    if len(text) <= limit:
        return text
    half = limit // 2 - 50
    return text[:half] + f"\n\n... ({len(text) - limit} chars truncated) ...\n\n" + text[-half:]


def execute_tool(name: str, inp: Dict[str, Any], progress_callback=None) -> str:
    """Execute a tool and return the result as text. Streaming tools accept progress_callback."""
    try:
        if name == "execute_command":
            return _tool_execute_command(inp, progress_callback=progress_callback)

        dispatch = {
            "read_file": _tool_read_file,
            "write_file": _tool_write_file,
            "edit_file": _tool_edit_file,
            "search_text": _tool_search_text,
            "list_directory": _tool_list_directory,
            "atlas_api": _tool_atlas_api,
            "interpreter": _tool_interpreter,
        }
        fn = dispatch.get(name)
        if not fn:
            return f"Error: unknown tool '{name}'"
        return fn(inp)
    except Exception as e:
        return f"Error executing {name}: {str(e)}"


def _tool_read_file(inp: Dict) -> str:
    path = Path(inp["path"])
    if not path.exists():
        return f"File not found: {path}"
    if not path.is_file():
        return f"Not a file: {path}"
    try:
        content = path.read_text(encoding="utf-8", errors="replace")
        lines = content.split("\n")
        numbered = "\n".join(f"{i+1:>5}|{line}" for i, line in enumerate(lines))
        return _truncate(numbered)
    except Exception as e:
        return f"Error reading {path}: {e}"


def _tool_write_file(inp: Dict) -> str:
    path = Path(inp["path"])
    path.parent.mkdir(parents=True, exist_ok=True)
    content = inp["content"]
    path.write_text(content, encoding="utf-8")
    lines = content.count("\n") + 1
    return f"Written {len(content)} bytes ({lines} lines) to {path}"


def _tool_edit_file(inp: Dict) -> str:
    path = Path(inp["path"])
    if not path.exists():
        return f"File not found: {path}"
    content = path.read_text(encoding="utf-8", errors="replace")
    old_text = inp["old_text"]
    new_text = inp["new_text"]
    count = content.count(old_text)
    if count == 0:
        preview = old_text[:200].replace("\n", "\\n")
        return f"old_text not found in {path}. Preview: '{preview}'"
    if count > 1:
        return f"old_text found {count} times in {path}. Make it unique by including more context."
    new_content = content.replace(old_text, new_text, 1)
    path.write_text(new_content, encoding="utf-8")
    return f"Edited {path}: replaced {len(old_text)} chars with {len(new_text)} chars"


def _tool_execute_command(inp: Dict, progress_callback=None) -> str:
    """Execute command with optional real-time progress streaming."""
    cmd = inp["command"]
    cwd = inp.get("working_directory", str(ATLAS_ROOT))
    timeout = min(inp.get("timeout", COMMAND_TIMEOUT), 300)
    try:
        proc = subprocess.Popen(
            ["powershell", "-Command", cmd],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            cwd=cwd, text=True, encoding="utf-8", errors="replace",
            bufsize=1,
        )

        stdout_lines = []
        start = time.perf_counter()

        while True:
            line = proc.stdout.readline()
            if line:
                stripped = line.rstrip("\n\r")
                stdout_lines.append(stripped)
                if progress_callback and stripped.strip():
                    progress_callback(stripped[:200])
            elif proc.poll() is not None:
                break

            if time.perf_counter() - start > timeout:
                proc.kill()
                return f"Command timed out after {timeout}s. Partial output:\n" + "\n".join(stdout_lines[-20:])

        remaining = proc.stdout.read()
        if remaining:
            for rl in remaining.strip().split("\n"):
                stdout_lines.append(rl)
                if progress_callback and rl.strip():
                    progress_callback(rl[:200])

        stderr_text = proc.stderr.read()

        out = "\n".join(stdout_lines)
        if stderr_text and stderr_text.strip():
            out += "\nSTDERR:\n" + stderr_text
        if proc.returncode != 0:
            out += f"\n[exit code: {proc.returncode}]"
        return _truncate(out.strip() or "(no output)")
    except Exception as e:
        return f"Command error: {e}"


def _tool_search_text(inp: Dict) -> str:
    pattern = inp["pattern"]
    directory = inp.get("directory", str(ATLAS_ROOT))
    file_glob = inp.get("file_glob")

    cmd = ["rg", "--no-heading", "-n", "--max-count", "30", pattern, directory]
    if file_glob:
        cmd.extend(["--glob", file_glob])
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=15,
            encoding="utf-8", errors="replace",
        )
        out = result.stdout.strip()
        if not out:
            return f"No matches for '{pattern}' in {directory}"
        return _truncate(out)
    except FileNotFoundError:
        cmd_ps = f'Get-ChildItem -Path "{directory}" -Recurse -Filter "{file_glob or "*.py"}" | Select-String -Pattern "{pattern}" | Select-Object -First 30'
        try:
            r = subprocess.run(
                ["powershell", "-Command", cmd_ps],
                capture_output=True, text=True, timeout=15,
                encoding="utf-8", errors="replace",
            )
            return _truncate(r.stdout.strip() or f"No matches for '{pattern}'")
        except Exception as e:
            return f"Search error: {e}"
    except Exception as e:
        return f"Search error: {e}"


def _tool_list_directory(inp: Dict) -> str:
    path = Path(inp["path"])
    if not path.exists():
        return f"Directory not found: {path}"
    if not path.is_dir():
        return f"Not a directory: {path}"
    items = []
    try:
        for entry in sorted(path.iterdir()):
            prefix = "DIR " if entry.is_dir() else "    "
            size = ""
            if entry.is_file():
                try:
                    size = f" ({entry.stat().st_size:,} bytes)"
                except OSError:
                    pass
            items.append(f"{prefix}{entry.name}{size}")
    except PermissionError:
        return f"Permission denied: {path}"
    return "\n".join(items[:100]) or "(empty directory)"


def _tool_atlas_api(inp: Dict) -> str:
    import requests
    method = inp["method"].upper()
    endpoint = inp["endpoint"]
    body = inp.get("body")
    url = f"http://127.0.0.1:8791{endpoint}"
    try:
        if method == "GET":
            r = requests.get(url, timeout=10)
        else:
            r = requests.post(url, json=body or {}, timeout=10)
        try:
            data = r.json()
            return _truncate(json.dumps(data, indent=2, ensure_ascii=False, default=str))
        except Exception:
            return _truncate(r.text[:3000])
    except Exception as e:
        return f"API error: {e}"


def _tool_interpreter(inp: Dict) -> str:
    """Execute a task via Open Interpreter, integrated as an agent tool."""
    task = inp["task"]
    language = inp.get("language", "")
    import requests

    try:
        payload = {"prompt": task, "auto_run": True}
        if language:
            payload["language"] = language
        r = requests.post(
            "http://127.0.0.1:8791/api/workspace/interpreter/quick",
            json=payload, timeout=120,
        )
        data = r.json()
        if data.get("ok"):
            output = data.get("output", "")
            exec_ms = data.get("ms", 0)
            model_used = data.get("model", "auto")
            return f"[Interpreter OK | {exec_ms}ms | model: {model_used}]\n{_truncate(output, 6000)}"
        else:
            return f"Interpreter error: {data.get('error', 'unknown')}"
    except requests.Timeout:
        return "Interpreter timed out after 120s"
    except Exception as e:
        return f"Interpreter error: {e}"


# ──────────────────────────────────────────────
#  Agent system prompt
# ──────────────────────────────────────────────
AGENT_SYSTEM_PROMPT = """Eres ATLAS, un agente tecnico autonomo con capacidad de ejecutar acciones reales en el sistema.

CAPACIDADES (herramientas disponibles):
- read_file, write_file, edit_file: Leer, crear y editar archivos
- execute_command: Ejecutar comandos de terminal (PowerShell en Windows)
- search_text: Buscar texto en el codigo fuente con ripgrep
- list_directory: Listar archivos y carpetas
- atlas_api: Consultar endpoints internos de ATLAS (/health, /status, /audit/tail, etc.)
- interpreter: Ejecutar tareas complejas autonomas (scripts multi-paso, analisis de datos, web scraping, programacion interactiva)

CUANDO USAR INTERPRETER vs EXECUTE_COMMAND:
- execute_command: comandos simples, one-liners, diagnosticos rapidos (python --version, git status, pip install X)
- interpreter: tareas que requieren multiples pasos de codigo, analisis de datos, procesamiento iterativo, tareas complejas de programacion

REGLAS:
1. Usa las herramientas para HACER, no solo para planificar
2. Cuando te pidan investigar algo, LEE el codigo real, los logs, la base de datos
3. Cuando te pidan arreglar algo, EDITA el archivo directamente
4. Cuando te pidan ejecutar algo, USA execute_command o interpreter segun la complejidad
5. Para diagnostico de ATLAS, usa atlas_api con /health, /audit/tail, /status
6. Siempre verifica tu trabajo: despues de editar, lee el archivo para confirmar
7. Responde en espanol, conciso y directo
8. La raiz del proyecto es """ + str(ATLAS_ROOT) + """
9. Maximo 3-5 acciones por tarea simple. Hasta 15 para tareas complejas.

PROHIBIDO:
- Generar planes abstractos sin ejecutarlos
- Sugerir pasos sin hacerlos
- Decir "podrias hacer X" en vez de HACER X"""


# ──────────────────────────────────────────────
#  Bedrock Converse API (boto3)
# ──────────────────────────────────────────────
def _bedrock_converse(model: str, system: str, messages: List[Dict], tools: List[Dict]) -> Dict:
    """Call Bedrock Converse API with tool support via boto3."""
    import boto3
    region = (os.getenv("AWS_REGION", "us-east-1") or "us-east-1").strip()
    client = boto3.client("bedrock-runtime", region_name=region)

    bedrock_tools = []
    for t in tools:
        bedrock_tools.append({
            "toolSpec": {
                "name": t["name"],
                "description": t["description"],
                "inputSchema": {"json": t["input_schema"]},
            }
        })

    bedrock_messages = []
    for msg in messages:
        role = msg["role"]
        content = msg["content"]
        if isinstance(content, str):
            bedrock_messages.append({"role": role, "content": [{"text": content}]})
        elif isinstance(content, list):
            blocks = []
            for item in content:
                if isinstance(item, dict):
                    if item.get("type") == "text":
                        blocks.append({"text": item["text"]})
                    elif item.get("type") == "tool_use":
                        blocks.append({
                            "toolUse": {
                                "toolUseId": item["id"],
                                "name": item["name"],
                                "input": item["input"],
                            }
                        })
                    elif item.get("type") == "tool_result":
                        blocks.append({
                            "toolResult": {
                                "toolUseId": item["tool_use_id"],
                                "content": [{"text": item["content"] if isinstance(item["content"], str) else json.dumps(item["content"])}],
                            }
                        })
            if blocks:
                bedrock_messages.append({"role": role, "content": blocks})

    resp = client.converse(
        modelId=model,
        system=[{"text": system}],
        messages=bedrock_messages,
        toolConfig={"tools": bedrock_tools},
        inferenceConfig={"maxTokens": 4096, "temperature": 0},
    )

    result_content = []
    for block in resp.get("output", {}).get("message", {}).get("content", []):
        if "text" in block:
            result_content.append(type("B", (), {"type": "text", "text": block["text"]})())
        elif "toolUse" in block:
            tu = block["toolUse"]
            result_content.append(type("B", (), {
                "type": "tool_use", "id": tu["toolUseId"],
                "name": tu["name"], "input": tu["input"],
            })())

    return type("R", (), {"content": result_content, "stop_reason": resp.get("stopReason", "")})()


# ──────────────────────────────────────────────
#  Ollama (structured prompt tool calling)
# ──────────────────────────────────────────────
def _ollama_available() -> Optional[str]:
    """Check if Ollama is running and return best model for tool calling."""
    try:
        import httpx
        r = httpx.get("http://127.0.0.1:11434/api/tags", timeout=3)
        if r.status_code != 200:
            return None
        models = [m["name"] for m in r.json().get("models", [])]
        preferred = ["qwen3:4b", "qwen2.5:7b", "llama3.1:latest", "llama3:latest",
                     "deepseek-r1:14b", "deepseek-r1:latest"]
        for p in preferred:
            if p in models:
                return p
        return models[0] if models else None
    except Exception:
        return None


def _ollama_chat(model: str, system: str, messages: List[Dict], tools: List[Dict]):
    """Call Ollama with tool-calling via structured prompt."""
    import httpx

    tools_desc = []
    for t in tools:
        params = t["input_schema"].get("properties", {})
        required = t["input_schema"].get("required", [])
        param_lines = []
        for pname, pinfo in params.items():
            req = " (required)" if pname in required else ""
            param_lines.append(f"  - {pname}: {pinfo.get('description', pinfo.get('type', 'string'))}{req}")
        tools_desc.append(f"### {t['name']}\n{t['description']}\nParameters:\n" + "\n".join(param_lines))

    tool_prompt = """You have access to the following tools:

""" + "\n\n".join(tools_desc) + """

When you need to use a tool, respond with EXACTLY this JSON format on a single line:
{"tool_call": {"name": "tool_name", "input": {"param1": "value1"}}}

When you have the final answer (no more tools needed), respond with normal text.
IMPORTANT: Use ONE tool call per response. After seeing the result, decide next action."""

    full_system = system + "\n\n" + tool_prompt

    ollama_msgs = [{"role": "system", "content": full_system}]
    for msg in messages:
        role = msg["role"]
        content = msg["content"]
        if isinstance(content, str):
            ollama_msgs.append({"role": role, "content": content})
        elif isinstance(content, list):
            parts = []
            for item in content:
                if isinstance(item, dict):
                    if item.get("type") == "text":
                        parts.append(item["text"])
                    elif item.get("type") == "tool_use":
                        parts.append(f'Tool call: {item["name"]}({json.dumps(item["input"], ensure_ascii=False)})')
                    elif item.get("type") == "tool_result":
                        parts.append(f'Tool result: {item["content"]}')
            if parts:
                ollama_msgs.append({"role": role, "content": "\n".join(parts)})

    resp = httpx.post(
        "http://127.0.0.1:11434/api/chat",
        json={"model": model, "messages": ollama_msgs, "stream": False,
              "options": {"temperature": 0, "num_predict": 2048}},
        timeout=120,
    )
    text = resp.json().get("message", {}).get("content", "").strip()

    import re
    match = re.search(r'\{"tool_call"\s*:\s*\{.*?\}\s*\}', text, re.DOTALL)
    if match:
        try:
            parsed = json.loads(match.group())
            tc = parsed["tool_call"]
            import uuid
            tool_block = type("B", (), {
                "type": "tool_use",
                "id": f"ollama_{uuid.uuid4().hex[:8]}",
                "name": tc["name"],
                "input": tc.get("input", {}),
            })()
            thinking = text[:match.start()].strip()
            content = []
            if thinking:
                content.append(type("B", (), {"type": "text", "text": thinking})())
            content.append(tool_block)
            return type("R", (), {"content": content, "stop_reason": "tool_use"})()
        except (json.JSONDecodeError, KeyError):
            pass

    return type("R", (), {
        "content": [type("B", (), {"type": "text", "text": text})()],
        "stop_reason": "end_turn",
    })()


# ──────────────────────────────────────────────
#  LLM backend selection
# ──────────────────────────────────────────────
def _get_llm_backend(model_override: Optional[str] = None):
    """Determine which LLM backend to use for tool calling.

    Priority: Bedrock boto3 > Anthropic direct > Ollama.
    Returns (backend_type, client_or_None, model_id_or_None).
    """
    region = (os.getenv("AWS_REGION", "us-east-1") or "us-east-1").strip()

    has_aws_creds = bool(
        ((os.getenv("AWS_ACCESS_KEY_ID") or "").strip() and (os.getenv("AWS_SECRET_ACCESS_KEY") or "").strip())
        or (os.getenv("AWS_PROFILE") or "").strip()
    )

    if has_aws_creds:
        try:
            import boto3
            boto3.client("bedrock-runtime", region_name=region)
            _log.info("Agent engine: Bedrock boto3 available")
            return "bedrock", None, model_override
        except Exception as e:
            _log.warning("Bedrock boto3 init failed: %s", e)

    api_key = os.getenv("ANTHROPIC_API_KEY", "")
    if not api_key:
        try:
            from modules.humanoid.ai.provider_credentials import get_provider_api_key
            api_key = get_provider_api_key("anthropic") or ""
        except Exception:
            pass

    if api_key and api_key.strip():
        try:
            from anthropic import Anthropic
            client = Anthropic(api_key=api_key.strip())
            _log.info("Agent engine: Anthropic direct available")
            return "anthropic", client, model_override
        except Exception as e:
            _log.warning("Anthropic client init failed: %s", e)

    ollama_model = _ollama_available()
    if ollama_model:
        _log.info("Agent engine: Ollama available (%s)", ollama_model)
        return "ollama", None, model_override or ollama_model

    _log.error("No LLM backend available for agent engine")
    return None, None, None


# ──────────────────────────────────────────────
#  Agentic loop (yields events for SSE)
# ──────────────────────────────────────────────
def run_agent(
    user_message: str,
    conversation_history: Optional[List[Dict]] = None,
    system_prompt: Optional[str] = None,
    model: Optional[str] = None,
) -> Generator[Dict[str, Any], None, None]:
    """
    Run the agentic tool-calling loop with intelligent model routing.

    Classifies task complexity and selects the appropriate model tier:
    - fast (Haiku): simple questions, greetings, status checks
    - balanced (Sonnet): medium tasks, file reads, searches
    - complex (Opus): refactoring, debugging, multi-step implementation

    Yields SSE events: thinking, tool_call, tool_result, text, done, error
    """
    backend_type, client, model_override = _get_llm_backend(model)
    if not backend_type:
        yield {"event": "error", "data": {"message": "No LLM backend available. Configure AWS credentials for Bedrock or set ANTHROPIC_API_KEY."}}
        return

    complexity = classify_complexity(user_message)
    selected_model = select_model(backend_type, complexity, model_override)

    sys_prompt = system_prompt or AGENT_SYSTEM_PROMPT
    messages = list(conversation_history or [])
    messages.append({"role": "user", "content": user_message})

    tools_used = []
    total_t0 = time.perf_counter()

    tier_labels = {"fast": "Rapido", "balanced": "Balanceado", "complex": "Complejo"}
    tier_label = tier_labels.get(complexity, complexity)
    model_short = selected_model.split(".")[-1] if "." in selected_model else selected_model

    yield {"event": "thinking", "data": {
        "message": f"Complejidad: {tier_label} → {model_short}",
        "complexity": complexity,
        "model": selected_model,
    }}

    for iteration in range(MAX_ITERATIONS):
        try:
            t0 = time.perf_counter()

            if backend_type == "bedrock":
                response = _bedrock_converse(selected_model, sys_prompt, messages, TOOLS)
            elif backend_type == "anthropic":
                response = client.messages.create(
                    model=selected_model, max_tokens=4096, system=sys_prompt,
                    tools=TOOLS, messages=messages,
                )
            elif backend_type == "ollama":
                response = _ollama_chat(selected_model, sys_prompt, messages, TOOLS)
            else:
                yield {"event": "error", "data": {"message": f"Unknown backend: {backend_type}"}}
                return

            llm_ms = int((time.perf_counter() - t0) * 1000)
        except Exception as e:
            err_msg = str(e)
            _log.error("LLM call failed (iteration %d, backend=%s, model=%s): %s", iteration, backend_type, selected_model, err_msg)

            if iteration == 0 and backend_type != "ollama" and (
                "credit" in err_msg.lower() or "balance" in err_msg.lower()
                or "auth" in err_msg.lower() or "401" in err_msg
            ):
                fallback_model = _ollama_available()
                if fallback_model:
                    _log.info("Falling back to Ollama (%s)", fallback_model)
                    yield {"event": "thinking", "data": {"message": f"API error, fallback a Ollama ({fallback_model})..."}}
                    backend_type = "ollama"
                    selected_model = fallback_model
                    client = None
                    continue
            yield {"event": "error", "data": {"message": f"LLM error: {err_msg[:300]}", "iteration": iteration}}
            return

        tool_use_blocks = []
        text_parts = []

        for block in response.content:
            if hasattr(block, "type"):
                if block.type == "tool_use":
                    tool_use_blocks.append(block)
                elif block.type == "text" and block.text.strip():
                    text_parts.append(block.text)

        if text_parts and not tool_use_blocks:
            final_text = "\n".join(text_parts)
            yield {"event": "text", "data": {"content": final_text, "llm_ms": llm_ms}}
            total_ms = int((time.perf_counter() - total_t0) * 1000)
            yield {"event": "done", "data": {
                "iterations": iteration + 1,
                "tools_used": tools_used,
                "ms": total_ms,
                "model": selected_model,
                "complexity": complexity,
            }}
            return

        if text_parts:
            for txt in text_parts:
                yield {"event": "thinking", "data": {"message": txt[:300]}}

        assistant_content = []
        for block in response.content:
            if block.type == "text":
                assistant_content.append({"type": "text", "text": block.text})
            elif block.type == "tool_use":
                assistant_content.append({
                    "type": "tool_use",
                    "id": block.id,
                    "name": block.name,
                    "input": block.input,
                })
        messages.append({"role": "assistant", "content": assistant_content})

        tool_results = []
        for tool_block in tool_use_blocks:
            tool_name = tool_block.name
            tool_input = tool_block.input

            yield {"event": "tool_call", "data": {
                "name": tool_name,
                "input": _sanitize_input(tool_name, tool_input),
                "iteration": iteration + 1,
            }}

            t0 = time.perf_counter()

            if tool_name in ("execute_command", "interpreter"):
                import queue, threading
                progress_q = queue.Queue()
                result_holder = [None]

                def _run_tool():
                    result_holder[0] = execute_tool(
                        tool_name, tool_input,
                        progress_callback=lambda line: progress_q.put(line),
                    )
                    progress_q.put(None)

                th = threading.Thread(target=_run_tool, daemon=True)
                th.start()

                last_emit = time.perf_counter()
                while True:
                    try:
                        line = progress_q.get(timeout=0.5)
                        if line is None:
                            break
                        now = time.perf_counter()
                        if now - last_emit > 0.3:
                            yield {"event": "tool_progress", "data": {
                                "name": tool_name,
                                "line": line[:200],
                                "elapsed_s": round(now - t0, 1),
                            }}
                            last_emit = now
                    except queue.Empty:
                        elapsed = time.perf_counter() - t0
                        if elapsed > 5 and int(elapsed) % 5 == 0:
                            yield {"event": "tool_progress", "data": {
                                "name": tool_name,
                                "line": f"... ejecutando ({int(elapsed)}s)",
                                "elapsed_s": round(elapsed, 1),
                            }}

                th.join(timeout=5)
                result_text = result_holder[0] or "Error: tool did not return"
            else:
                result_text = execute_tool(tool_name, tool_input)

            tool_ms = int((time.perf_counter() - t0) * 1000)

            tools_used.append({"name": tool_name, "ms": tool_ms})

            yield {"event": "tool_result", "data": {
                "name": tool_name,
                "output": result_text[:500],
                "ms": tool_ms,
                "ok": not result_text.startswith("Error"),
            }}

            tool_results.append({
                "type": "tool_result",
                "tool_use_id": tool_block.id,
                "content": result_text,
            })

        messages.append({"role": "user", "content": tool_results})
        _audit_tool_calls(tool_use_blocks, tools_used[-len(tool_use_blocks):])

    total_ms = int((time.perf_counter() - total_t0) * 1000)
    yield {"event": "text", "data": {"content": "Alcancé el límite de iteraciones.", "llm_ms": 0}}
    yield {"event": "done", "data": {"iterations": MAX_ITERATIONS, "tools_used": tools_used, "ms": total_ms, "model": selected_model, "complexity": complexity}}


def _sanitize_input(name: str, inp: Dict) -> Dict:
    """Truncate large inputs for SSE display."""
    sanitized = {}
    for k, v in inp.items():
        if isinstance(v, str) and len(v) > 200:
            sanitized[k] = v[:200] + "..."
        else:
            sanitized[k] = v
    return sanitized


def _audit_tool_calls(blocks: list, results: list) -> None:
    """Log tool calls to Atlas audit."""
    try:
        from modules.humanoid.audit import get_audit_logger
        logger = get_audit_logger()
        for block, res in zip(blocks, results):
            logger.log_event(
                "agent_engine", "system", "agent",
                f"tool:{block.name}", True, res.get("ms", 0),
                None, {"input_keys": list(block.input.keys())}, None,
            )
    except Exception:
        pass
