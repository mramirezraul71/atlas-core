"""
Agentic execution engine for Atlas Workspace.

Implements a Cursor-like tool-calling loop with:
  - Intelligent model routing by task complexity (fast/balanced/complex)
  - Multi-provider failover chain ordered by cost
  - No iteration limit — automatic model escalation on failure
  - Real-time progress tracking with percentage
  - Providers: Bedrock, OpenAI, Gemini, Grok (xAI), DeepSeek, Groq, Ollama
"""
from __future__ import annotations

import json
import logging
import os
import re
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Generator, List, Optional, Tuple
from .execution_runner import ExecutionRunner

_log = logging.getLogger("atlas.agent_engine")

ATLAS_ROOT = Path(os.getenv("ATLAS_ROOT", r"C:\ATLAS_PUSH"))
SAFETY_MAX_ITERATIONS = 50  # Reducido de 100 para prevención de ciclos
MAX_TOOL_OUTPUT = 8000
COMMAND_TIMEOUT = 60

_COMPLETION_PATTERNS = (
    r"\b(completad[oa]|finalizad[oa]|hech[oa]|list[oa]|resuelt[oa])\b",
    r"\b(task|tarea)\s+(completad[ao]|finalizad[ao])\b",
)

# Importar sistema de prevención de ciclos (desactivado por defecto para "modo limpio")
_CYCLE_PREVENTION_ENABLED = os.getenv("AGENT_CYCLE_PREVENTION_ENABLED", "false").strip().lower() in ("1", "true", "yes", "y", "on")
try:
    from .agent_cycle_prevention import get_cycle_prevention
    CYCLE_PREVENTION_AVAILABLE = bool(_CYCLE_PREVENTION_ENABLED)
except ImportError:
    CYCLE_PREVENTION_AVAILABLE = False
    _log.warning("Cycle prevention not available")

# ──────────────────────────────────────────────
#  Multi-provider model catalog
# ──────────────────────────────────────────────
PROVIDER_MODELS = {
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
    "openai": {
        "fast":    "gpt-4.1-mini",
        "balanced": "gpt-4.1-mini",
        "complex": "gpt-4.1",
    },
    "gemini": {
        "fast":    "gemini-2.5-flash",
        "balanced": "gemini-2.5-pro",
        "complex": "gemini-2.5-pro",
    },
    "grok": {
        "fast":    "grok-3-mini",
        "balanced": "grok-3-mini",
        "complex": "grok-3",
    },
    "deepseek": {
        "fast":    "deepseek-chat",
        "balanced": "deepseek-chat",
        "complex": "deepseek-reasoner",
    },
    "groq": {
        "fast":    "llama-3.3-70b-versatile",
        "balanced": "llama-3.3-70b-versatile",
        "complex": "llama-3.3-70b-versatile",
    },
}

FAILOVER_CHAINS = {
    "fast": ["bedrock", "groq", "ollama", "openai", "deepseek", "grok", "gemini"],
    "balanced": ["bedrock", "openai", "gemini", "groq", "grok", "deepseek", "ollama"],
    "complex": ["bedrock", "openai", "gemini", "grok", "deepseek", "groq", "ollama"],
}

# ──────────────────────────────────────────────
#  Complexity classification
# ──────────────────────────────────────────────
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


# ──────────────────────────────────────────────
#  Tool definitions (Anthropic format — converted per provider)
# ──────────────────────────────────────────────
TOOLS = [
    {
        "name": "read_file",
        "description": "Read a file and return its contents with line numbers.",
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
        "description": "Replace exact text in a file. old_text must match exactly.",
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
        "description": "Execute a shell command (PowerShell on Windows). Returns stdout + stderr.",
        "input_schema": {
            "type": "object",
            "properties": {
                "command": {"type": "string"},
                "working_directory": {"type": "string", "description": "Working directory (default: ATLAS_ROOT)"},
                "timeout": {"type": "integer", "description": "Timeout in seconds (default: 60, max: 300)"},
            },
            "required": ["command"],
        },
    },
    {
        "name": "search_text",
        "description": "Search for text pattern across files using ripgrep.",
        "input_schema": {
            "type": "object",
            "properties": {
                "pattern": {"type": "string", "description": "Regex pattern to search for"},
                "directory": {"type": "string", "description": "Directory to search in (default: ATLAS_ROOT)"},
                "file_glob": {"type": "string", "description": "File filter, e.g. '*.py'"},
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
        "description": "Call an internal ATLAS REST endpoint. Endpoints: /health, /audit/tail, /status, /api/autonomy/status, /watchdog/status, /api/monitor/snapshot.",
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
        "description": "Execute a complex autonomous task using Open Interpreter. Use for multi-step code execution, data analysis, web scraping, or iterative programming.",
        "input_schema": {
            "type": "object",
            "properties": {
                "task": {"type": "string", "description": "Natural language description of the task"},
                "language": {"type": "string", "description": "Preferred language: python, shell, javascript (default: auto)"},
            },
            "required": ["task"],
        },
    },
]


def _tools_to_openai_format() -> List[Dict]:
    """Convert Anthropic-format tools to OpenAI function-calling format."""
    return [
        {
            "type": "function",
            "function": {
                "name": t["name"],
                "description": t["description"],
                "parameters": t["input_schema"],
            },
        }
        for t in TOOLS
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
        return f"old_text found {count} times in {path}. Make it unique."
    new_content = content.replace(old_text, new_text, 1)
    path.write_text(new_content, encoding="utf-8")
    return f"Edited {path}: replaced {len(old_text)} chars with {len(new_text)} chars"


def _tool_execute_command(inp: Dict, progress_callback=None) -> str:
    cmd = inp["command"]
    cwd = inp.get("working_directory", str(ATLAS_ROOT))
    timeout = min(inp.get("timeout", COMMAND_TIMEOUT), 300)
    try:
        proc = subprocess.Popen(
            ["powershell", "-Command", cmd],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            cwd=cwd, text=True, encoding="utf-8", errors="replace", bufsize=1,
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
                return f"Command timed out after {timeout}s. Partial:\n" + "\n".join(stdout_lines[-20:])
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
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15, encoding="utf-8", errors="replace")
        out = result.stdout.strip()
        return _truncate(out) if out else f"No matches for '{pattern}' in {directory}"
    except FileNotFoundError:
        cmd_ps = f'Get-ChildItem -Path "{directory}" -Recurse -Filter "{file_glob or "*.py"}" | Select-String -Pattern "{pattern}" | Select-Object -First 30'
        try:
            r = subprocess.run(["powershell", "-Command", cmd_ps], capture_output=True, text=True, timeout=15, encoding="utf-8", errors="replace")
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
        r = requests.get(url, timeout=10) if method == "GET" else requests.post(url, json=body or {}, timeout=10)
        try:
            return _truncate(json.dumps(r.json(), indent=2, ensure_ascii=False, default=str))
        except Exception:
            return _truncate(r.text[:3000])
    except Exception as e:
        return f"API error: {e}"


def _tool_interpreter(inp: Dict) -> str:
    import requests
    task = inp["task"]
    language = inp.get("language", "")
    try:
        payload = {"prompt": task, "auto_run": True}
        if language:
            payload["language"] = language
        r = requests.post("http://127.0.0.1:8791/api/workspace/interpreter/quick", json=payload, timeout=120)
        data = r.json()
        if data.get("ok"):
            return f"[Interpreter OK | {data.get('ms', 0)}ms | model: {data.get('model', 'auto')}]\n{_truncate(data.get('output', ''), 6000)}"
        return f"Interpreter error: {data.get('error', 'unknown')}"
    except Exception as e:
        return f"Interpreter error: {e}"


# ──────────────────────────────────────────────
#  Agent system prompt
# ──────────────────────────────────────────────
AGENT_SYSTEM_PROMPT = """Eres ATLAS, un agente tecnico autonomo con capacidad de ejecutar acciones reales en el sistema.

CAPACIDADES:
- read_file, write_file, edit_file: Leer, crear y editar archivos
- execute_command: Ejecutar comandos de terminal (PowerShell en Windows)
- search_text: Buscar texto en el codigo fuente con ripgrep
- list_directory: Listar archivos y carpetas
- atlas_api: Consultar endpoints internos de ATLAS (/health, /status, /audit/tail)
- interpreter: Ejecutar tareas complejas autonomas (scripts multi-paso, analisis de datos)

REGLAS:
1. Usa las herramientas para HACER, no solo para planificar
2. Cuando te pidan investigar, LEE el codigo real, los logs, la base de datos
3. Cuando te pidan arreglar, EDITA el archivo directamente
4. Para diagnostico de ATLAS, usa atlas_api
5. Verifica tu trabajo: despues de editar, lee el archivo para confirmar
6. Responde en espanol, conciso y directo
7. La raiz del proyecto es """ + str(ATLAS_ROOT) + """

PROHIBIDO:
- Generar planes abstractos sin ejecutarlos
- Sugerir pasos sin hacerlos
- Decir "podrias hacer X" en vez de HACER X"""


# ──────────────────────────────────────────────
#  Provider-specific LLM callers
# ──────────────────────────────────────────────
def _bedrock_converse(model: str, system: str, messages: List[Dict], tools: List[Dict]) -> Dict:
    """Call Bedrock Converse API with tool support via boto3."""
    import boto3
    region = (os.getenv("AWS_REGION", "us-east-1") or "us-east-1").strip()
    client = boto3.client("bedrock-runtime", region_name=region)

    bedrock_tools = [{"toolSpec": {"name": t["name"], "description": t["description"], "inputSchema": {"json": t["input_schema"]}}} for t in tools]

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
                        blocks.append({"toolUse": {"toolUseId": item["id"], "name": item["name"], "input": item["input"]}})
                    elif item.get("type") == "tool_result":
                        blocks.append({"toolResult": {"toolUseId": item["tool_use_id"], "content": [{"text": item["content"] if isinstance(item["content"], str) else json.dumps(item["content"])}]}})
            if blocks:
                bedrock_messages.append({"role": role, "content": blocks})

    resp = client.converse(
        modelId=model, system=[{"text": system}], messages=bedrock_messages,
        toolConfig={"tools": bedrock_tools}, inferenceConfig={"maxTokens": 4096, "temperature": 0},
    )

    result_content = []
    for block in resp.get("output", {}).get("message", {}).get("content", []):
        if "text" in block:
            result_content.append(type("B", (), {"type": "text", "text": block["text"]})())
        elif "toolUse" in block:
            tu = block["toolUse"]
            result_content.append(type("B", (), {"type": "tool_use", "id": tu["toolUseId"], "name": tu["name"], "input": tu["input"]})())
    return type("R", (), {"content": result_content, "stop_reason": resp.get("stopReason", "")})()


def _openai_compatible_call(base_url: str, api_key: str, model: str, system: str, messages: List[Dict], tools: List[Dict]):
    """Call OpenAI-compatible API (OpenAI, Grok, DeepSeek, Groq). Returns Anthropic-like response."""
    import httpx

    oai_messages = [{"role": "system", "content": system}]
    for msg in messages:
        role = msg["role"]
        content = msg["content"]
        if isinstance(content, str):
            oai_messages.append({"role": role, "content": content})
        elif isinstance(content, list):
            parts_text = []
            tool_calls_out = []
            tool_results_out = []
            for item in content:
                if isinstance(item, dict):
                    if item.get("type") == "text":
                        parts_text.append(item["text"])
                    elif item.get("type") == "tool_use":
                        tool_calls_out.append({
                            "id": item["id"], "type": "function",
                            "function": {"name": item["name"], "arguments": json.dumps(item["input"], ensure_ascii=False)},
                        })
                    elif item.get("type") == "tool_result":
                        tool_results_out.append({
                            "role": "tool", "tool_call_id": item["tool_use_id"],
                            "content": item["content"] if isinstance(item["content"], str) else json.dumps(item["content"]),
                        })
            if tool_calls_out:
                oai_messages.append({"role": "assistant", "content": "\n".join(parts_text) if parts_text else None, "tool_calls": tool_calls_out})
            elif parts_text:
                oai_messages.append({"role": role, "content": "\n".join(parts_text)})
            for tr in tool_results_out:
                oai_messages.append(tr)

    oai_tools = _tools_to_openai_format()

    resp = httpx.post(
        f"{base_url}/chat/completions",
        headers={"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"},
        json={"model": model, "messages": oai_messages, "tools": oai_tools, "temperature": 0, "max_tokens": 4096},
        timeout=120,
    )
    resp.raise_for_status()
    data = resp.json()
    choice = data["choices"][0]
    msg_out = choice.get("message", {})

    result_content = []
    if msg_out.get("content"):
        result_content.append(type("B", (), {"type": "text", "text": msg_out["content"]})())
    for tc in msg_out.get("tool_calls", []):
        fn = tc.get("function", {})
        try:
            args = json.loads(fn.get("arguments", "{}"))
        except json.JSONDecodeError:
            args = {}
        result_content.append(type("B", (), {
            "type": "tool_use", "id": tc["id"], "name": fn["name"], "input": args,
        })())

    stop = "tool_use" if msg_out.get("tool_calls") else "end_turn"
    return type("R", (), {"content": result_content, "stop_reason": stop})()


def _gemini_call(api_key: str, model: str, system: str, messages: List[Dict], tools: List[Dict]):
    """Call Google Gemini API with function calling."""
    import httpx

    gemini_tools = [{"function_declarations": [
        {"name": t["name"], "description": t["description"], "parameters": t["input_schema"]}
        for t in tools
    ]}]

    gemini_contents = []
    for msg in messages:
        role = "user" if msg["role"] == "user" else "model"
        content = msg["content"]
        if isinstance(content, str):
            gemini_contents.append({"role": role, "parts": [{"text": content}]})
        elif isinstance(content, list):
            parts = []
            for item in content:
                if isinstance(item, dict):
                    if item.get("type") == "text":
                        parts.append({"text": item["text"]})
                    elif item.get("type") == "tool_use":
                        parts.append({"functionCall": {"name": item["name"], "args": item["input"]}})
                    elif item.get("type") == "tool_result":
                        parts.append({"functionResponse": {"name": "tool", "response": {"result": item["content"] if isinstance(item["content"], str) else json.dumps(item["content"])}}})
            if parts:
                gemini_contents.append({"role": role, "parts": parts})

    url = f"https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent?key={api_key}"
    resp = httpx.post(url, json={
        "contents": gemini_contents,
        "tools": gemini_tools,
        "systemInstruction": {"parts": [{"text": system}]},
        "generationConfig": {"temperature": 0, "maxOutputTokens": 4096},
    }, timeout=120)
    resp.raise_for_status()
    data = resp.json()

    result_content = []
    import uuid
    for part in data.get("candidates", [{}])[0].get("content", {}).get("parts", []):
        if "text" in part:
            result_content.append(type("B", (), {"type": "text", "text": part["text"]})())
        elif "functionCall" in part:
            fc = part["functionCall"]
            result_content.append(type("B", (), {
                "type": "tool_use", "id": f"gemini_{uuid.uuid4().hex[:8]}",
                "name": fc["name"], "input": fc.get("args", {}),
            })())

    stop = "tool_use" if any(hasattr(b, "type") and b.type == "tool_use" for b in result_content) else "end_turn"
    return type("R", (), {"content": result_content, "stop_reason": stop})()


def _ollama_available() -> Optional[str]:
    try:
        import httpx
        r = httpx.get("http://127.0.0.1:11434/api/tags", timeout=3)
        if r.status_code != 200:
            return None
        models = [m["name"] for m in r.json().get("models", [])]
        preferred = ["qwen3:4b", "qwen2.5:7b", "llama3.1:latest", "llama3:latest", "deepseek-r1:14b"]
        for p in preferred:
            if p in models:
                return p
        return models[0] if models else None
    except Exception:
        return None


def _ollama_chat(model: str, system: str, messages: List[Dict], tools: List[Dict]):
    import httpx
    tools_desc = []
    for t in tools:
        params = t["input_schema"].get("properties", {})
        required = t["input_schema"].get("required", [])
        param_lines = [f"  - {pn}: {pi.get('description', pi.get('type', 'string'))}{' (required)' if pn in required else ''}" for pn, pi in params.items()]
        tools_desc.append(f"### {t['name']}\n{t['description']}\nParameters:\n" + "\n".join(param_lines))

    tool_prompt = "You have tools:\n\n" + "\n\n".join(tools_desc) + '\n\nTo use a tool respond with: {"tool_call": {"name": "tool_name", "input": {...}}}\nOtherwise respond with normal text.'
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

    resp = httpx.post("http://127.0.0.1:11434/api/chat", json={"model": model, "messages": ollama_msgs, "stream": False, "options": {"temperature": 0, "num_predict": 2048}}, timeout=120)
    text = resp.json().get("message", {}).get("content", "").strip()

    import re, uuid
    match = re.search(r'\{"tool_call"\s*:\s*\{.*?\}\s*\}', text, re.DOTALL)
    if match:
        try:
            parsed = json.loads(match.group())
            tc = parsed["tool_call"]
            tb = type("B", (), {"type": "tool_use", "id": f"ollama_{uuid.uuid4().hex[:8]}", "name": tc["name"], "input": tc.get("input", {})})()
            thinking = text[:match.start()].strip()
            content = []
            if thinking:
                content.append(type("B", (), {"type": "text", "text": thinking})())
            content.append(tb)
            return type("R", (), {"content": content, "stop_reason": "tool_use"})()
        except (json.JSONDecodeError, KeyError):
            pass
    return type("R", (), {"content": [type("B", (), {"type": "text", "text": text})()], "stop_reason": "end_turn"})()


# ──────────────────────────────────────────────
#  Provider availability detection
# ──────────────────────────────────────────────
def _detect_available_providers() -> Dict[str, Dict]:
    """Detect all available LLM providers. Returns {provider: {key, base_url, ...}}."""
    available = {}
    region = (os.getenv("AWS_REGION", "us-east-1") or "us-east-1").strip()

    has_aws = bool(
        ((os.getenv("AWS_ACCESS_KEY_ID") or "").strip() and (os.getenv("AWS_SECRET_ACCESS_KEY") or "").strip())
        or (os.getenv("AWS_PROFILE") or "").strip()
    )
    if has_aws:
        try:
            import boto3
            boto3.client("bedrock-runtime", region_name=region)
            available["bedrock"] = {"region": region}
        except Exception:
            pass

    api_key = os.getenv("ANTHROPIC_API_KEY", "")
    if not api_key:
        try:
            from modules.humanoid.ai.provider_credentials import get_provider_api_key
            api_key = get_provider_api_key("anthropic") or ""
        except Exception:
            pass
    if api_key and api_key.strip():
        available["anthropic"] = {"key": api_key.strip()}

    oai_key = (os.getenv("OPENAI_API_KEY") or "").strip()
    if oai_key:
        available["openai"] = {"key": oai_key, "base_url": "https://api.openai.com/v1"}

    gemini_key = (os.getenv("GEMINI_API_KEY") or "").strip()
    if gemini_key:
        available["gemini"] = {"key": gemini_key}

    grok_key = (os.getenv("XAI_API_KEY") or "").strip()
    if grok_key:
        available["grok"] = {"key": grok_key, "base_url": "https://api.x.ai/v1"}

    deepseek_key = (os.getenv("DEEPSEEK_API_KEY") or "").strip()
    if deepseek_key:
        available["deepseek"] = {"key": deepseek_key, "base_url": "https://api.deepseek.com/v1"}

    groq_key = (os.getenv("GROQ_API_KEY") or "").strip()
    if groq_key:
        available["groq"] = {"key": groq_key, "base_url": "https://api.groq.com/openai/v1"}

    ollama_model = _ollama_available()
    if ollama_model:
        available["ollama"] = {"model": ollama_model}

    return available


def _call_provider(provider: str, provider_info: Dict, model: str, system: str, messages: List[Dict], tools: List[Dict]):
    """Call a specific LLM provider. Returns response in unified format."""
    if provider == "bedrock":
        return _bedrock_converse(model, system, messages, tools)
    elif provider == "anthropic":
        from anthropic import Anthropic
        client = Anthropic(api_key=provider_info["key"])
        return client.messages.create(model=model, max_tokens=4096, system=system, tools=tools, messages=messages)
    elif provider in ("openai", "grok", "deepseek", "groq"):
        return _openai_compatible_call(provider_info["base_url"], provider_info["key"], model, system, messages, tools)
    elif provider == "gemini":
        return _gemini_call(provider_info["key"], model, system, messages, tools)
    elif provider == "ollama":
        return _ollama_chat(model, system, messages, tools)
    else:
        raise ValueError(f"Unknown provider: {provider}")


# ──────────────────────────────────────────────
#  Agentic loop — unlimited iterations, multi-provider failover
# ──────────────────────────────────────────────
def run_agent(
    user_message: str,
    conversation_history: Optional[List[Dict]] = None,
    system_prompt: Optional[str] = None,
    model: Optional[str] = None,
) -> Generator[Dict[str, Any], None, None]:
    """
    Run the agentic tool-calling loop.

    - No iteration limit (safety cap at 100)
    - Classifies complexity and picks cheapest model
    - On failure, escalates to next provider in failover chain
    - Yields progress events with percentage
    """
    available = _detect_available_providers()
    if not available:
        yield {"event": "error", "data": {"message": "No LLM providers available. Configure at least one: AWS Bedrock, OpenAI, Gemini, Grok, DeepSeek, Groq, or Ollama."}}
        return

    complexity = classify_complexity(user_message)
    chain = FAILOVER_CHAINS.get(complexity, FAILOVER_CHAINS["balanced"])
    active_chain = [p for p in chain if p in available]
    if not active_chain:
        yield {"event": "error", "data": {"message": f"No providers available for complexity '{complexity}'. Available: {list(available.keys())}"}}
        return

    provider_idx = 0
    current_provider = active_chain[provider_idx]
    provider_info = available[current_provider]

    if model:
        selected_model = model
    else:
        models = PROVIDER_MODELS.get(current_provider, {})
        selected_model = models.get(complexity, models.get("balanced", ""))
        if not selected_model and current_provider == "ollama":
            selected_model = provider_info.get("model", "llama3:latest")

    sys_prompt = system_prompt or AGENT_SYSTEM_PROMPT
    messages = list(conversation_history or [])
    messages.append({"role": "user", "content": user_message})

    runner = ExecutionRunner(user_message, ATLAS_ROOT)
    tools_used = []
    checks: List[Dict[str, Any]] = []
    task_contract = _build_task_contract(user_message)
    runner.move_to_plan(task_contract)
    pre_checks = _run_pre_checks(task_contract)
    runner.move_to_execute(pre_checks)
    post_checks: List[Dict[str, Any]] = []
    total_t0 = time.perf_counter()
    consecutive_errors = 0

    # Inicializar prevención de ciclos
    cycle_prevention = None
    if CYCLE_PREVENTION_AVAILABLE:
        cycle_prevention = get_cycle_prevention()
        cycle_prevention.reset()
        _log.info("Cycle prevention enabled")

    tier_labels = {"fast": "Rapido", "balanced": "Balanceado", "complex": "Complejo"}
    tier_label = tier_labels.get(complexity, complexity)
    providers_str = " → ".join(active_chain)

    yield {"event": "thinking", "data": {
        "message": f"Complejidad: {tier_label} | {current_provider}:{selected_model}",
        "complexity": complexity,
        "model": selected_model,
        "provider": current_provider,
        "failover_chain": active_chain,
    }}
    yield {"event": "precheck", "data": {
        "task_contract": task_contract,
        "pre_checks": pre_checks,
    }}

    iteration = 0
    while iteration < SAFETY_MAX_ITERATIONS:
        iteration += 1
        progress_pct = min(int((iteration / SAFETY_MAX_ITERATIONS) * 100), 95)

        # Verificar prevención de ciclos
        if cycle_prevention:
            cycle_check = cycle_prevention.check_iteration_limits()
            if cycle_check["should_escape"]:
                _log.warning(f"Cycle detected: {cycle_check['escape_reason']}")
                
                # Generar mensaje de escape
                escape_message = cycle_prevention.generate_escape_message(
                    cycle_check["escape_reason"],
                    cycle_check["escape_strategy"]
                )
                
                yield {"event": "cycle_detected", "data": {
                    "reason": cycle_check["escape_reason"],
                    "strategy": cycle_check["escape_strategy"],
                    "iteration": iteration,
                    "warnings": cycle_check["warnings"]
                }}
                
                yield {"event": "text", "data": {"content": escape_message, "llm_ms": 0}}
                
                total_ms = int((time.perf_counter() - total_t0) * 1000)
                yield {"event": "done", "data": {
                    "iterations": iteration,
                    "tools_used": tools_used,
                    "task_contract": task_contract,
                    "pre_checks": pre_checks,
                    "post_checks": [{
                        "name": "cycle_escape",
                        "ok": True,
                        "detail": cycle_check["escape_reason"],
                    }],
                    "verification_passed": False,
                    "final_state": "cycle_escaped",
                    "runner_state": runner.state,
                    "runner_timeline": runner.timeline,
                    "kpis": runner.kpis(),
                    "ms": total_ms,
                    "model": selected_model,
                    "provider": current_provider,
                    "complexity": complexity,
                    "progress_pct": progress_pct,
                    "cycle_escaped": True,
                    "escape_reason": cycle_check["escape_reason"]
                }}
                return

        try:
            t0 = time.perf_counter()
            response = _call_provider(current_provider, provider_info, selected_model, sys_prompt, messages, TOOLS)
            llm_ms = int((time.perf_counter() - t0) * 1000)
            consecutive_errors = 0

        except Exception as e:
            err_msg = str(e)
            _log.error("LLM error [%s/%s iter=%d]: %s", current_provider, selected_model, iteration, err_msg[:200])
            consecutive_errors += 1

            if consecutive_errors <= len(active_chain) and provider_idx < len(active_chain) - 1:
                provider_idx += 1
                current_provider = active_chain[provider_idx]
                provider_info = available[current_provider]
                models = PROVIDER_MODELS.get(current_provider, {})
                selected_model = models.get(complexity, models.get("balanced", ""))
                if not selected_model and current_provider == "ollama":
                    selected_model = provider_info.get("model", "llama3:latest")
                _log.info("Failover → %s (%s)", current_provider, selected_model)
                yield {"event": "thinking", "data": {
                    "message": f"Failover → {current_provider}:{selected_model}",
                    "provider": current_provider,
                    "model": selected_model,
                }}
                continue
            else:
                yield {"event": "error", "data": {"message": f"All providers exhausted. Last error ({current_provider}): {err_msg[:200]}", "iteration": iteration}}
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
            post_checks = _run_post_checks(task_contract, pre_checks, checks, final_text)
            runner.move_to_verify(post_checks)
            verification_passed = all(c.get("ok") for c in post_checks) if post_checks else False
            if _looks_like_completion_claim(final_text) and not verification_passed:
                final_text = _blocked_no_verification_message()
            rollback_results: List[Dict[str, Any]] = []
            if runner.should_auto_rollback(verification_passed):
                rollback_results = runner.rollback()
            runner.move_to_report("completed")
            yield {"event": "text", "data": {"content": final_text, "llm_ms": llm_ms}}
            total_ms = int((time.perf_counter() - total_t0) * 1000)
            yield {"event": "done", "data": {
                "iterations": iteration,
                "tools_used": tools_used,
                "checks": checks,
                "task_contract": task_contract,
                "pre_checks": pre_checks,
                "post_checks": post_checks,
                "rollback": rollback_results,
                "verification_passed": verification_passed,
                "final_state": "success_verified" if verification_passed else "blocked_no_verification",
                "runner_state": runner.state,
                "runner_timeline": runner.timeline,
                "kpis": runner.kpis(),
                "ms": total_ms,
                "model": selected_model,
                "provider": current_provider,
                "complexity": complexity,
                "progress_pct": 100,
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
                assistant_content.append({"type": "tool_use", "id": block.id, "name": block.name, "input": block.input})
        messages.append({"role": "assistant", "content": assistant_content})

        tool_results = []
        for tool_block in tool_use_blocks:
            tool_name = tool_block.name
            tool_input = tool_block.input
            runner.before_tool(tool_name, tool_input)

            yield {"event": "tool_call", "data": {
                "name": tool_name,
                "input": _sanitize_input(tool_name, tool_input),
                "iteration": iteration,
                "progress_pct": progress_pct,
            }}

            # Registrar para prevención de ciclos
            if cycle_prevention:
                cycle_prevention.record_tool_call(tool_name, tool_input)

            t0 = time.perf_counter()

            if tool_name in ("execute_command", "interpreter"):
                import queue, threading
                progress_q = queue.Queue()
                result_holder = [None]

                def _run_tool(tn=tool_name, ti=tool_input, pq=progress_q, rh=result_holder):
                    rh[0] = execute_tool(tn, ti, progress_callback=lambda line: pq.put(line))
                    pq.put(None)

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
                            yield {"event": "tool_progress", "data": {"name": tool_name, "line": line[:200], "elapsed_s": round(now - t0, 1)}}
                            last_emit = now
                    except Exception:
                        elapsed = time.perf_counter() - t0
                        if elapsed > 5 and int(elapsed) % 5 == 0:
                            yield {"event": "tool_progress", "data": {"name": tool_name, "line": f"... ejecutando ({int(elapsed)}s)", "elapsed_s": round(elapsed, 1)}}
                th.join(timeout=5)
                result_text = result_holder[0] or "Error: tool did not return"
            else:
                result_text = execute_tool(tool_name, tool_input)

            tool_ms = int((time.perf_counter() - t0) * 1000)
            tools_used.append({"name": tool_name, "ms": tool_ms})
            checks.append({
                "step": len(checks) + 1,
                "tool": tool_name,
                "ok": not result_text.startswith("Error"),
                "evidence": result_text[:180],
                "ms": tool_ms,
            })
            runner.record_tool_result(
                tool_name=tool_name,
                ok=not result_text.startswith("Error"),
                ms=tool_ms,
                output_preview=result_text[:180],
            )

            yield {"event": "tool_result", "data": {
                "name": tool_name,
                "output": result_text[:500],
                "ms": tool_ms,
                "ok": not result_text.startswith("Error"),
                "progress_pct": progress_pct,
            }}

            # Registrar progreso para prevención de ciclos
            if cycle_prevention:
                cycle_prevention.record_progress(progress_pct)

            tool_results.append({"type": "tool_result", "tool_use_id": tool_block.id, "content": result_text})

        messages.append({"role": "user", "content": tool_results})
        _audit_tool_calls(tool_use_blocks, tools_used[-len(tool_use_blocks):])

    total_ms = int((time.perf_counter() - total_t0) * 1000)
    post_checks = _run_post_checks(task_contract, pre_checks, checks, "")
    runner.move_to_verify(post_checks)
    verification_passed = all(c.get("ok") for c in post_checks) if post_checks else False
    final_text = f"Tarea completada tras {SAFETY_MAX_ITERATIONS} iteraciones."
    if not verification_passed:
        final_text = _blocked_no_verification_message()
    rollback_results: List[Dict[str, Any]] = []
    if runner.should_auto_rollback(verification_passed):
        rollback_results = runner.rollback()
    runner.move_to_report("max_iterations_reached")
    yield {"event": "text", "data": {"content": final_text, "llm_ms": 0}}
    yield {"event": "done", "data": {
        "iterations": SAFETY_MAX_ITERATIONS,
        "tools_used": tools_used,
        "checks": checks,
        "task_contract": task_contract,
        "pre_checks": pre_checks,
        "post_checks": post_checks,
        "rollback": rollback_results,
        "verification_passed": verification_passed,
        "final_state": "success_verified" if verification_passed else "blocked_no_verification",
        "runner_state": runner.state,
        "runner_timeline": runner.timeline,
        "kpis": runner.kpis(),
        "ms": total_ms,
        "model": selected_model,
        "provider": current_provider,
        "complexity": complexity,
        "progress_pct": 100
    }}


def _sanitize_input(name: str, inp: Dict) -> Dict:
    sanitized = {}
    for k, v in inp.items():
        if isinstance(v, str) and len(v) > 200:
            sanitized[k] = v[:200] + "..."
        else:
            sanitized[k] = v
    return sanitized


def _build_task_contract(user_message: str) -> Dict[str, Any]:
    lower = (user_message or "").lower()
    needs_runtime = any(k in lower for k in ("puerto", "http://", "https://", "/ui", "/api", "status", "health", "conexion", "conexión"))
    needs_files = any(k in lower for k in ("archivo", "file", ".py", ".json", "ruta", "path", "write", "edit"))
    expected_tools = []
    if needs_runtime:
        expected_tools.extend(["atlas_api", "execute_command"])
    if needs_files:
        expected_tools.extend(["read_file", "write_file", "edit_file"])
    if not expected_tools:
        expected_tools = ["read_file", "execute_command", "atlas_api"]
    return {
        "task_type": "runtime" if needs_runtime else ("file_ops" if needs_files else "generic"),
        "requires_runtime_evidence": needs_runtime,
        "requires_file_evidence": needs_files,
        "expected_tools": expected_tools,
        "success_criteria": [
            "at_least_one_successful_tool_call",
            "no_unverified_completion_claim",
        ],
    }


def _run_pre_checks(contract: Dict[str, Any]) -> List[Dict[str, Any]]:
    checks: List[Dict[str, Any]] = []
    root_ok = ATLAS_ROOT.exists()
    checks.append({
        "name": "atlas_root_exists",
        "ok": root_ok,
        "detail": str(ATLAS_ROOT),
    })
    if contract.get("requires_runtime_evidence"):
        health_out = _tool_atlas_api({"method": "GET", "endpoint": "/health"})
        health_ok = "error" not in health_out.lower()
        checks.append({
            "name": "atlas_health_reachable",
            "ok": health_ok,
            "detail": health_out[:180],
        })
    return checks


def _run_post_checks(contract: Dict[str, Any], pre_checks: List[Dict[str, Any]], tool_checks: List[Dict[str, Any]], final_text: str) -> List[Dict[str, Any]]:
    successful_tools = [c for c in tool_checks if c.get("ok")]
    tool_names = {c.get("tool") for c in successful_tools}
    checks: List[Dict[str, Any]] = [
        {
            "name": "at_least_one_successful_tool_call",
            "ok": len(successful_tools) > 0,
            "detail": f"successful_tools={len(successful_tools)}",
        }
    ]
    if contract.get("requires_runtime_evidence"):
        checks.append({
            "name": "runtime_evidence_present",
            "ok": any(t in tool_names for t in ("atlas_api", "execute_command", "interpreter")),
            "detail": "needs one of atlas_api/execute_command/interpreter",
        })
    if contract.get("requires_file_evidence"):
        checks.append({
            "name": "file_evidence_present",
            "ok": any(t in tool_names for t in ("read_file", "write_file", "edit_file")),
            "detail": "needs one of read_file/write_file/edit_file",
        })
    unverified_claim = _looks_like_completion_claim(final_text or "") and len(successful_tools) == 0
    checks.append({
        "name": "no_unverified_completion_claim",
        "ok": not unverified_claim,
        "detail": "completion claim requires evidence",
    })
    checks.append({
        "name": "pre_checks_ok",
        "ok": all(c.get("ok") for c in pre_checks),
        "detail": f"pre_checks={len(pre_checks)}",
    })
    return checks


def _looks_like_completion_claim(text: str) -> bool:
    t = (text or "").lower()
    return any(re.search(p, t, re.IGNORECASE) for p in _COMPLETION_PATTERNS)


def _blocked_no_verification_message() -> str:
    return (
        "Estado: BLOCKED_NO_VERIFICATION\n"
        "No puedo declarar cumplimiento sin evidencia verificable de ejecución.\n"
        "Ejecuta al menos un paso comprobable (comando/API/archivo) y vuelve a validar."
    )


def _audit_tool_calls(blocks: list, results: list) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        logger = get_audit_logger()
        for block, res in zip(blocks, results):
            logger.log_event("agent_engine", "system", "agent", f"tool:{block.name}", True, res.get("ms", 0), None, {"input_keys": list(block.input.keys())}, None)
    except Exception:
        pass
