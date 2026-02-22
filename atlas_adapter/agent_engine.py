"""
Agentic execution engine for Atlas Workspace.

Implements a Cursor-like tool-calling loop:
  User message → LLM (with tools) → tool_use → execute → result → LLM → ... → final text

Supports: Bedrock (Claude), Anthropic, OpenAI, Gemini via their native tool calling APIs.
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
COMMAND_TIMEOUT = 30

# ──────────────────────────────────────────────
#  Tool definitions (Claude/Anthropic format)
# ──────────────────────────────────────────────
TOOLS = [
    {
        "name": "read_file",
        "description": "Read a file and return its contents. Use for inspecting source code, configs, logs.",
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
        "description": "Replace exact text in a file. old_text must match exactly (including whitespace).",
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
            },
            "required": ["command"],
        },
    },
    {
        "name": "search_text",
        "description": "Search for text pattern across files using ripgrep. Returns matching lines with file paths.",
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
        "description": "List files and subdirectories at a path.",
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
        "description": "Call an internal ATLAS REST endpoint. Use for diagnostics: /health, /audit/tail, /status, /api/autonomy/status, /watchdog/status, /api/libro-vida/status, etc.",
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
]


# ──────────────────────────────────────────────
#  Tool execution
# ──────────────────────────────────────────────
def _truncate(text: str, limit: int = MAX_TOOL_OUTPUT) -> str:
    if len(text) <= limit:
        return text
    half = limit // 2 - 50
    return text[:half] + f"\n\n... ({len(text) - limit} chars truncated) ...\n\n" + text[-half:]


def execute_tool(name: str, inp: Dict[str, Any]) -> str:
    """Execute a tool and return the result as text."""
    try:
        if name == "read_file":
            return _tool_read_file(inp)
        elif name == "write_file":
            return _tool_write_file(inp)
        elif name == "edit_file":
            return _tool_edit_file(inp)
        elif name == "execute_command":
            return _tool_execute_command(inp)
        elif name == "search_text":
            return _tool_search_text(inp)
        elif name == "list_directory":
            return _tool_list_directory(inp)
        elif name == "atlas_api":
            return _tool_atlas_api(inp)
        else:
            return f"Error: unknown tool '{name}'"
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


def _tool_execute_command(inp: Dict) -> str:
    cmd = inp["command"]
    cwd = inp.get("working_directory", str(ATLAS_ROOT))
    try:
        result = subprocess.run(
            ["powershell", "-Command", cmd],
            capture_output=True, text=True, timeout=COMMAND_TIMEOUT,
            cwd=cwd, encoding="utf-8", errors="replace",
        )
        out = ""
        if result.stdout:
            out += result.stdout
        if result.stderr:
            out += "\nSTDERR:\n" + result.stderr
        if result.returncode != 0:
            out += f"\n[exit code: {result.returncode}]"
        return _truncate(out.strip() or "(no output)")
    except subprocess.TimeoutExpired:
        return f"Command timed out after {COMMAND_TIMEOUT}s"
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
        # rg not available, fallback to findstr on Windows
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


# ──────────────────────────────────────────────
#  Agent system prompt
# ──────────────────────────────────────────────
AGENT_SYSTEM_PROMPT = """Eres ATLAS, un agente tecnico autonomo con capacidad de ejecutar acciones reales en el sistema.

CAPACIDADES:
- Leer, crear y editar archivos del sistema
- Ejecutar comandos de terminal (PowerShell en Windows)
- Buscar texto en el codigo fuente
- Consultar endpoints internos de ATLAS para diagnostico

REGLAS:
1. Usa las herramientas para HACER, no solo para planificar
2. Cuando te pidan investigar algo, LEE el codigo real, los logs, la base de datos
3. Cuando te pidan arreglar algo, EDITA el archivo directamente
4. Cuando te pidan ejecutar algo, USA execute_command
5. Para diagnostico de ATLAS, usa atlas_api con /health, /audit/tail, /status, etc.
6. Siempre verifica tu trabajo: despues de editar, lee el archivo para confirmar
7. Responde en espanol, conciso y directo
8. La raiz del proyecto es """ + str(ATLAS_ROOT) + """
9. Maximo 3-5 acciones por tarea. No sobre-analices.

PROHIBIDO:
- Generar planes abstractos sin ejecutarlos
- Sugerir pasos sin hacerlos
- Decir "podrias hacer X" en vez de HACER X"""


# ──────────────────────────────────────────────
#  Agentic loop (yields events for SSE)
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
    """Call Ollama with tool-calling via structured prompt.
    
    Returns a response object compatible with the Anthropic format.
    """
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


def _get_llm_backend(model_override: Optional[str] = None):
    """Determine which LLM backend to use for tool calling.
    
    Priority: Bedrock boto3 > Anthropic direct > Ollama.
    Returns (backend_type, client_or_None, model_id).
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
            model = model_override or "us.anthropic.claude-opus-4-6-v1"
            _log.info("Agent engine: using Bedrock boto3 (%s)", model)
            return "bedrock_boto3", None, model
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
            model = model_override or "claude-sonnet-4-20250514"
            _log.info("Agent engine: using Anthropic direct (%s)", model)
            return "anthropic", client, model
        except Exception as e:
            _log.warning("Anthropic client init failed: %s", e)
    ollama_model = _ollama_available()
    if ollama_model:
        model = model_override or ollama_model
        _log.info("Agent engine: using Ollama (%s)", model)
        return "ollama", None, model

    _log.error("No LLM backend available for agent engine")
    return None, None, None


def run_agent(
    user_message: str,
    conversation_history: Optional[List[Dict]] = None,
    system_prompt: Optional[str] = None,
    model: Optional[str] = None,
) -> Generator[Dict[str, Any], None, None]:
    """
    Run the agentic tool-calling loop. Yields event dicts for SSE streaming.

    Uses Bedrock Converse API (boto3) for reliable tool calling with Claude.
    """
    backend, client, default_model = _get_llm_backend(model)
    if not backend:
        yield {"event": "error", "data": {"message": "No LLM backend available. Configure AWS credentials for Bedrock or set ANTHROPIC_API_KEY."}}
        return

    model = model or default_model
    sys_prompt = system_prompt or AGENT_SYSTEM_PROMPT

    messages = list(conversation_history or [])
    messages.append({"role": "user", "content": user_message})

    tools_used = []
    total_t0 = time.perf_counter()

    yield {"event": "thinking", "data": {"message": f"Conectando con {model.split(':')[-1] if ':' in model else model}..."}}

    for iteration in range(MAX_ITERATIONS):
        try:
            t0 = time.perf_counter()

            if backend == "bedrock_boto3":
                response = _bedrock_converse(model, sys_prompt, messages, TOOLS)
            elif backend in ("bedrock_anthropic", "anthropic"):
                response = client.messages.create(
                    model=model, max_tokens=4096, system=sys_prompt,
                    tools=TOOLS, messages=messages,
                )
            elif backend == "ollama":
                response = _ollama_chat(model, sys_prompt, messages, TOOLS)
            else:
                yield {"event": "error", "data": {"message": f"Unknown backend: {backend}"}}
                return

            llm_ms = int((time.perf_counter() - t0) * 1000)
        except Exception as e:
            err_msg = str(e)
            _log.error("LLM call failed (iteration %d, backend=%s): %s", iteration, backend, err_msg)
            if iteration == 0 and backend != "ollama" and ("credit" in err_msg.lower() or "balance" in err_msg.lower()
                                                           or "auth" in err_msg.lower() or "401" in err_msg):
                fallback_model = _ollama_available()
                if fallback_model:
                    _log.info("Falling back to Ollama (%s)", fallback_model)
                    yield {"event": "thinking", "data": {"message": f"API error, switching to Ollama ({fallback_model})..."}}
                    backend = "ollama"
                    model = fallback_model
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

        # Final text response (no more tool calls)
        if text_parts and not tool_use_blocks:
            final_text = "\n".join(text_parts)
            yield {"event": "text", "data": {"content": final_text, "llm_ms": llm_ms}}
            total_ms = int((time.perf_counter() - total_t0) * 1000)
            yield {"event": "done", "data": {
                "iterations": iteration + 1,
                "tools_used": tools_used,
                "ms": total_ms,
                "model": model,
            }}
            return

        # Intermediate thinking text
        if text_parts:
            for txt in text_parts:
                yield {"event": "thinking", "data": {"message": txt[:300]}}

        # Build assistant message with tool_use blocks
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

        # Execute tools and collect results
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
    yield {"event": "done", "data": {"iterations": MAX_ITERATIONS, "tools_used": tools_used, "ms": total_ms, "model": model}}


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
