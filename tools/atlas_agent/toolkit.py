"""Local tools executed by ATLAS autonomous agent."""
from __future__ import annotations

import json
import subprocess
from difflib import unified_diff
from hashlib import sha256
from pathlib import Path
from typing import Any, Dict, List

try:
    from .config import AgentConfig
except Exception:  # pragma: no cover - script-mode fallback
    from config import AgentConfig


DENY_PATTERNS = [
    "git reset --hard",
    "git clean -fd",
    "del /f /q",
    "rd /s /q",
    "format ",
    "shutdown ",
    "reboot",
]


def _limit_text(text: str, max_chars: int) -> str:
    if len(text) <= max_chars:
        return text
    return text[:max_chars] + f"\n...[truncated {len(text)-max_chars} chars]"


def _hash_text(text: str) -> str:
    return sha256(text.encode("utf-8", errors="ignore")).hexdigest()[:16]


class AtlasToolkit:
    """Tool registry and dispatch with workspace safety constraints."""

    def __init__(self, config: AgentConfig):
        self.config = config
        self.workspace = config.workspace.resolve()

    def _safe_path(self, value: str) -> Path:
        path = (self.workspace / value).resolve()
        if self.workspace not in path.parents and path != self.workspace:
            raise ValueError(f"path out of workspace: {value}")
        return path

    def describe_tools(self) -> List[Dict[str, Any]]:
        return [
            {
                "name": "list_files",
                "args_schema": {"path": "string (optional)", "max_entries": "int (optional)"},
                "description": "List files under workspace.",
            },
            {
                "name": "read_file",
                "args_schema": {"path": "string", "max_chars": "int (optional)"},
                "description": "Read file content as text.",
            },
            {
                "name": "search_text",
                "args_schema": {"pattern": "string", "path": "string (optional)"},
                "description": "Search text using ripgrep if available.",
            },
            {
                "name": "write_file",
                "args_schema": {"path": "string", "content": "string"},
                "description": "Write full file content.",
            },
            {
                "name": "run_shell",
                "args_schema": {"command": "string"},
                "description": "Run shell command inside workspace.",
            },
        ]

    def dispatch(self, tool: str, args: Dict[str, Any]) -> Dict[str, Any]:
        try:
            if self.config.dry_run_tools:
                return {"ok": True, "tool": tool, "dry_run": True, "args": args}
            if tool == "list_files":
                return self.list_files(
                    path=str(args.get("path") or "."),
                    max_entries=int(args.get("max_entries") or 200),
                )
            if tool == "read_file":
                return self.read_file(
                    path=str(args.get("path") or ""),
                    max_chars=int(args.get("max_chars") or self.config.max_tool_output_chars),
                )
            if tool == "search_text":
                return self.search_text(
                    pattern=str(args.get("pattern") or ""),
                    path=str(args.get("path") or "."),
                )
            if tool == "write_file":
                return self.write_file(
                    path=str(args.get("path") or ""),
                    content=str(args.get("content") or ""),
                )
            if tool == "run_shell":
                return self.run_shell(command=str(args.get("command") or ""))
            return {"ok": False, "error": f"unknown tool: {tool}"}
        except Exception as exc:
            return {"ok": False, "error": str(exc), "tool": tool}

    def list_files(self, path: str = ".", max_entries: int = 200) -> Dict[str, Any]:
        try:
            root = self._safe_path(path)
        except Exception as exc:
            return {"ok": False, "error": str(exc)}
        entries: List[str] = []
        if root.is_file():
            entries = [str(root.relative_to(self.workspace))]
        else:
            for p in sorted(root.rglob("*")):
                if len(entries) >= max_entries:
                    break
                if ".git" in p.parts:
                    continue
                entries.append(str(p.relative_to(self.workspace)))
        return {"ok": True, "path": str(root), "entries": entries}

    def read_file(self, path: str, max_chars: int = 6000) -> Dict[str, Any]:
        if not path:
            return {"ok": False, "error": "path is required"}
        try:
            file_path = self._safe_path(path)
        except Exception as exc:
            return {"ok": False, "error": str(exc)}
        if not file_path.exists():
            return {"ok": False, "error": f"file not found: {path}"}
        if file_path.is_dir():
            return {"ok": False, "error": f"path is a directory: {path}"}
        text = file_path.read_text(encoding="utf-8", errors="replace")
        return {
            "ok": True,
            "path": str(file_path.relative_to(self.workspace)),
            "content": _limit_text(text, max_chars),
            "size": file_path.stat().st_size,
        }

    def search_text(self, pattern: str, path: str = ".") -> Dict[str, Any]:
        if not pattern:
            return {"ok": False, "error": "pattern is required"}
        try:
            target = self._safe_path(path)
        except Exception as exc:
            return {"ok": False, "error": str(exc)}
        rg_cmd = ["rg", "-n", pattern, str(target)]
        try:
            proc = subprocess.run(
                rg_cmd,
                cwd=str(self.workspace),
                text=True,
                capture_output=True,
                timeout=self.config.tool_timeout_sec,
            )
            out = proc.stdout if proc.returncode in (0, 1) else proc.stderr
            return {
                "ok": proc.returncode in (0, 1),
                "returncode": proc.returncode,
                "output": _limit_text(out or "", self.config.max_tool_output_chars),
            }
        except FileNotFoundError:
            # Fallback to a python-based grep.
            matches: List[str] = []
            for p in target.rglob("*"):
                if p.is_dir() or ".git" in p.parts:
                    continue
                try:
                    text = p.read_text(encoding="utf-8", errors="ignore")
                except Exception:
                    continue
                for idx, line in enumerate(text.splitlines(), start=1):
                    if pattern in line:
                        matches.append(
                            f"{p.relative_to(self.workspace)}:{idx}:{line[:180]}"
                        )
                        if len(matches) >= 200:
                            break
                if len(matches) >= 200:
                    break
            return {"ok": True, "returncode": 0, "output": "\n".join(matches)}

    def write_file(self, path: str, content: str) -> Dict[str, Any]:
        if not path:
            return {"ok": False, "error": "path is required"}
        try:
            file_path = self._safe_path(path)
        except Exception as exc:
            return {"ok": False, "error": str(exc)}
        file_path.parent.mkdir(parents=True, exist_ok=True)
        before = ""
        existed = file_path.exists()
        if existed and file_path.is_file():
            before = file_path.read_text(encoding="utf-8", errors="replace")
        file_path.write_text(content, encoding="utf-8")
        diff_text = "\n".join(
            unified_diff(
                before.splitlines(),
                content.splitlines(),
                fromfile=f"a/{file_path.relative_to(self.workspace)}",
                tofile=f"b/{file_path.relative_to(self.workspace)}",
                lineterm="",
            )
        )
        return {
            "ok": True,
            "path": str(file_path.relative_to(self.workspace)),
            "bytes": len(content.encode("utf-8")),
            "existed": existed,
            "changed": before != content,
            "before_hash": _hash_text(before),
            "after_hash": _hash_text(content),
            "diff": _limit_text(diff_text, self.config.max_tool_output_chars),
        }

    def run_shell(self, command: str) -> Dict[str, Any]:
        if not self.config.allow_shell:
            return {"ok": False, "error": "shell tool is disabled"}
        clean = (command or "").strip()
        if not clean:
            return {"ok": False, "error": "command is empty"}
        lower = clean.lower()
        for pat in DENY_PATTERNS:
            if pat in lower:
                return {"ok": False, "error": f"blocked command pattern: {pat}"}
        proc = subprocess.run(
            clean,
            cwd=str(self.workspace),
            shell=True,
            text=True,
            capture_output=True,
            timeout=self.config.tool_timeout_sec,
        )
        payload = {
            "ok": proc.returncode == 0,
            "returncode": proc.returncode,
            "stdout": _limit_text(proc.stdout or "", self.config.max_tool_output_chars),
            "stderr": _limit_text(proc.stderr or "", self.config.max_tool_output_chars),
        }
        # Keep response shape compact for model context.
        return json.loads(json.dumps(payload, ensure_ascii=False))
