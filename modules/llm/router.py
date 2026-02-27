from __future__ import annotations
import re
from dataclasses import dataclass
from typing import Optional, Literal

RouteName = Literal["FAST", "CHAT", "CODE", "REASON", "TOOLS"]

CODE_HINTS = (
    "python", "fastapi", "uvicorn", "httpx", "pydantic",
    "typescript", "javascript", "node", "pnpm", "vite",
    "docker", "powershell", "ps1", "bash",
    "traceback", "stack trace", "error:", "exception",
    "import ", "def ", "class ", "SELECT ", "CREATE TABLE"
)

TOOLS_HINTS = (
    "tool", "tools", "function calling", "schema", "json schema",
    "invoke", "execute", "endpoint", "/execute", "router", "intent"
)

REASON_HINTS = (
    "razona", "reason", "demuestra", "prueba", "teorema", "análisis profundo",
    "paso a paso", "evalúa opciones", "decide la mejor estrategia"
)

@dataclass
class RouteDecision:
    route: RouteName
    reason: str

class HybridRouter:
    def __init__(self, fast_max_chars: int, chat_max_chars: int, reason_min_chars: int) -> None:
        self.fast_max_chars = fast_max_chars
        self.chat_max_chars = chat_max_chars
        self.reason_min_chars = reason_min_chars

    def decide(self, prompt: str, forced: Optional[str] = None) -> RouteDecision:
        if forced:
            r = forced.upper()
            if r in ("FAST", "CHAT", "CODE", "REASON", "TOOLS"):
                return RouteDecision(route=r, reason="forced_route")

        p = prompt.strip()
        n = len(p)

        if n <= self.fast_max_chars and not self._looks_like_code(p):
            return RouteDecision(route="FAST", reason=f"short_prompt_len={n}")

        if self._looks_like_code(p):
            return RouteDecision(route="CODE", reason="code_hints_detected")

        if self._looks_like_tools(p):
            return RouteDecision(route="TOOLS", reason="tools_hints_detected")

        if n >= self.reason_min_chars and self._looks_like_reason(p):
            return RouteDecision(route="REASON", reason=f"reason_hints_len={n}")

        if n <= self.chat_max_chars:
            return RouteDecision(route="CHAT", reason=f"default_chat_len={n}")

        return RouteDecision(route="REASON", reason=f"very_long_prompt_len={n}")

    def _looks_like_code(self, p: str) -> bool:
        low = p.lower()
        if any(h in low for h in CODE_HINTS):
            return True
        if "```" in p:
            return True
        if re.search(r"\bfrom\s+\w+\s+import\b|\bimport\s+\w+", p):
            return True
        if re.search(r"^\s*(Traceback|File \")", p, re.MULTILINE):
            return True
        return False

    def _looks_like_tools(self, p: str) -> bool:
        low = p.lower()
        return any(h in low for h in TOOLS_HINTS)

    def _looks_like_reason(self, p: str) -> bool:
        low = p.lower()
        return any(h in low for h in REASON_HINTS)
