"""Configuration for ATLAS autonomous agent."""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path


def _to_bool(value: str | None, default: bool) -> bool:
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "y", "on"}


def _normalize_mode(value: str | None) -> str:
    mode = (value or "safe").strip().lower()
    return mode if mode in {"safe", "aggressive"} else "safe"


@dataclass
class AgentConfig:
    """Runtime configuration loaded from env and CLI."""

    model: str = "gpt-4o-mini"
    temperature: float = 0.1
    max_steps: int = 12
    max_tokens: int = 1200
    objective_max_items: int = 6
    request_timeout_sec: int = 45
    tool_timeout_sec: int = 30
    max_tool_output_chars: int = 6000
    mode: str = "safe"  # safe | aggressive
    allow_shell: bool = True
    dry_run_tools: bool = False
    embedding_model: str = "text-embedding-3-small"
    memory_top_k: int = 3
    workspace: Path = Path(".")
    runs_dir: Path = Path("tools/atlas_agent/runs")
    memory_db: Path = Path("tools/atlas_agent/memory/episodes.sqlite")

    @classmethod
    def from_env(cls) -> "AgentConfig":
        repo_root = Path(__file__).resolve().parents[2]
        try:
            from dotenv import load_dotenv

            load_dotenv(repo_root / ".env", override=False)
            load_dotenv(repo_root / "config" / "atlas.env", override=False)
            # Some runtime initializers preload OPENAI_API_KEY as empty from atlas.env.
            # If that happened, recover the actual key from .env.
            if not (os.getenv("OPENAI_API_KEY") or "").strip():
                load_dotenv(repo_root / ".env", override=True)
        except Exception:
            # dotenv is optional at runtime.
            pass
        workspace = Path(
            os.getenv("ATLAS_AGENT_WORKSPACE") or str(repo_root)
        ).resolve()
        runs_dir = Path(
            os.getenv("ATLAS_AGENT_RUNS_DIR")
            or str((Path(__file__).resolve().parent / "runs"))
        ).resolve()
        return cls(
            model=os.getenv("ATLAS_AGENT_MODEL", "gpt-4o-mini"),
            temperature=float(os.getenv("ATLAS_AGENT_TEMPERATURE", "0.1")),
            max_steps=int(os.getenv("ATLAS_AGENT_MAX_STEPS", "12")),
            max_tokens=int(os.getenv("ATLAS_AGENT_MAX_TOKENS", "1200")),
            objective_max_items=int(os.getenv("ATLAS_AGENT_OBJECTIVE_MAX_ITEMS", "6")),
            request_timeout_sec=int(os.getenv("ATLAS_AGENT_REQUEST_TIMEOUT_SEC", "45")),
            tool_timeout_sec=int(os.getenv("ATLAS_AGENT_TOOL_TIMEOUT_SEC", "30")),
            max_tool_output_chars=int(
                os.getenv("ATLAS_AGENT_MAX_TOOL_OUTPUT_CHARS", "6000")
            ),
            mode=_normalize_mode(os.getenv("ATLAS_AGENT_MODE", "safe")),
            allow_shell=_to_bool(os.getenv("ATLAS_AGENT_ALLOW_SHELL"), True),
            dry_run_tools=_to_bool(os.getenv("ATLAS_AGENT_DRY_RUN_TOOLS"), False),
            embedding_model=os.getenv(
                "ATLAS_AGENT_EMBEDDING_MODEL", "text-embedding-3-small"
            ),
            memory_top_k=int(os.getenv("ATLAS_AGENT_MEMORY_TOP_K", "3")),
            workspace=workspace,
            runs_dir=runs_dir,
            memory_db=Path(
                os.getenv("ATLAS_AGENT_MEMORY_DB")
                or str((Path(__file__).resolve().parent / "memory" / "episodes.sqlite"))
            ).resolve(),
        )

    def with_overrides(
        self,
        *,
        model: str | None = None,
        max_steps: int | None = None,
        workspace: Path | None = None,
        allow_shell: bool | None = None,
        dry_run_tools: bool | None = None,
        mode: str | None = None,
    ) -> "AgentConfig":
        return AgentConfig(
            model=model or self.model,
            temperature=self.temperature,
            max_steps=max_steps if max_steps is not None else self.max_steps,
            max_tokens=self.max_tokens,
            objective_max_items=self.objective_max_items,
            request_timeout_sec=self.request_timeout_sec,
            tool_timeout_sec=self.tool_timeout_sec,
            max_tool_output_chars=self.max_tool_output_chars,
            mode=_normalize_mode(mode or self.mode),
            allow_shell=self.allow_shell if allow_shell is None else allow_shell,
            dry_run_tools=(
                self.dry_run_tools if dry_run_tools is None else dry_run_tools
            ),
            embedding_model=self.embedding_model,
            memory_top_k=self.memory_top_k,
            workspace=(workspace or self.workspace).resolve(),
            runs_dir=self.runs_dir.resolve(),
            memory_db=self.memory_db.resolve(),
        )
