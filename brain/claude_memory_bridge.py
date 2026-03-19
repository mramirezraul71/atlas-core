"""Claude-Atlas Memory Bridge v1.0

Fusión bidireccional entre la memoria persistente de Claude Code y el cerebro de Atlas.

Flujo:
  Claude MEMORY.md  ──► parse ──► EpisodicMemory (SQLite) + strategic_memory (JSON)
  Atlas brain        ──► export ──► memory/atlas_brain_context.json  (Claude lo lee al inicio)

Uso directo:
    bridge = ClaudeMemoryBridge()
    bridge.sync_claude_to_atlas()   # MEMORY.md → Atlas brain
    bridge.sync_atlas_to_claude()   # Atlas brain → claude_context.json
    bridge.bidirectional_sync()     # Ambos en una sola llamada
"""
from __future__ import annotations

import json
import os
import sqlite3
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

# ─── Rutas base ───────────────────────────────────────────────────────────────
_ROOT = Path(__file__).resolve().parent.parent           # C:\ATLAS_PUSH
_CLAUDE_MEMORY_DIR = Path(os.environ.get(
    "CLAUDE_MEMORY_DIR",
    r"C:\Users\r6957\.claude\projects\c--ATLAS-PUSH\memory"
))
_STRATEGIC_DIR    = _ROOT / "memory_engine" / "strategic_memory"
_DECISIONS_LOG    = _STRATEGIC_DIR / "decisions.log"
_PATTERNS_JSON    = _STRATEGIC_DIR / "learned_patterns.json"
_EPISODIC_DB      = _ROOT / "logs" / "learning_episodic.sqlite"
_BRIDGE_STATE     = _ROOT / "memory_engine" / "claude_bridge_state.json"
_CLAUDE_CONTEXT   = _CLAUDE_MEMORY_DIR / "atlas_brain_context.json"   # Claude lee esto


# ─── Helpers internos ─────────────────────────────────────────────────────────

def _now_iso() -> str:
    return datetime.now().isoformat(timespec="seconds")


def _load_json(path: Path, default: Any = None) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return default if default is not None else {}


def _save_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def _append_decision(msg: str, level: str = "INFO", subsystem: str = "claude_bridge") -> None:
    _DECISIONS_LOG.parent.mkdir(parents=True, exist_ok=True)
    line = f"[{_now_iso()}] [{level}] {subsystem} :: {msg}\n"
    with open(_DECISIONS_LOG, "a", encoding="utf-8") as f:
        f.write(line)


# ─── Parser de MEMORY.md ──────────────────────────────────────────────────────

def _parse_memory_md(memory_dir: Path) -> Dict[str, Any]:
    """
    Lee MEMORY.md y archivos de tema en el directorio de memoria de Claude.
    Retorna un dict con secciones parseadas como: {section_title: content_text}.
    """
    result: Dict[str, Any] = {"sections": {}, "raw": "", "files": {}}
    main_path = memory_dir / "MEMORY.md"
    if not main_path.exists():
        return result
    raw = main_path.read_text(encoding="utf-8")
    result["raw"] = raw

    # Parsear secciones H2 (## Título)
    current_section = "_preamble"
    current_lines: List[str] = []
    for line in raw.splitlines():
        if line.startswith("## "):
            if current_lines:
                result["sections"][current_section] = "\n".join(current_lines).strip()
            current_section = line[3:].strip()
            current_lines = []
        else:
            current_lines.append(line)
    if current_lines:
        result["sections"][current_section] = "\n".join(current_lines).strip()

    # Leer archivos de tema adicionales (*.md excepto MEMORY.md)
    for f in memory_dir.glob("*.md"):
        if f.name != "MEMORY.md":
            result["files"][f.stem] = f.read_text(encoding="utf-8")

    return result


# ─── Escritura en EpisodicMemory (SQLite directo) ─────────────────────────────

def _ensure_episodic_schema(conn: sqlite3.Connection) -> None:
    conn.execute("""
        CREATE TABLE IF NOT EXISTS episodes (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp REAL NOT NULL,
            situation TEXT NOT NULL,
            context TEXT,
            action_taken TEXT,
            result TEXT,
            success INTEGER,
            uncertainty_score REAL DEFAULT 0.0,
            asked_for_help INTEGER DEFAULT 0,
            new_knowledge_count INTEGER DEFAULT 0,
            task_type TEXT DEFAULT 'general',
            tags TEXT,
            importance REAL DEFAULT 0.5,
            times_recalled INTEGER DEFAULT 0,
            created_at TEXT,
            metadata TEXT
        )
    """)
    conn.commit()


def _write_episode(
    situation: str,
    context: Dict[str, Any],
    action_taken: str,
    result: str,
    task_type: str,
    tags: List[str],
    importance: float = 0.7,
    success: bool = True,
) -> int:
    """Escribe un episodio en la BD SQLite de EpisodicMemory. Retorna episode_id."""
    _EPISODIC_DB.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(_EPISODIC_DB)
    _ensure_episodic_schema(conn)
    now = time.time()
    cursor = conn.execute(
        """
        INSERT INTO episodes (
            timestamp, situation, context, action_taken, result, success,
            task_type, tags, importance, created_at, metadata
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            now,
            situation[:2000],
            json.dumps(context, ensure_ascii=False)[:5000],
            action_taken[:500],
            result[:2000],
            1 if success else 0,
            task_type[:200],
            ",".join(tags),
            importance,
            _now_iso(),
            json.dumps({"source": "claude_bridge"}, ensure_ascii=False),
        ),
    )
    episode_id = cursor.lastrowid or 0
    conn.commit()
    conn.close()
    return episode_id


# ─── Mapeo sección MEMORY.md → tipo episodio ─────────────────────────────────

_SECTION_MAP: Dict[str, Dict[str, Any]] = {
    "Arquitectura General":    {"task_type": "architecture",      "importance": 0.9, "tags": ["arquitectura", "claude_memory"]},
    "Componentes Críticos":    {"task_type": "architecture",      "importance": 0.85,"tags": ["componentes", "claude_memory"]},
    "Batería de Modelos":      {"task_type": "model_config",      "importance": 0.9, "tags": ["modelos", "ai", "claude_memory"]},
    "Endpoints v4 añadidos":   {"task_type": "api_knowledge",     "importance": 0.8, "tags": ["endpoints", "api", "claude_memory"]},
    "Dependencias del venv":   {"task_type": "dependencies",      "importance": 0.7, "tags": ["deps", "venv", "claude_memory"]},
    "Patrones de código":      {"task_type": "code_patterns",     "importance": 0.85,"tags": ["patterns", "code", "claude_memory"]},
    "Dashboard v4 — Estado":   {"task_type": "ui_state",          "importance": 0.8, "tags": ["dashboard", "ui", "claude_memory"]},
    "Preferencias del Usuario":{"task_type": "user_preferences",  "importance": 0.95,"tags": ["preferencias", "usuario", "claude_memory"]},
    "Hardening Aplicado":      {"task_type": "security_fix",      "importance": 0.85,"tags": ["hardening", "security", "claude_memory"]},
    "Estado del Branch":       {"task_type": "git_state",         "importance": 0.75,"tags": ["git", "branch", "claude_memory"]},
    "Notas de Seguridad":      {"task_type": "security_notes",    "importance": 0.9, "tags": ["security", "notes", "claude_memory"]},
    "Protocolo de Jerarquía":  {"task_type": "protocol",          "importance": 0.85,"tags": ["jerarquía", "protocolo", "claude_memory"]},
}


# ─── ClaudeMemoryBridge ────────────────────────────────────────────────────────

class ClaudeMemoryBridge:
    """
    Puente bidireccional Claude Code ↔ Atlas Brain.

    - sync_claude_to_atlas(): Lee MEMORY.md → escribe episodios en brain
    - sync_atlas_to_claude(): Lee Atlas brain → escribe atlas_brain_context.json para Claude
    - bidirectional_sync()  : Ambos en una sola llamada
    - write_insight(...)    : Escribe un insight puntual de la sesión Claude al brain
    - get_bridge_stats()    : Estado del puente
    """

    def __init__(
        self,
        claude_memory_dir: Optional[str] = None,
        verbose: bool = False,
    ) -> None:
        self.memory_dir = Path(claude_memory_dir) if claude_memory_dir else _CLAUDE_MEMORY_DIR
        self.verbose = verbose
        self._state = _load_json(_BRIDGE_STATE, {"syncs": 0, "last_sync": None, "episodes_written": 0})

    # ── Claude → Atlas ─────────────────────────────────────────────────────────

    def sync_claude_to_atlas(self) -> Dict[str, Any]:
        """
        Lee MEMORY.md + archivos de tema de Claude → escribe en Atlas brain.
        Retorna resumen: {episodes_written, patterns_updated, sections_processed}.
        """
        parsed = _parse_memory_md(self.memory_dir)
        if not parsed["raw"]:
            return {"ok": False, "error": "MEMORY.md no encontrado", "episodes_written": 0}

        episodes_written = 0
        sections_processed: List[str] = []
        patterns_updated: List[str] = []

        for section, content in parsed["sections"].items():
            if not content or section == "_preamble":
                continue
            cfg = _SECTION_MAP.get(section, {
                "task_type": "general_memory",
                "importance": 0.6,
                "tags": ["claude_memory", "misc"],
            })

            eid = _write_episode(
                situation=f"[Claude Memory] {section}",
                context={"section": section, "source_file": "MEMORY.md"},
                action_taken="Sincronización automática Claude→Atlas",
                result=content[:2000],
                task_type=cfg["task_type"],
                tags=cfg["tags"],
                importance=cfg["importance"],
                success=True,
            )
            episodes_written += 1
            sections_processed.append(section)

            # Secciones de código → actualizar learned_patterns.json
            if cfg["task_type"] in ("code_patterns", "architecture", "protocol"):
                patterns_updated.append(section)

        # Sincronizar archivos de tema adicionales
        for fname, fcontent in parsed["files"].items():
            eid = _write_episode(
                situation=f"[Claude Memory File] {fname}",
                context={"file": fname, "source_dir": str(self.memory_dir)},
                action_taken="Sincronización archivo tema Claude→Atlas",
                result=fcontent[:2000],
                task_type="topic_file",
                tags=["claude_memory", "topic", fname],
                importance=0.65,
                success=True,
            )
            episodes_written += 1

        # Actualizar learned_patterns.json con las secciones de patrones
        if patterns_updated:
            pdata = _load_json(_PATTERNS_JSON, {"version": 1, "updated_at": _now_iso(), "patterns": []})
            existing_names = {p.get("name") for p in pdata.get("patterns", [])}
            added = 0
            for sec in patterns_updated:
                pattern_name = f"claude_memory:{sec.lower().replace(' ', '_')}"
                if pattern_name not in existing_names:
                    pdata["patterns"].append({
                        "name": pattern_name,
                        "source": "claude_memory_bridge",
                        "section": sec,
                        "learned_at": _now_iso(),
                        "confidence": 0.9,
                    })
                    added += 1
            pdata["updated_at"] = _now_iso()
            _save_json(_PATTERNS_JSON, pdata)

        # Log decisión estratégica
        _append_decision(
            f"sync_claude_to_atlas completado: {episodes_written} episodios, "
            f"{len(patterns_updated)} patrones actualizados",
            level="INFO",
            subsystem="claude_bridge",
        )

        # Actualizar estado del puente
        self._state["syncs"] = self._state.get("syncs", 0) + 1
        self._state["last_sync"] = _now_iso()
        self._state["episodes_written"] = self._state.get("episodes_written", 0) + episodes_written
        self._state["last_claude_to_atlas"] = _now_iso()
        _save_json(_BRIDGE_STATE, self._state)

        return {
            "ok": True,
            "episodes_written": episodes_written,
            "sections_processed": sections_processed,
            "patterns_updated": patterns_updated,
            "timestamp": _now_iso(),
        }

    # ── Atlas → Claude ─────────────────────────────────────────────────────────

    def sync_atlas_to_claude(self, top_n: int = 30) -> Dict[str, Any]:
        """
        Lee Atlas brain (episodios importantes + patrones + decisiones recientes)
        → escribe atlas_brain_context.json en el directorio de memoria de Claude.
        """
        # Episodios de alta importancia
        episodes: List[Dict[str, Any]] = []
        if _EPISODIC_DB.exists():
            try:
                conn = sqlite3.connect(_EPISODIC_DB)
                conn.row_factory = sqlite3.Row
                rows = conn.execute(
                    """
                    SELECT id, situation, action_taken, result, task_type, tags,
                           importance, created_at, success
                    FROM episodes
                    WHERE importance >= 0.75
                    ORDER BY importance DESC, timestamp DESC
                    LIMIT ?
                    """,
                    (top_n,),
                ).fetchall()
                conn.close()
                for r in rows:
                    episodes.append({
                        "id": r["id"],
                        "situation": r["situation"],
                        "action": r["action_taken"],
                        "result": (r["result"] or "")[:300],
                        "type": r["task_type"],
                        "tags": r["tags"],
                        "importance": r["importance"],
                        "at": r["created_at"],
                        "success": bool(r["success"]),
                    })
            except Exception as e:
                episodes = [{"error": str(e)}]

        # Patrones aprendidos
        patterns = _load_json(_PATTERNS_JSON, {"patterns": []}).get("patterns", [])

        # Últimas 15 líneas del decisions.log
        recent_decisions: List[str] = []
        if _DECISIONS_LOG.exists():
            lines = _DECISIONS_LOG.read_text(encoding="utf-8").splitlines()
            recent_decisions = [l for l in lines if l.strip()][-15:]

        # Estado del bridge
        bridge_state = _load_json(_BRIDGE_STATE, {})

        context: Dict[str, Any] = {
            "generated_at": _now_iso(),
            "atlas_version": self._read_version(),
            "bridge_state": bridge_state,
            "top_episodes": episodes,
            "learned_patterns": patterns[-20:],
            "recent_decisions": recent_decisions,
            "summary": {
                "total_episodes_exported": len(episodes),
                "total_patterns": len(patterns),
                "decisions_tail": len(recent_decisions),
            },
        }

        self.memory_dir.mkdir(parents=True, exist_ok=True)
        _save_json(_CLAUDE_CONTEXT, context)

        _append_decision(
            f"sync_atlas_to_claude: {len(episodes)} episodios, {len(patterns)} patrones exportados",
            level="INFO",
        )

        self._state["last_atlas_to_claude"] = _now_iso()
        _save_json(_BRIDGE_STATE, self._state)

        return {
            "ok": True,
            "episodes_exported": len(episodes),
            "patterns_exported": len(patterns),
            "output_file": str(_CLAUDE_CONTEXT),
            "timestamp": _now_iso(),
        }

    # ── Bidireccional ──────────────────────────────────────────────────────────

    def bidirectional_sync(self) -> Dict[str, Any]:
        """Sincronización completa en ambas direcciones."""
        r1 = self.sync_claude_to_atlas()
        r2 = self.sync_atlas_to_claude()
        return {
            "ok": r1["ok"] and r2["ok"],
            "claude_to_atlas": r1,
            "atlas_to_claude": r2,
            "timestamp": _now_iso(),
        }

    # ── Insight puntual ───────────────────────────────────────────────────────

    def write_insight(
        self,
        insight: str,
        category: str = "general",
        importance: float = 0.8,
        tags: Optional[List[str]] = None,
        context: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """
        Escribe un insight puntual de la sesión Claude directamente al brain.
        Útil para capturar decisiones, descubrimientos o errores durante la sesión.
        """
        eid = _write_episode(
            situation=f"[Claude Insight] {category}: {insight[:100]}",
            context=context or {"category": category},
            action_taken="Insight registrado desde sesión Claude",
            result=insight,
            task_type=f"claude_insight_{category}",
            tags=(tags or []) + ["claude_insight", category],
            importance=importance,
            success=True,
        )
        _append_decision(f"insight registrado ({category}): {insight[:80]}", level="INFO")
        return {"ok": True, "episode_id": eid, "category": category, "timestamp": _now_iso()}

    # ── Estadísticas del puente ───────────────────────────────────────────────

    def get_bridge_stats(self) -> Dict[str, Any]:
        """Retorna estado y estadísticas del puente."""
        total_episodes = 0
        claude_episodes = 0
        if _EPISODIC_DB.exists():
            try:
                conn = sqlite3.connect(_EPISODIC_DB)
                total_episodes = conn.execute("SELECT COUNT(*) FROM episodes").fetchone()[0]
                claude_episodes = conn.execute(
                    "SELECT COUNT(*) FROM episodes WHERE tags LIKE '%claude_memory%' OR tags LIKE '%claude_insight%'"
                ).fetchone()[0]
                conn.close()
            except Exception:
                pass
        state = _load_json(_BRIDGE_STATE, {})
        patterns = _load_json(_PATTERNS_JSON, {"patterns": []})
        return {
            "bridge_version": "1.0",
            "claude_memory_dir": str(self.memory_dir),
            "memory_md_exists": (self.memory_dir / "MEMORY.md").exists(),
            "context_file_exists": _CLAUDE_CONTEXT.exists(),
            "total_episodes_in_brain": total_episodes,
            "claude_bridge_episodes": claude_episodes,
            "patterns_count": len(patterns.get("patterns", [])),
            "last_sync": state.get("last_sync"),
            "total_syncs": state.get("syncs", 0),
            "last_claude_to_atlas": state.get("last_claude_to_atlas"),
            "last_atlas_to_claude": state.get("last_atlas_to_claude"),
        }

    # ── Interno ───────────────────────────────────────────────────────────────

    def _read_version(self) -> str:
        vf = _ROOT / "VERSION"
        try:
            return vf.read_text(encoding="utf-8").strip()
        except Exception:
            return "unknown"


# ─── Singleton accesible globalmente ─────────────────────────────────────────

_bridge_instance: Optional[ClaudeMemoryBridge] = None


def get_bridge() -> ClaudeMemoryBridge:
    global _bridge_instance
    if _bridge_instance is None:
        _bridge_instance = ClaudeMemoryBridge()
    return _bridge_instance
