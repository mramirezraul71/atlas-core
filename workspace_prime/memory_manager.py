import json
import re
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List

MEMORY_ROOT = Path(r"C:\ATLAS_PUSH\workspace_prime\memory")
MEMORY_ROOT.mkdir(parents=True, exist_ok=True)


class MemoryManager:
    def __init__(self):
        self.working_path = MEMORY_ROOT / "working.json"
        self.longterm_path = MEMORY_ROOT / "longterm.json"
        self.episodes_path = MEMORY_ROOT / "episodes"
        self.episodes_path.mkdir(parents=True, exist_ok=True)

    def _read_json(self, path: Path, default: Dict[str, Any] | List[Any]):
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return default

    def _write_json_atomic(self, path: Path, data: Dict[str, Any] | List[Any]) -> bool:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        tmp.replace(path)
        return True

    def read_working(self) -> dict:
        return self._read_json(self.working_path, {})

    def write_working(self, data: dict) -> bool:
        payload = dict(data or {})
        payload["last_updated"] = datetime.now().isoformat()
        return self._write_json_atomic(self.working_path, payload)

    def update_field(self, key: str, value) -> bool:
        data = self.read_working()
        data[key] = value
        return self.write_working(data)

    def read_longterm(self) -> dict:
        data = self._read_json(self.longterm_path, {})
        if not isinstance(data, dict):
            data = {}
        data.setdefault("rules", [])
        data.setdefault("updated_at", "")
        return data

    def write_longterm(self, data: dict) -> bool:
        payload = dict(data or {})
        payload["updated_at"] = datetime.now().isoformat()
        payload.setdefault("rules", [])
        return self._write_json_atomic(self.longterm_path, payload)

    def save_episode(
        self,
        task: str,
        result: str,
        success: bool,
        metadata: dict | None = None,
    ):
        ep = {
            "timestamp": datetime.now().isoformat(),
            "task": task,
            "result": result,
            "success": success,
            "metadata": metadata or {},
        }
        ep_file = (
            self.episodes_path / f"ep_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.json"
        )
        self._write_json_atomic(ep_file, ep)

    def get_recent_episodes(self, n: int = 5) -> list:
        files = sorted(self.episodes_path.glob("*.json"), reverse=True)[:n]
        episodes = []
        for f in files:
            try:
                episodes.append(json.loads(f.read_text(encoding="utf-8")))
            except Exception:
                continue
        return episodes

    def _keywords(self, text: str) -> set[str]:
        tokens = re.findall(r"[a-zA-Z0-9_áéíóúñ]{4,}", (text or "").lower())
        stopwords = {
            "para",
            "como",
            "esta",
            "este",
            "desde",
            "hasta",
            "sobre",
            "quiero",
            "necesito",
            "hacer",
            "tarea",
            "atlas",
            "workspace",
        }
        return {tok for tok in tokens if tok not in stopwords}

    def get_relevant_context(
        self, task: str, episode_limit: int = 3, rule_limit: int = 3
    ) -> dict:
        task_keywords = self._keywords(task)
        recent = self.get_recent_episodes(12)
        ranked_episodes = []
        for ep in recent:
            ep_text = f"{ep.get('task', '')} {ep.get('result', '')}"
            overlap = len(task_keywords & self._keywords(ep_text))
            if overlap > 0 or not task_keywords:
                ranked_episodes.append((overlap, ep))
        ranked_episodes.sort(key=lambda item: item[0], reverse=True)

        longterm = self.read_longterm()
        ranked_rules = []
        for rule in longterm.get("rules", []):
            overlap = len(task_keywords & set(rule.get("keywords", [])))
            if overlap > 0 or rule.get("category") == "general":
                ranked_rules.append((overlap, rule))
        ranked_rules.sort(key=lambda item: item[0], reverse=True)

        return {
            "recent_episodes": [ep for _, ep in ranked_episodes[:episode_limit]],
            "rules": [rule for _, rule in ranked_rules[:rule_limit]],
        }

    def learn_from_execution(
        self,
        task: str,
        success: bool,
        plan: dict | None = None,
        final_output: dict | None = None,
        errors: list | None = None,
        metadata: dict | None = None,
    ) -> None:
        longterm = self.read_longterm()
        rules = longterm.get("rules", [])
        final_blob = json.dumps(final_output or {}, ensure_ascii=False).lower()
        error_blob = " | ".join(str(e) for e in (errors or [])).lower()
        keywords = sorted(self._keywords(task))

        recommendation = ""
        category = "general"
        if "sorry" in final_blob or "captcha" in final_blob:
            category = "web_anti_bot"
            recommendation = (
                "Si una web responde con CAPTCHA o pagina sorry, evitar Playwright directo "
                "y priorizar smart_browser_task o una fuente alternativa."
            )
        elif "acción desconocida" in error_blob or "accion desconocida" in error_blob:
            category = "planner_alignment"
            recommendation = (
                "Limitar el plan a acciones soportadas por PlanExecutor y normalizar aliases "
                "antes de ejecutar."
            )
        elif success and plan and plan.get("requires_smart_browser"):
            category = "smart_browser"
            recommendation = (
                "Para tareas web complejas similares, priorizar smart_browser_task desde el plan."
            )
        elif success:
            category = "general_success"
            recommendation = (
                "Reutilizar la misma familia de acciones cuando la tarea sea semánticamente parecida."
            )
        else:
            recommendation = (
                "Ante un fallo sin patron claro, replanificar con menos pasos y verificar evidencia "
                "despues de cada accion critica."
            )

        key = f"{category}:{recommendation}"
        existing = next((rule for rule in rules if rule.get("key") == key), None)
        if existing:
            existing["last_seen"] = datetime.now().isoformat()
            existing["hits"] = int(existing.get("hits", 1)) + 1
            existing["success_examples"] = int(existing.get("success_examples", 0)) + int(
                bool(success)
            )
            if keywords:
                existing["keywords"] = sorted(
                    set(existing.get("keywords", [])) | set(keywords[:8])
                )
        else:
            rules.append(
                {
                    "key": key,
                    "category": category,
                    "recommendation": recommendation,
                    "keywords": keywords[:8],
                    "hits": 1,
                    "success_examples": int(bool(success)),
                    "last_seen": datetime.now().isoformat(),
                    "metadata": metadata or {},
                }
            )

        longterm["rules"] = rules[-30:]
        self.write_longterm(longterm)


if __name__ == "__main__":
    import sys

    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    mm = MemoryManager()
    mm.update_field("session_count", 1)
    print("OK MemoryManager:", mm.read_working())
