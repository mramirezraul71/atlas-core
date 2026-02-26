import json
from datetime import datetime
from pathlib import Path

MEMORY_ROOT = Path(r"C:\ATLAS_PUSH\workspace_prime\memory")


class MemoryManager:
    def __init__(self):
        self.working_path = MEMORY_ROOT / "working.json"
        self.longterm_path = MEMORY_ROOT / "longterm.json"
        self.episodes_path = MEMORY_ROOT / "episodes"
        self.episodes_path.mkdir(parents=True, exist_ok=True)

    def read_working(self) -> dict:
        try:
            with open(self.working_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except:
            return {}

    def write_working(self, data: dict) -> bool:
        data["last_updated"] = datetime.now().isoformat()
        with open(self.working_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        return True

    def update_field(self, key: str, value) -> bool:
        data = self.read_working()
        data[key] = value
        return self.write_working(data)

    def save_episode(self, task: str, result: str, success: bool):
        ep = {
            "timestamp": datetime.now().isoformat(),
            "task": task,
            "result": result,
            "success": success,
        }
        ep_file = (
            self.episodes_path / f"ep_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
        with open(ep_file, "w", encoding="utf-8") as f:
            json.dump(ep, f, indent=2, ensure_ascii=False)

    def get_recent_episodes(self, n: int = 5) -> list:
        files = sorted(self.episodes_path.glob("*.json"), reverse=True)[:n]
        return [json.load(open(f, encoding="utf-8")) for f in files]


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
