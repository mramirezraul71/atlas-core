from __future__ import annotations

import json
import os
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional


def _repo_root() -> Path:
    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT") or ""
    if root:
        return Path(root).resolve()
    return Path(__file__).resolve().parents[2]


def _snap_dir(repo_root: Optional[Path] = None) -> Path:
    r = (repo_root or _repo_root()).resolve()
    d = r / "snapshots" / "architect"
    d.mkdir(parents=True, exist_ok=True)
    return d


@dataclass
class ProjectIndex:
    root: str
    generated_ts_utc: str
    kind: str  # python | node | mixed | unknown
    files_sample: List[str]
    detected: Dict[str, Any]
    dependencies: Dict[str, List[str]]
    notes: List[str]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "root": self.root,
            "generated_ts_utc": self.generated_ts_utc,
            "kind": self.kind,
            "files_sample": list(self.files_sample),
            "detected": dict(self.detected),
            "dependencies": {k: list(v) for k, v in (self.dependencies or {}).items()},
            "notes": list(self.notes),
        }


class ProjectIndexer:
    """Indexador genérico: sirve para ATLAS y para cualquier app nueva (scaffolded)."""

    def __init__(self, repo_root: Optional[Path] = None) -> None:
        self.repo_root = (repo_root or _repo_root()).resolve()

    def index(self, project_root: Path, *, max_files: int = 400) -> ProjectIndex:
        root = Path(project_root).resolve()
        files = self._sample_files(root, max_files=max_files)
        detected = self._detect(root, files)
        deps = self._dependencies(root, detected)
        kind = self._classify(detected)
        notes = self._notes(detected, kind)
        return ProjectIndex(
            root=str(root),
            generated_ts_utc=datetime.now(timezone.utc).isoformat(),
            kind=kind,
            files_sample=files,
            detected=detected,
            dependencies=deps,
            notes=notes,
        )

    def persist(self, idx: ProjectIndex, name: str = "project_index") -> str:
        safe = "".join([c if c.isalnum() or c in ("-", "_") else "_" for c in (name or "project_index")])[:48]
        p = _snap_dir(self.repo_root) / f"{safe}.json"
        p.write_text(json.dumps(idx.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")
        return str(p)

    def _sample_files(self, root: Path, *, max_files: int) -> List[str]:
        out: List[str] = []
        if not root.exists():
            return out
        for fp in root.rglob("*"):
            if fp.is_dir():
                continue
            rel = str(fp.relative_to(root)).replace("\\", "/")
            # evitar ruido
            if rel.startswith(("node_modules/", ".venv/", "venv/", "logs/", "snapshots/", "__pycache__/")):
                continue
            out.append(rel)
            if len(out) >= max_files:
                break
        return out

    def _detect(self, root: Path, files: List[str]) -> Dict[str, Any]:
        s = set(files)
        det: Dict[str, Any] = {
            "has_pyproject": "pyproject.toml" in s,
            "has_requirements": "requirements.txt" in s or any(p.endswith("/requirements.txt") for p in s),
            "has_package_json": "package.json" in s or any(p.endswith("/package.json") for p in s),
            "has_docker_compose": "docker-compose.yml" in s,
            "python_packages": [],
            "node_packages": [],
        }

        # python deps from requirements (top-level only)
        req = root / "requirements.txt"
        if req.exists():
            try:
                lines = [ln.strip() for ln in req.read_text(encoding="utf-8", errors="ignore").splitlines()]
                pkgs = [ln.split("==")[0].split(">=")[0].split("<=")[0].strip() for ln in lines if ln and not ln.startswith("#")]
                det["python_packages"] = [p for p in pkgs if p]
            except Exception:
                pass

        # node deps from package.json (top-level only)
        pj = root / "package.json"
        if pj.exists():
            try:
                data = json.loads(pj.read_text(encoding="utf-8", errors="ignore"))
                deps = list((data.get("dependencies") or {}).keys())
                dev = list((data.get("devDependencies") or {}).keys())
                det["node_packages"] = sorted(set(deps + dev))
            except Exception:
                pass

        return det

    def _dependencies(self, root: Path, det: Dict[str, Any]) -> Dict[str, List[str]]:
        deps: Dict[str, List[str]] = {}
        if det.get("has_docker_compose"):
            deps["docker-compose.yml"] = ["services/*"]
        if det.get("has_pyproject"):
            deps["pyproject.toml"] = ["python packages (pyproject)"]
        if det.get("has_requirements"):
            deps["requirements.txt"] = det.get("python_packages") or []
        if det.get("has_package_json"):
            deps["package.json"] = det.get("node_packages") or []
        return deps

    def _classify(self, det: Dict[str, Any]) -> str:
        py = bool(det.get("has_pyproject") or det.get("has_requirements"))
        node = bool(det.get("has_package_json"))
        if py and node:
            return "mixed"
        if py:
            return "python"
        if node:
            return "node"
        return "unknown"

    def _notes(self, det: Dict[str, Any], kind: str) -> List[str]:
        notes: List[str] = []
        if kind == "python":
            notes.append("Proyecto Python detectado (pyproject/requirements).")
        if kind == "node":
            notes.append("Proyecto Node detectado (package.json).")
        if kind == "mixed":
            notes.append("Proyecto mixto detectado (Python + Node).")
        if det.get("has_docker_compose"):
            notes.append("docker-compose.yml presente: hay arquitectura multi-servicio.")
        return notes

