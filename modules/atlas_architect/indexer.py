from __future__ import annotations

import os
import json
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml


def _repo_root() -> Path:
    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT") or ""
    if root:
        return Path(root).resolve()
    # modules/atlas_architect/indexer.py -> repo_root
    return Path(__file__).resolve().parents[2]


def _snap_dir(repo_root: Optional[Path] = None) -> Path:
    r = (repo_root or _repo_root()).resolve()
    d = r / "snapshots" / "architect"
    d.mkdir(parents=True, exist_ok=True)
    return d


@dataclass(frozen=True)
class ServiceEndpoint:
    name: str
    port: int
    base_url: str
    role: str  # push | nexus | robot | redis | unknown


@dataclass
class ArchitectureIndex:
    """Mapa mínimo (pero útil) de arquitectura y dependencias entre ATLAS_PUSH y ATLAS_NEXUS."""

    repo_root: str
    generated_ts_utc: str
    services: List[ServiceEndpoint]
    env: Dict[str, str]
    dependencies: List[Dict[str, Any]]
    notes: List[str]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "repo_root": self.repo_root,
            "generated_ts_utc": self.generated_ts_utc,
            "services": [asdict(s) for s in self.services],
            "env": dict(self.env),
            "dependencies": list(self.dependencies),
            "notes": list(self.notes),
        }


class ArchitectureIndexer:
    def __init__(self, repo_root: Optional[Path] = None) -> None:
        self.repo_root = (repo_root or _repo_root()).resolve()

    def _read_compose(self) -> Dict[str, Any]:
        path = self.repo_root / "docker-compose.yml"
        if not path.exists():
            return {}
        try:
            return yaml.safe_load(path.read_text(encoding="utf-8", errors="ignore")) or {}
        except Exception:
            return {}

    def build(self) -> ArchitectureIndex:
        compose = self._read_compose()
        services: List[ServiceEndpoint] = []
        env: Dict[str, str] = {}
        notes: List[str] = []

        # Ports/URLs defaults (local dev)
        push_url = os.getenv("PUSH_BASE_URL", "http://127.0.0.1:8791")
        nexus_url = os.getenv("NEXUS_BASE_URL", "http://127.0.0.1:8000")
        robot_url = os.getenv("NEXUS_ROBOT_API_URL", os.getenv("ROBOT_BASE_URL", "http://127.0.0.1:8002"))

        env["PUSH_BASE_URL"] = push_url
        env["NEXUS_BASE_URL"] = nexus_url
        env["NEXUS_ROBOT_API_URL"] = robot_url

        # Compose-derived mapping (if present)
        try:
            svc = (compose or {}).get("services") or {}
            push = svc.get("push") or {}
            nexus = svc.get("nexus") or {}
            robot = svc.get("robot") or {}
            # docker-compose ports are strings "host:container"
            def _first_port(svc_def: Dict[str, Any], default: int) -> int:
                ports = svc_def.get("ports") or []
                if ports and isinstance(ports, list):
                    p0 = str(ports[0])
                    if ":" in p0:
                        return int(p0.split(":", 1)[0].strip().strip('"').strip("'"))
                return default

            push_port = _first_port(push, 8791)
            nexus_port = _first_port(nexus, 8000)
            robot_port = _first_port(robot, 8002)

            services.extend(
                [
                    ServiceEndpoint(name="atlas_push", port=push_port, base_url=f"http://127.0.0.1:{push_port}", role="push"),
                    ServiceEndpoint(name="atlas_nexus", port=nexus_port, base_url=f"http://127.0.0.1:{nexus_port}", role="nexus"),
                    ServiceEndpoint(name="atlas_robot", port=robot_port, base_url=f"http://127.0.0.1:{robot_port}", role="robot"),
                ]
            )

            # environment variables in compose push service
            for it in (push.get("environment") or []):
                try:
                    if isinstance(it, str) and "=" in it:
                        k, v = it.split("=", 1)
                        env[k.strip()] = v.strip()
                except Exception:
                    pass
        except Exception:
            services.extend(
                [
                    ServiceEndpoint(name="atlas_push", port=8791, base_url=push_url, role="push"),
                    ServiceEndpoint(name="atlas_nexus", port=8000, base_url=nexus_url, role="nexus"),
                    ServiceEndpoint(name="atlas_robot", port=8002, base_url=robot_url, role="robot"),
                ]
            )

        # Dependencies graph (mínimo útil)
        deps = [
            {"from": "atlas_push", "to": "atlas_nexus", "why": "proxy/ws/actions/log y servicios", "port": 8000},
            {"from": "atlas_push", "to": "atlas_robot", "why": "servicio de cámaras / visión", "port": 8002},
            {"from": "UI/dashboard", "to": "atlas_push", "why": "control/estado y orquestación Cursor", "port": 8791},
        ]

        notes.append("Puertos clave: PUSH=8791, NEXUS=8000, ROBOT=8002.")
        notes.append("Si NEXUS está en docker, PUSH usa NEXUS_BASE_URL=http://nexus:8000 y ROBOT_BASE_URL=http://robot:8002.")

        return ArchitectureIndex(
            repo_root=str(self.repo_root),
            generated_ts_utc=datetime.now(timezone.utc).isoformat(),
            services=services,
            env=env,
            dependencies=deps,
            notes=notes,
        )

    def persist(self, index: ArchitectureIndex) -> str:
        p = _snap_dir(self.repo_root) / "arch_index.json"
        p.write_text(json.dumps(index.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")
        return str(p)

