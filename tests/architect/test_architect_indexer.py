from __future__ import annotations

from pathlib import Path


def test_architecture_indexer_reads_compose_ports(tmp_path: Path):
    from modules.atlas_architect.indexer import ArchitectureIndexer

    # Crear docker-compose.yml mínimo en tmp
    (tmp_path / "docker-compose.yml").write_text(
        """
version: '3.8'
services:
  push:
    ports: ["8791:8791"]
    environment:
      - NEXUS_BASE_URL=http://nexus:8000
      - ROBOT_BASE_URL=http://robot:8002
  nexus:
    ports: ["8000:8000"]
  robot:
    ports: ["8002:8002"]
""".strip(),
        encoding="utf-8",
    )

    idx = ArchitectureIndexer(repo_root=tmp_path).build()
    data = idx.to_dict()
    ports = {s["port"] for s in data["services"]}
    assert {8791, 8000, 8002}.issubset(ports)
    assert "NEXUS_BASE_URL" in data["env"]

