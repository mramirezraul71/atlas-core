"""MEMORY_ENGINE_V2: memoria persistente enterprise para ATLAS.

Capacidades:
- Versionado automático de estado con retención configurable.
- Checkpoints transaccionales PRE/POST con hash y git HEAD.
- Integridad (schema + hash), detección de corrupción y auto-repair.
- Rollback automático si fallan smoke tests.
- Memoria estratégica separada (decisiones, evolución y patrones).
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any


def _now_iso() -> str:
    return datetime.now().isoformat(timespec="seconds")


def _stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


@dataclass
class MemoryPaths:
    root: Path
    versioned_state: Path
    current_state: Path
    versions_dir: Path
    state_index: Path
    strategic_memory: Path
    decisions_log: Path
    architectural_evolution: Path
    learned_patterns: Path
    checkpoints: Path
    checkpoint_index: Path
    recovery: Path
    corruption_log: Path
    auto_repair_log: Path
    integrity: Path
    hashes: Path
    last_integrity_scan: Path


class MemoryEngineV2:
    """Motor de memoria persistente versionada para operación enterprise."""

    def __init__(
        self,
        root: str | Path = "memory_engine",
        max_versions: int | None = None,
        project_root: str | Path | None = None,
        signing_key: str | None = None,
    ) -> None:
        self.root = Path(root)
        cfg_max = int(os.getenv("ATLAS_STATE_VERSIONS_MAX", "50"))
        self.max_versions = max(
            5, int(max_versions) if max_versions is not None else cfg_max
        )
        self.project_root = Path(project_root) if project_root else Path.cwd()
        self.signing_key = signing_key or os.getenv(
            "ATLAS_MEMORY_SIGNING_KEY", "atlas-memory-engine-v2"
        )
        self.legacy_memory_root = Path(
            os.getenv("ATLAS_MEMORY_DIR", r"C:\ATLAS\MEMORY")
        )
        self.legacy_decisions = self.legacy_memory_root / "decisions.json"
        self.legacy_modules = self.legacy_memory_root / "modules_state.json"
        self.legacy_ltc = self.legacy_memory_root / "long_term_context.md"
        self.legacy_last_snapshot = (
            self.legacy_memory_root / "last_snapshot_pointer.txt"
        )
        self.log_file = Path(os.getenv("ATLAS_LOG_FILE", r"C:\ATLAS\logs\atlas.log"))
        self.manage_legacy_v1 = os.getenv("ATLAS_MEMORY_V1_MANAGED", "1") in {
            "1",
            "true",
            "TRUE",
            "yes",
            "on",
        }
        self.paths = MemoryPaths(
            root=self.root,
            versioned_state=self.root / "versioned_state",
            current_state=self.root / "versioned_state" / "current_state.json",
            versions_dir=self.root / "versioned_state" / "versions",
            state_index=self.root / "versioned_state" / "state_index.json",
            strategic_memory=self.root / "strategic_memory",
            decisions_log=self.root / "strategic_memory" / "decisions.log",
            architectural_evolution=self.root
            / "strategic_memory"
            / "architectural_evolution.md",
            learned_patterns=self.root / "strategic_memory" / "learned_patterns.json",
            checkpoints=self.root / "checkpoints",
            checkpoint_index=self.root / "checkpoints" / "checkpoint_index.json",
            recovery=self.root / "recovery",
            corruption_log=self.root / "recovery" / "corruption_log.json",
            auto_repair_log=self.root / "recovery" / "auto_repair.log",
            integrity=self.root / "integrity",
            hashes=self.root / "integrity" / "hashes.json",
            last_integrity_scan=self.root / "integrity" / "last_integrity_scan.json",
        )

    # -------------------------
    # Puentes memoria V1 (legacy)
    # -------------------------
    def _legacy_bridge_meta(self) -> dict[str, Any]:
        return {
            "legacy_root": str(self.legacy_memory_root),
            "managed": self.manage_legacy_v1,
            "files": {
                "decisions_json": {
                    "path": str(self.legacy_decisions),
                    "exists": self.legacy_decisions.exists(),
                },
                "modules_state_json": {
                    "path": str(self.legacy_modules),
                    "exists": self.legacy_modules.exists(),
                },
                "long_term_context_md": {
                    "path": str(self.legacy_ltc),
                    "exists": self.legacy_ltc.exists(),
                },
                "last_snapshot_pointer_txt": {
                    "path": str(self.legacy_last_snapshot),
                    "exists": self.legacy_last_snapshot.exists(),
                },
            },
        }

    def _capture_legacy_snapshot(self) -> dict[str, Any]:
        """Captura snapshot de memoria V1 para rollback controlado."""
        payload: dict[str, Any] = {"managed": self.manage_legacy_v1, "files": {}}
        for name, path in {
            "decisions.json": self.legacy_decisions,
            "modules_state.json": self.legacy_modules,
            "long_term_context.md": self.legacy_ltc,
            "last_snapshot_pointer.txt": self.legacy_last_snapshot,
        }.items():
            if not path.exists():
                payload["files"][name] = {"exists": False, "content": None}
                continue
            payload["files"][name] = {
                "exists": True,
                "content": path.read_text(encoding="utf-8", errors="replace"),
            }
        return payload

    def _restore_legacy_snapshot(self, snapshot: dict[str, Any]) -> dict[str, Any]:
        """Restaura archivos V1 desde checkpoint cuando están bajo control del engine."""
        if not self.manage_legacy_v1:
            return {"restored": False, "reason": "legacy_not_managed"}

        files = snapshot.get("files", {}) if isinstance(snapshot, dict) else {}
        restored: list[str] = []
        for name, path in {
            "decisions.json": self.legacy_decisions,
            "modules_state.json": self.legacy_modules,
            "long_term_context.md": self.legacy_ltc,
            "last_snapshot_pointer.txt": self.legacy_last_snapshot,
        }.items():
            item = files.get(name, {}) if isinstance(files, dict) else {}
            if not isinstance(item, dict):
                continue
            if not item.get("exists", False):
                # No eliminar archivos existentes para evitar impacto no controlado.
                continue
            content = item.get("content")
            if isinstance(content, str):
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content, encoding="utf-8")
                restored.append(name)

        return {"restored": bool(restored), "files": restored}

    # -------------------------
    # Bootstrap + utilidades I/O
    # -------------------------
    def bootstrap(self) -> None:
        """Crea estructura base y archivos mínimos de operación."""
        for folder in (
            self.paths.root,
            self.paths.versioned_state,
            self.paths.versions_dir,
            self.paths.strategic_memory,
            self.paths.checkpoints,
            self.paths.recovery,
            self.paths.integrity,
        ):
            folder.mkdir(parents=True, exist_ok=True)

        if not self.paths.current_state.exists():
            self._write_json(
                self.paths.current_state,
                {
                    "version": 0,
                    "updated_at": _now_iso(),
                    "reason": "bootstrap",
                    "actor": "memory_engine_v2",
                    "state": {
                        "legacy_bridge": self._legacy_bridge_meta(),
                    },
                },
            )

        if not self.paths.state_index.exists():
            self._write_json(
                self.paths.state_index,
                {
                    "version": 1,
                    "updated_at": _now_iso(),
                    "max_versions": self.max_versions,
                    "current_version": 0,
                    "versions": [],
                },
            )

        if not self.paths.decisions_log.exists():
            self.paths.decisions_log.write_text(
                "# decisions.log - memoria estratégica\n",
                encoding="utf-8",
            )

        if not self.paths.architectural_evolution.exists():
            self.paths.architectural_evolution.write_text(
                "# Architectural Evolution\n\n"
                "## Inicio\n"
                "- Se inicializa MEMORY_ENGINE_V2 como fuente de verdad persistente.\n",
                encoding="utf-8",
            )

        if not self.paths.learned_patterns.exists():
            self._write_json(
                self.paths.learned_patterns,
                {"version": 1, "updated_at": _now_iso(), "patterns": []},
            )

        if not self.paths.checkpoint_index.exists():
            self._write_json(
                self.paths.checkpoint_index,
                {
                    "version": 1,
                    "updated_at": _now_iso(),
                    "last_checkpoint": None,
                    "checkpoints": [],
                },
            )

        if not self.paths.corruption_log.exists():
            self._write_json(
                self.paths.corruption_log,
                {"version": 1, "updated_at": _now_iso(), "events": []},
            )

        if not self.paths.auto_repair_log.exists():
            self.paths.auto_repair_log.write_text(
                "# auto_repair.log\n",
                encoding="utf-8",
            )

        if not self.paths.hashes.exists():
            self._write_json(self.paths.hashes, {"updated_at": _now_iso(), "files": {}})

        if not self.paths.last_integrity_scan.exists():
            self._write_json(
                self.paths.last_integrity_scan,
                {
                    "timestamp": _now_iso(),
                    "status": "BOOTSTRAP",
                    "errors": [],
                    "auto_repaired": False,
                },
            )

        self._refresh_hashes()

    def _read_json(
        self, path: Path, default: dict[str, Any] | None = None
    ) -> dict[str, Any]:
        if not path.exists():
            return default or {}
        data = json.loads(path.read_text(encoding="utf-8", errors="replace"))
        schema_errors = self._validate_known_schema(path, data)
        if schema_errors:
            raise ValueError(f"Schema inválido en {path}: {schema_errors}")
        return data

    def _write_json(self, path: Path, data: dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp.replace(path)

    def _sha256_file(self, path: Path) -> str:
        h = hashlib.sha256()
        with path.open("rb") as f:
            for chunk in iter(lambda: f.read(65536), b""):
                h.update(chunk)
        return h.hexdigest()

    def _refresh_hashes(self) -> None:
        tracked = [
            self.paths.current_state,
            self.paths.state_index,
            self.paths.learned_patterns,
            self.paths.checkpoint_index,
            self.paths.corruption_log,
            self.paths.last_integrity_scan,
        ]
        files: dict[str, str] = {}
        for p in tracked:
            if p.exists():
                files[str(p.relative_to(self.paths.root))] = self._sha256_file(p)
        hashes_payload = {"updated_at": _now_iso(), "files": files}
        self._validate_hashes_schema_payload(hashes_payload)
        self._write_json(self.paths.hashes, hashes_payload)

    def _git_head(self) -> str:
        try:
            p = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=str(self.project_root),
                capture_output=True,
                text=True,
                timeout=10,
                check=False,
            )
            return (p.stdout or "").strip() or "unknown"
        except Exception:
            return "unknown"

    def _checkpoint_signature(self, payload: dict[str, Any]) -> str:
        """Firma SHA256 del checkpoint para detectar manipulación/corrupción."""
        base = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        signed = f"{self.signing_key}|{base}".encode("utf-8", errors="replace")
        return hashlib.sha256(signed).hexdigest()

    # -------------------------
    # Contratos / schemas ligeros
    # -------------------------
    def _validate_current_state_payload(self, payload: dict[str, Any]) -> None:
        if not isinstance(payload, dict):
            raise ValueError("current_state payload debe ser dict")
        for key in ("version", "updated_at", "reason", "actor", "state"):
            if key not in payload:
                raise ValueError(f"current_state missing key: {key}")
        if not isinstance(payload.get("state"), dict):
            raise ValueError("current_state.state debe ser dict")

    def _validate_checkpoint_payload(self, payload: dict[str, Any]) -> None:
        if not isinstance(payload, dict):
            raise ValueError("checkpoint payload debe ser dict")
        required = (
            "id",
            "timestamp",
            "stage",
            "summary",
            "git_head",
            "state_version",
            "state_hash",
            "state_snapshot",
            "signature",
        )
        for key in required:
            if key not in payload:
                raise ValueError(f"checkpoint missing key: {key}")
        if not isinstance(payload.get("state_snapshot"), dict):
            raise ValueError("checkpoint.state_snapshot debe ser dict")

    def _validate_hashes_schema_payload(self, payload: dict[str, Any]) -> None:
        if not isinstance(payload, dict):
            raise ValueError("hashes payload debe ser dict")
        if not isinstance(payload.get("files", {}), dict):
            raise ValueError("hashes.files debe ser dict")

    def _validate_learned_patterns_payload(self, payload: dict[str, Any]) -> None:
        if not isinstance(payload, dict):
            raise ValueError("learned_patterns payload debe ser dict")
        if not isinstance(payload.get("patterns", []), list):
            raise ValueError("learned_patterns.patterns debe ser list")

    def _validate_known_schema(self, path: Path, data: dict[str, Any]) -> list[str]:
        errors: list[str] = []
        try:
            if path == self.paths.current_state:
                self._validate_current_state_payload(data)
            elif path == self.paths.hashes:
                self._validate_hashes_schema_payload(data)
            elif path == self.paths.learned_patterns:
                self._validate_learned_patterns_payload(data)
            elif (
                path.parent == self.paths.checkpoints
                and path.name.startswith("checkpoint_")
                and path.name != "checkpoint_index.json"
            ):
                self._validate_checkpoint_payload(data)
        except Exception as e:
            errors.append(str(e))
        return errors

    # -------------------------
    # Versionado de estado
    # -------------------------
    def save_state(
        self,
        state: dict[str, Any],
        reason: str,
        actor: str = "system",
        strategic_note: str | None = None,
    ) -> dict[str, Any]:
        """Guarda estado actual + versión incremental + índice global."""
        self.bootstrap()
        if not isinstance(state, dict):
            raise ValueError("state debe ser dict")

        index = self._read_json(self.paths.state_index)
        next_version = int(index.get("current_version", 0)) + 1
        ts = _now_iso()

        payload = {
            "version": next_version,
            "updated_at": ts,
            "reason": reason,
            "actor": actor,
            "state": state,
        }
        self._validate_current_state_payload(payload)
        self._write_json(self.paths.current_state, payload)

        version_file = f"state_v{next_version:06d}_{_stamp()}.json"
        version_path = self.paths.versions_dir / version_file
        self._write_json(version_path, payload)

        entry = {
            "version": next_version,
            "file": version_file,
            "timestamp": ts,
            "reason": reason,
            "actor": actor,
            "git_head": self._git_head(),
            "sha256": self._sha256_file(version_path),
        }

        index.setdefault("versions", [])
        index["version"] = 1
        index["updated_at"] = ts
        index["max_versions"] = self.max_versions
        index["current_version"] = next_version
        index["versions"].append(entry)

        # Rotación de versiones antiguas
        while len(index["versions"]) > self.max_versions:
            old = index["versions"].pop(0)
            old_path = self.paths.versions_dir / str(old.get("file", ""))
            if old_path.exists():
                old_path.unlink()

        self._write_json(self.paths.state_index, index)
        self._refresh_hashes()

        if strategic_note:
            self.log_decision("state_update", strategic_note, level="info")

        return entry

    # -------------------------
    # Checkpoints transaccionales
    # -------------------------
    def _normalize_checkpoint_stage(self, stage: str) -> str:
        """Normaliza stage de checkpoint manteniendo compatibilidad backward."""
        if not isinstance(stage, str) or not stage.strip():
            raise ValueError(
                "stage inválido. Permitidos: PRE, POST, BASELINE o formato interno (ej: PRE_CAMBIO)."
            )

        normalized = stage.strip().upper()
        alias_map = {
            "PRE": "PRE_CAMBIO",
            "POST": "POST_CAMBIO",
            "BASELINE": "BASELINE",
        }
        if normalized in alias_map:
            return alias_map[normalized]

        # Si ya viene en formato interno (ej: PRE_CAMBIO, POST_CAMBIO, PRE_SMOKE), se respeta.
        if re.match(r"^[A-Z]+(?:_[A-Z0-9]+)*$", normalized):
            return normalized

        raise ValueError(
            "stage desconocido. Permitidos: PRE, POST, BASELINE o formato interno (ej: PRE_CAMBIO, POST_CAMBIO, PRE_SMOKE)."
        )

    def _normalize_checkpoint_summary(
        self, summary: str | None, reason: str | None
    ) -> str:
        """Compatibilidad: reason actúa como alias de summary."""
        summary_norm = summary.strip() if isinstance(summary, str) else ""
        reason_norm = reason.strip() if isinstance(reason, str) else ""

        if not summary_norm and reason_norm:
            summary_norm = reason_norm
        elif summary_norm and reason_norm and summary_norm != reason_norm:
            self.log_decision(
                title="checkpoint_reason_ignored",
                details=(
                    "create_checkpoint recibió summary y reason simultáneamente; "
                    "se prioriza summary por compatibilidad."
                ),
                level="info",
            )

        if not summary_norm:
            raise ValueError(
                "create_checkpoint requiere summary válido (o reason como alias)."
            )

        return summary_norm

    def create_checkpoint(
        self,
        stage: str,
        summary: str | None = None,
        kind: str = "structural_change",
        extra: dict[str, Any] | None = None,
        reason: str | None = None,
    ) -> dict[str, Any]:
        """Crea checkpoint con hash, git HEAD y snapshot del estado.

        Compatibilidad:
        - `summary` sigue siendo el contrato oficial.
        - `reason` funciona como alias de `summary`.
        - stages simples PRE/POST/BASELINE se normalizan internamente.
        """
        self.bootstrap()
        stage_norm = self._normalize_checkpoint_stage(stage)
        summary_norm = self._normalize_checkpoint_summary(summary, reason)
        current = self._read_json(self.paths.current_state)
        state_index = self._read_json(self.paths.state_index)

        stamp = _stamp()
        checkpoint_id = f"ckpt_{stamp}_{stage_norm.lower()}"
        fname = f"checkpoint_{stamp}_{stage_norm.lower()}.json"
        path = self.paths.checkpoints / fname
        payload = {
            "id": checkpoint_id,
            "timestamp": _now_iso(),
            "stage": stage_norm,
            "kind": kind,
            "summary": summary_norm,
            "git_head": self._git_head(),
            "state_version": state_index.get("current_version", 0),
            "state_hash": hashlib.sha256(
                json.dumps(current, ensure_ascii=False, sort_keys=True).encode("utf-8")
            ).hexdigest(),
            "state_snapshot": current,
            "pointers": {
                "last_snapshot_pointer": self.legacy_last_snapshot.read_text(
                    encoding="utf-8", errors="replace"
                ).strip()
                if self.legacy_last_snapshot.exists()
                else None,
                "log_file": str(self.log_file),
                "legacy_memory_root": str(self.legacy_memory_root),
                "legacy_bridge": self._legacy_bridge_meta(),
            },
            "legacy_v1_snapshot": self._capture_legacy_snapshot(),
            "extra": extra or {},
        }
        payload["signature"] = self._checkpoint_signature(payload)
        self._validate_checkpoint_payload(payload)
        self._write_json(path, payload)

        index = self._read_json(self.paths.checkpoint_index)
        index.setdefault("checkpoints", [])
        index["version"] = 1
        index["updated_at"] = _now_iso()
        index["last_checkpoint"] = checkpoint_id
        index["checkpoints"].append(
            {
                "id": checkpoint_id,
                "file": fname,
                "timestamp": payload["timestamp"],
                "stage": stage_norm,
                "kind": kind,
                "summary": summary_norm,
                "git_head": payload["git_head"],
                "state_version": payload["state_version"],
                "state_hash": payload["state_hash"],
                "signature": payload["signature"],
            }
        )
        self._write_json(self.paths.checkpoint_index, index)
        self._refresh_hashes()
        return payload

    # -------------------------
    # Integridad + auto-repair
    # -------------------------
    def _validate_core_schema(self) -> list[str]:
        errors: list[str] = []
        try:
            cs = self._read_json(self.paths.current_state)
            if not isinstance(cs, dict) or not isinstance(cs.get("state"), dict):
                errors.append("current_state schema inválido")
        except Exception as e:
            errors.append(f"current_state JSON inválido: {e}")

        try:
            si = self._read_json(self.paths.state_index)
            if not isinstance(si, dict) or not isinstance(si.get("versions", []), list):
                errors.append("state_index schema inválido")
        except Exception as e:
            errors.append(f"state_index JSON inválido: {e}")

        try:
            ci = self._read_json(self.paths.checkpoint_index)
            if not isinstance(ci, dict) or not isinstance(
                ci.get("checkpoints", []), list
            ):
                errors.append("checkpoint_index schema inválido")
        except Exception as e:
            errors.append(f"checkpoint_index JSON inválido: {e}")

        return errors

    def _validate_hashes(self) -> list[str]:
        errors: list[str] = []
        hashes = self._read_json(self.paths.hashes, default={"files": {}})
        files = hashes.get("files", {}) if isinstance(hashes, dict) else {}
        for rel, expected in files.items():
            p = self.paths.root / rel
            if not p.exists():
                errors.append(f"hash file faltante: {rel}")
                continue
            current = self._sha256_file(p)
            if current != expected:
                errors.append(f"hash mismatch: {rel}")
        return errors

    def _validate_checkpoint_signatures(self) -> list[str]:
        """Valida firma de checkpoints para detectar manipulación."""
        errors: list[str] = []
        index = self._read_json(
            self.paths.checkpoint_index, default={"checkpoints": []}
        )
        for item in list(index.get("checkpoints", [])):
            file_name = str(item.get("file", ""))
            if not file_name:
                errors.append("checkpoint_index entry sin file")
                continue

            cp_path = self.paths.checkpoints / file_name
            if not cp_path.exists():
                errors.append(f"checkpoint faltante: {file_name}")
                continue

            try:
                payload = self._read_json(cp_path)
                signature = payload.get("signature")
                if not signature:
                    errors.append(f"checkpoint sin firma: {file_name}")
                    continue

                payload_no_sig = dict(payload)
                payload_no_sig.pop("signature", None)
                expected = self._checkpoint_signature(payload_no_sig)
                if signature != expected:
                    errors.append(f"checkpoint firma inválida: {file_name}")
            except Exception as e:
                errors.append(f"checkpoint inválido {file_name}: {e}")

        return errors

    def _append_corruption_event(
        self, errors: list[str], repaired: bool, note: str
    ) -> None:
        data = self._read_json(self.paths.corruption_log)
        data.setdefault("events", [])
        data["updated_at"] = _now_iso()
        data["events"].append(
            {
                "timestamp": _now_iso(),
                "errors": errors,
                "repaired": repaired,
                "note": note,
            }
        )
        self._write_json(self.paths.corruption_log, data)

    def _auto_repair_from_latest_version(self, errors: list[str]) -> dict[str, Any]:
        index = self._read_json(self.paths.state_index)
        versions = list(index.get("versions", []))

        for entry in reversed(versions):
            p = self.paths.versions_dir / str(entry.get("file", ""))
            if not p.exists():
                continue
            try:
                candidate = self._read_json(p)
                if isinstance(candidate, dict) and isinstance(
                    candidate.get("state"), dict
                ):
                    self._write_json(self.paths.current_state, candidate)
                    self._refresh_hashes()
                    log_line = (
                        f"[{_now_iso()}] Auto-repair aplicado con {p.name} "
                        f"por errores: {errors}\n"
                    )
                    with self.paths.auto_repair_log.open("a", encoding="utf-8") as f:
                        f.write(log_line)
                    return {
                        "repaired": True,
                        "source_version": entry.get("version"),
                        "source_file": p.name,
                    }
            except Exception:
                continue

        return {"repaired": False, "source_version": None, "source_file": None}

    def _verify_integrity_internal(
        self,
        auto_repair: bool = True,
        _skip_bootstrap: bool = False,
    ) -> dict[str, Any]:
        """Implementación interna para evitar recursión durante bootstrap."""
        if not _skip_bootstrap:
            self.bootstrap()

        errors = (
            self._validate_core_schema()
            + self._validate_hashes()
            + self._validate_checkpoint_signatures()
        )
        repaired = {"repaired": False, "source_version": None, "source_file": None}

        status = "OK"
        if errors:
            status = "FAIL"
            if auto_repair:
                repaired = self._auto_repair_from_latest_version(errors)
                if repaired.get("repaired"):
                    status = "REPAIRED"

            self._append_corruption_event(
                errors, bool(repaired.get("repaired")), status
            )

        report = {
            "timestamp": _now_iso(),
            "status": status,
            "errors": errors,
            "auto_repaired": bool(repaired.get("repaired")),
            "repair_source_version": repaired.get("source_version"),
            "repair_source_file": repaired.get("source_file"),
        }
        self._write_json(self.paths.last_integrity_scan, report)
        self._refresh_hashes()
        return report

    def verify_integrity(
        self, auto_repair: bool = True, _skip_bootstrap: bool = False
    ) -> dict[str, Any]:
        """Valida schema+hash. Si falla, intenta auto-repair y registra recuperación."""
        return self._verify_integrity_internal(
            auto_repair=auto_repair, _skip_bootstrap=_skip_bootstrap
        )

    def integrity_scan(self, auto_repair: bool = True) -> dict[str, Any]:
        """Alias explícito enterprise para escaneo de integridad."""
        return self.verify_integrity(auto_repair=auto_repair)

    # -------------------------
    # Rollback automático + transacción
    # -------------------------
    def _run_smoke(self, command: str) -> dict[str, Any]:
        p = subprocess.run(
            command,
            cwd=str(self.project_root),
            shell=True,
            capture_output=True,
            text=True,
            timeout=180,
            check=False,
        )
        out = (p.stdout or "") + ("\n" + p.stderr if p.stderr else "")
        return {
            "command": command,
            "returncode": p.returncode,
            "output": out.strip()[:4000],
        }

    def rollback_to_checkpoint(
        self, checkpoint_id: str | None = None, reason: str = ""
    ) -> dict[str, Any]:
        """Restaura estado desde checkpoint y deja trazabilidad en versiones."""
        index = self._read_json(self.paths.checkpoint_index)
        checkpoints = list(index.get("checkpoints", []))
        if not checkpoints:
            return {"ok": False, "error": "No hay checkpoints"}

        target = None
        if checkpoint_id:
            for item in checkpoints:
                if item.get("id") == checkpoint_id:
                    target = item
                    break
        if target is None:
            target = checkpoints[-1]

        cp_file = self.paths.checkpoints / str(target.get("file", ""))
        if not cp_file.exists():
            return {"ok": False, "error": f"Checkpoint no encontrado: {cp_file}"}

        cp = self._read_json(cp_file)
        snap = cp.get("state_snapshot")
        if not isinstance(snap, dict) or not isinstance(snap.get("state", {}), dict):
            return {"ok": False, "error": "state_snapshot inválido"}

        restored_entry = self.save_state(
            state=snap.get("state", {}),
            reason=f"rollback:{reason or 'unspecified'}",
            actor="auto_recovery",
            strategic_note=f"Rollback ejecutado hacia checkpoint {target.get('id')}",
        )

        legacy_restore = {"restored": False, "files": []}
        if self.manage_legacy_v1:
            legacy_restore = self._restore_legacy_snapshot(
                cp.get("legacy_v1_snapshot", {})
            )

        return {
            "ok": True,
            "checkpoint_id": target.get("id"),
            "restored_version": restored_entry.get("version"),
            "legacy_restored": legacy_restore,
        }

    def run_transactional_guard(
        self,
        change_name: str,
        smoke_commands: list[str],
        ai_summary: str = "",
    ) -> dict[str, Any]:
        """Aplica política PRE->SMOKE->POST/ROLLBACK con reporte supervisor."""
        pre = self.create_checkpoint(
            stage="PRE_CAMBIO",
            summary=f"PRE {change_name}",
            extra={"ai_summary": ai_summary},
        )

        smoke_results = [self._run_smoke(cmd) for cmd in smoke_commands]
        smoke_ok = all(r.get("returncode", 1) == 0 for r in smoke_results)

        post = None
        rollback = None
        if smoke_ok:
            post = self.create_checkpoint(
                stage="POST_CAMBIO",
                summary=f"POST {change_name}",
                extra={"ai_summary": ai_summary},
            )
            self.log_decision(
                title="transaction_ok",
                details=f"Cambio {change_name} validado por smoke tests.",
                level="info",
            )
        else:
            rollback = self.rollback_to_checkpoint(
                checkpoint_id=pre.get("id"),
                reason=f"smoke_fail:{change_name}",
            )
            self.log_decision(
                title="transaction_rollback",
                details=f"Rollback automático por fallo smoke en {change_name}",
                level="critical",
            )

        integrity = self.verify_integrity(auto_repair=True)
        summary = self.supervisor_report(
            status="OK" if smoke_ok else "ROLLBACK",
            risk="LOW" if smoke_ok else "HIGH",
            next_action="Continuar evolución controlada"
            if smoke_ok
            else "Revisar cambio antes de reintentar",
        )
        return {
            "change_name": change_name,
            "smoke_ok": smoke_ok,
            "pre_checkpoint": pre,
            "post_checkpoint": post,
            "rollback": rollback,
            "smoke_results": smoke_results,
            "integrity": integrity,
            "supervisor_report": summary,
        }

    def transactional_state_change(
        self,
        change_name: str,
        new_state: dict[str, Any],
        smoke_commands: list[str],
        actor: str = "system",
        ai_summary: str = "",
    ) -> dict[str, Any]:
        """Política segura completa para cambio estructural de estado.

        Flujo:
        1) Checkpoint PRE_CAMBIO
        2) Aplicar nuevo estado (versionado automático)
        3) Smoke tests
        4) POST_CAMBIO si OK, rollback automático si FAIL
        """
        pre = self.create_checkpoint(
            stage="PRE_CAMBIO",
            summary=f"PRE {change_name}",
            extra={"ai_summary": ai_summary},
        )

        state_entry = self.save_state(
            state=new_state,
            reason=f"structural_change:{change_name}",
            actor=actor,
            strategic_note=f"Cambio estructural aplicado: {change_name}",
        )

        smoke_results = [self._run_smoke(cmd) for cmd in smoke_commands]
        smoke_ok = all(r.get("returncode", 1) == 0 for r in smoke_results)

        post = None
        rollback = None
        if smoke_ok:
            post = self.create_checkpoint(
                stage="POST_CAMBIO",
                summary=f"POST {change_name}",
                extra={"ai_summary": ai_summary},
            )
        else:
            rollback = self.rollback_to_checkpoint(
                checkpoint_id=pre.get("id"),
                reason=f"smoke_fail:{change_name}",
            )

        integrity = self.verify_integrity(auto_repair=True)
        report = self.supervisor_report(
            status="OK" if smoke_ok else "ROLLBACK",
            risk="LOW" if smoke_ok else "HIGH",
            next_action="Continuar evolución controlada"
            if smoke_ok
            else "Corregir cambio y reintentar",
        )

        return {
            "change_name": change_name,
            "state_entry": state_entry,
            "smoke_ok": smoke_ok,
            "pre_checkpoint": pre,
            "post_checkpoint": post,
            "rollback": rollback,
            "smoke_results": smoke_results,
            "integrity": integrity,
            "supervisor_report": report,
        }

    # -------------------------
    # Memoria estratégica
    # -------------------------
    def log_decision(self, title: str, details: str, level: str = "info") -> None:
        self.bootstrap()
        line = f"[{_now_iso()}] [{level.upper()}] {title} :: {details}\n"
        with self.paths.decisions_log.open("a", encoding="utf-8") as f:
            f.write(line)

    def log_architectural_evolution(self, note: str) -> None:
        self.bootstrap()
        with self.paths.architectural_evolution.open("a", encoding="utf-8") as f:
            f.write(f"\n## {_now_iso()}\n- {note}\n")

    def log_pattern(
        self,
        pattern_name: str,
        context: str,
        action: str,
        outcome: str,
        tags: list[str] | None = None,
    ) -> None:
        self.bootstrap()
        data = self._read_json(self.paths.learned_patterns)
        data.setdefault("patterns", [])
        data["updated_at"] = _now_iso()
        data["patterns"].append(
            {
                "timestamp": _now_iso(),
                "pattern": pattern_name,
                "context": context,
                "action": action,
                "outcome": outcome,
                "tags": tags or [],
            }
        )
        self._write_json(self.paths.learned_patterns, data)
        self._refresh_hashes()

    # -------------------------
    # Estado + reporte supervisor
    # -------------------------
    def get_status_summary(self) -> dict[str, Any]:
        self.bootstrap()
        si = self._read_json(self.paths.state_index)
        ci = self._read_json(self.paths.checkpoint_index)
        integ = self._read_json(self.paths.last_integrity_scan)
        hashes = self._read_json(self.paths.hashes)
        return {
            "state_version": si.get("current_version", 0),
            "state_versions_count": len(si.get("versions", [])),
            "max_versions": si.get("max_versions", self.max_versions),
            "last_checkpoint": ci.get("last_checkpoint"),
            "checkpoint_count": len(ci.get("checkpoints", [])),
            "integrity_status": integ.get("status", "UNKNOWN"),
            "integrity_timestamp": integ.get("timestamp"),
            "hashes_updated_at": hashes.get("updated_at"),
            "legacy_bridge": self._legacy_bridge_meta(),
        }

    def supervisor_report(
        self, status: str, risk: str, next_action: str
    ) -> dict[str, Any]:
        summary = self.get_status_summary()
        return {
            "estado_general": status,
            "riesgo": risk,
            "version_actual_state": summary.get("state_version"),
            "ultimo_checkpoint": summary.get("last_checkpoint"),
            "integridad": summary.get("integrity_status"),
            "accion_futura_recomendada": next_action,
            "timestamp": _now_iso(),
        }
