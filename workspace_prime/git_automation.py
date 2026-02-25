"""
Atlas Git Automation - Herramientas especializadas para gestión de repositorios
Integración con GitPython, pre-commit y automatización avanzada
"""

import json
import logging
import os
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional


class AtlasGitAutomation:
    """Sistema de automatización Git para Atlas con capacidades avanzadas"""

    def __init__(self, repo_path: str = None):
        self.repo_path = repo_path or str(Path.cwd())
        self.logger = self._setup_logger()

        # Configuración de Git
        self.git_config = {
            "user.name": "Atlas Agent",
            "user.email": "agent@atlas.ai",
            "auto.setup.branch": "true",
            "init.defaultBranch": "main",
            "pull.rebase": "false",
            "push.autoSetupRemote": "true",
        }

        # 🛡️ SEGURIDAD: Archivos excluidos automáticamente
        self.excluded_patterns = [
            "*.key",
            "*.pem",
            "*.env",
            "*.secret",
            "*.token",
            "__pycache__/",
            "*.pyc",
            "*.tmp",
            "*.log",
            "*.cache",
            "node_modules/",
            ".vscode/",
            ".idea/",
            "*.swp",
            "*.swo",
            "data/",
            "cache/",
            "temp/",
            "backup/",
            "*.bak",
            ".DS_Store",
            "Thumbs.db",
            "*.pid",
            "*.lock",
        ]

        # 🛡️ SEGURIDAD: Límites de seguridad
        self.max_files_per_commit = 50
        self.max_file_size_mb = 10
        self.require_confirmation_for_large_commits = True

        # Configuración de pre-commit
        self.precommit_config = {
            "repos": [
                {
                    "repo": "https://github.com/pre-commit/pre-commit-hooks",
                    "rev": "v4.5.0",
                    "hooks": [
                        {"id": "trailing-whitespace"},
                        {"id": "end-of-file-fixer"},
                        {"id": "check-yaml"},
                        {"id": "check-added-large-files"},
                        {"id": "check-json"},
                        {"id": "check-merge-conflict"},
                        {"id": "debug-statements"},
                    ],
                },
                {
                    "repo": "https://github.com/psf/black",
                    "rev": "23.12.1",
                    "hooks": [{"id": "black"}],
                },
                {
                    "repo": "https://github.com/pycqa/isort",
                    "rev": "5.13.2",
                    "hooks": [{"id": "isort"}],
                },
            ]
        }

        self._initialize_git()

    def _should_exclude_file(self, file_path: str) -> bool:
        """🛡️ Verifica si un archivo debe ser excluido por seguridad"""
        file_name = os.path.basename(file_path)

        # Verificar patrones de exclusión
        for pattern in self.excluded_patterns:
            if pattern.startswith("*/"):
                # Directorio
                if pattern[2:] in file_path:
                    return True
            elif pattern.startswith("*"):
                # Extensión
                if file_name.endswith(pattern[1:]):
                    return True
            elif pattern in file_path:
                # Contenido exacto
                return True

        return False

    def _check_file_sizes(self, files: List[str]) -> Dict[str, Any]:
        """🛡️ Verifica tamaños de archivos para seguridad"""
        large_files = []

        for file_path in files:
            try:
                if os.path.exists(file_path):
                    size_mb = os.path.getsize(file_path) / (1024 * 1024)
                    if size_mb > self.max_file_size_mb:
                        large_files.append(f"{file_path} ({size_mb:.1f}MB)")
            except Exception as e:
                self.logger.warning(f"No se pudo verificar tamaño de {file_path}: {e}")

        if large_files:
            return {
                "ok": False,
                "error": f"Archivos grandes detectados (límite: {self.max_file_size_mb}MB): {large_files}",
            }
        return {"ok": True}

    def _filter_safe_files(self, files: List[str]) -> List[str]:
        """🛡️ Filtra archivos seguros para commit"""
        safe_files = []
        excluded_files = []

        for file_path in files:
            if self._should_exclude_file(file_path):
                excluded_files.append(file_path)
                self.logger.warning(f"Archivo excluido por seguridad: {file_path}")
            else:
                safe_files.append(file_path)

        if excluded_files:
            self.logger.info(f"Archivos excluidos: {len(excluded_files)}")

        return safe_files

    def _setup_logger(self) -> logging.Logger:
        logger = logging.getLogger("AtlasGit")
        logger.setLevel(logging.INFO)

        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)

        return logger

    def _initialize_git(self):
        """Inicializa configuración Git y pre-commit"""
        try:
            import git

            self.repo = git.Repo(self.repo_path)
            self.logger.info(f"Repositorio Git encontrado: {self.repo_path}")
        except Exception:
            self.logger.info("Inicializando nuevo repositorio Git...")
            import git

            self.repo = git.Repo.init(self.repo_path)

        # Aplicar configuración Git
        for key, value in self.git_config.items():
            try:
                with self.repo.config_writer() as config:
                    config.set_value("user", key.split(".")[1], value)
            except Exception as e:
                self.logger.warning(f"No se pudo configurar {key}: {e}")

        # Configurar pre-commit
        self._setup_precommit()

    def _setup_precommit(self):
        """Configura pre-commit hooks"""
        precommit_file = Path(self.repo_path) / ".pre-commit-config.yaml"

        if not precommit_file.exists():
            import yaml

            with open(precommit_file, "w") as f:
                yaml.dump(self.precommit_config, f, default_flow_style=False)

            self.logger.info("Configuración pre-commit creada")

            # Instalar hooks
            try:
                subprocess.run(
                    ["pre-commit", "install"],
                    cwd=self.repo_path,
                    check=True,
                    capture_output=True,
                )
                self.logger.info("Pre-commit hooks instalados")
            except subprocess.CalledProcessError as e:
                self.logger.warning(f"No se pudieron instalar pre-commit hooks: {e}")

    def smart_commit(
        self,
        message: str,
        files: List[str] = None,
        auto_stage: bool = True,
        dry_run: bool = False,
    ) -> Dict[str, Any]:
        """Commit inteligente con análisis automático y seguridad mejorada"""
        try:
            import git

            # SEGURIDAD: Obtener lista de archivos
            if files:
                # Filtrar archivos proporcionados
                safe_files = self._filter_safe_files(files)
            elif auto_stage:
                # Obtener todos los archivos modificados
                all_files = [item.a_path for item in self.repo.index.diff(None)]
                all_files.extend(self.repo.untracked_files)
                safe_files = self._filter_safe_files(all_files)
            else:
                safe_files = []

            # SEGURIDAD: Verificar límite de archivos
            if len(safe_files) > self.max_files_per_commit:
                return {
                    "ok": False,
                    "error": f"Demasiados archivos para commit automático (límite: {self.max_files_per_commit}, encontrados: {len(safe_files)})",
                }

            # SEGURIDAD: Verificar tamaños de archivos
            size_check = self._check_file_sizes(safe_files)
            if not size_check.get("ok"):
                return size_check

            # SEGURIDAD: Modo dry-run
            if dry_run:
                return {
                    "ok": True,
                    "dry_run": True,
                    "would_commit": safe_files,
                    "message": message,
                    "files_count": len(safe_files),
                }

            # Analizar cambios
            changes = self._analyze_changes()

            # Generar mensaje mejorado si es necesario
            if len(message) < 20:
                message = self._generate_smart_message(message, changes)

            # Staging seguro
            for file in safe_files:
                try:
                    self.repo.index.add([file])
                    self.logger.info(f"Staged: {file}")
                except Exception as e:
                    self.logger.warning(f"No se pudo stage {file}: {e}")

            # Ejecutar pre-commit si está configurado
            if self._has_precommit():
                try:
                    subprocess.run(
                        ["pre-commit", "run", "--all-files"],
                        cwd=self.repo_path,
                        check=True,
                        capture_output=True,
                    )
                    self.logger.info("Pre-commit checks pasados")
                except subprocess.CalledProcessError as e:
                    self.logger.error(f"Pre-commit checks fallaron: {e}")
                    return {"ok": False, "error": "Pre-commit checks fallaron"}

            # Realizar commit
            commit = self.repo.index.commit(message)

            result = {
                "ok": True,
                "commit_hash": commit.hexsha,
                "message": message,
                "files_changed": len(safe_files),
                "changes": changes,
                "timestamp": datetime.now().isoformat(),
                "security": {
                    "excluded_count": len(files) - len(safe_files) if files else 0,
                    "size_check": "passed",
                    "limit_check": "passed",
                },
            }

            self.logger.info(f"Commit exitoso: {commit.hexsha[:8]}")
            return result

        except Exception as e:
            self.logger.error(f"Error en commit: {e}")
            return {"ok": False, "error": str(e)}

    def _analyze_changes(self) -> List[Dict[str, Any]]:
        """Analiza los cambios en el repositorio"""
        changes = []

        try:
            # Archivos modificados
            for item in self.repo.index.diff(None):
                changes.append(
                    {
                        "file": item.a_path,
                        "type": "modified",
                        "changes": item.diff.decode("utf-8", errors="ignore")[:500],
                    }
                )

            # Archivos nuevos
            for item in self.repo.untracked_files:
                changes.append(
                    {"file": item, "type": "untracked", "changes": "new file"}
                )

        except Exception as e:
            self.logger.warning(f"Error analizando cambios: {e}")

        return changes

    def _generate_smart_message(self, base_message: str, changes: List[Dict]) -> str:
        """Genera mensaje de commit mejorado basado en cambios"""
        if not changes:
            return base_message

        # Contar tipos de cambios
        types = {}
        for change in changes:
            file_type = (
                change["file"].split(".")[-1] if "." in change["file"] else "other"
            )
            types[file_type] = types.get(file_type, 0) + 1

        # Construir mensaje
        type_summary = ", ".join(
            [f"{count} {file_type}" for file_type, count in types.items()]
        )

        enhanced_message = (
            f"{base_message}\n\nChanges: {len(changes)} files ({type_summary})"
        )

        return enhanced_message

    def _has_precommit(self) -> bool:
        """Verifica si pre-commit está disponible"""
        precommit_file = Path(self.repo_path) / ".pre-commit-config.yaml"
        return precommit_file.exists()

    def smart_push(
        self, branch: str = None, force: bool = False, create_backup: bool = True
    ) -> Dict[str, Any]:
        """Push inteligente con verificación y backup automático"""
        try:
            import git

            # Determinar branch
            if not branch:
                branch = self.repo.active_branch.name

            # Verificar estado del repositorio
            status = self.repo.git.status("--porcelain")
            if status.strip():
                return {"ok": False, "error": "Hay cambios sin commitear"}

            # SEGURIDAD: Backup antes de force push
            if force and create_backup:
                backup_tag = (
                    f"backup/before-force-{datetime.now().strftime('%Y%m%d-%H%M%S')}"
                )
                try:
                    self.repo.create_tag(backup_tag, self.repo.head.commit)
                    self.logger.info(f"Backup tag creado: {backup_tag}")
                except Exception as e:
                    self.logger.warning(f"No se pudo crear backup tag: {e}")

            # Verificar si está ahead de remote
            try:
                ahead = self.repo.git.rev_list("--count", f"origin/{branch}..{branch}")
                ahead = int(ahead)
            except:
                ahead = 0

            if ahead == 0 and not force:
                return {
                    "ok": True,
                    "message": "No hay commits para hacer push",
                    "pushed": 0,
                }

            # Realizar push
            if force:
                origin = self.repo.remote(name="origin")
                result = origin.push(branch, force=True)
                self.logger.warning(f"Force push realizado a {branch}")
            else:
                origin = self.repo.remote(name="origin")
                result = origin.push(branch)

            pushed_commits = len(result) if result else 0

            return {
                "ok": True,
                "branch": branch,
                "pushed_commits": pushed_commits,
                "message": f"Push exitoso a {branch}",
                "timestamp": datetime.now().isoformat(),
                "security": {
                    "force_used": force,
                    "backup_created": backup_tag if force and create_backup else None,
                    "ahead_count": ahead,
                },
            }

        except Exception as e:
            self.logger.error(f"Error en push: {e}")
            return {"ok": False, "error": str(e)}

    def create_branch(
        self, branch_name: str, from_branch: str = None
    ) -> Dict[str, Any]:
        """Crea nueva branch con configuración automática"""
        try:
            import git

            # Determinar branch base
            if from_branch:
                base = self.repo.branches[from_branch]
            else:
                base = self.repo.active_branch

            # Crear nueva branch
            new_branch = self.repo.create_head(branch_name, base)
            new_branch.checkout()

            # Configurar tracking remoto
            try:
                origin = self.repo.remote(name="origin")
                origin.push(branch_name, set_upstream=True)
            except:
                self.logger.warning("No se pudo configurar tracking remoto")

            return {
                "ok": True,
                "branch": branch_name,
                "from_branch": base.name,
                "message": f"Branch {branch_name} creada exitosamente",
            }

        except Exception as e:
            self.logger.error(f"Error creando branch: {e}")
            return {"ok": False, "error": str(e)}

    def merge_branch(
        self, source_branch: str, target_branch: str = None, strategy: str = "merge"
    ) -> Dict[str, Any]:
        """Merge inteligente con resolución de conflictos"""
        try:
            import git

            # Determinar branch target
            if not target_branch:
                target_branch = self.repo.active_branch.name

            # Cambiar a target branch
            self.repo.heads[target_branch].checkout()

            # Realizar merge
            if strategy == "merge":
                merge_result = self.repo.merge(source_branch)
            else:
                merge_result = self.repo.git.merge(f"{source_branch}", "--no-ff")

            return {
                "ok": True,
                "source": source_branch,
                "target": target_branch,
                "strategy": strategy,
                "message": f"Merge de {source_branch} a {target_branch} exitoso",
            }

        except Exception as e:
            self.logger.error(f"Error en merge: {e}")
            return {"ok": False, "error": str(e)}

    def get_repo_status(self) -> Dict[str, Any]:
        """Obtiene estado completo del repositorio"""
        try:
            import git

            status = {
                "branch": self.repo.active_branch.name,
                "is_clean": self.repo.is_dirty(),
                "untracked_files": self.repo.untracked_files,
                "modified_files": [item.a_path for item in self.repo.index.diff(None)],
                "staged_files": [item.a_path for item in self.repo.index.diff("HEAD")],
                "last_commit": {
                    "hash": self.repo.head.commit.hexsha,
                    "message": self.repo.head.commit.message,
                    "author": str(self.repo.head.commit.author),
                    "date": self.repo.head.commit.committed_datetime.isoformat(),
                },
                "remotes": [remote.name for remote in self.repo.remotes],
            }

            return status

        except Exception as e:
            self.logger.error(f"Error obteniendo status: {e}")
            return {"ok": False, "error": str(e)}

    def automated_workflow(self, action: str, **kwargs) -> Dict[str, Any]:
        """Workflow automatizado completo"""
        workflows = {
            "commit_and_push": self._workflow_commit_push,
            "create_feature_branch": self._workflow_feature_branch,
            "update_and_sync": self._workflow_update_sync,
            "cleanup_branch": self._workflow_cleanup_branch,
        }

        if action not in workflows:
            return {"ok": False, "error": f"Workflow {action} no encontrado"}

        return workflows[action](**kwargs)

    def _workflow_commit_push(self, message: str, files: List[str] = None, **kwargs):
        """Workflow: Commit y Push automático"""
        # 1. Commit inteligente
        commit_result = self.smart_commit(message, files)
        if not commit_result.get("ok"):
            return commit_result

        # 2. Push automático
        push_result = self.smart_push()

        return {
            "ok": True,
            "commit": commit_result,
            "push": push_result,
            "workflow": "commit_and_push",
        }

    def _workflow_feature_branch(self, feature_name: str, **kwargs):
        """Workflow: Crear feature branch"""
        branch_name = f"feature/{feature_name}"

        # 1. Crear branch
        branch_result = self.create_branch(branch_name)

        return {
            "ok": branch_result.get("ok"),
            "branch": branch_result,
            "workflow": "create_feature_branch",
        }

    def _workflow_update_sync(self, **kwargs):
        """Workflow: Actualizar y sincronizar con remote"""
        try:
            import git

            # 1. Pull latest changes
            origin = self.repo.remote(name="origin")
            origin.pull()

            # 2. Push local changes si hay
            push_result = self.smart_push()

            return {
                "ok": True,
                "pull": "completed",
                "push": push_result,
                "workflow": "update_sync",
            }

        except Exception as e:
            return {"ok": False, "error": str(e)}

    def _workflow_cleanup_branch(self, branch_name: str, **kwargs):
        """Workflow: Limpieza de branch"""
        try:
            import git

            # Eliminar branch local
            if branch_name in self.repo.heads:
                self.repo.delete_head(branch_name, force=True)

            # Eliminar branch remoto
            try:
                origin = self.repo.remote(name="origin")
                origin.push(f":{branch_name}")
            except:
                pass

            return {"ok": True, "branch": branch_name, "workflow": "cleanup_branch"}

        except Exception as e:
            return {"ok": False, "error": str(e)}


# Instancia global para uso en Atlas
git_automation = AtlasGitAutomation()

if __name__ == "__main__":
    # Demostración de capacidades
    print("🔧 Atlas Git Automation - Demo")
    print("=" * 40)

    # Status del repo
    status = git_automation.get_repo_status()
    print(f"📁 Branch actual: {status.get('branch', 'N/A')}")
    print(f"🧹 Repo limpio: {status.get('is_clean', 'N/A')}")

    # Ejemplo de workflow
    if not status.get("is_clean"):
        result = git_automation.automated_workflow(
            "commit_and_push",
            message="Actualización automatizada desde Atlas",
            files=["workspace_prime/git_automation.py"],
        )
        print(f"✅ Workflow resultado: {result.get('ok', False)}")
    else:
        print("📋 No hay cambios para commitear")
