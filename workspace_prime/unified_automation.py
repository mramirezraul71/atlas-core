"""
Atlas Unified Automation System
Unifica todos los sistemas automáticos existentes:
- Schedule (simple)
- Humanoid Scheduler (avanzado)
- Celery (disponible)
- Git Automation (nuevo)
- Workspace Tools (integrado)
"""

import asyncio
import logging
import os
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from typing import Any, Callable, Dict, Optional

# Importar sistemas existentes
try:
    import schedule as simple_schedule

    SIMPLE_SCHEDULER_AVAILABLE = True
except ImportError:
    SIMPLE_SCHEDULER_AVAILABLE = False

try:
    from modules.humanoid.scheduler.db import SchedulerDB
    from modules.humanoid.scheduler.engine import (start_scheduler,
                                                   stop_scheduler)

    HUMANOID_SCHEDULER_AVAILABLE = True
except ImportError:
    HUMANOID_SCHEDULER_AVAILABLE = False

try:
    import celery

    CELERY_AVAILABLE = True
except ImportError:
    CELERY_AVAILABLE = False

from git_automation import git_automation
from tools_integration import tools


class AutomationLevel(Enum):
    """Niveles de automatización"""

    BASIC = "basic"  # Schedule simple
    ADVANCED = "advanced"  # Humanoid Scheduler
    ENTERPRISE = "enterprise"  # Celery + Redis
    UNIFIED = "unified"  # Todos unificados


@dataclass
class AutomationTask:
    """Tarea de automatización unificada"""

    id: str
    name: str
    level: AutomationLevel
    schedule: str  # cron-like expression
    function: Callable
    enabled: bool = True
    max_retries: int = 3
    timeout: int = 300
    metadata: Dict[str, Any] = None


class AtlasUnifiedAutomation:
    """Sistema unificado de automatización para Atlas"""

    def __init__(self):
        self.logger = self._setup_logger()
        self.tasks: Dict[str, AutomationTask] = {}
        self.active_level = AutomationLevel.BASIC
        self.running = False
        self.scheduler_task: Optional[asyncio.Task] = None

        # Verificar disponibilidad de sistemas
        self.available_systems = {
            "simple_schedule": SIMPLE_SCHEDULER_AVAILABLE,
            "humanoid_scheduler": HUMANOID_SCHEDULER_AVAILABLE,
            "celery": CELERY_AVAILABLE,
            "git_automation": True,
            "workspace_tools": True,
        }

        self.logger.info(f"Sistemas disponibles: {self.available_systems}")

    def _setup_logger(self) -> logging.Logger:
        """Configura logging unificado"""
        logger = logging.getLogger("AtlasUnifiedAutomation")
        logger.setLevel(logging.INFO)

        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)

        return logger

    def register_task(self, task: AutomationTask) -> bool:
        """Registra una tarea de automatización"""
        try:
            # Validar tarea
            if not task.id or not task.function:
                self.logger.error(f"Tarea inválida: {task.id}")
                return False

            # Verificar compatibilidad con nivel actual
            if task.level.value > self.active_level.value:
                self.logger.warning(
                    f"Tarea {task.id} requiere nivel {task.level.value}, actual: {self.active_level.value}"
                )

            self.tasks[task.id] = task
            self.logger.info(f"Tarea registrada: {task.id} ({task.level.value})")
            return True

        except Exception as e:
            self.logger.error(f"Error registrando tarea {task.id}: {e}")
            return False

    def set_automation_level(self, level: AutomationLevel) -> bool:
        """Establece el nivel de automatización"""
        try:
            # Detener sistema actual
            if self.running:
                self.stop()

            # Cambiar nivel
            self.active_level = level
            self.logger.info(f"Nivel de automatización: {level.value}")

            # Iniciar nuevo sistema
            return self.start()

        except Exception as e:
            self.logger.error(f"Error cambiando nivel: {e}")
            return False

    def start(self) -> bool:
        """Inicia el sistema de automatización"""
        try:
            if self.running:
                self.logger.warning("Sistema ya está corriendo")
                return True

            # Iniciar según nivel
            if self.active_level == AutomationLevel.BASIC:
                return self._start_basic_scheduler()
            elif self.active_level == AutomationLevel.ADVANCED:
                return self._start_advanced_scheduler()
            elif self.active_level == AutomationLevel.ENTERPRISE:
                return self._start_enterprise_scheduler()
            elif self.active_level == AutomationLevel.UNIFIED:
                return self._start_unified_scheduler()
            else:
                self.logger.error(f"Nivel no soportado: {self.active_level}")
                return False

        except Exception as e:
            self.logger.error(f"Error iniciando automatización: {e}")
            return False

    def stop(self) -> bool:
        """Detiene el sistema de automatización"""
        try:
            if not self.running:
                return True

            self.running = False

            # Detener scheduler task
            if self.scheduler_task:
                self.scheduler_task.cancel()
                self.scheduler_task = None

            # Detener sistemas específicos
            if self.active_level in [AutomationLevel.ADVANCED, AutomationLevel.UNIFIED]:
                if HUMANOID_SCHEDULER_AVAILABLE:
                    stop_scheduler()

            self.logger.info("Sistema de automatización detenido")
            return True

        except Exception as e:
            self.logger.error(f"Error deteniendo automatización: {e}")
            return False

    def _start_basic_scheduler(self) -> bool:
        """Inicia scheduler básico con schedule library"""
        if not SIMPLE_SCHEDULER_AVAILABLE:
            self.logger.error("Schedule library no disponible")
            return False

        try:
            # Configurar tareas básicas
            for task in self.tasks.values():
                if task.level == AutomationLevel.BASIC and task.enabled:
                    self._schedule_basic_task(task)

            # Iniciar loop básico
            self.running = True
            self.scheduler_task = asyncio.create_task(self._run_basic_loop())

            self.logger.info("Scheduler básico iniciado")
            return True

        except Exception as e:
            self.logger.error(f"Error iniciando scheduler básico: {e}")
            return False

    def _start_advanced_scheduler(self) -> bool:
        """Inicia scheduler avanzado Humanoid"""
        if not HUMANOID_SCHEDULER_AVAILABLE:
            self.logger.error("Humanoid scheduler no disponible")
            return False

        try:
            # Iniciar Humanoid Scheduler
            start_scheduler()

            # Registrar tareas en Humanoid
            self._register_humanoid_tasks()

            self.running = True
            self.logger.info("Scheduler avanzado (Humanoid) iniciado")
            return True

        except Exception as e:
            self.logger.error(f"Error iniciando scheduler avanzado: {e}")
            return False

    def _start_enterprise_scheduler(self) -> bool:
        """Inicia scheduler enterprise con Celery"""
        if not CELERY_AVAILABLE:
            self.logger.error("Celery no disponible")
            return False

        try:
            # Configurar Celery app
            from celery import Celery

            self.celery_app = Celery("atlas_automation")

            # Configurar Redis broker
            self.celery_app.conf.update(
                broker_url="redis://localhost:6379/0",
                result_backend="redis://localhost:6379/0",
                task_serializer="json",
                accept_content=["json"],
                result_serializer="json",
                timezone="UTC",
                enable_utc=True,
            )

            # Registrar tareas Celery
            self._register_celery_tasks()

            self.running = True
            self.logger.info("Scheduler enterprise (Celery) iniciado")
            return True

        except Exception as e:
            self.logger.error(f"Error iniciando scheduler enterprise: {e}")
            return False

    def _start_unified_scheduler(self) -> bool:
        """Inicia scheduler unificado (todos los sistemas)"""
        try:
            # Iniciar todos los sistemas disponibles
            systems_started = []

            if SIMPLE_SCHEDULER_AVAILABLE:
                self._start_basic_scheduler()
                systems_started.append("basic")

            if HUMANOID_SCHEDULER_AVAILABLE:
                self._start_advanced_scheduler()
                systems_started.append("advanced")

            if CELERY_AVAILABLE:
                self._start_enterprise_scheduler()
                systems_started.append("enterprise")

            self.running = True
            self.logger.info(f"Scheduler unificado iniciado: {systems_started}")
            return True

        except Exception as e:
            self.logger.error(f"Error iniciando scheduler unificado: {e}")
            return False

    def _schedule_basic_task(self, task: AutomationTask):
        """Programa tarea en scheduler básico"""
        try:
            # Parsear schedule (simplificado)
            if "every" in task.schedule.lower():
                if "minute" in task.schedule.lower():
                    simple_schedule.every().minute.do(task.function)
                elif "hour" in task.schedule.lower():
                    simple_schedule.every().hour.do(task.function)
                elif "day" in task.schedule.lower():
                    simple_schedule.every().day.do(task.function)
            else:
                self.logger.warning(f"Schedule no soportado: {task.schedule}")

        except Exception as e:
            self.logger.error(f"Error programando tarea básica {task.id}: {e}")

    async def _run_basic_loop(self):
        """Loop de ejecución para scheduler básico"""
        while self.running:
            try:
                simple_schedule.run_pending()
                await asyncio.sleep(1)
            except Exception as e:
                self.logger.error(f"Error en loop básico: {e}")
                await asyncio.sleep(5)

    def _register_humanoid_tasks(self):
        """Registra tareas en Humanoid Scheduler"""
        try:
            if not HUMANOID_SCHEDULER_AVAILABLE:
                return

            db = SchedulerDB()

            for task in self.tasks.values():
                if (
                    task.level in [AutomationLevel.ADVANCED, AutomationLevel.UNIFIED]
                    and task.enabled
                ):
                    # Crear job en Humanoid
                    job_data = {
                        "id": task.id,
                        "name": task.name,
                        "kind": "custom",
                        "schedule": task.schedule,
                        "payload": {
                            "function_name": task.function.__name__,
                            "metadata": task.metadata or {},
                        },
                        "max_retries": task.max_retries,
                        "timeout": task.timeout,
                    }

                    db.add_job(job_data)
                    self.logger.info(f"Tarea registrada en Humanoid: {task.id}")

        except Exception as e:
            self.logger.error(f"Error registrando tareas Humanoid: {e}")

    def _register_celery_tasks(self):
        """Registra tareas en Celery"""
        try:
            if not CELERY_AVAILABLE:
                return

            for task in self.tasks.values():
                if (
                    task.level in [AutomationLevel.ENTERPRISE, AutomationLevel.UNIFIED]
                    and task.enabled
                ):
                    # Crear tarea Celery
                    @self.celery_app.task(name=task.id)
                    def celery_task():
                        return task.function()

                    self.logger.info(f"Tarea registrada en Celery: {task.id}")

        except Exception as e:
            self.logger.error(f"Error registrando tareas Celery: {e}")

    def get_status(self) -> Dict[str, Any]:
        """Obtiene estado completo del sistema"""
        return {
            "running": self.running,
            "active_level": self.active_level.value,
            "available_systems": self.available_systems,
            "registered_tasks": len(self.tasks),
            "enabled_tasks": len([t for t in self.tasks.values() if t.enabled]),
            "task_details": {
                task_id: {
                    "name": task.name,
                    "level": task.level.value,
                    "enabled": task.enabled,
                    "schedule": task.schedule,
                }
                for task_id, task in self.tasks.items()
            },
        }

    def execute_task_now(self, task_id: str) -> Dict[str, Any]:
        """Ejecuta tarea inmediatamente"""
        try:
            if task_id not in self.tasks:
                return {"ok": False, "error": f"Tarea {task_id} no encontrada"}

            task = self.tasks[task_id]

            if not task.enabled:
                return {"ok": False, "error": f"Tarea {task_id} deshabilitada"}

            # Ejecutar según nivel
            if task.level == AutomationLevel.BASIC:
                result = task.function()
            elif task.level in [AutomationLevel.ADVANCED, AutomationLevel.UNIFIED]:
                if HUMANOID_SCHEDULER_AVAILABLE:
                    # Usar Humanoid runner
                    job_data = {
                        "id": task_id,
                        "kind": "custom",
                        "payload": {"function_name": task.function.__name__},
                    }
                    from modules.humanoid.scheduler.runner import run_job_sync

                    result = run_job_sync(job_data)
                else:
                    result = task.function()
            else:
                result = task.function()

            return {
                "ok": True,
                "task_id": task_id,
                "result": result,
                "timestamp": datetime.now(timezone.utc).isoformat(),
            }

        except Exception as e:
            self.logger.error(f"Error ejecutando tarea {task_id}: {e}")
            return {"ok": False, "error": str(e)}


# Instancia global
unified_automation = AtlasUnifiedAutomation()


# Tareas predefinidas para Atlas
def setup_atlas_tasks():
    """Configura tareas predefinidas de Atlas"""

    # 1. Git Backup Task
    git_backup_task = AutomationTask(
        id="git_backup",
        name="Git Repository Backup",
        level=AutomationLevel.BASIC,
        schedule="every 1 hour",
        function=lambda: git_automation.automated_workflow(
            "commit_and_push", message="backup(automated): scheduled backup"
        ),
        metadata={"category": "version_control", "priority": "medium"},
    )

    # 2. System Health Check
    health_check_task = AutomationTask(
        id="health_check",
        name="System Health Check",
        level=AutomationLevel.BASIC,
        schedule="every 5 minutes",
        function=lambda: tools.get_all_status(),
        metadata={"category": "monitoring", "priority": "high"},
    )

    # 3. Workspace Cleanup
    cleanup_task = AutomationTask(
        id="workspace_cleanup",
        name="Workspace Cleanup",
        level=AutomationLevel.ADVANCED,
        schedule="every 6 hours",
        function=lambda: _workspace_cleanup(),
        metadata={"category": "maintenance", "priority": "low"},
    )

    # 4. Trading Analysis
    trading_task = AutomationTask(
        id="trading_analysis",
        name="Automated Trading Analysis",
        level=AutomationLevel.UNIFIED,
        schedule="every 30 minutes",
        function=lambda: _trading_analysis(),
        metadata={"category": "trading", "priority": "high"},
    )

    # Registrar tareas
    tasks = [git_backup_task, health_check_task, cleanup_task, trading_task]

    for task in tasks:
        unified_automation.register_task(task)

    return len(tasks)


def _workspace_cleanup():
    """Limpieza automática del workspace"""
    try:
        # Limpiar archivos temporales
        import shutil
        import tempfile

        temp_dir = tempfile.gettempdir()
        cleaned = 0

        for item in os.listdir(temp_dir):
            if item.startswith("atlas_"):
                item_path = os.path.join(temp_dir, item)
                try:
                    if os.path.isfile(item_path):
                        os.remove(item_path)
                    else:
                        shutil.rmtree(item_path)
                    cleaned += 1
                except:
                    pass

        return {"cleaned_files": cleaned, "timestamp": datetime.now().isoformat()}

    except Exception as e:
        return {"error": str(e)}


def _trading_analysis():
    """Análisis de trading automatizado"""
    try:
        # Usar Ollama para análisis
        result = tools.ollama_generate(
            "Analiza condiciones actuales del mercado EUR/USD y sugiere posibles entradas",
            model="llama3.2:3b",
        )

        return {
            "analysis": result.get("response", ""),
            "model": "llama3.2:3b",
            "timestamp": datetime.now().isoformat(),
        }

    except Exception as e:
        return {"error": str(e)}


if __name__ == "__main__":
    # Demo del sistema unificado
    print("🤖 Atlas Unified Automation System")
    print("=" * 50)

    # Configurar tareas
    task_count = setup_atlas_tasks()
    print(f"📋 Tareas configuradas: {task_count}")

    # Establecer nivel unificado
    unified_automation.set_automation_level(AutomationLevel.UNIFIED)

    # Mostrar estado
    status = unified_automation.get_status()
    print(f"📊 Estado: {status}")

    # Ejecutar una tarea de prueba
    result = unified_automation.execute_task_now("health_check")
    print(f"✅ Health check: {result.get('ok', False)}")

    print("\n🎯 Sistema unificado listo para uso")
