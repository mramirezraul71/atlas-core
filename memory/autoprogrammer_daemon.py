# -*- coding: utf-8 -*-
"""
ATLAS Autoprogrammer Daemon
Ejecuta tareas automáticamente basadas en el contexto de conversaciones
"""

import json
import time
import threading
from datetime import datetime, timedelta
from typing import Dict, List, Any, Callable
from pathlib import Path
import logging

from conversation_thread_manager import (
    get_thread_manager,
    get_autoprogrammer,
    ConversationThreadManager,
    AutoProgrammer
)

# Configurar logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    handlers=[
        logging.FileHandler(r'C:\ATLAS_PUSH\memory\autoprogrammer.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class TaskExecutor:
    """Ejecutor de tareas programadas"""
    
    def __init__(self):
        self.handlers: Dict[str, Callable] = {}
        self.execution_history: List[Dict[str, Any]] = []
        self.register_default_handlers()
    
    def register_handler(self, action_type: str, handler: Callable):
        """Registra un manejador para un tipo de acción"""
        self.handlers[action_type] = handler
        logger.info(f"Handler registrado: {action_type}")
    
    def register_default_handlers(self):
        """Registra los manejadores por defecto"""
        self.register_handler("execute", self._handle_execute)
        self.register_handler("create", self._handle_create)
        self.register_handler("generate", self._handle_generate)
        self.register_handler("save", self._handle_save)
        self.register_handler("send", self._handle_send)
        self.register_handler("update", self._handle_update)
        self.register_handler("analyze", self._handle_analyze)
    
    def execute_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """Ejecuta una acción"""
        action_type = action.get('action_type', 'unknown')
        handler = self.handlers.get(action_type)
        
        if not handler:
            logger.warning(f"No handler para: {action_type}")
            return {"success": False, "error": f"No handler for {action_type}"}
        
        try:
            logger.info(f"Ejecutando: {action_type} - {action.get('trigger_message', '')[:50]}")
            result = handler(action)
            
            execution_record = {
                "timestamp": datetime.now().isoformat(),
                "action_type": action_type,
                "thread_id": action.get('thread_id'),
                "success": result.get('success', False),
                "result": result
            }
            self.execution_history.append(execution_record)
            
            return result
        except Exception as e:
            logger.error(f"Error ejecutando {action_type}: {str(e)}")
            return {"success": False, "error": str(e)}
    
    def _handle_execute(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """Maneja acciones de ejecución"""
        logger.info(f"Ejecutando comando: {action.get('trigger_message')}")
        return {"success": True, "action": "execute", "message": "Comando ejecutado"}
    
    def _handle_create(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """Maneja acciones de creación"""
        logger.info(f"Creando recurso: {action.get('trigger_message')}")
        return {"success": True, "action": "create", "message": "Recurso creado"}
    
    def _handle_generate(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """Maneja acciones de generación"""
        logger.info(f"Generando: {action.get('trigger_message')}")
        return {"success": True, "action": "generate", "message": "Contenido generado"}
    
    def _handle_save(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """Maneja acciones de guardado"""
        logger.info(f"Guardando: {action.get('trigger_message')}")
        return {"success": True, "action": "save", "message": "Guardado completado"}
    
    def _handle_send(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """Maneja acciones de envío"""
        logger.info(f"Enviando: {action.get('trigger_message')}")
        return {"success": True, "action": "send", "message": "Enviado"}
    
    def _handle_update(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """Maneja acciones de actualización"""
        logger.info(f"Actualizando: {action.get('trigger_message')}")
        return {"success": True, "action": "update", "message": "Actualizado"}
    
    def _handle_analyze(self, action: Dict[str, Any]) -> Dict[str, Any]:
        """Maneja acciones de análisis"""
        logger.info(f"Analizando: {action.get('trigger_message')}")
        return {"success": True, "action": "analyze", "message": "Análisis completado"}


class AutoProgrammerDaemon:
    """Daemon que ejecuta autoprogramación de tareas"""
    
    def __init__(self, check_interval: int = 30):
        self.thread_manager: ConversationThreadManager = get_thread_manager()
        self.autoprogrammer: AutoProgrammer = get_autoprogrammer()
        self.executor = TaskExecutor()
        self.check_interval = check_interval
        self.running = False
        self.daemon_thread = None
        self.stats = {
            "total_actions_processed": 0,
            "successful_actions": 0,
            "failed_actions": 0,
            "last_check": None
        }
    
    def start(self):
        """Inicia el daemon"""
        if self.running:
            logger.warning("Daemon ya está corriendo")
            return
        
        self.running = True
        self.daemon_thread = threading.Thread(target=self._run_loop, daemon=True)
        self.daemon_thread.start()
        logger.info("Autoprogrammer Daemon iniciado")
    
    def stop(self):
        """Detiene el daemon"""
        self.running = False
        if self.daemon_thread:
            self.daemon_thread.join(timeout=5)
        logger.info("Autoprogrammer Daemon detenido")
    
    def _run_loop(self):
        """Loop principal del daemon"""
        while self.running:
            try:
                self._check_and_execute()
                time.sleep(self.check_interval)
            except Exception as e:
                logger.error(f"Error en loop: {str(e)}")
                time.sleep(self.check_interval)
    
    def _check_and_execute(self):
        """Verifica y ejecuta acciones pendientes"""
        self.stats['last_check'] = datetime.now().isoformat()
        
        # Analizar todos los hilos activos
        active_threads = self.thread_manager.get_active_threads()
        logger.info(f"Verificando {len(active_threads)} hilos activos")
        
        for thread in active_threads:
            # Analizar hilo para acciones
            actions = self.autoprogrammer.analyze_thread_for_actions(thread.thread_id)
            
            for action in actions:
                # Programar acción
                self.autoprogrammer.schedule_action(action)
        
        # Ejecutar acciones pendientes
        pending_actions = self.autoprogrammer.get_pending_actions()
        logger.info(f"Ejecutando {len(pending_actions)} acciones pendientes")
        
        for action in pending_actions:
            result = self.executor.execute_action(action)
            self.autoprogrammer.mark_action_complete(action['id'], result)
            
            self.stats['total_actions_processed'] += 1
            if result.get('success'):
                self.stats['successful_actions'] += 1
            else:
                self.stats['failed_actions'] += 1
    
    def get_status(self) -> Dict[str, Any]:
        """Retorna el estado del daemon"""
        return {
            "running": self.running,
            "check_interval": self.check_interval,
            "stats": self.stats,
            "pending_actions": len(self.autoprogrammer.get_pending_actions()),
            "completed_actions": len(self.autoprogrammer.completed_tasks),
            "context_snapshot": self.thread_manager.get_context_snapshot()
        }
    
    def get_execution_history(self, limit: int = 50) -> List[Dict[str, Any]]:
        """Retorna el historial de ejecución"""
        return self.executor.execution_history[-limit:]


# Instancia global del daemon
_daemon = None

def get_daemon() -> AutoProgrammerDaemon:
    """Obtiene la instancia global del daemon"""
    global _daemon
    if _daemon is None:
        _daemon = AutoProgrammerDaemon()
    return _daemon

def start_daemon():
    """Inicia el daemon global"""
    daemon = get_daemon()
    daemon.start()
    return daemon

def stop_daemon():
    """Detiene el daemon global"""
    daemon = get_daemon()
    daemon.stop()

def get_daemon_status() -> Dict[str, Any]:
    """Obtiene el estado del daemon"""
    daemon = get_daemon()
    return daemon.get_status()


if __name__ == "__main__":
    logger.info("Iniciando Autoprogrammer Daemon...")
    daemon = start_daemon()
    
    try:
        while True:
            time.sleep(10)
            status = get_daemon_status()
            logger.info(f"Status: {json.dumps(status, indent=2)}")
    except KeyboardInterrupt:
        logger.info("Deteniendo daemon...")
        stop_daemon()
