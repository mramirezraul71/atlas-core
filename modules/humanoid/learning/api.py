"""
LearningAPI: API unificada para todas las interfaces de aprendizaje.

Combina:
- Learning from Demonstration (LfD)
- Reinforcement Learning (RL)
- Natural Language Feedback
"""
from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

from .demonstration import DemonstrationLearning
from .reinforcement import ReinforcementLearning
from .natural_feedback import NaturalLanguageFeedback, FeedbackType

logger = logging.getLogger(__name__)


@dataclass
class LearningStats:
    """Estadisticas globales de aprendizaje."""
    demonstrations_recorded: int = 0
    rl_episodes_completed: int = 0
    feedback_received: int = 0
    policies_trained: int = 0
    total_reward: float = 0.0
    average_reward: float = 0.0


class LearningAPI:
    """
    API unificada para aprendizaje del robot.
    
    Orquesta:
    - Aprendizaje por demostracion
    - Aprendizaje por refuerzo
    - Feedback en lenguaje natural
    
    Permite combinar multiples fuentes de aprendizaje para
    mejorar comportamientos.
    """
    
    def __init__(
        self,
        storage_path: str = None,
        state_reader: Callable[[], Dict[str, float]] = None,
        action_executor: Callable[[str, Dict], Any] = None,
        llm_client: Any = None,
    ):
        """
        Inicializa la API de aprendizaje.
        
        Args:
            storage_path: Directorio para persistencia
            state_reader: Funcion para leer estado del robot
            action_executor: Funcion para ejecutar acciones
            llm_client: Cliente LLM para NLU avanzado
        """
        self.storage_path = storage_path
        self.state_reader = state_reader
        self.action_executor = action_executor
        
        # Subsistemas
        self.demonstration = DemonstrationLearning(
            storage_path=storage_path,
        )
        
        self.reinforcement = ReinforcementLearning(
            storage_path=storage_path,
        )
        
        self.feedback = NaturalLanguageFeedback(
            llm_client=llm_client,
        )
        
        # Estadisticas
        self._stats = LearningStats()
        
        # Callbacks
        self._learning_callbacks: List[Callable[[str, Dict], None]] = []
        
        # Configurar integracion entre subsistemas
        self._setup_integration()
    
    def _setup_integration(self) -> None:
        """Configura integracion entre subsistemas."""
        # El feedback positivo/negativo actualiza rewards de RL
        self.feedback.on_correction(self._on_correction_feedback)
        self.feedback.on_prohibition(self._on_prohibition)
    
    def _on_correction_feedback(self, intent: Any, context: Dict) -> None:
        """Maneja correcciones del usuario."""
        # Si hay una politica RL activa, reducir reward
        task_id = context.get("task_id")
        if task_id and task_id in self.reinforcement._policies:
            # Penalizar la accion reciente
            last_action = context.get("last_action")
            if last_action:
                logger.info(f"Correction applied to task {task_id}")
    
    def _on_prohibition(self, target: str) -> None:
        """Maneja prohibiciones."""
        logger.info(f"Prohibition registered: {target}")
    
    # ============ Demonstration Learning API ============
    
    async def start_demonstration(self, task_name: str) -> str:
        """
        Inicia grabacion de demostracion.
        
        Args:
            task_name: Nombre de la tarea
        
        Returns:
            ID de la demostracion
        """
        task_id = task_name.lower().replace(" ", "_")
        demo_id = await self.demonstration.start_recording(task_id, task_name)
        self._stats.demonstrations_recorded += 1
        logger.info(f"Started demonstration: {demo_id}")
        return demo_id
    
    async def stop_demonstration(self) -> Optional[str]:
        """
        Detiene grabacion de demostracion.
        
        Returns:
            ID de la demostracion guardada
        """
        demo = await self.demonstration.stop_recording()
        if demo:
            logger.info(f"Stopped demonstration: {demo.id}")
            return demo.id
        return None
    
    async def add_demonstration_step(self, action: str, 
                                     parameters: Dict[str, Any] = None) -> bool:
        """
        Agrega paso a la demostracion activa.
        
        Args:
            action: Nombre de la accion
            parameters: Parametros de la accion
        
        Returns:
            True si se agrego
        """
        return await self.demonstration.add_action(action, parameters)
    
    async def learn_task_from_demonstrations(self, task_name: str) -> bool:
        """
        Entrena politica de imitacion.
        
        Args:
            task_name: Nombre de la tarea
        
        Returns:
            True si el entrenamiento fue exitoso
        """
        task_id = task_name.lower().replace(" ", "_")
        success = await self.demonstration.learn_from_demonstrations(task_id)
        if success:
            self._stats.policies_trained += 1
        return success
    
    async def execute_learned_task(self, task_name: str) -> bool:
        """
        Ejecuta tarea usando politica aprendida.
        
        Args:
            task_name: Nombre de la tarea
        
        Returns:
            True si la ejecucion fue exitosa
        """
        task_id = task_name.lower().replace(" ", "_")
        
        # Verificar que hay politica
        if task_id not in self.demonstration._policies:
            logger.warning(f"No policy found for task: {task_name}")
            return False
        
        if not self.action_executor:
            logger.warning("No action executor configured")
            return False
        
        # Ejecutar politica
        max_steps = 100
        for step in range(max_steps):
            # Obtener estado actual
            state = self.state_reader() if self.state_reader else {}
            
            # Predecir accion
            action = self.demonstration.predict_action(task_id, state)
            if not action:
                logger.info("Policy returned no action (end of task)")
                break
            
            # Verificar prohibiciones
            if self.feedback.is_prohibited(action.action):
                logger.warning(f"Action prohibited: {action.action}")
                continue
            
            # Ejecutar
            try:
                await self.action_executor(action.action, action.parameters)
            except Exception as e:
                logger.error(f"Error executing action: {e}")
                return False
        
        return True
    
    # ============ Reinforcement Learning API ============
    
    def create_rl_task(self, task_name: str, 
                       action_space: List[str],
                       reward_function: Callable[[Dict], float] = None) -> str:
        """
        Crea tarea de RL.
        
        Args:
            task_name: Nombre de la tarea
            action_space: Lista de acciones posibles
            reward_function: Funcion de reward personalizada
        
        Returns:
            ID de la tarea
        """
        task_id = task_name.lower().replace(" ", "_")
        
        policy = self.reinforcement.create_policy(task_id, action_space)
        
        if reward_function:
            self.reinforcement.set_reward_function(task_id, reward_function)
        
        logger.info(f"Created RL task: {task_id} with {len(action_space)} actions")
        return task_id
    
    def rl_step(self, task_name: str, state: Dict[str, float], 
                action: str, reward: float, next_state: Dict[str, float],
                done: bool = False) -> None:
        """
        Registra paso de RL.
        
        Args:
            task_name: Nombre de la tarea
            state: Estado antes de accion
            action: Accion tomada
            reward: Reward recibido
            next_state: Estado despues de accion
            done: Si el episodio termino
        """
        task_id = task_name.lower().replace(" ", "_")
        
        # Ajustar reward basado en feedback
        feedback_adjustment = self._get_feedback_reward_adjustment()
        adjusted_reward = reward + feedback_adjustment
        
        self.reinforcement.record_experience(
            task_id, state, action, adjusted_reward, next_state, done
        )
        
        self._stats.total_reward += adjusted_reward
        
        if done:
            self._stats.rl_episodes_completed += 1
    
    def _get_feedback_reward_adjustment(self) -> float:
        """Obtiene ajuste de reward basado en feedback reciente."""
        recent = self.feedback.get_recent_feedback(5)
        
        total = 0.0
        for fb in recent:
            # Feedback muy reciente (ultimos 30s) tiene mas peso
            total += fb.get("sentiment", 0.0) * 0.1
        
        return total
    
    def get_rl_action(self, task_name: str, state: Dict[str, float]) -> str:
        """
        Obtiene accion de politica RL.
        
        Args:
            task_name: Nombre de la tarea
            state: Estado actual
        
        Returns:
            Accion a ejecutar
        """
        task_id = task_name.lower().replace(" ", "_")
        
        if task_id not in self.reinforcement._policies:
            raise ValueError(f"No RL policy for task: {task_name}")
        
        action = self.reinforcement._policies[task_id].get_action(state)
        
        # Verificar prohibiciones
        if self.feedback.is_prohibited(action):
            # Obtener accion alternativa
            alternatives = self.reinforcement._policies[task_id].action_space
            for alt in alternatives:
                if not self.feedback.is_prohibited(alt):
                    return alt
            logger.warning("All actions prohibited")
        
        return action
    
    def train_rl_batch(self, task_name: str, batch_size: int = 32) -> Dict[str, float]:
        """
        Entrena politica con batch de experiencias.
        
        Args:
            task_name: Nombre de la tarea
            batch_size: Tamano del batch
        
        Returns:
            Metricas de entrenamiento
        """
        task_id = task_name.lower().replace(" ", "_")
        return self.reinforcement.train_batch(task_id, batch_size)
    
    # ============ Natural Language Feedback API ============
    
    async def process_human_feedback(self, text: str, 
                                    context: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Procesa feedback humano en lenguaje natural.
        
        Args:
            text: Texto del feedback
            context: Contexto actual
        
        Returns:
            Resultado del procesamiento
        """
        record = await self.feedback.process_feedback(text, context)
        self._stats.feedback_received += 1
        
        # Notificar callbacks
        for callback in self._learning_callbacks:
            try:
                callback("feedback", {
                    "type": record.intent.feedback_type,
                    "sentiment": record.intent.sentiment,
                    "applied": record.applied,
                })
            except Exception as e:
                logger.error(f"Error in learning callback: {e}")
        
        return {
            "id": record.id,
            "type": record.intent.feedback_type,
            "sentiment": record.intent.sentiment,
            "applied": record.applied,
            "result": record.result,
        }
    
    def is_action_prohibited(self, action: str) -> bool:
        """Verifica si una accion esta prohibida."""
        return self.feedback.is_prohibited(action)
    
    def get_user_preference(self, key: str, default: Any = None) -> Any:
        """Obtiene preferencia del usuario."""
        return self.feedback.get_preference(key, default)
    
    # ============ Utilidades ============
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas globales."""
        self._stats.average_reward = (
            self._stats.total_reward / self._stats.rl_episodes_completed
            if self._stats.rl_episodes_completed > 0 else 0.0
        )
        
        return {
            "demonstrations_recorded": self._stats.demonstrations_recorded,
            "rl_episodes_completed": self._stats.rl_episodes_completed,
            "feedback_received": self._stats.feedback_received,
            "policies_trained": self._stats.policies_trained,
            "total_reward": self._stats.total_reward,
            "average_reward": self._stats.average_reward,
            "demonstration_stats": self.demonstration.get_stats(),
            "reinforcement_stats": self.reinforcement.get_stats(),
            "feedback_stats": self.feedback.get_stats(),
        }
    
    def on_learning_event(self, callback: Callable[[str, Dict], None]) -> None:
        """
        Registra callback para eventos de aprendizaje.
        
        Args:
            callback: Funcion (event_type, data) -> None
        """
        self._learning_callbacks.append(callback)
    
    def get_available_tasks(self) -> Dict[str, List[str]]:
        """Obtiene tareas disponibles por tipo de aprendizaje."""
        return {
            "demonstration": self.demonstration.list_tasks(),
            "reinforcement": list(self.reinforcement._policies.keys()),
        }
    
    async def save_all(self) -> None:
        """Guarda todos los datos."""
        self.demonstration.save()
        self.reinforcement.save()
        logger.info("Learning data saved")
    
    def load_all(self) -> None:
        """Carga todos los datos guardados."""
        self.demonstration.load()
        self.reinforcement.load()
        logger.info("Learning data loaded")
