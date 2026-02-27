"""
DemonstrationLearning: Aprendizaje por demostracion (LfD/Imitation Learning).

Permite al robot aprender nuevas tareas observando demostraciones humanas.
"""
from __future__ import annotations

import asyncio
import json
import logging
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)


def _bitacora(msg: str, ok: bool = True) -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log
        append_evolution_log(msg, ok=ok, source="learning")
    except Exception:
        pass


@dataclass
class StateObservation:
    """Observacion de estado durante demostracion."""
    timestamp_ns: int
    joint_positions: Dict[str, float]
    joint_velocities: Dict[str, float]
    gripper_state: Dict[str, float]
    end_effector_pose: Optional[Dict[str, float]] = None
    force_torque: Optional[Dict[str, float]] = None
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp_ns": self.timestamp_ns,
            "joint_positions": self.joint_positions,
            "joint_velocities": self.joint_velocities,
            "gripper_state": self.gripper_state,
            "end_effector_pose": self.end_effector_pose,
            "force_torque": self.force_torque,
        }


@dataclass
class Demonstration:
    """Demostracion grabada."""
    id: str
    task_id: str
    task_name: str
    observations: List[StateObservation] = field(default_factory=list)
    
    # Metadatos
    created_at_ns: int = field(default_factory=lambda: time.time_ns())
    duration_ms: float = 0.0
    sample_rate_hz: float = 100.0
    
    # Estado
    is_complete: bool = False
    is_successful: bool = False
    quality_score: float = 0.0  # 0-1, evaluado post-grabacion
    
    # Contexto
    initial_state: Optional[Dict[str, Any]] = None
    final_state: Optional[Dict[str, Any]] = None
    annotations: List[Dict[str, Any]] = field(default_factory=list)
    
    def add_observation(self, obs: StateObservation) -> None:
        """Agrega observacion."""
        self.observations.append(obs)
    
    def complete(self, successful: bool) -> None:
        """Marca demostracion como completa."""
        self.is_complete = True
        self.is_successful = successful
        if self.observations:
            first_ts = self.observations[0].timestamp_ns
            last_ts = self.observations[-1].timestamp_ns
            self.duration_ms = (last_ts - first_ts) / 1e6
    
    def get_trajectory(self) -> List[Dict[str, float]]:
        """Obtiene trayectoria de posiciones articulares."""
        return [obs.joint_positions for obs in self.observations]
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "task_id": self.task_id,
            "task_name": self.task_name,
            "observations_count": len(self.observations),
            "duration_ms": self.duration_ms,
            "sample_rate_hz": self.sample_rate_hz,
            "is_complete": self.is_complete,
            "is_successful": self.is_successful,
            "quality_score": self.quality_score,
        }
    
    def to_full_dict(self) -> Dict[str, Any]:
        """Diccionario completo con observaciones."""
        d = self.to_dict()
        d["observations"] = [obs.to_dict() for obs in self.observations]
        d["initial_state"] = self.initial_state
        d["final_state"] = self.final_state
        d["annotations"] = self.annotations
        return d


class BehavioralCloning:
    """
    Behavioral Cloning simple.
    
    Aprende mapeo estado -> accion de demostraciones.
    """
    
    def __init__(self):
        self._trained = False
        self._states: List[Dict[str, float]] = []
        self._actions: List[Dict[str, float]] = []
    
    def fit(self, demonstrations: List[Demonstration]) -> None:
        """
        Entrena desde demostraciones.
        
        Args:
            demonstrations: Lista de demostraciones
        """
        self._states = []
        self._actions = []
        
        for demo in demonstrations:
            if not demo.is_successful:
                continue
            
            obs = demo.observations
            for i in range(len(obs) - 1):
                # Estado actual
                state = obs[i].joint_positions
                # Accion = diferencia al siguiente estado
                action = {
                    k: obs[i+1].joint_positions.get(k, 0) - v
                    for k, v in state.items()
                }
                
                self._states.append(state)
                self._actions.append(action)
        
        self._trained = len(self._states) > 0
        logger.info(f"BehavioralCloning trained with {len(self._states)} samples")
    
    def predict(self, state: Dict[str, float]) -> Dict[str, float]:
        """
        Predice accion para un estado.
        
        Usa K-NN simple para encontrar estado similar.
        """
        if not self._trained:
            return {}
        
        # Encontrar estado mas similar (distancia euclidiana)
        best_idx = 0
        best_dist = float("inf")
        
        for i, s in enumerate(self._states):
            dist = sum((state.get(k, 0) - v) ** 2 for k, v in s.items())
            if dist < best_dist:
                best_dist = dist
                best_idx = i
        
        return self._actions[best_idx]
    
    def is_trained(self) -> bool:
        return self._trained


class DemonstrationLearning:
    """
    Sistema de aprendizaje por demostracion.
    
    Permite:
    - Grabar demostraciones en modo compliant
    - Aprender politicas de demostraciones
    - Reproducir movimientos aprendidos
    """
    
    def __init__(self, 
                 storage_path: str = None,
                 sample_rate_hz: float = 100.0):
        """
        Inicializa el sistema de LfD.
        
        Args:
            storage_path: Ruta para guardar demostraciones
            sample_rate_hz: Tasa de muestreo
        """
        self.storage_path = Path(storage_path) if storage_path else None
        self.sample_rate_hz = sample_rate_hz
        
        # Demostraciones
        self._demonstrations: Dict[str, Demonstration] = {}
        
        # Politicas aprendidas
        self._policies: Dict[str, BehavioralCloning] = {}
        
        # Estado de grabacion
        self._recording = False
        self._current_demo: Optional[Demonstration] = None
        self._record_task: Optional[asyncio.Task] = None
        
        # Callback para obtener estado actual
        self._get_state_callback: Optional[Callable[[], StateObservation]] = None
        
        # Cargar demostraciones guardadas
        if self.storage_path:
            self._load_demonstrations()
    
    def _load_demonstrations(self) -> None:
        """Carga demostraciones desde disco."""
        if not self.storage_path or not self.storage_path.exists():
            return
        
        demos_file = self.storage_path / "demonstrations.json"
        if not demos_file.exists():
            return
        
        try:
            with open(demos_file, "r", encoding="utf-8") as f:
                data = json.load(f)
            
            for d in data.get("demonstrations", []):
                demo = Demonstration(
                    id=d["id"],
                    task_id=d["task_id"],
                    task_name=d["task_name"],
                    duration_ms=d.get("duration_ms", 0),
                    is_complete=d.get("is_complete", False),
                    is_successful=d.get("is_successful", False),
                    quality_score=d.get("quality_score", 0),
                )
                self._demonstrations[demo.id] = demo
            
            logger.info(f"Loaded {len(self._demonstrations)} demonstrations")
            
        except Exception as e:
            logger.error(f"Error loading demonstrations: {e}")
    
    def _save_demonstrations(self) -> None:
        """Guarda demostraciones a disco."""
        if not self.storage_path:
            return
        
        self.storage_path.mkdir(parents=True, exist_ok=True)
        demos_file = self.storage_path / "demonstrations.json"
        
        try:
            data = {
                "version": "1.0",
                "count": len(self._demonstrations),
                "demonstrations": [
                    d.to_dict() for d in self._demonstrations.values()
                ],
            }
            
            with open(demos_file, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
                
        except Exception as e:
            logger.error(f"Error saving demonstrations: {e}")
    
    def set_state_callback(self, callback: Callable[[], StateObservation]) -> None:
        """
        Establece callback para obtener estado actual del robot.
        
        Args:
            callback: Funcion que retorna StateObservation
        """
        self._get_state_callback = callback
    
    async def add_action(self, action: str, parameters: Dict[str, Any] = None) -> bool:
        """
        Agrega anotacion de accion a la demostracion activa.
        
        Args:
            action: Nombre de la accion
            parameters: Parametros de la accion
        
        Returns:
            True si se agrego exitosamente
        """
        if not self._recording or self._current_demo is None:
            logger.warning("Cannot add action: not recording")
            return False
        
        self._current_demo.annotations.append({
            "timestamp_ns": time.time_ns(),
            "action": action,
            "parameters": parameters or {},
        })
        return True
    
    async def start_recording(self, task_id: str, task_name: str) -> str:
        """
        Inicia grabacion de demostracion.
        
        Args:
            task_id: ID de la tarea
            task_name: Nombre de la tarea
        
        Returns:
            ID de la demostracion
        """
        if self._recording:
            raise RuntimeError("Already recording a demonstration")
        
        demo_id = f"demo_{uuid.uuid4().hex[:12]}"
        
        self._current_demo = Demonstration(
            id=demo_id,
            task_id=task_id,
            task_name=task_name,
            sample_rate_hz=self.sample_rate_hz,
        )
        
        self._recording = True
        self._record_task = asyncio.create_task(self._record_loop())
        
        logger.info(f"Started recording demonstration: {demo_id}")
        _bitacora(f"Demo iniciada: {task_id}")
        return demo_id
    
    async def _record_loop(self) -> None:
        """Loop de grabacion."""
        interval = 1.0 / self.sample_rate_hz
        
        while self._recording:
            try:
                if self._get_state_callback:
                    obs = self._get_state_callback()
                else:
                    # Estado dummy para testing
                    obs = StateObservation(
                        timestamp_ns=time.time_ns(),
                        joint_positions={},
                        joint_velocities={},
                        gripper_state={},
                    )
                
                self._current_demo.add_observation(obs)
                await asyncio.sleep(interval)
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in recording loop: {e}")
    
    async def stop_recording(self, successful: bool = True) -> Demonstration:
        """
        Detiene grabacion y guarda demostracion.
        
        Args:
            successful: Si la demostracion fue exitosa
        
        Returns:
            Demostracion grabada
        """
        if not self._recording:
            raise RuntimeError("Not recording")
        
        self._recording = False
        if self._record_task:
            self._record_task.cancel()
            try:
                await self._record_task
            except asyncio.CancelledError:
                pass
        
        self._current_demo.complete(successful)
        
        # Guardar
        self._demonstrations[self._current_demo.id] = self._current_demo
        self._save_demonstrations()
        
        # Guardar observaciones detalladas
        if self.storage_path:
            obs_file = self.storage_path / f"{self._current_demo.id}_observations.json"
            try:
                with open(obs_file, "w") as f:
                    json.dump(self._current_demo.to_full_dict(), f)
            except Exception as e:
                logger.error(f"Error saving observations: {e}")
        
        demo = self._current_demo
        self._current_demo = None
        
        logger.info(f"Stopped recording: {demo.id}, "
                   f"observations={len(demo.observations)}, "
                   f"duration={demo.duration_ms:.0f}ms")
        
        _bitacora(f"Demo finalizada: {demo.id} obs={len(demo.observations)}")
        
        return demo
    
    async def learn_from_demonstrations(self, task_id: str) -> bool:
        """
        Aprende politica de demostraciones existentes.
        
        Args:
            task_id: ID de la tarea
        
        Returns:
            True si se aprendio exitosamente
        """
        # Obtener demostraciones para esta tarea
        demos = [
            d for d in self._demonstrations.values()
            if d.task_id == task_id and d.is_successful
        ]
        
        if not demos:
            logger.warning(f"No successful demonstrations for task {task_id}")
            return False
        
        # Cargar observaciones completas
        full_demos = []
        for demo in demos:
            if self.storage_path:
                obs_file = self.storage_path / f"{demo.id}_observations.json"
                if obs_file.exists():
                    try:
                        with open(obs_file, "r") as f:
                            data = json.load(f)
                        demo.observations = [
                            StateObservation(**obs) 
                            for obs in data.get("observations", [])
                        ]
                        full_demos.append(demo)
                    except Exception as e:
                        logger.error(f"Error loading observations for {demo.id}: {e}")
        
        if not full_demos:
            logger.warning(f"No demonstrations with observations for task {task_id}")
            return False
        
        # Entrenar politica
        policy = BehavioralCloning()
        policy.fit(full_demos)
        
        if policy.is_trained():
            self._policies[task_id] = policy
            logger.info(f"Learned policy for task {task_id}")
            return True
        
        return False
    
    def get_policy(self, task_id: str) -> Optional[BehavioralCloning]:
        """Obtiene politica aprendida para una tarea."""
        return self._policies.get(task_id)
    
    def predict_action(self, task_id: str, 
                      current_state: Dict[str, float]) -> Optional[Dict[str, float]]:
        """
        Predice accion usando politica aprendida.
        
        Args:
            task_id: ID de la tarea
            current_state: Estado actual
        
        Returns:
            Accion predicha o None
        """
        policy = self._policies.get(task_id)
        if not policy or not policy.is_trained():
            return None
        
        return policy.predict(current_state)
    
    def list_demonstrations(self, task_id: str = None) -> List[Dict[str, Any]]:
        """Lista demostraciones disponibles."""
        demos = self._demonstrations.values()
        if task_id:
            demos = [d for d in demos if d.task_id == task_id]
        return [d.to_dict() for d in demos]
    
    def list_policies(self) -> List[str]:
        """Lista tareas con politicas aprendidas."""
        return list(self._policies.keys())
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas."""
        return {
            "total_demonstrations": len(self._demonstrations),
            "successful_demonstrations": sum(
                1 for d in self._demonstrations.values() if d.is_successful
            ),
            "policies_count": len(self._policies),
            "is_recording": self._recording,
        }
