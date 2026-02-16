"""
Sim2Real Transfer: Transferencia simulación a realidad.
========================================================
Técnicas para transferir políticas entrenadas en simulación.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional
import numpy as np

_log = logging.getLogger("simulation.sim2real")


@dataclass
class TransferConfig:
    """Configuración de transferencia sim2real."""
    use_domain_randomization: bool = True
    use_system_identification: bool = False
    use_adaptation: bool = True
    adaptation_steps: int = 100
    real_world_noise: float = 0.02


class Sim2RealTransfer:
    """
    Sistema de transferencia Sim-to-Real.
    
    Implementa técnicas para mejorar la transferencia
    de políticas entrenadas en simulación a hardware real.
    """
    
    def __init__(self, config: Optional[TransferConfig] = None):
        self.config = config or TransferConfig()
        self._policy = None
        self._adapted_policy = None
        self._real_world_data: List[Dict[str, Any]] = []
    
    def prepare_policy(self, policy: Any) -> Any:
        """
        Prepara una política para transferencia.
        
        Args:
            policy: Política entrenada en simulación
            
        Returns:
            Política preparada para el mundo real
        """
        self._policy = policy
        
        # Aplicar técnicas de transferencia
        adapted = policy
        
        if self.config.use_adaptation:
            adapted = self._apply_noise_injection(adapted)
        
        self._adapted_policy = adapted
        _log.info("Policy prepared for sim2real transfer")
        
        return adapted
    
    def _apply_noise_injection(self, policy: Any) -> Any:
        """Inyecta ruido para robustez."""
        # Placeholder: en implementación real, modificar la política
        _log.debug("Applied noise injection to policy")
        return policy
    
    def collect_real_data(self, state: Dict[str, Any], action: Any, next_state: Dict[str, Any]) -> None:
        """
        Recolecta datos del mundo real para adaptación.
        
        Args:
            state: Estado actual
            action: Acción tomada
            next_state: Estado resultante
        """
        self._real_world_data.append({
            "state": state,
            "action": action,
            "next_state": next_state,
        })
        
        # Adaptar si hay suficientes datos
        if len(self._real_world_data) >= self.config.adaptation_steps:
            self._adapt_policy()
    
    def _adapt_policy(self) -> None:
        """Adapta la política usando datos reales."""
        if not self._real_world_data:
            return
        
        # Placeholder: implementar adaptación real
        _log.info("Adapting policy with %d real-world samples", len(self._real_world_data))
        
        # Limpiar datos usados
        self._real_world_data = []
    
    def evaluate_gap(self, sim_states: List[Dict], real_states: List[Dict]) -> Dict[str, float]:
        """
        Evalúa la brecha sim-to-real.
        
        Args:
            sim_states: Estados de simulación
            real_states: Estados del mundo real
            
        Returns:
            Métricas de la brecha
        """
        if not sim_states or not real_states:
            return {"error": "No data"}
        
        # Calcular diferencias (simplificado)
        pos_errors = []
        vel_errors = []
        
        for sim, real in zip(sim_states[:len(real_states)], real_states):
            if "robot_pos" in sim and "robot_pos" in real:
                pos_err = np.linalg.norm(
                    np.array(sim["robot_pos"]) - np.array(real["robot_pos"])
                )
                pos_errors.append(pos_err)
            
            if "robot_vel" in sim and "robot_vel" in real:
                vel_err = np.linalg.norm(
                    np.array(sim["robot_vel"]) - np.array(real["robot_vel"])
                )
                vel_errors.append(vel_err)
        
        return {
            "position_error_mean": np.mean(pos_errors) if pos_errors else 0,
            "position_error_std": np.std(pos_errors) if pos_errors else 0,
            "velocity_error_mean": np.mean(vel_errors) if vel_errors else 0,
            "velocity_error_std": np.std(vel_errors) if vel_errors else 0,
            "samples_compared": len(pos_errors),
        }
    
    def get_transfer_report(self) -> Dict[str, Any]:
        """Genera reporte de transferencia."""
        return {
            "policy_prepared": self._adapted_policy is not None,
            "real_data_collected": len(self._real_world_data),
            "config": {
                "domain_randomization": self.config.use_domain_randomization,
                "system_identification": self.config.use_system_identification,
                "adaptation": self.config.use_adaptation,
            },
        }
