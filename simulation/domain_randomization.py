"""
Domain Randomization: Variación de parámetros de simulación.
=============================================================
Mejora la transferencia sim-to-real variando parámetros.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

_log = logging.getLogger("simulation.domain_randomization")


@dataclass
class RandomizationRange:
    """Rango de randomización para un parámetro."""
    name: str
    min_val: float
    max_val: float
    distribution: str = "uniform"  # uniform, normal, log_uniform


@dataclass
class DomainConfig:
    """Configuración de domain randomization."""
    enabled: bool = True
    randomize_physics: bool = True
    randomize_visuals: bool = True
    randomize_dynamics: bool = True
    seed: Optional[int] = None


class DomainRandomizer:
    """
    Sistema de Domain Randomization.
    
    Varía parámetros de simulación para mejorar
    la robustez de políticas entrenadas.
    """
    
    def __init__(self, config: Optional[DomainConfig] = None):
        self.config = config or DomainConfig()
        
        if self.config.seed is not None:
            np.random.seed(self.config.seed)
        
        # Rangos por defecto
        self._physics_ranges: List[RandomizationRange] = [
            RandomizationRange("gravity_z", -10.5, -9.0),
            RandomizationRange("timestep", 0.001, 0.004),
            RandomizationRange("friction", 0.5, 1.5),
            RandomizationRange("restitution", 0.0, 0.3),
        ]
        
        self._dynamics_ranges: List[RandomizationRange] = [
            RandomizationRange("mass_scale", 0.8, 1.2),
            RandomizationRange("inertia_scale", 0.8, 1.2),
            RandomizationRange("joint_damping", 0.1, 2.0),
            RandomizationRange("joint_friction", 0.01, 0.5),
            RandomizationRange("motor_strength", 0.8, 1.2),
        ]
        
        self._visual_ranges: List[RandomizationRange] = [
            RandomizationRange("light_intensity", 0.5, 1.5),
            RandomizationRange("camera_fov", 50, 90),
            RandomizationRange("texture_scale", 0.5, 2.0),
        ]
        
        self._current_params: Dict[str, float] = {}
    
    def randomize(self) -> Dict[str, float]:
        """
        Genera un nuevo conjunto de parámetros aleatorios.
        
        Returns:
            Diccionario con los parámetros randomizados
        """
        if not self.config.enabled:
            return {}
        
        params = {}
        
        if self.config.randomize_physics:
            for r in self._physics_ranges:
                params[r.name] = self._sample(r)
        
        if self.config.randomize_dynamics:
            for r in self._dynamics_ranges:
                params[r.name] = self._sample(r)
        
        if self.config.randomize_visuals:
            for r in self._visual_ranges:
                params[r.name] = self._sample(r)
        
        self._current_params = params
        return params
    
    def _sample(self, r: RandomizationRange) -> float:
        """Samplea un valor según la distribución."""
        if r.distribution == "uniform":
            return np.random.uniform(r.min_val, r.max_val)
        elif r.distribution == "normal":
            mean = (r.min_val + r.max_val) / 2
            std = (r.max_val - r.min_val) / 4
            return np.clip(np.random.normal(mean, std), r.min_val, r.max_val)
        elif r.distribution == "log_uniform":
            return np.exp(np.random.uniform(np.log(r.min_val), np.log(r.max_val)))
        else:
            return np.random.uniform(r.min_val, r.max_val)
    
    def add_range(self, name: str, min_val: float, max_val: float, 
                  category: str = "dynamics", distribution: str = "uniform") -> None:
        """Añade un nuevo rango de randomización."""
        r = RandomizationRange(name, min_val, max_val, distribution)
        
        if category == "physics":
            self._physics_ranges.append(r)
        elif category == "dynamics":
            self._dynamics_ranges.append(r)
        elif category == "visual":
            self._visual_ranges.append(r)
    
    def get_current_params(self) -> Dict[str, float]:
        """Retorna los parámetros actuales."""
        return self._current_params.copy()
    
    def apply_to_env(self, env: Any, params: Optional[Dict[str, float]] = None) -> None:
        """
        Aplica parámetros a un entorno (placeholder).
        
        Args:
            env: Entorno de simulación
            params: Parámetros a aplicar (o usar actuales)
        """
        params = params or self._current_params
        
        # TODO: Implementar aplicación real a MuJoCo/PyBullet
        _log.debug("Applied %d randomized parameters", len(params))
    
    def get_ranges(self) -> Dict[str, List[Dict[str, Any]]]:
        """Retorna todos los rangos configurados."""
        return {
            "physics": [{"name": r.name, "min": r.min_val, "max": r.max_val} for r in self._physics_ranges],
            "dynamics": [{"name": r.name, "min": r.min_val, "max": r.max_val} for r in self._dynamics_ranges],
            "visual": [{"name": r.name, "min": r.min_val, "max": r.max_val} for r in self._visual_ranges],
        }
