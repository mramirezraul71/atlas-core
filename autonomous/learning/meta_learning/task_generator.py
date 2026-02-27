"""
Generador de tareas sintéticas para meta-entrenamiento MAML.
"""
from __future__ import annotations

from typing import Dict, List

import numpy as np

STATE_DIM = 128  # Compatible con MAMLPolicy(state_dim=128)


def _pad_state(arr: np.ndarray, dim: int = STATE_DIM) -> np.ndarray:
    out = np.zeros(dim, dtype=np.float32)
    out[: len(arr)] = arr
    return out


class TaskGenerator:
    """Genera tareas sintéticas para meta-entrenamiento."""

    @staticmethod
    def generate_navigation_tasks(num_tasks: int = 50) -> List[Dict]:
        """Tareas de navegación: origen aleatorio → meta fija."""
        tasks = []
        for i in range(num_tasks):
            goal_x = np.random.uniform(-10, 10)
            goal_y = np.random.uniform(-10, 10)
            demonstrations = []
            validation = []
            for demo_idx in range(20):
                start_x = np.random.uniform(-10, 10)
                start_y = np.random.uniform(-10, 10)
                distance = np.sqrt(
                    (goal_x - start_x) ** 2 + (goal_y - start_y) ** 2
                )
                state = _pad_state(
                    np.array(
                        [start_x, start_y, goal_x, goal_y, distance],
                        dtype=np.float32,
                    )
                )
                move_x = (
                    (goal_x - start_x) / distance
                    if distance > 1e-6
                    else 0.0
                )
                move_y = (
                    (goal_y - start_y) / distance
                    if distance > 1e-6
                    else 0.0
                )
                action = np.array([move_x, move_y], dtype=np.float32)
                action = _pad_state(action, 64)
                reward = float(-distance)
                if demo_idx < 16:
                    demonstrations.append((state, action, reward))
                else:
                    validation.append((state, action, reward))
            tasks.append(
                {
                    "name": f"navigate_to_{goal_x:.1f}_{goal_y:.1f}",
                    "goal_position": (goal_x, goal_y),
                    "demonstrations": demonstrations,
                    "validation_data": validation,
                }
            )
        return tasks

    @staticmethod
    def generate_manipulation_tasks(num_tasks: int = 30) -> List[Dict]:
        """Tareas de manipulación: distintas masa/fricción."""
        tasks = []
        for i in range(num_tasks):
            mass = np.random.uniform(0.1, 5.0)
            friction = np.random.uniform(0.1, 0.9)
            demonstrations = []
            validation = []
            for demo_idx in range(15):
                gripper_force = np.random.uniform(0, 50)
                lifted = 1.0 if gripper_force > mass * 10 else 0.0
                state = _pad_state(
                    np.array(
                        [gripper_force, mass, friction, lifted],
                        dtype=np.float32,
                    )
                )
                optimal_force = mass * 10 * (1 + friction)
                action = np.array([optimal_force], dtype=np.float32)
                action = _pad_state(action, 64)
                reward = lifted * 10 - abs(gripper_force - optimal_force) * 0.1
                if demo_idx < 12:
                    demonstrations.append((state, action, reward))
                else:
                    validation.append((state, action, reward))
            tasks.append(
                {
                    "name": f"pick_object_mass_{mass:.1f}_friction_{friction:.1f}",
                    "object_properties": {"mass": mass, "friction": friction},
                    "demonstrations": demonstrations,
                    "validation_data": validation,
                }
            )
        return tasks
