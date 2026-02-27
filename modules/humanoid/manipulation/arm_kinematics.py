"""
Arm Kinematics: Cinemática de brazos.
======================================
Cinemática directa e inversa para brazos robóticos.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

_log = logging.getLogger("humanoid.manipulation.arm_kinematics")


@dataclass
class ArmConfig:
    """Configuración del brazo."""
    name: str = "right_arm"
    num_joints: int = 7
    # Longitudes de segmentos (m)
    link_lengths: Tuple[float, ...] = (0.0, 0.0, 0.35, 0.0, 0.35, 0.0, 0.1)
    # Límites de joints (rad)
    joint_limits_lower: Tuple[float, ...] = (-2.9, -1.7, -2.9, -0.1, -2.9, -1.4, -3.0)
    joint_limits_upper: Tuple[float, ...] = (2.9, 1.7, 2.9, 2.8, 2.9, 1.4, 3.0)


class ArmKinematics:
    """
    Cinemática para brazos de ATLAS.
    
    Implementa:
    - Forward Kinematics (FK)
    - Inverse Kinematics (IK) numérica
    - Jacobiano
    """
    
    def __init__(self, config: Optional[ArmConfig] = None):
        self.config = config or ArmConfig()
        
        # Estado actual
        self._joint_positions = np.zeros(self.config.num_joints)
        
        # Parámetros DH simplificados
        self._dh_params = self._compute_dh_params()
    
    def _compute_dh_params(self) -> List[Dict[str, float]]:
        """Calcula parámetros DH del brazo."""
        # Simplificado: brazo tipo KUKA/Panda
        # [theta, d, a, alpha]
        dh = [
            {"d": 0.0, "a": 0.0, "alpha": -np.pi/2},     # Joint 1
            {"d": 0.0, "a": 0.0, "alpha": np.pi/2},      # Joint 2
            {"d": 0.35, "a": 0.0, "alpha": np.pi/2},     # Joint 3
            {"d": 0.0, "a": 0.0, "alpha": -np.pi/2},     # Joint 4
            {"d": 0.35, "a": 0.0, "alpha": -np.pi/2},    # Joint 5
            {"d": 0.0, "a": 0.0, "alpha": np.pi/2},      # Joint 6
            {"d": 0.1, "a": 0.0, "alpha": 0.0},          # Joint 7 (end effector)
        ]
        return dh
    
    def forward_kinematics(self, joint_positions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calcula cinemática directa.
        
        Args:
            joint_positions: Posiciones de joints (7,)
            
        Returns:
            (position [x,y,z], rotation_matrix [3x3])
        """
        T = np.eye(4)
        
        for i, (q, dh) in enumerate(zip(joint_positions, self._dh_params)):
            T_i = self._dh_matrix(q, dh["d"], dh["a"], dh["alpha"])
            T = T @ T_i
        
        position = T[:3, 3]
        rotation = T[:3, :3]
        
        self._joint_positions = joint_positions.copy()
        
        return position, rotation
    
    def _dh_matrix(self, theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        """Matriz de transformación DH."""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        
        return np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1],
        ])
    
    def inverse_kinematics(
        self,
        target_position: np.ndarray,
        target_orientation: Optional[np.ndarray] = None,
        initial_guess: Optional[np.ndarray] = None,
        max_iterations: int = 100,
        tolerance: float = 1e-4,
    ) -> Optional[np.ndarray]:
        """
        Calcula cinemática inversa usando Newton-Raphson.
        
        Args:
            target_position: Posición objetivo [x, y, z]
            target_orientation: Matriz de rotación objetivo (opcional)
            initial_guess: Configuración inicial
            max_iterations: Máximo de iteraciones
            tolerance: Tolerancia de convergencia
            
        Returns:
            Posiciones de joints o None si no converge
        """
        # Configuración inicial
        q = initial_guess if initial_guess is not None else self._joint_positions.copy()
        
        for iteration in range(max_iterations):
            # FK actual
            pos, rot = self.forward_kinematics(q)
            
            # Error de posición
            pos_error = target_position - pos
            
            # Error de orientación (si se especifica)
            if target_orientation is not None:
                # Usar representación de eje-ángulo
                R_error = target_orientation @ rot.T
                trace = np.clip((np.trace(R_error) - 1) / 2, -1, 1)
                angle = np.arccos(trace)
                if angle > 1e-6:
                    axis = np.array([
                        R_error[2, 1] - R_error[1, 2],
                        R_error[0, 2] - R_error[2, 0],
                        R_error[1, 0] - R_error[0, 1],
                    ]) / (2 * np.sin(angle))
                    ori_error = axis * angle
                else:
                    ori_error = np.zeros(3)
                
                error = np.concatenate([pos_error, ori_error])
            else:
                error = pos_error
            
            # Verificar convergencia
            if np.linalg.norm(error) < tolerance:
                _log.debug("IK converged in %d iterations", iteration)
                return self._apply_joint_limits(q)
            
            # Jacobiano
            J = self._jacobian(q)
            if target_orientation is None:
                J = J[:3, :]  # Solo posición
            
            # Pseudo-inversa con regularización (damped least squares)
            damping = 0.1
            J_pinv = J.T @ np.linalg.inv(J @ J.T + damping**2 * np.eye(J.shape[0]))
            
            # Actualizar joints
            dq = J_pinv @ error
            q = q + 0.5 * dq  # Factor de paso
        
        _log.warning("IK did not converge")
        return None
    
    def _jacobian(self, q: np.ndarray) -> np.ndarray:
        """Calcula el Jacobiano analítico."""
        n = self.config.num_joints
        J = np.zeros((6, n))
        
        T = np.eye(4)
        transforms = [T.copy()]
        
        for i, dh in enumerate(self._dh_params):
            T_i = self._dh_matrix(q[i], dh["d"], dh["a"], dh["alpha"])
            T = T @ T_i
            transforms.append(T.copy())
        
        end_effector_pos = T[:3, 3]
        
        for i in range(n):
            T_i = transforms[i]
            z_i = T_i[:3, 2]  # Eje z del frame i
            p_i = T_i[:3, 3]  # Posición del frame i
            
            # Jacobiano lineal
            J[:3, i] = np.cross(z_i, end_effector_pos - p_i)
            # Jacobiano angular
            J[3:, i] = z_i
        
        return J
    
    def _apply_joint_limits(self, q: np.ndarray) -> np.ndarray:
        """Aplica límites de joints."""
        return np.clip(
            q,
            self.config.joint_limits_lower,
            self.config.joint_limits_upper,
        )
    
    def get_workspace_point(self, radius: float = 0.6, theta: float = 0, phi: float = 0) -> np.ndarray:
        """Genera un punto en el espacio de trabajo."""
        x = radius * np.sin(phi) * np.cos(theta)
        y = radius * np.sin(phi) * np.sin(theta)
        z = radius * np.cos(phi) + 0.5  # Offset en z
        return np.array([x, y, z])
    
    def is_reachable(self, position: np.ndarray) -> bool:
        """Verifica si una posición es alcanzable."""
        result = self.inverse_kinematics(position, max_iterations=50)
        return result is not None
    
    def to_dict(self) -> Dict[str, Any]:
        pos, rot = self.forward_kinematics(self._joint_positions)
        return {
            "arm_name": self.config.name,
            "num_joints": self.config.num_joints,
            "joint_positions": self._joint_positions.tolist(),
            "end_effector_position": pos.tolist(),
        }
