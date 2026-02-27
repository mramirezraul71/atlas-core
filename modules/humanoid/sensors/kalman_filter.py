"""
Kalman Filters: Filtros para estimación de estado.
===================================================
Implementaciones de KF, EKF y UKF.
"""
from __future__ import annotations

import logging
from typing import Optional
import numpy as np

_log = logging.getLogger("humanoid.sensors.kalman")


class KalmanFilter:
    """
    Filtro de Kalman lineal.
    
    Estado: x(k+1) = A*x(k) + B*u(k) + w
    Medición: z(k) = H*x(k) + v
    """
    
    def __init__(
        self,
        state_dim: int,
        measurement_dim: int,
        control_dim: int = 0,
    ):
        self.n = state_dim
        self.m = measurement_dim
        self.p = control_dim
        
        # Estado
        self.x = np.zeros(self.n)
        
        # Covarianza del estado
        self.P = np.eye(self.n)
        
        # Matrices del sistema
        self.A = np.eye(self.n)           # Transición de estado
        self.B = np.zeros((self.n, max(1, self.p)))  # Control
        self.H = np.zeros((self.m, self.n))  # Observación
        self.H[:min(self.m, self.n), :min(self.m, self.n)] = np.eye(min(self.m, self.n))
        
        # Ruido
        self.Q = np.eye(self.n) * 0.01    # Ruido de proceso
        self.R = np.eye(self.m) * 0.1     # Ruido de medición
    
    def predict(self, u: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Paso de predicción.
        
        Args:
            u: Control input (opcional)
            
        Returns:
            Estado predicho
        """
        # Predicción del estado
        self.x = self.A @ self.x
        if u is not None and self.p > 0:
            self.x += self.B @ u
        
        # Predicción de la covarianza
        self.P = self.A @ self.P @ self.A.T + self.Q
        
        return self.x.copy()
    
    def update(self, z: np.ndarray) -> np.ndarray:
        """
        Paso de actualización.
        
        Args:
            z: Medición
            
        Returns:
            Estado actualizado
        """
        # Innovación
        y = z - self.H @ self.x
        
        # Covarianza de la innovación
        S = self.H @ self.P @ self.H.T + self.R
        
        # Ganancia de Kalman
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Actualización del estado
        self.x = self.x + K @ y
        
        # Actualización de la covarianza
        I = np.eye(self.n)
        self.P = (I - K @ self.H) @ self.P
        
        return self.x.copy()
    
    def get_state(self) -> np.ndarray:
        """Retorna el estado actual."""
        return self.x.copy()
    
    def set_state(self, x: np.ndarray) -> None:
        """Establece el estado."""
        self.x = x.copy()


class ExtendedKalmanFilter:
    """
    Filtro de Kalman Extendido.
    
    Para sistemas no lineales con jacobiano.
    """
    
    def __init__(
        self,
        state_dim: int,
        measurement_dim: int,
    ):
        self.n = state_dim
        self.m = measurement_dim
        
        # Estado: [x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz, ...]
        self.x = np.zeros(self.n)
        if self.n >= 10:
            self.x[6] = 1.0  # Quaternion w = 1 (identidad)
        
        # Covarianza
        self.P = np.eye(self.n) * 0.1
        
        # Ruido
        self.Q = np.eye(self.n) * 0.001
        self.R = np.eye(self.m) * 0.01
        
        # Delta time
        self.dt = 0.01
    
    def predict(self, dt: Optional[float] = None) -> np.ndarray:
        """Predicción con modelo de movimiento."""
        if dt is not None:
            self.dt = dt
        
        # Modelo de movimiento simple (constante velocity)
        # Posición += velocidad * dt
        if self.n >= 6:
            self.x[:3] += self.x[3:6] * self.dt
        
        # Jacobiano de la transición (simplificado)
        F = np.eye(self.n)
        if self.n >= 6:
            F[:3, 3:6] = np.eye(3) * self.dt
        
        # Actualizar covarianza
        self.P = F @ self.P @ F.T + self.Q
        
        return self.x.copy()
    
    def update(self, z: np.ndarray) -> np.ndarray:
        """Actualización con medición."""
        # Jacobiano de observación (simplificado: observamos posición y vel angular)
        H = np.zeros((self.m, self.n))
        H[:min(self.m, 3), :3] = np.eye(min(self.m, 3))  # Posición
        if self.m > 3 and self.n > 10:
            H[3:min(self.m, 6), 10:min(self.n, 13)] = np.eye(min(self.m - 3, 3))  # Vel angular
        
        # Innovación
        h_x = H @ self.x  # Predicción de medición
        y = z[:self.m] - h_x
        
        # Covarianza de innovación
        S = H @ self.P @ H.T + self.R
        
        # Ganancia de Kalman
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            K = self.P @ H.T @ np.linalg.pinv(S)
        
        # Actualizar estado
        self.x = self.x + K @ y
        
        # Normalizar quaternion si existe
        if self.n >= 10:
            quat = self.x[6:10]
            norm = np.linalg.norm(quat)
            if norm > 0:
                self.x[6:10] = quat / norm
        
        # Actualizar covarianza
        I = np.eye(self.n)
        self.P = (I - K @ H) @ self.P
        
        return self.x.copy()
    
    def get_state(self) -> np.ndarray:
        return self.x.copy()
    
    def get_position(self) -> np.ndarray:
        return self.x[:3].copy()
    
    def get_velocity(self) -> np.ndarray:
        return self.x[3:6].copy() if self.n >= 6 else np.zeros(3)
    
    def get_orientation(self) -> np.ndarray:
        return self.x[6:10].copy() if self.n >= 10 else np.array([1, 0, 0, 0])
