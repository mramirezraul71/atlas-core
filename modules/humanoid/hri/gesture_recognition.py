"""
Gesture Recognition: Detección de gestos humanos.
==================================================
Reconoce gestos de manos y cuerpo.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

_log = logging.getLogger("humanoid.hri.gesture")


class GestureType(str, Enum):
    """Tipos de gestos reconocidos."""
    NONE = "none"
    WAVE = "wave"
    POINT = "point"
    THUMBS_UP = "thumbs_up"
    THUMBS_DOWN = "thumbs_down"
    STOP = "stop"
    COME_HERE = "come_here"
    OK = "ok"
    GRAB = "grab"
    RELEASE = "release"


@dataclass
class Gesture:
    """Gesto detectado."""
    type: GestureType
    confidence: float
    hand: str = "right"  # right, left, both
    position: Optional[np.ndarray] = None  # 3D position if available
    direction: Optional[np.ndarray] = None  # Direction for pointing
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type.value,
            "confidence": self.confidence,
            "hand": self.hand,
            "position": self.position.tolist() if self.position is not None else None,
            "direction": self.direction.tolist() if self.direction is not None else None,
        }


class GestureRecognizer:
    """
    Reconocedor de gestos para ATLAS.
    
    Métodos:
    - MediaPipe Hands
    - Pose estimation
    - Custom gesture classifiers
    """
    
    def __init__(self):
        self._last_gestures: List[Gesture] = []
        self._gesture_history: List[Tuple[float, Gesture]] = []
        self._history_max_len = 100
        
        # Landmarks de mano (MediaPipe format)
        self._hand_landmarks: Optional[np.ndarray] = None
        
        _log.info("Gesture recognizer initialized")
    
    def process_frame(self, rgb_frame: np.ndarray) -> List[Gesture]:
        """
        Procesa un frame y detecta gestos.
        
        Args:
            rgb_frame: Imagen RGB (H x W x 3)
            
        Returns:
            Lista de gestos detectados
        """
        gestures = []
        
        # Placeholder: Detección real con MediaPipe
        # import mediapipe as mp
        # mp_hands = mp.solutions.hands
        # with mp_hands.Hands() as hands:
        #     results = hands.process(rgb_frame)
        #     if results.multi_hand_landmarks:
        #         for hand_landmarks in results.multi_hand_landmarks:
        #             gesture = self._classify_gesture(hand_landmarks)
        #             if gesture:
        #                 gestures.append(gesture)
        
        self._last_gestures = gestures
        
        # Añadir al historial
        import time
        for g in gestures:
            self._gesture_history.append((time.time(), g))
            if len(self._gesture_history) > self._history_max_len:
                self._gesture_history.pop(0)
        
        return gestures
    
    def process_landmarks(self, landmarks: np.ndarray, hand: str = "right") -> Optional[Gesture]:
        """
        Clasifica gesto desde landmarks de mano.
        
        Args:
            landmarks: Landmarks de MediaPipe (21 x 3)
            hand: Mano (right/left)
            
        Returns:
            Gesto detectado o None
        """
        if len(landmarks) != 21:
            return None
        
        self._hand_landmarks = landmarks
        
        # Extraer puntos clave
        wrist = landmarks[0]
        thumb_tip = landmarks[4]
        index_tip = landmarks[8]
        middle_tip = landmarks[12]
        ring_tip = landmarks[16]
        pinky_tip = landmarks[20]
        
        index_mcp = landmarks[5]
        
        # Detectar gestos por geometría
        
        # Thumbs up: pulgar arriba, otros dedos cerrados
        thumb_up = thumb_tip[1] < wrist[1] - 0.1
        fingers_closed = all(
            landmarks[tip][1] > landmarks[tip - 2][1]
            for tip in [8, 12, 16, 20]
        )
        if thumb_up and fingers_closed:
            return Gesture(
                type=GestureType.THUMBS_UP,
                confidence=0.8,
                hand=hand,
                position=wrist,
            )
        
        # Stop: palma abierta
        fingers_extended = all(
            np.linalg.norm(landmarks[tip] - wrist) > 0.15
            for tip in [4, 8, 12, 16, 20]
        )
        if fingers_extended:
            return Gesture(
                type=GestureType.STOP,
                confidence=0.7,
                hand=hand,
                position=wrist,
            )
        
        # Point: índice extendido, otros cerrados
        index_extended = np.linalg.norm(index_tip - wrist) > 0.15
        others_closed = all(
            np.linalg.norm(landmarks[tip] - wrist) < 0.12
            for tip in [12, 16, 20]
        )
        if index_extended and others_closed:
            # Calcular dirección del pointing
            direction = index_tip - index_mcp
            direction = direction / np.linalg.norm(direction)
            
            return Gesture(
                type=GestureType.POINT,
                confidence=0.85,
                hand=hand,
                position=index_tip,
                direction=direction,
            )
        
        return None
    
    def detect_wave(self, history_window_seconds: float = 2.0) -> bool:
        """
        Detecta gesto de saludo (wave).
        
        Requiere movimiento oscilatorio de mano.
        """
        import time
        now = time.time()
        
        # Filtrar historial reciente
        recent = [
            g for t, g in self._gesture_history
            if now - t < history_window_seconds
        ]
        
        if len(recent) < 5:
            return False
        
        # Verificar oscilación horizontal
        positions = [g.position for g in recent if g.position is not None]
        if len(positions) < 5:
            return False
        
        positions = np.array(positions)
        x_values = positions[:, 0]
        
        # Detectar cambios de dirección
        direction_changes = 0
        for i in range(1, len(x_values) - 1):
            if (x_values[i] - x_values[i-1]) * (x_values[i+1] - x_values[i]) < 0:
                direction_changes += 1
        
        return direction_changes >= 3
    
    def get_pointing_target(self, camera_transform: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """
        Calcula el punto objetivo del gesto pointing.
        
        Args:
            camera_transform: Transformación de cámara a mundo
            
        Returns:
            Punto 3D objetivo o None
        """
        # Buscar gesto de pointing reciente
        pointing = None
        for g in reversed(self._last_gestures):
            if g.type == GestureType.POINT and g.direction is not None:
                pointing = g
                break
        
        if pointing is None:
            return None
        
        # Proyectar rayo (simplificado)
        origin = pointing.position
        direction = pointing.direction
        distance = 2.0  # Asumir 2m de distancia
        
        target = origin + direction * distance
        
        # Transformar a coordenadas mundo si hay transform
        if camera_transform is not None:
            target_h = np.append(target, 1)
            target = (camera_transform @ target_h)[:3]
        
        return target
    
    def get_last_gesture(self) -> Optional[Gesture]:
        """Retorna el último gesto detectado."""
        if self._last_gestures:
            return self._last_gestures[-1]
        return None
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "last_gestures": [g.to_dict() for g in self._last_gestures],
            "history_size": len(self._gesture_history),
        }
