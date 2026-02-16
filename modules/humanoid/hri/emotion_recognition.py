"""
Emotion Recognition: Detección de emociones.
=============================================
Reconoce emociones de rostros y voz.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional
import numpy as np

_log = logging.getLogger("humanoid.hri.emotion")


class EmotionType(str, Enum):
    """Emociones básicas (Ekman)."""
    NEUTRAL = "neutral"
    HAPPY = "happy"
    SAD = "sad"
    ANGRY = "angry"
    FEAR = "fear"
    SURPRISE = "surprise"
    DISGUST = "disgust"


@dataclass
class Emotion:
    """Emoción detectada."""
    type: EmotionType
    confidence: float
    source: str = "face"  # face, voice, combined
    valence: float = 0.0  # -1 (negative) to 1 (positive)
    arousal: float = 0.0  # 0 (calm) to 1 (excited)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type.value,
            "confidence": self.confidence,
            "source": self.source,
            "valence": self.valence,
            "arousal": self.arousal,
        }


class EmotionRecognizer:
    """
    Reconocedor de emociones para ATLAS.
    
    Fuentes:
    - Expresión facial (face landmarks, CNN)
    - Voz (prosodia, tono)
    - Lenguaje corporal
    """
    
    # Mapeo emoción a valence/arousal
    EMOTION_VA_MAP = {
        EmotionType.NEUTRAL: (0.0, 0.0),
        EmotionType.HAPPY: (0.8, 0.5),
        EmotionType.SAD: (-0.6, -0.3),
        EmotionType.ANGRY: (-0.7, 0.8),
        EmotionType.FEAR: (-0.8, 0.6),
        EmotionType.SURPRISE: (0.2, 0.8),
        EmotionType.DISGUST: (-0.5, 0.3),
    }
    
    def __init__(self):
        self._last_emotion: Optional[Emotion] = None
        self._emotion_history: List[Emotion] = []
        self._history_max = 100
        
        _log.info("Emotion recognizer initialized")
    
    def recognize_from_face(
        self,
        face_image: np.ndarray,
        face_landmarks: Optional[np.ndarray] = None,
    ) -> Optional[Emotion]:
        """
        Reconoce emoción desde imagen de rostro.
        
        Args:
            face_image: Imagen del rostro (H x W x 3)
            face_landmarks: Landmarks faciales opcionales
            
        Returns:
            Emoción detectada
        """
        # Placeholder: Reconocimiento real con DeepFace, FER, etc.
        # from deepface import DeepFace
        # result = DeepFace.analyze(face_image, actions=['emotion'])
        # dominant = result[0]['dominant_emotion']
        
        # Simulación basada en landmarks
        if face_landmarks is not None:
            return self._analyze_landmarks(face_landmarks)
        
        # Default
        return Emotion(
            type=EmotionType.NEUTRAL,
            confidence=0.5,
            source="face",
            valence=0.0,
            arousal=0.0,
        )
    
    def _analyze_landmarks(self, landmarks: np.ndarray) -> Emotion:
        """Analiza landmarks faciales para emoción."""
        # Simplificado: analizar geometría facial
        # En implementación real: usar Action Units (AU)
        
        # Placeholder: asignar neutral
        emotion = EmotionType.NEUTRAL
        confidence = 0.6
        
        valence, arousal = self.EMOTION_VA_MAP[emotion]
        
        return Emotion(
            type=emotion,
            confidence=confidence,
            source="face",
            valence=valence,
            arousal=arousal,
        )
    
    def recognize_from_voice(
        self,
        audio_features: Dict[str, float],
    ) -> Optional[Emotion]:
        """
        Reconoce emoción desde características de voz.
        
        Args:
            audio_features: Features como pitch, energy, etc.
            
        Returns:
            Emoción detectada
        """
        # Features esperados: pitch_mean, pitch_var, energy, speech_rate
        pitch = audio_features.get("pitch_mean", 0)
        energy = audio_features.get("energy", 0)
        rate = audio_features.get("speech_rate", 1.0)
        
        # Heurísticas simples
        if energy > 0.7 and rate > 1.3:
            emotion = EmotionType.ANGRY
            confidence = 0.6
        elif pitch > 0.6 and energy > 0.5:
            emotion = EmotionType.HAPPY
            confidence = 0.5
        elif pitch < 0.3 and energy < 0.3:
            emotion = EmotionType.SAD
            confidence = 0.5
        else:
            emotion = EmotionType.NEUTRAL
            confidence = 0.4
        
        valence, arousal = self.EMOTION_VA_MAP[emotion]
        
        return Emotion(
            type=emotion,
            confidence=confidence,
            source="voice",
            valence=valence,
            arousal=arousal,
        )
    
    def combine_emotions(
        self,
        face_emotion: Optional[Emotion],
        voice_emotion: Optional[Emotion],
    ) -> Emotion:
        """
        Combina emociones de múltiples fuentes.
        
        Args:
            face_emotion: Emoción de rostro
            voice_emotion: Emoción de voz
            
        Returns:
            Emoción combinada
        """
        if face_emotion is None and voice_emotion is None:
            return Emotion(type=EmotionType.NEUTRAL, confidence=0.0, source="none")
        
        if face_emotion is None:
            return voice_emotion
        
        if voice_emotion is None:
            return face_emotion
        
        # Combinar por peso (rostro generalmente más confiable)
        face_weight = 0.7
        voice_weight = 0.3
        
        # Promediar valence y arousal
        combined_valence = (
            face_weight * face_emotion.valence +
            voice_weight * voice_emotion.valence
        )
        combined_arousal = (
            face_weight * face_emotion.arousal +
            voice_weight * voice_emotion.arousal
        )
        
        # Determinar emoción por V/A
        best_emotion = EmotionType.NEUTRAL
        best_dist = float('inf')
        
        for emotion, (v, a) in self.EMOTION_VA_MAP.items():
            dist = (v - combined_valence)**2 + (a - combined_arousal)**2
            if dist < best_dist:
                best_dist = dist
                best_emotion = emotion
        
        # Combinar confianza
        combined_confidence = (
            face_weight * face_emotion.confidence +
            voice_weight * voice_emotion.confidence
        )
        
        result = Emotion(
            type=best_emotion,
            confidence=combined_confidence,
            source="combined",
            valence=combined_valence,
            arousal=combined_arousal,
        )
        
        self._last_emotion = result
        self._emotion_history.append(result)
        if len(self._emotion_history) > self._history_max:
            self._emotion_history.pop(0)
        
        return result
    
    def get_emotional_state(self) -> Dict[str, Any]:
        """
        Retorna el estado emocional promedio reciente.
        """
        if not self._emotion_history:
            return {"emotion": "neutral", "stability": 0.0}
        
        recent = self._emotion_history[-10:]
        
        # Emoción más frecuente
        emotion_counts: Dict[EmotionType, int] = {}
        for e in recent:
            emotion_counts[e.type] = emotion_counts.get(e.type, 0) + 1
        
        dominant = max(emotion_counts, key=emotion_counts.get)
        
        # Estabilidad (qué tan consistente)
        stability = emotion_counts[dominant] / len(recent)
        
        # Promedio V/A
        avg_valence = sum(e.valence for e in recent) / len(recent)
        avg_arousal = sum(e.arousal for e in recent) / len(recent)
        
        return {
            "emotion": dominant.value,
            "stability": stability,
            "valence": avg_valence,
            "arousal": avg_arousal,
            "samples": len(recent),
        }
    
    def get_last_emotion(self) -> Optional[Emotion]:
        return self._last_emotion
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "last_emotion": self._last_emotion.to_dict() if self._last_emotion else None,
            "state": self.get_emotional_state(),
        }
