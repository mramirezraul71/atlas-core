"""
HRI System: Sistema integrado de interacción humano-robot.
==========================================================
Integra todos los componentes de HRI.
"""
from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

from .voice_interface import VoiceInterface, VoiceConfig
from .gesture_recognition import GestureRecognizer, Gesture
from .emotion_recognition import EmotionRecognizer, Emotion
from .intent_parser import IntentParser, Intent
from .dialog_manager import DialogManager, DialogState
from .safety_monitor import SafetyMonitor, SafetyLevel

_log = logging.getLogger("humanoid.hri.system")


@dataclass
class HRIConfig:
    """Configuración del sistema HRI."""
    voice_config: VoiceConfig = None
    enable_voice: bool = True
    enable_gestures: bool = True
    enable_emotions: bool = True
    enable_safety: bool = True
    
    def __post_init__(self):
        if self.voice_config is None:
            self.voice_config = VoiceConfig()


class HRISystem:
    """
    Sistema integrado de interacción humano-robot.
    
    Coordina:
    - Interfaz de voz
    - Reconocimiento de gestos
    - Reconocimiento de emociones
    - Parser de intenciones
    - Gestor de diálogo
    - Monitor de seguridad
    """
    
    def __init__(self, config: Optional[HRIConfig] = None):
        self.config = config or HRIConfig()
        
        # Componentes
        self._voice = VoiceInterface(self.config.voice_config) if self.config.enable_voice else None
        self._gestures = GestureRecognizer() if self.config.enable_gestures else None
        self._emotions = EmotionRecognizer() if self.config.enable_emotions else None
        self._intent_parser = IntentParser()
        self._dialog = DialogManager()
        self._safety = SafetyMonitor() if self.config.enable_safety else None
        
        # Estado
        self._running = False
        self._lock = threading.Lock()
        
        # Callbacks
        self._action_callback: Optional[Callable[[Dict[str, Any]], None]] = None
        
        # Conectar callbacks internos
        if self._voice:
            self._voice.set_speech_callback(self._on_speech_recognized)
        
        _log.info("HRI system initialized")
    
    def start(self) -> None:
        """Inicia el sistema HRI."""
        self._running = True
        _log.info("HRI system started")
    
    def stop(self) -> None:
        """Detiene el sistema HRI."""
        self._running = False
        _log.info("HRI system stopped")
    
    def process_text_input(self, text: str) -> str:
        """
        Procesa entrada de texto.
        
        Args:
            text: Texto del usuario
            
        Returns:
            Respuesta del robot
        """
        # Parsear intención
        intent = self._intent_parser.parse(text)
        
        # Procesar con gestor de diálogo
        response = self._dialog.process_intent(intent)
        
        # Verificar si hay acción a ejecutar
        context = self._dialog.get_context()
        if context.state == DialogState.EXECUTING and context.pending_action:
            self._execute_action(context.pending_action)
        
        return response
    
    def _on_speech_recognized(self, text: str) -> None:
        """Callback cuando se reconoce habla."""
        _log.info("Speech recognized: %s", text)
        
        response = self.process_text_input(text)
        
        # Hablar respuesta
        if self._voice and response:
            self._voice.speak(response)
    
    def process_visual_input(
        self,
        rgb_frame,
        depth_frame = None,
        persons: Optional[List[Dict[str, Any]]] = None,
    ) -> Dict[str, Any]:
        """
        Procesa entrada visual.
        
        Args:
            rgb_frame: Frame RGB
            depth_frame: Frame de profundidad opcional
            persons: Personas detectadas con posiciones
            
        Returns:
            Resultados del procesamiento
        """
        results = {}
        
        # Actualizar seguridad
        if self._safety and persons:
            safety_level = self._safety.update_persons(persons)
            results["safety_level"] = safety_level.value
            results["safe_to_move"], results["safety_reason"] = self._safety.is_safe_to_move()
        
        # Reconocer gestos
        if self._gestures and rgb_frame is not None:
            gestures = self._gestures.process_frame(rgb_frame)
            results["gestures"] = [g.to_dict() for g in gestures]
            
            # Procesar gestos como comandos
            for gesture in gestures:
                self._process_gesture(gesture)
        
        # Reconocer emociones
        if self._emotions and rgb_frame is not None:
            # Asumiendo que tenemos rostros detectados
            # emotion = self._emotions.recognize_from_face(face_image)
            # results["emotion"] = emotion.to_dict() if emotion else None
            pass
        
        return results
    
    def _process_gesture(self, gesture: Gesture) -> None:
        """Procesa un gesto como comando."""
        from .gesture_recognition import GestureType
        
        if gesture.type == GestureType.STOP:
            _log.info("Stop gesture detected")
            # Simular comando de parada
            self.process_text_input("para")
        
        elif gesture.type == GestureType.COME_HERE:
            _log.info("Come here gesture detected")
            self.process_text_input("ven aquí")
        
        elif gesture.type == GestureType.WAVE:
            _log.info("Wave gesture detected")
            self.process_text_input("hola")
    
    def _execute_action(self, action: Dict[str, Any]) -> None:
        """Ejecuta una acción."""
        _log.info("Executing action: %s", action)
        
        if self._action_callback:
            self._action_callback(action)
        
        # Limpiar acción del diálogo
        self._dialog.clear_action()
    
    def set_action_callback(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """Registra callback para acciones."""
        self._action_callback = callback
    
    def speak(self, text: str) -> bool:
        """Hace que el robot hable."""
        if self._voice:
            return self._voice.speak(text)
        return False
    
    def get_safety_status(self) -> Dict[str, Any]:
        """Retorna estado de seguridad."""
        if self._safety:
            return self._safety.to_dict()
        return {"enabled": False}
    
    def emergency_stop(self) -> None:
        """Activa parada de emergencia."""
        if self._safety:
            self._safety.emergency_stop()
    
    def reset_emergency(self) -> bool:
        """Resetea parada de emergencia."""
        if self._safety:
            return self._safety.reset_emergency()
        return True
    
    def get_dialog_context(self) -> Dict[str, Any]:
        """Retorna contexto de diálogo."""
        return self._dialog.to_dict()
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "running": self._running,
            "voice_enabled": self._voice is not None,
            "gestures_enabled": self._gestures is not None,
            "emotions_enabled": self._emotions is not None,
            "safety_enabled": self._safety is not None,
            "dialog": self._dialog.to_dict(),
            "safety": self._safety.to_dict() if self._safety else None,
        }
