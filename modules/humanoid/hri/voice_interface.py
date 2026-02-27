"""
Voice Interface: Reconocimiento y síntesis de voz.
====================================================
Interfaz de voz para interacción con humanos.
"""
from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional
from enum import Enum

_log = logging.getLogger("humanoid.hri.voice")


class VoiceState(str, Enum):
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING = "processing"
    SPEAKING = "speaking"


@dataclass
class VoiceConfig:
    """Configuración de interfaz de voz."""
    language: str = "es-ES"
    wake_word: str = "atlas"
    timeout_seconds: float = 10.0
    sample_rate: int = 16000
    use_local_stt: bool = True    # Speech-to-text local
    use_local_tts: bool = True    # Text-to-speech local
    voice_name: str = "default"
    volume: float = 0.8


class VoiceInterface:
    """
    Interfaz de voz para ATLAS.
    
    Soporta:
    - Reconocimiento de voz (STT)
    - Síntesis de voz (TTS)
    - Wake word detection
    - Múltiples idiomas
    """
    
    def __init__(self, config: Optional[VoiceConfig] = None):
        self.config = config or VoiceConfig()
        self._state = VoiceState.IDLE
        self._lock = threading.Lock()
        
        # Callbacks
        self._on_speech_callback: Optional[Callable[[str], None]] = None
        self._on_wake_word_callback: Optional[Callable[[], None]] = None
        
        # Buffer de audio
        self._audio_buffer: List[bytes] = []
        
        # Historial
        self._transcript_history: List[Dict[str, Any]] = []
        
        _log.info("Voice interface initialized for %s", self.config.language)
    
    def start_listening(self) -> bool:
        """Inicia la escucha de voz."""
        with self._lock:
            if self._state == VoiceState.SPEAKING:
                return False
            
            self._state = VoiceState.LISTENING
            self._audio_buffer = []
            _log.debug("Started listening")
            return True
    
    def stop_listening(self) -> Optional[str]:
        """Detiene la escucha y procesa el audio."""
        with self._lock:
            if self._state != VoiceState.LISTENING:
                return None
            
            self._state = VoiceState.PROCESSING
        
        # Procesar audio
        transcript = self._process_audio()
        
        with self._lock:
            self._state = VoiceState.IDLE
        
        if transcript and self._on_speech_callback:
            self._on_speech_callback(transcript)
        
        return transcript
    
    def _process_audio(self) -> Optional[str]:
        """Procesa el buffer de audio (STT)."""
        if not self._audio_buffer:
            return None
        
        # Placeholder: STT real requiere Whisper, Vosk, etc.
        # Simulación
        _log.debug("Processing %d audio chunks", len(self._audio_buffer))
        
        # En implementación real:
        # if self.config.use_local_stt:
        #     import whisper
        #     model = whisper.load_model("base")
        #     result = model.transcribe(audio_path)
        #     return result["text"]
        
        return None
    
    def speak(self, text: str, blocking: bool = True) -> bool:
        """
        Sintetiza y reproduce texto.
        
        Args:
            text: Texto a hablar
            blocking: Si esperar a que termine
            
        Returns:
            True si exitoso
        """
        with self._lock:
            if self._state == VoiceState.SPEAKING:
                _log.warning("Already speaking")
                return False
            
            self._state = VoiceState.SPEAKING
        
        _log.info("Speaking: %s", text[:50])
        
        # Placeholder: TTS real requiere pyttsx3, gtts, etc.
        try:
            self._synthesize_and_play(text)
            success = True
        except Exception as e:
            _log.exception("TTS error: %s", e)
            success = False
        finally:
            with self._lock:
                self._state = VoiceState.IDLE
        
        return success
    
    def _synthesize_and_play(self, text: str) -> None:
        """Sintetiza y reproduce (placeholder)."""
        # En implementación real:
        # import pyttsx3
        # engine = pyttsx3.init()
        # engine.setProperty('voice', self.config.voice_name)
        # engine.setProperty('volume', self.config.volume)
        # engine.say(text)
        # engine.runAndWait()
        
        import time
        # Simular duración de habla
        words = len(text.split())
        duration = words * 0.3  # 0.3s por palabra
        time.sleep(min(duration, 2.0))
    
    def recognize_wake_word(self, audio_chunk: bytes) -> bool:
        """
        Verifica si hay wake word en el audio.
        
        Args:
            audio_chunk: Fragmento de audio
            
        Returns:
            True si se detectó wake word
        """
        # Placeholder: Wake word real requiere Porcupine, Snowboy, etc.
        # En implementación real:
        # detector = pvporcupine.create(keywords=[self.config.wake_word])
        # result = detector.process(audio_chunk)
        # return result >= 0
        
        return False
    
    def add_audio(self, audio_chunk: bytes) -> None:
        """Añade audio al buffer."""
        if self._state == VoiceState.LISTENING:
            self._audio_buffer.append(audio_chunk)
    
    def set_speech_callback(self, callback: Callable[[str], None]) -> None:
        """Registra callback para cuando se reconoce habla."""
        self._on_speech_callback = callback
    
    def set_wake_word_callback(self, callback: Callable[[], None]) -> None:
        """Registra callback para wake word."""
        self._on_wake_word_callback = callback
    
    def get_supported_languages(self) -> List[str]:
        """Retorna idiomas soportados."""
        return ["es-ES", "en-US", "en-GB", "fr-FR", "de-DE", "it-IT", "pt-BR"]
    
    @property
    def state(self) -> VoiceState:
        return self._state
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "state": self._state.value,
            "language": self.config.language,
            "wake_word": self.config.wake_word,
            "history_size": len(self._transcript_history),
        }
