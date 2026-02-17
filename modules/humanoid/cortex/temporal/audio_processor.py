"""
AudioProcessor: Procesamiento de audio del lóbulo temporal.

Análogo biológico: Corteza auditiva primaria y secundaria
- ASR (Automatic Speech Recognition)
- VAD (Voice Activity Detection)
- Clasificación de sonidos
"""
from __future__ import annotations

import asyncio
import logging
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, AsyncIterator, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)


@dataclass
class AudioEvent:
    """Evento de audio procesado."""
    event_type: str  # "speech", "sound", "silence", "wake_word"
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    data: Any = None
    confidence: float = 1.0
    duration_ms: float = 0.0
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "event_type": self.event_type,
            "timestamp_ns": self.timestamp_ns,
            "data": self.data,
            "confidence": self.confidence,
            "duration_ms": self.duration_ms,
        }


@dataclass
class Transcript:
    """Transcripción de habla."""
    text: str
    language: str = "es"
    confidence: float = 1.0
    is_final: bool = True
    start_time_ns: int = 0
    end_time_ns: int = 0
    words: List[Dict[str, Any]] = field(default_factory=list)  # [{word, start, end, confidence}]
    
    def duration_ms(self) -> float:
        return (self.end_time_ns - self.start_time_ns) / 1e6


@dataclass
class SoundClassification:
    """Clasificación de sonido no vocal."""
    sound_class: str  # "alarm", "knock", "glass_break", "dog_bark", etc.
    confidence: float
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())


class VAD:
    """
    Voice Activity Detection simple.
    
    Detecta si hay voz en un chunk de audio basándose en energía.
    """
    
    def __init__(self, energy_threshold: float = 0.01, 
                 speech_pad_ms: int = 300):
        """
        Inicializa VAD.
        
        Args:
            energy_threshold: Umbral de energía para detectar actividad
            speech_pad_ms: Padding después de detectar fin de habla
        """
        self.energy_threshold = energy_threshold
        self.speech_pad_ms = speech_pad_ms
        
        self._is_speaking = False
        self._silence_start_ns = 0
        self._energy_history = deque(maxlen=10)
    
    def is_speech(self, audio_chunk: bytes, sample_rate: int = 16000) -> bool:
        """
        Detecta si hay habla en el chunk de audio.
        
        Args:
            audio_chunk: Bytes de audio PCM (int16)
            sample_rate: Tasa de muestreo
        
        Returns:
            True si se detecta habla
        """
        # Calcular energía RMS
        try:
            import numpy as np
            audio_array = np.frombuffer(audio_chunk, dtype=np.int16)
            energy = np.sqrt(np.mean(audio_array.astype(float) ** 2)) / 32768.0
        except ImportError:
            # Fallback sin numpy
            samples = [int.from_bytes(audio_chunk[i:i+2], 'little', signed=True) 
                      for i in range(0, len(audio_chunk), 2)]
            if not samples:
                return False
            energy = (sum(s**2 for s in samples) / len(samples)) ** 0.5 / 32768.0
        
        self._energy_history.append(energy)
        avg_energy = sum(self._energy_history) / len(self._energy_history)
        
        # Detectar transiciones
        if avg_energy > self.energy_threshold:
            self._is_speaking = True
            self._silence_start_ns = 0
        elif self._is_speaking:
            # Posible fin de habla
            if self._silence_start_ns == 0:
                self._silence_start_ns = time.time_ns()
            elif (time.time_ns() - self._silence_start_ns) / 1e6 > self.speech_pad_ms:
                # Silencio suficiente, fin de habla
                self._is_speaking = False
        
        return self._is_speaking
    
    def reset(self) -> None:
        """Resetea el estado del VAD."""
        self._is_speaking = False
        self._silence_start_ns = 0
        self._energy_history.clear()


class AudioProcessor:
    """
    Procesador de audio del lóbulo temporal.
    
    Integra:
    - VAD para detectar actividad de voz
    - ASR para transcribir habla
    - Clasificación de sonidos ambientales
    - Detección de wake words
    """
    
    def __init__(self, 
                 wake_word: str = "atlas",
                 language: str = "es",
                 asr_model: str = "whisper-base"):
        """
        Inicializa el procesador de audio.
        
        Args:
            wake_word: Palabra de activación
            language: Idioma principal
            asr_model: Modelo de ASR a usar
        """
        self.wake_word = wake_word.lower()
        self.language = language
        self.asr_model = asr_model
        
        # VAD
        self.vad = VAD()
        
        # ASR (lazy loading)
        self._asr = None
        
        # Buffer de audio
        self._audio_buffer = deque(maxlen=32000 * 30)  # ~30 segundos a 16kHz
        
        # Callbacks
        self._callbacks: Dict[str, List[Callable]] = {
            "speech": [],
            "sound": [],
            "wake_word": [],
        }
        
        # Estado
        self._is_listening = False
        self._wake_word_detected = False
        self._current_utterance_start = 0
    
    def _load_asr(self) -> None:
        """Carga modelo ASR de forma lazy con CUDA si disponible."""
        if self._asr is not None:
            return
        
        try:
            import torch
            device = "cuda" if torch.cuda.is_available() else "cpu"
            
            # Intentar cargar Whisper
            import whisper
            model_name = self.asr_model.replace("whisper-", "")
            self._asr = whisper.load_model(model_name, device=device)
            logger.info(f"Loaded Whisper model '{model_name}' on {device}")
        except ImportError:
            logger.warning("Whisper not available, using mock ASR")
            self._asr = "mock"
        except Exception as e:
            logger.error(f"Error loading ASR: {e}")
            self._asr = "mock"
    
    async def process_chunk(self, audio_chunk: bytes, 
                           sample_rate: int = 16000) -> List[AudioEvent]:
        """
        Procesa un chunk de audio.
        
        Args:
            audio_chunk: Bytes de audio PCM (int16)
            sample_rate: Tasa de muestreo
        
        Returns:
            Lista de eventos detectados
        """
        events = []
        
        # Agregar al buffer
        self._audio_buffer.extend(audio_chunk)
        
        # VAD
        is_speech = self.vad.is_speech(audio_chunk, sample_rate)
        
        if is_speech and not self._is_listening:
            # Inicio de habla
            self._is_listening = True
            self._current_utterance_start = time.time_ns()
            
        elif not is_speech and self._is_listening:
            # Fin de habla - transcribir
            self._is_listening = False
            duration_ms = (time.time_ns() - self._current_utterance_start) / 1e6
            
            if duration_ms > 300:  # Mínimo 300ms para transcribir
                # Obtener audio del buffer
                audio_data = bytes(self._audio_buffer)
                
                # Transcribir
                transcript = await self._transcribe(audio_data, sample_rate)
                
                if transcript and transcript.text:
                    # Verificar wake word
                    if self.wake_word in transcript.text.lower():
                        self._wake_word_detected = True
                        events.append(AudioEvent(
                            event_type="wake_word",
                            data={"word": self.wake_word, "transcript": transcript.text},
                            confidence=transcript.confidence,
                            duration_ms=duration_ms,
                        ))
                        
                        # Notificar callbacks
                        await self._notify_callbacks("wake_word", transcript)
                    
                    # Evento de habla
                    events.append(AudioEvent(
                        event_type="speech",
                        data=transcript,
                        confidence=transcript.confidence,
                        duration_ms=duration_ms,
                    ))
                    
                    await self._notify_callbacks("speech", transcript)
                
                # Limpiar buffer
                self._audio_buffer.clear()
        
        return events
    
    async def _transcribe(self, audio_data: bytes, 
                         sample_rate: int) -> Optional[Transcript]:
        """Transcribe audio usando ASR."""
        self._load_asr()
        
        if self._asr == "mock":
            # Mock ASR para testing
            return Transcript(
                text="(mock transcription)",
                language=self.language,
                confidence=0.5,
            )
        
        try:
            import numpy as np
            import tempfile
            import os
            
            # Convertir bytes a array
            audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
            
            # Whisper espera 16kHz
            if sample_rate != 16000:
                # Resample simple (debería usar librosa para mejor calidad)
                factor = 16000 / sample_rate
                new_len = int(len(audio_array) * factor)
                indices = np.linspace(0, len(audio_array) - 1, new_len).astype(int)
                audio_array = audio_array[indices]
            
            # Transcribir
            result = await asyncio.to_thread(
                self._asr.transcribe,
                audio_array,
                language=self.language[:2],  # "es", "en", etc.
                fp16=False,
            )
            
            return Transcript(
                text=result["text"].strip(),
                language=result.get("language", self.language),
                confidence=1.0 - (result.get("no_speech_prob", 0) or 0),
                is_final=True,
            )
            
        except Exception as e:
            logger.error(f"Transcription error: {e}")
            return None
    
    async def process_stream(self, audio_stream: AsyncIterator[bytes],
                            sample_rate: int = 16000) -> AsyncIterator[AudioEvent]:
        """
        Procesa stream de audio continuo.
        
        Args:
            audio_stream: Iterador asíncrono de chunks de audio
            sample_rate: Tasa de muestreo
        
        Yields:
            Eventos de audio detectados
        """
        async for chunk in audio_stream:
            events = await self.process_chunk(chunk, sample_rate)
            for event in events:
                yield event
    
    def register_callback(self, event_type: str, 
                         callback: Callable[[Any], None]) -> None:
        """
        Registra callback para tipo de evento.
        
        Args:
            event_type: "speech", "sound", "wake_word"
            callback: Función a llamar
        """
        if event_type in self._callbacks:
            self._callbacks[event_type].append(callback)
    
    async def _notify_callbacks(self, event_type: str, data: Any) -> None:
        """Notifica callbacks de un evento."""
        for callback in self._callbacks.get(event_type, []):
            try:
                if asyncio.iscoroutinefunction(callback):
                    await callback(data)
                else:
                    callback(data)
            except Exception as e:
                logger.error(f"Error in audio callback: {e}")
    
    def is_wake_word_active(self) -> bool:
        """Verifica si el wake word fue detectado recientemente."""
        return self._wake_word_detected
    
    def clear_wake_word(self) -> None:
        """Limpia el estado del wake word."""
        self._wake_word_detected = False
    
    def reset(self) -> None:
        """Resetea todo el procesador."""
        self.vad.reset()
        self._audio_buffer.clear()
        self._is_listening = False
        self._wake_word_detected = False
