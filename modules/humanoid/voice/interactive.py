"""
ATLAS Interactive Voice System - Estilo Gemini/Alexa
=====================================================

Sistema de interacción por voz continua:
- Escucha activa con wake word "Atlas"
- Reconocimiento de voz en español
- Respuestas inteligentes via Brain/LLM
- Text-to-Speech para respuestas
- Estado visual en consola

Uso:
    python -m modules.humanoid.voice.interactive
    # o
    from modules.humanoid.voice.interactive import VoiceAssistant
    assistant = VoiceAssistant()
    assistant.run()
"""
from __future__ import annotations

import os
import sys
import time
import threading
import queue
import logging
from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass
from enum import Enum

# Configurar logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
_log = logging.getLogger("atlas.voice")

# ============================================================================
# ESTADO DEL ASISTENTE
# ============================================================================

class AssistantState(Enum):
    IDLE = "idle"           # Esperando wake word
    LISTENING = "listening" # Escuchando comando
    THINKING = "thinking"   # Procesando con LLM
    SPEAKING = "speaking"   # Hablando respuesta
    ERROR = "error"         # Error temporal


@dataclass
class VoiceConfig:
    """Configuración del asistente de voz."""
    wake_words: tuple = ("atlas", "átlas", "atlas,")
    language: str = "es-ES"
    energy_threshold: int = 300
    pause_threshold: float = 0.8
    phrase_time_limit: int = 10
    listen_timeout: int = 5
    tts_rate: int = 150
    tts_volume: float = 1.0
    use_llm: bool = True
    debug: bool = False


# ============================================================================
# MOTOR TTS (Text-to-Speech)
# ============================================================================

class TTSEngine:
    """Motor de síntesis de voz."""
    
    def __init__(self, rate: int = 150, volume: float = 1.0):
        self._engine = None
        self._rate = rate
        self._volume = volume
        self._lock = threading.Lock()
        self._init_engine()
    
    def _init_engine(self):
        try:
            import pyttsx3
            self._engine = pyttsx3.init()
            self._engine.setProperty('rate', self._rate)
            self._engine.setProperty('volume', self._volume)
            # Intentar voz en español
            voices = self._engine.getProperty('voices')
            for v in voices:
                if 'spanish' in v.name.lower() or 'español' in v.name.lower():
                    self._engine.setProperty('voice', v.id)
                    break
        except Exception as e:
            _log.error(f"Error inicializando TTS: {e}")
            self._engine = None
    
    def speak(self, text: str, block: bool = True) -> bool:
        """Habla el texto dado."""
        if not self._engine or not text:
            return False
        
        with self._lock:
            try:
                self._engine.say(text)
                if block:
                    self._engine.runAndWait()
                return True
            except Exception as e:
                _log.error(f"Error en TTS: {e}")
                return False
    
    def stop(self):
        """Detiene el habla actual."""
        if self._engine:
            try:
                self._engine.stop()
            except Exception:
                pass


# ============================================================================
# MOTOR STT (Speech-to-Text)
# ============================================================================

class STTEngine:
    """Motor de reconocimiento de voz."""
    
    def __init__(self, config: VoiceConfig):
        self.config = config
        self._recognizer = None
        self._microphone = None
        self._init_engine()
    
    def _init_engine(self):
        try:
            import speech_recognition as sr
            self._recognizer = sr.Recognizer()
            self._recognizer.energy_threshold = self.config.energy_threshold
            self._recognizer.pause_threshold = self.config.pause_threshold
            self._recognizer.dynamic_energy_threshold = True
            self._microphone = sr.Microphone()
            
            # Calibrar con ruido ambiente
            _log.info("Calibrando micrófono...")
            with self._microphone as source:
                self._recognizer.adjust_for_ambient_noise(source, duration=1)
            _log.info(f"Micrófono calibrado. Umbral de energía: {self._recognizer.energy_threshold}")
        except Exception as e:
            _log.error(f"Error inicializando STT: {e}")
            self._recognizer = None
    
    def listen(self, timeout: Optional[int] = None) -> Optional[str]:
        """Escucha y retorna el texto reconocido."""
        if not self._recognizer or not self._microphone:
            return None
        
        import speech_recognition as sr
        
        try:
            with self._microphone as source:
                _log.debug("Escuchando...")
                audio = self._recognizer.listen(
                    source,
                    timeout=timeout or self.config.listen_timeout,
                    phrase_time_limit=self.config.phrase_time_limit
                )
            
            # Reconocer con Google (gratis y bueno para español)
            text = self._recognizer.recognize_google(audio, language=self.config.language)
            return text.strip() if text else None
            
        except sr.WaitTimeoutError:
            _log.debug("Timeout esperando audio")
            return None
        except sr.UnknownValueError:
            _log.debug("No se entendió el audio")
            return None
        except sr.RequestError as e:
            _log.error(f"Error en servicio de reconocimiento: {e}")
            return None
        except Exception as e:
            _log.error(f"Error en STT: {e}")
            return None
    
    def is_available(self) -> bool:
        return self._recognizer is not None and self._microphone is not None


# ============================================================================
# PROCESADOR DE COMANDOS (Brain/LLM)
# ============================================================================

class CommandProcessor:
    """Procesa comandos y genera respuestas."""
    
    def __init__(self, use_llm: bool = True):
        self.use_llm = use_llm
        self._brain_available = False
        self._check_brain()
    
    def _check_brain(self):
        """Verifica si el Brain está disponible."""
        try:
            import urllib.request
            req = urllib.request.Request("http://127.0.0.1:8791/api/brain/status")
            with urllib.request.urlopen(req, timeout=2) as r:
                self._brain_available = r.status == 200
        except Exception:
            self._brain_available = False
    
    def process(self, text: str) -> str:
        """Procesa el texto y retorna una respuesta."""
        if not text:
            return "No entendí lo que dijiste."
        
        text_lower = text.lower().strip()
        
        # Comandos especiales (respuestas rápidas)
        if any(w in text_lower for w in ["hola", "buenos días", "buenas tardes", "buenas noches"]):
            return self._greeting()
        
        if any(w in text_lower for w in ["hora", "qué hora es", "dime la hora"]):
            return self._get_time()
        
        if any(w in text_lower for w in ["fecha", "qué día es", "qué fecha"]):
            return self._get_date()
        
        if any(w in text_lower for w in ["cómo estás", "cómo te encuentras", "qué tal"]):
            return "Estoy funcionando perfectamente. Todos mis sistemas están operativos. ¿En qué puedo ayudarte?"
        
        if any(w in text_lower for w in ["quién eres", "qué eres", "cuál es tu nombre"]):
            return "Soy Atlas, tu asistente de inteligencia artificial. Puedo ayudarte con tareas, responder preguntas y controlar sistemas."
        
        if any(w in text_lower for w in ["gracias", "muchas gracias"]):
            return "De nada. Estoy aquí para ayudarte."
        
        if any(w in text_lower for w in ["adiós", "hasta luego", "chao", "nos vemos"]):
            return "Hasta luego. Estaré aquí cuando me necesites."
        
        if any(w in text_lower for w in ["estado", "status", "diagnóstico", "cómo está el sistema"]):
            return self._get_system_status()
        
        # Si el Brain está disponible, usar LLM
        if self.use_llm and self._brain_available:
            return self._ask_brain(text)
        
        # Respuesta genérica
        return f"Escuché: {text}. ¿Puedes ser más específico con lo que necesitas?"
    
    def _greeting(self) -> str:
        from datetime import datetime
        hour = datetime.now().hour
        if hour < 12:
            saludo = "Buenos días"
        elif hour < 19:
            saludo = "Buenas tardes"
        else:
            saludo = "Buenas noches"
        return f"{saludo}. Soy Atlas, tu asistente. ¿En qué puedo ayudarte?"
    
    def _get_time(self) -> str:
        from datetime import datetime
        now = datetime.now()
        return f"Son las {now.hour}:{now.minute:02d}."
    
    def _get_date(self) -> str:
        from datetime import datetime
        meses = ["enero", "febrero", "marzo", "abril", "mayo", "junio",
                 "julio", "agosto", "septiembre", "octubre", "noviembre", "diciembre"]
        dias = ["lunes", "martes", "miércoles", "jueves", "viernes", "sábado", "domingo"]
        now = datetime.now()
        dia_semana = dias[now.weekday()]
        mes = meses[now.month - 1]
        return f"Hoy es {dia_semana}, {now.day} de {mes} de {now.year}."
    
    def _get_system_status(self) -> str:
        try:
            import urllib.request
            import json
            req = urllib.request.Request("http://127.0.0.1:8791/ans/status")
            with urllib.request.urlopen(req, timeout=3) as r:
                data = json.loads(r.read().decode())
                if data.get("ok"):
                    return "El sistema está funcionando correctamente. Todos los módulos están operativos."
        except Exception:
            pass
        return "El sistema está activo. Algunos servicios podrían estar iniciándose."
    
    def _ask_brain(self, text: str) -> str:
        """Consulta al Brain/LLM para respuestas complejas."""
        try:
            import urllib.request
            import json
            
            payload = json.dumps({
                "message": text,
                "context": "voice_interaction",
                "max_tokens": 150
            }).encode()
            
            req = urllib.request.Request(
                "http://127.0.0.1:8791/api/brain/chat",
                data=payload,
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            
            with urllib.request.urlopen(req, timeout=10) as r:
                data = json.loads(r.read().decode())
                if data.get("ok") and data.get("response"):
                    return data["response"][:300]  # Limitar longitud para TTS
        except Exception as e:
            _log.debug(f"Brain no disponible: {e}")
        
        return f"Entendí tu pregunta sobre: {text[:50]}. Déjame procesar eso."


# ============================================================================
# ASISTENTE DE VOZ PRINCIPAL
# ============================================================================

class VoiceAssistant:
    """Asistente de voz interactivo estilo Gemini."""
    
    def __init__(self, config: Optional[VoiceConfig] = None):
        self.config = config or VoiceConfig()
        self.state = AssistantState.IDLE
        self._running = False
        self._tts = TTSEngine(rate=self.config.tts_rate, volume=self.config.tts_volume)
        self._stt = STTEngine(self.config)
        self._processor = CommandProcessor(use_llm=self.config.use_llm)
        self._state_callback: Optional[Callable[[AssistantState, str], None]] = None
    
    def set_state_callback(self, callback: Callable[[AssistantState, str], None]):
        """Establece callback para cambios de estado (útil para UI)."""
        self._state_callback = callback
    
    def _set_state(self, state: AssistantState, info: str = ""):
        self.state = state
        if self._state_callback:
            try:
                self._state_callback(state, info)
            except Exception:
                pass
        
        # Indicador visual en consola (sin emojis para compatibilidad Windows)
        icons = {
            AssistantState.IDLE: "[...]",
            AssistantState.LISTENING: "[OIR]",
            AssistantState.THINKING: "[CPU]",
            AssistantState.SPEAKING: "[VOZ]",
            AssistantState.ERROR: "[ERR]",
        }
        icon = icons.get(state, "")
        try:
            print(f"\r{icon} [{state.value.upper()}] {info[:60]:<60}", end="", flush=True)
        except Exception:
            pass
    
    def _is_wake_word(self, text: str) -> bool:
        """Detecta si el texto contiene la wake word."""
        if not text:
            return False
        text_lower = text.lower().strip()
        for wake in self.config.wake_words:
            if wake in text_lower or text_lower.startswith(wake):
                return True
        return False
    
    def _extract_command(self, text: str) -> str:
        """Extrae el comando después de la wake word."""
        if not text:
            return ""
        text_lower = text.lower()
        for wake in self.config.wake_words:
            if wake in text_lower:
                # Obtener todo después de la wake word
                idx = text_lower.find(wake)
                command = text[idx + len(wake):].strip()
                # Quitar comas o puntos iniciales
                command = command.lstrip(",. ")
                if command:
                    return command
        return text  # Retornar todo si no hay wake word clara
    
    def run(self):
        """Ejecuta el bucle principal del asistente."""
        if not self._stt.is_available():
            print("\n❌ Error: Micrófono no disponible")
            print("   Asegúrate de tener un micrófono conectado y permisos activados")
            return
        
        self._running = True
        print("\n" + "=" * 60)
        print("ATLAS Voice Assistant - Iniciado")
        print("=" * 60)
        print(f"   Wake word: 'Atlas'")
        print(f"   Idioma: {self.config.language}")
        print(f"   Para salir: Ctrl+C o di 'Atlas, apagate'")
        print("=" * 60 + "\n")
        
        # Saludo inicial
        self._tts.speak("Hola, soy Atlas. Estoy listo para ayudarte.")
        
        try:
            while self._running:
                self._loop_iteration()
        except KeyboardInterrupt:
            print("\n\nAsistente detenido por usuario")
        finally:
            self._running = False
            self._tts.speak("Hasta luego.")
    
    def _loop_iteration(self):
        """Una iteración del bucle principal."""
        # Estado: Esperando wake word
        self._set_state(AssistantState.IDLE, "Di 'Atlas' para activarme...")
        
        # Escuchar
        text = self._stt.listen(timeout=None)  # Escucha continua
        
        if not text:
            return
        
        _log.info(f"Detectado: {text}")
        
        # Verificar wake word
        if not self._is_wake_word(text):
            return
        
        # Extraer comando
        command = self._extract_command(text)
        
        # Si solo dijo "Atlas", pedir más información
        if not command or len(command) < 2:
            self._set_state(AssistantState.LISTENING, "Te escucho...")
            self._tts.speak("¿Sí? Te escucho.")
            
            # Escuchar comando
            command = self._stt.listen(timeout=8)
            
            if not command:
                self._tts.speak("No escuché nada. Intenta de nuevo.")
                return
        
        # Comando especial: apagar
        if any(w in command.lower() for w in ["apágate", "apagar", "detente", "para"]):
            self._running = False
            return
        
        # Procesar comando
        self._set_state(AssistantState.THINKING, f"Procesando: {command[:40]}...")
        response = self._processor.process(command)
        
        # Responder
        self._set_state(AssistantState.SPEAKING, response[:40])
        self._tts.speak(response)
    
    def stop(self):
        """Detiene el asistente."""
        self._running = False
        self._tts.stop()


# ============================================================================
# INTERFAZ DE LÍNEA DE COMANDOS
# ============================================================================

def main():
    """Punto de entrada principal."""
    import argparse
    
    parser = argparse.ArgumentParser(description="ATLAS Voice Assistant")
    parser.add_argument("--debug", action="store_true", help="Modo debug")
    parser.add_argument("--no-llm", action="store_true", help="Desactivar LLM/Brain")
    parser.add_argument("--rate", type=int, default=150, help="Velocidad TTS (default: 150)")
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    config = VoiceConfig(
        debug=args.debug,
        use_llm=not args.no_llm,
        tts_rate=args.rate
    )
    
    assistant = VoiceAssistant(config)
    assistant.run()


if __name__ == "__main__":
    main()
