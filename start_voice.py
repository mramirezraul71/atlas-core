#!/usr/bin/env python
"""
ATLAS Voice Assistant - Script de Inicio Rápido
================================================

Ejecuta: python start_voice.py

Comandos de voz:
- "Atlas" - Activa el asistente
- "Atlas, ¿qué hora es?"
- "Atlas, ¿cómo estás?"
- "Atlas, dime la fecha"
- "Atlas, estado del sistema"
- "Atlas, apágate" - Detiene el asistente
"""

import sys
import os

# Asegurar que el path está configurado
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def main():
    print("""
╔══════════════════════════════════════════════════════════════╗
║           🤖 ATLAS Voice Assistant                           ║
╠══════════════════════════════════════════════════════════════╣
║  Comandos:                                                   ║
║    • Di "Atlas" para activar                                 ║
║    • "Atlas, ¿qué hora es?"                                  ║
║    • "Atlas, ¿cómo estás?"                                   ║
║    • "Atlas, estado del sistema"                             ║
║    • "Atlas, apágate" - para salir                           ║
║                                                              ║
║  Presiona Ctrl+C para salir                                  ║
╚══════════════════════════════════════════════════════════════╝
    """)
    
    try:
        from modules.humanoid.voice.interactive import VoiceAssistant, VoiceConfig
        
        config = VoiceConfig(
            tts_rate=140,  # Velocidad de habla
            use_llm=True,  # Usar Brain para respuestas inteligentes
        )
        
        assistant = VoiceAssistant(config)
        assistant.run()
        
    except ImportError as e:
        print(f"\n❌ Error importando módulos: {e}")
        print("\nInstalando dependencias...")
        os.system("pip install SpeechRecognition PyAudio pyttsx3")
        print("\nIntenta ejecutar de nuevo: python start_voice.py")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
