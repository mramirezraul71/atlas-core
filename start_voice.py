#!/usr/bin/env python
"""
ATLAS Voice Assistant - Script de Inicio Rapido
================================================

Ejecuta: python start_voice.py

Comandos de voz:
- "Atlas" - Activa el asistente
- "Atlas, que hora es?"
- "Atlas, como estas?"
- "Atlas, dime la fecha"
- "Atlas, estado del sistema"
- "Atlas, apagate" - Detiene el asistente
"""

import sys
import os
import subprocess

# Asegurar que el path está configurado
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Verificar Python correcto (necesitamos 3.11 donde están las dependencias)
PYTHON311 = r"C:\Users\r6957\AppData\Local\Programs\Python\Python311\python.exe"
if os.path.exists(PYTHON311) and "Python311" not in sys.executable:
    # Re-ejecutar con Python 3.11
    os.execv(PYTHON311, [PYTHON311] + sys.argv)

def main():
    # Configurar consola para UTF-8 en Windows
    import sys
    if sys.platform == 'win32':
        import os
        os.system('chcp 65001 > nul 2>&1')
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    
    print("""
================================================================
           ATLAS Voice Assistant                           
================================================================
  Comandos:                                                   
    - Di "Atlas" para activar                                 
    - "Atlas, que hora es?"                                  
    - "Atlas, como estas?"                                   
    - "Atlas, estado del sistema"                             
    - "Atlas, apagate" - para salir                           
                                                              
  Presiona Ctrl+C para salir                                  
================================================================
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
