"""
ATLAS Voice - Asistente de Voz Interactivo
===========================================
"""
import sys
import os

# Usar Python 3.11
PYTHON311 = r"C:\Users\r6957\AppData\Local\Programs\Python\Python311\python.exe"
if os.path.exists(PYTHON311) and "Python311" not in sys.executable:
    os.execv(PYTHON311, [PYTHON311] + sys.argv)

import speech_recognition as sr
import pyttsx3
from datetime import datetime

class AtlasVoice:
    def __init__(self):
        # TTS con pyttsx3
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)
        self.engine.setProperty('volume', 1.0)
        
        # Buscar voz en espanol
        voices = self.engine.getProperty('voices')
        for v in voices:
            if 'spanish' in v.name.lower() or 'sabina' in v.name.lower():
                self.engine.setProperty('voice', v.id)
                print(f"Usando voz: {v.name}")
                break
        
        # STT
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 200
        self.recognizer.pause_threshold = 0.7
        self.mic = sr.Microphone()
        
        # Calibrar
        print("Calibrando microfono...")
        with self.mic as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
        print(f"Umbral de energia: {self.recognizer.energy_threshold}")
    
    def hablar(self, texto):
        """ATLAS habla."""
        print(f"ATLAS: {texto}")
        self.engine.say(texto)
        self.engine.runAndWait()
    
    def escuchar(self, timeout=15):
        """Escucha y retorna texto."""
        try:
            with self.mic as source:
                print("  [Escuchando...]")
                audio = self.recognizer.listen(source, timeout=timeout, phrase_time_limit=10)
            
            print("  [Procesando...]")
            texto = self.recognizer.recognize_google(audio, language='es-ES')
            print(f"  Tu: {texto}")
            return texto.lower().strip()
        except sr.WaitTimeoutError:
            return None
        except sr.UnknownValueError:
            return None
        except Exception as e:
            print(f"  Error: {e}")
            return None
    
    def procesar(self, texto):
        """Procesa el comando."""
        if not texto:
            return "No te escuche bien."
        
        if "hora" in texto:
            now = datetime.now()
            return f"Son las {now.hour} con {now.minute} minutos"
        
        if "fecha" in texto or "dia" in texto:
            meses = ["enero", "febrero", "marzo", "abril", "mayo", "junio",
                     "julio", "agosto", "septiembre", "octubre", "noviembre", "diciembre"]
            now = datetime.now()
            return f"Hoy es {now.day} de {meses[now.month-1]} de {now.year}"
        
        if "como estas" in texto:
            return "Estoy funcionando perfectamente. Todos mis sistemas estan operativos."
        
        if "quien eres" in texto or "que eres" in texto:
            return "Soy Atlas, tu asistente de inteligencia artificial."
        
        if "gracias" in texto:
            return "De nada."
        
        if "hola" in texto or "buenos" in texto:
            hour = datetime.now().hour
            if hour < 12:
                return "Buenos dias. Soy Atlas. En que puedo ayudarte?"
            elif hour < 19:
                return "Buenas tardes. Soy Atlas. En que puedo ayudarte?"
            else:
                return "Buenas noches. Soy Atlas. En que puedo ayudarte?"
        
        if "adios" in texto or "chao" in texto:
            return "__EXIT__"
        
        if "apaga" in texto or "detente" in texto or "salir" in texto:
            return "__EXIT__"
        
        return f"Escuche: {texto}"
    
    def run(self):
        """Bucle principal."""
        print("\n" + "="*50)
        print("  ATLAS VOICE - Asistente Interactivo")
        print("="*50)
        print("  Di 'Atlas' seguido de tu comando")
        print("  Ejemplo: 'Atlas, que hora es?'")
        print("  Para salir: 'Atlas, apagate'")
        print("="*50 + "\n")
        
        self.hablar("Hola, soy Atlas. Estoy listo.")
        
        while True:
            print("\n[Esperando 'Atlas'...]")
            texto = self.escuchar(timeout=None)
            
            if not texto:
                continue
            
            if "atlas" not in texto:
                continue
            
            # Extraer comando
            partes = texto.split("atlas", 1)
            comando = partes[1].strip(" ,") if len(partes) > 1 else ""
            
            if not comando or len(comando) < 2:
                self.hablar("Dime")
                comando = self.escuchar(timeout=8)
                if not comando:
                    continue
            
            respuesta = self.procesar(comando)
            
            if respuesta == "__EXIT__":
                self.hablar("Hasta luego")
                break
            
            self.hablar(respuesta)


if __name__ == "__main__":
    try:
        atlas = AtlasVoice()
        atlas.run()
    except KeyboardInterrupt:
        print("\nDetenido.")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        input("Presiona Enter...")
