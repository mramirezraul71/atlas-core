"""
ATLAS Voice - Asistente de Voz Interactivo
===========================================
Usa PowerShell para TTS (funciona en Windows)
"""
import sys
import os
import subprocess

# Usar Python 3.11
PYTHON311 = r"C:\Users\r6957\AppData\Local\Programs\Python\Python311\python.exe"
if os.path.exists(PYTHON311) and "Python311" not in sys.executable:
    os.execv(PYTHON311, [PYTHON311] + sys.argv)

import speech_recognition as sr
from datetime import datetime

def hablar(texto):
    """Habla usando PowerShell System.Speech."""
    print(f"ATLAS: {texto}")
    # Escapar comillas
    texto_escaped = texto.replace('"', '`"').replace("'", "`'")
    cmd = f'''powershell -Command "Add-Type -AssemblyName System.Speech; $s = New-Object System.Speech.Synthesis.SpeechSynthesizer; $s.Volume = 100; $s.Rate = 1; $s.Speak('{texto_escaped}')"'''
    subprocess.run(cmd, shell=True, capture_output=True)

class AtlasVoice:
    def __init__(self):
        # STT
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 200
        self.recognizer.pause_threshold = 0.7
        self.mic = sr.Microphone()
        
        # Calibrar
        print("Calibrando microfono...")
        with self.mic as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
        print(f"Umbral: {self.recognizer.energy_threshold}")
    
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
            return "No te escuche."
        
        if "hora" in texto:
            now = datetime.now()
            return f"Son las {now.hour} con {now.minute} minutos"
        
        if "fecha" in texto or "dia" in texto:
            meses = ["enero", "febrero", "marzo", "abril", "mayo", "junio",
                     "julio", "agosto", "septiembre", "octubre", "noviembre", "diciembre"]
            now = datetime.now()
            return f"Hoy es {now.day} de {meses[now.month-1]} de {now.year}"
        
        if "como estas" in texto:
            return "Estoy bien, gracias."
        
        if "quien eres" in texto or "que eres" in texto:
            return "Soy Atlas, tu asistente."
        
        if "gracias" in texto:
            return "De nada."
        
        if "hola" in texto or "buenos" in texto:
            return "Hola, en que te ayudo?"
        
        if "adios" in texto or "chao" in texto or "apaga" in texto or "salir" in texto:
            return "__EXIT__"
        
        return f"Escuche: {texto}"
    
    def run(self):
        """Bucle principal."""
        print("\n" + "="*50)
        print("  ATLAS VOICE")
        print("="*50)
        print("  Di 'Atlas' seguido de tu comando")
        print("  Para salir: 'Atlas, adios'")
        print("="*50 + "\n")
        
        hablar("Hola, soy Atlas")
        
        while True:
            print("\n[Esperando...]")
            texto = self.escuchar(timeout=None)
            
            if not texto:
                continue
            
            if "atlas" not in texto:
                continue
            
            partes = texto.split("atlas", 1)
            comando = partes[1].strip(" ,") if len(partes) > 1 else ""
            
            if not comando or len(comando) < 2:
                hablar("Dime")
                comando = self.escuchar(timeout=8)
                if not comando:
                    continue
            
            respuesta = self.procesar(comando)
            
            if respuesta == "__EXIT__":
                hablar("Adios")
                break
            
            hablar(respuesta)


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
        input("Enter para cerrar...")
