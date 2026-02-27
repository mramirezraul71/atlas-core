import pyttsx3
import os

engine = pyttsx3.init()
engine.setProperty('rate', 150)
engine.setProperty('volume', 0.9)

texto = 'Informe de conclusiones del sistema ATLAS. Estado general: Excelente. Puntuacion de salud: noventa y cinco de cien. Todos los componentes criticos estan operativos. Latencia cero milisegundos. Tasa de error cero por ciento. Conectividad completa con Nexus y Robot. Hay cinco incidentes abiertos pero ninguno es critico. El sistema esta estable y funcionando normalmente. Recomendaciones activadas: Monitoreo continuo de incidentes, alertas de rendimiento en tiempo real, validacion de conectividad cada cinco minutos, reporte automatico de anomalias, optimizacion de recursos del sistema, y auditoria de seguridad continua. Fin del informe.'

audio_path = r'C:\ATLAS_PUSH\informe_audio.mp3'
engine.save_to_file(texto, audio_path)
engine.runAndWait()

print('Audio generado exitosamente en: ' + audio_path)
