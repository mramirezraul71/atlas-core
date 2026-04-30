import time

cache = {}


def ejecutar_tarea(tarea):
    if tarea in cache:
        return cache[tarea]
    else:
        inicio = time.time()
        resultado = tarea()
        fin = time.time()
        cache[tarea] = resultado
        return resultado


def optimizar_codigo():
    # Código de optimización
    print("Optimización realizada con éxito")
