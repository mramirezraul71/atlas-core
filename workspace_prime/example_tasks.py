"""
ATLAS-WORKSPACE-PRIME — Tareas de ejemplo listas para ejecutar
Muestra la capacidad de lenguaje natural → ejecución autónoma
"""

import asyncio

from atlas_prime import AtlasPrime


async def run_examples():
    atlas = AtlasPrime(headless_browser=False, auto_confirm=False)

    examples = [
        # ── WEB ──────────────────────────────────────────────────────
        "Abre Google y busca el precio del Bitcoin hoy, dime el resultado",
        "Ve a reddit.com/r/technology y dime los 3 posts más populares de hoy",
        "Entra a weather.com, busca el clima de Miami hoy y dime temperatura y condiciones",
        "Abre YouTube, busca 'piano jazz' y dime los primeros 3 videos que aparecen",
        # ── GMAIL / PRODUCTIVIDAD ─────────────────────────────────────
        "Ve a gmail.com, inicia sesión y dime cuántos correos sin leer tengo",
        "Abre Google Calendar y dime qué eventos tengo esta semana",
        "Ve a Google Drive y lista los 5 archivos más recientes",
        # ── ATLAS INTERNO ─────────────────────────────────────────────
        "Abre la consola PowerShell y dime el estado de los 3 servicios de ATLAS",
        "Lee el archivo C:\\ATLAS_PUSH\\README.md y dame un resumen",
        "Verifica que los puertos 8000, 8791 y 8002 estén activos en ATLAS",
        # ── SISTEMA ───────────────────────────────────────────────────
        "Toma un screenshot del escritorio y dime qué aplicaciones están abiertas",
        "Abre el explorador de archivos en C:\\ATLAS_PUSH y dime qué carpetas hay",
    ]

    print("EJEMPLOS DE TAREAS EN LENGUAJE NATURAL")
    print("=" * 50)
    for i, task in enumerate(examples, 1):
        print(f"{i:2}. {task}")

    print(
        "\n¿Qué número de ejemplo quieres ejecutar? (o escribe tu propia tarea): ",
        end="",
    )
    choice = input().strip()

    if choice.isdigit() and 1 <= int(choice) <= len(examples):
        task = examples[int(choice) - 1]
    else:
        task = choice

    print(f"\nEjecutando: {task}\n")
    result = await atlas.run(task)
    return result


if __name__ == "__main__":
    asyncio.run(run_examples())
