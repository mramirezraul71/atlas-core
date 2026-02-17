#!/usr/bin/env python3
"""Test del sistema de IA Full Automatico."""
import os
import sys

# Asegurar path
ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from modules.humanoid.ai import (
    get_brain_state,
    set_full_auto_mode,
    auto_route,
    TaskType,
    get_auto_router
)

def main():
    print("=" * 60)
    print("  ATLAS - Test Sistema IA Full Automatico")
    print("=" * 60)
    print()
    
    # Configurar modo full auto
    state = set_full_auto_mode()
    print("=== ESTADO DEL CEREBRO ===")
    print(f"  Modo: {state.get('mode')}")
    print(f"  Auto Route: {state.get('auto_route')}")
    print(f"  Especialistas: {len(state.get('specialists', {}))}")
    
    specialists = state.get('specialists', {})
    for task, model in specialists.items():
        print(f"    - {task}: {model}")
    
    features = state.get('features', {})
    print(f"  Features habilitadas:")
    for feat, val in features.items():
        print(f"    - {feat}: {val}")
    
    print()
    print("=== PRUEBA AUTO ROUTER ===")
    router = get_auto_router()
    status = router.get_status()
    print(f"  Modelos configurados: {status['models_configured']}")
    
    for key, info in status['models'].items():
        print(f"    [{info['provider']}] {info['model']} -> {info['tasks']}")
    
    print()
    print("=== PRUEBA DE ROUTING ===")
    tests = [
        "Hola, como estas?",
        "Escribe una funcion en Python que ordene una lista",
        "Explica paso a paso como funciona la memoria cache",
        "Analiza esta imagen y describe lo que ves",
        "Disena la arquitectura de un sistema de microservicios",
        "Optimiza este codigo para que sea mas eficiente",
        "Escribe una historia corta sobre un robot",
    ]
    
    for prompt in tests:
        result = auto_route(prompt)
        task = result.task_type.value
        model = result.model_config.model_name
        conf = f"{result.confidence:.0%}"
        print(f"  [{task:10}] ({conf:>4}) -> {model:25} | {prompt[:35]}...")
    
    print()
    print("=" * 60)
    print("  [OK] Sistema IA Full Automatico operativo")
    print("=" * 60)


if __name__ == "__main__":
    main()
