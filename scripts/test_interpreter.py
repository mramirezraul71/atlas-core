"""
Test de integracion: Open Interpreter Bridge para ATLAS Workspace.

Ejecuta desde la raiz del proyecto:
    .venv\\Scripts\\python.exe scripts\\test_interpreter.py
"""
from __future__ import annotations

import asyncio
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dotenv import load_dotenv

_env = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "atlas.env")
if os.path.exists(_env):
    load_dotenv(_env, override=True)
else:
    load_dotenv()


def test_import():
    print("[1/5] Importando Open Interpreter...", end=" ")
    from interpreter import OpenInterpreter
    print(f"OK - {OpenInterpreter}")


def test_bridge_import():
    print("[2/5] Importando interpreter_bridge...", end=" ")
    from modules.humanoid.hands.interpreter_bridge import (
        resolve_model,
        list_available_models,
        interpreter_status,
        get_session_manager,
    )
    print("OK")
    return resolve_model, list_available_models, interpreter_status, get_session_manager


def test_model_resolution(resolve_model, list_available_models):
    print("[3/5] Resolucion de modelos IA...")
    default = resolve_model()
    print(f"      Modelo por defecto: {default}")
    models = list_available_models()
    print(f"      Modelos disponibles: {len(models)}")
    for m in models[:8]:
        print(f"        - {m['id']} ({m['tier']})")
    if len(models) > 8:
        print(f"        ... y {len(models) - 8} mas")


def test_status(interpreter_status):
    print("[4/5] Estado del motor...")
    st = interpreter_status()
    print(f"      Instalado: {st['installed']} (v{st['version']})")
    print(f"      AI mode: {st['ai_mode']}")
    print(f"      Max sessions: {st['max_sessions']}")
    print(f"      Session TTL: {st['session_ttl_sec']}s")


def test_session_lifecycle(get_session_manager):
    print("[5/5] Ciclo de vida de sesiones...")
    mgr = get_session_manager()
    s = mgr.get_or_create(model="ollama/llama3.1:latest")
    print(f"      Sesion creada: {s.session_id} (model={s.model})")
    sessions = mgr.list_sessions()
    print(f"      Sesiones activas: {len(sessions)}")
    closed = mgr.close(s.session_id)
    print(f"      Sesion cerrada: {closed}")
    sessions2 = mgr.list_sessions()
    print(f"      Sesiones restantes: {len(sessions2)}")
    assert closed, "Session should have been closed"
    assert len(sessions2) == len(sessions) - 1, "Session count mismatch"


def main():
    print("=" * 60)
    print("ATLAS Workspace -- Open Interpreter Integration Test")
    print("=" * 60)

    test_import()

    resolve_model, list_available_models, interpreter_status, get_session_manager = test_bridge_import()
    test_model_resolution(resolve_model, list_available_models)
    test_status(interpreter_status)
    test_session_lifecycle(get_session_manager)

    print("=" * 60)
    print("Todos los tests pasaron.")
    print("=" * 60)


if __name__ == "__main__":
    main()
