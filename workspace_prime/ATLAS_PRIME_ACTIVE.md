# ATLAS Workspace Prime — Activo

- **Fecha activación**: 2025-02-25
- **Python**: 3.11 (usar explícitamente `C:\Users\r6957\AppData\Local\Programs\Python\Python311\python.exe` en scripts y terminal)
- **Fase 14**: Tests OK (memory_manager, web_tools, vision_eyes, desktop_hands, smart_browser, visual_agent, nl_parser, plan_executor, browser_hands)
- **Nota**: `nl_parser` requiere credenciales AWS/Bedrock para parsear instrucciones; el resto funciona sin Bedrock.
- **Playwright**: Chromium instalado para Python 3.11 (`python -m playwright install chromium`)

Para ejecutar el agente:
```powershell
cd C:\ATLAS_PUSH\workspace_prime
& "C:\Users\r6957\AppData\Local\Programs\Python\Python311\python.exe" atlas_prime.py
```

Para re-ejecutar todos los tests de Fase 14:
```powershell
cd C:\ATLAS_PUSH\workspace_prime
.\run_phase14_py311.ps1
```
