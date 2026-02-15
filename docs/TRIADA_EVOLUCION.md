# Tríada de Crecimiento (GitHub, PyPI, Hugging Face)

## Comprobación: ¿se ha ejecutado alguna actualización?

- **Antes:** No había registro de ejecución (`logs/evolution_last_cycle.json` no existía) porque el daemon no se había arrancado o no se había forzado un ciclo.
- **Ahora:**
  - **Endpoint:** `POST /api/evolution/trigger` — lanza un ciclo en segundo plano sin esperar.
  - **Modo run-once:** `python evolution_daemon.py --run-once` — ejecuta un ciclo y termina.
  - **Script:** `scripts/force_evolution_cycle.ps1` — fuerza ciclo (por API si PUSH está en 8791, si no ejecuta `--run-once`).
- **Verificación:** Tras un ciclo correcto se crea/actualiza `logs/evolution_last_cycle.json` con `{"ok": true, "message": "Asimilación Exitosa", ...}`.

## Causas por las que la Tríada no se ejecutaba

1. **Daemon no en marcha:** El ciclo corre cada 12 h solo si `evolution_daemon.py` está en ejecución (o se usa `--run-once` / el trigger).
2. **Credenciales:** GitHub y Hugging Face requieren token en la Bóveda; si faltan, esos workers se desactivan. PyPI puede escanear sin token.
   - Bóveda: `ATLAS_CREDENTIALS_PATH` o `CREDENTIALS_PATH` en env; por defecto `C:\dev\credenciales.txt`. Claves: `GITHUB_TOKEN`, `HF_TOKEN`/`HUGGINGFACE_TOKEN`, `PYPI_TOKEN`.
3. **Bug corregido:** Al actualizar `requirements.txt`, el daemon unía líneas sin `\n`; ya se preservan los saltos de línea.

## Cómo forzar una actualización

1. **Desde la API (PUSH en 8791):**
   ```http
   POST http://127.0.0.1:8791/api/evolution/trigger
   ```
2. **Desde PowerShell:**
   ```powershell
   .\scripts\force_evolution_cycle.ps1        # ciclo directo
   .\scripts\force_evolution_cycle.ps1 -Api   # vía API
   ```
3. **Desde línea de comandos:**
   ```bash
   python evolution_daemon.py --run-once
   ```

## Estado tras la corrección

- Ciclo ejecutado con éxito: PyPI (p. ej. pydantic actualizado), GitHub (atlas-core, commits), Hugging Face (modelos VLA/visión).
- `logs/evolution_last_cycle.json` generado con `"Asimilación Exitosa"`.
- `requirements.txt` restaurado a formato una-línea-por-paquete (y el daemon ya no lo corrompe en futuras actualizaciones).
