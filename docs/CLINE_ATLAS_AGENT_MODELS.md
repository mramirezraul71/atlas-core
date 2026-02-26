# Vinculación Cline y ATLAS Agent con tu batería de modelos

Para que **Cline** y **ATLAS Agent** (extensiones de Cursor) usen **exclusivamente tus modelos** (Bóveda + Ollama + APIs externas) y **no el crédito del editor**.

Template de settings: `config/vscode_settings_template.json` (cópialo a `.vscode/settings.json` si quieres).

## 1. Cargar variables de entorno desde la Bóveda

En la raíz del repo:

```bash
python scripts/sync_env_from_vault.py
```

Esto genera `.env` desde `C:\dev\credenciales.txt` (o `ATLAS_VAULT_PATH`). Cursor no lee `.env` automáticamente; hay que exponer las variables al proceso (ver abajo).

## 2. Cursor: usar API propia (no crédito del editor)

- **Cursor Settings** → **Features** / **Models**:
  - Activa **"Use your own API keys"** o **"Custom API"**.
- **Cline** (si está instalado):
  - Settings → Cline → **API Keys**: elegir **"From environment variables"**.
  - Asegúrate de que el proceso de Cursor reciba `OPENAI_API_KEY`, `ANTHROPIC_API_KEY`, `GEMINI_API_KEY`, etc.

## 3. Exponer env al abrir Cursor

Para que Cursor (y Cline) vean las claves:

**Opción A — Lanzar Cursor desde una terminal donde ya cargaste el .env:**

```powershell
cd C:\ATLAS_PUSH
# Cargar .env en la sesión (PowerShell)
Get-Content .env | ForEach-Object { if ($_ -match '^([^#=]+)=(.*)$') { [Environment]::SetEnvironmentVariable($matches[1].Trim(), $matches[2].Trim().Trim('"'), 'Process') } }
& "C:\Users\<tu_usuario>\AppData\Local\Programs\cursor\Cursor.exe" "C:\ATLAS_PUSH"
```

**Opción B — Script de arranque que genera .env y abre Cursor:**

Crea `scripts/start_cursor_with_atlas_env.ps1` que ejecute `sync_env_from_vault.py`, cargue las variables y lance Cursor.

## 4. Modelos con mayor ventana de contexto (arquitectura)

Para tareas de arquitectura el sistema prioriza (según `config/atlas.env` y `scripts/load_atlas_models.py`):

- **Ollama:** `deepseek-r1:14b`, `deepseek-r1:latest`
- **Anthropic:** Claude 3.5 Sonnet
- **OpenAI:** GPT-4o

Ruta de modelo: `AI_ARCHITECT_MODEL=ollama:deepseek-r1:14b` (editable en `config/atlas.env`).

## 5. Comprobar que ATLAS usa tu infraestructura

- **PUSH (:8791):** Al arrancar carga `config/atlas.env` y la Bóveda (`ATLAS_VAULT_PATH`). Las llamadas a IA pasan por `modules/humanoid/ai/router.py` y `provider_credentials` (env + `config/provider_api_keys.json`).
- **Chat del Dashboard:** Si el front usa el backend PUSH para IA, ya está usando tus modelos.
- **Cursor/Cline:** Si activaste "Use your own API keys" y las variables están en el proceso, el chat de Cursor muestra que usas tu propia infraestructura (no "Cursor subscription").

## 6. Template de settings (VSCode/Cursor)

Si quieres un template de configuración del workspace para referencias de env, copia `config/vscode_settings_template.json` a `.vscode/settings.json` (el directorio `.vscode` no se sube al repo). Ese template solo contiene placeholders y comentarios; las claves nunca se guardan en settings.
