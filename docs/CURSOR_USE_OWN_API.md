# Activar "Use your own API" en Cursor

## Por qué sale "No Settings Found" en ATLAS Folder

El ámbito **ATLAS Folder** solo muestra opciones que define la extensión Atlas para esa carpeta.
La opción **Use your own API keys** es de **Cursor**, no de la extensión, por eso **nunca** aparecerá en "ATLAS Folder".

## Dónde está ya activado (por código)

- **Usuario Cursor:** `%AppData%\Roaming\Cursor\User\settings.json`
  - `cursor.general.useOwnApiKeys`: true
  - `cursor.useOwnApiKeys`: true
- **Workspace:** `.vscode/settings.json` en la raíz del repo
  - Mismas claves anteriores

Eso ya está configurado. Para que Cursor **use** tus keys y desaparezca el "100% API usage":

## Pasos en Cursor (UI)

1. **Abrir Cursor Settings**
   - Pestaña **Cursor Settings** (junto a "Settings"), **o**
   - Menú **File → Preferences → Cursor Settings**.

2. **Models / API**
   - Busca la sección **Models** o **API Keys**.
   - Activa **"Use your own API keys"** (o equivalente).
   - Añade al menos una clave (OpenAI, Anthropic o la que uses) en los campos que muestre Cursor.
   - Así el chat usará tus keys y dejará de consumir el cupo incluido.

3. **Comprobar en Settings (User)**
   - Pestaña **Settings** → **User** (no "ATLAS Folder").
   - Busca `use own` o `useOwnApiKeys`.
   - Deberías ver la opción y que esté en **true**.

4. **Recargar**
   - **Ctrl+Shift+P** → **Developer: Reload Window**.

Resumen: no busques "use own api" en **ATLAS Folder**; en **User** sí aparece, y el interruptor efectivo está en **Cursor Settings → Models / API keys**.
