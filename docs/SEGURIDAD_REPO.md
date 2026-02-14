# Seguridad del repositorio — Sin credenciales ni archivos que nos puedan comprometer

Este repo **no debe contener** credenciales ni archivos sensibles. Reglas y buenas prácticas.

---

## Nunca commitear

- **`config/atlas.env`** — Config con secretos, API keys, rutas locales. Usar `config/atlas.env.example` como plantilla.
- **`.env`**, **`*.env`** (excepto `.env.example` / `atlas.env.example` como plantillas sin secretos).
- **`credenciales.txt`** o cualquier archivo con claves (Bóveda, Telegram, APIs).
- **Claves privadas:** `*.pem`, `*.key` (salvo ejemplos como `*.key.example`).
- **`license.key`** u otros archivos de licencia con firma/secretos.
- Carpetas **`secrets/`**, **`.secrets/`** y cualquier archivo con contraseñas o tokens.

---

## .gitignore

El `.gitignore` ya incluye:

- `config/atlas.env`, `config/.env`
- `credenciales.txt`, `**/credenciales*.txt`
- `*.pem`, `*.key`, `license.key`
- `secrets/`, `.secrets/`

Si añades nuevos archivos de configuración con secretos, **inclúyelos en .gitignore** antes de trabajar.

---

## Historial limpio

Se ha usado **git filter-repo** para eliminar `config/atlas.env` de todo el historial. Si en el futuro se sube por error algún archivo sensible:

1. Añádelo a `.gitignore`.
2. Deja de trackearlo: `git rm --cached <archivo>`.
3. Si ya estaba en commits anteriores, elimínalo del historial con `git filter-repo` y luego haz **force-push** (avisa al equipo).

---

## Push después de limpiar historial

Si se reescribió historial, hay que volver a configurar el remoto y hacer push forzado:

```bash
git remote add origin https://github.com/mramirezraul71/atlas-core.git   # si se eliminó
git push --force-with-lease origin <rama>
```

Cualquier persona que tenga un clon antiguo deberá re-clonar o hacer `git fetch origin && git reset --hard origin/<rama>`.

---

*Mantener el repo sin credenciales protege a todo el equipo y evita fugas.*  

*Generado como parte del blindaje del repo atlas-core.*
