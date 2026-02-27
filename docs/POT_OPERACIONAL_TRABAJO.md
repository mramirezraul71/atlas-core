## POT (Procedimiento Operacional de Trabajo) — ATLAS

Objetivo: que ATLAS trabaje como un operario senior **sin acumular cambios**, con **evidencia** por Audio/Telegram y con commits **pequeños, reversibles y auditables**.

### Principios (Reglas de Oro)

- **Cadencia**: no acumular. Commit por hito funcional (máx. 15–25 archivos por commit).
- **Separación**: 1 commit = 1 tema (Vision / Approvals / Architect / Learning / Dashboard / Scripts).
- **Evidencia obligatoria**:
  - Antes: estado + pruebas.
  - Después: commit + push + confirmación por Telegram/Audio.
- **Seguridad**:
  - Nunca commitear `.env`, `credenciales.txt`, bases de datos, logs o snapshots.
  - Si hay duda, se excluye y se reporta.

### Flujo estándar (Checklist)

1) **Diagnóstico**
- `git status -b --porcelain`
- `git diff --stat`

2) **Pruebas (mínimo)**
- `pytest -q`
- Si falla: **no commit**. Reparar → reintentar.

3) **Plan de commits (anti-megacommits)**
- Si hay más de **25 archivos** tocados o cambios en múltiples áreas, dividir por módulos:
  - `modules/atlas_architect/*`
  - `modules/humanoid/vision/*`
  - `modules/humanoid/approvals/*` + `modules/humanoid/comms/*`
  - `atlas_adapter/static/dashboard.html`
  - `brain/learning/*` + `training/*` + `tests/*`
  - `scripts/*` + configs

4) **Stage**
- `git add -A` (o por grupo si se va a dividir)
- `git diff --staged --stat`

5) **Commit**
- Mensaje: `feat|fix|chore: <resumen humano>`
- Cuerpo: 3–6 bullets con “por qué”.
- En Windows/PowerShell: evitar HEREDOC tipo bash.

6) **Push**
- `git push`
- Confirmar: `git status -b --porcelain` (no debe quedar ahead sin push)

7) **Notificación (Telegram/Audio)**
- Al crear commit: “Nuevo cambio guardado en Git: <resumen>”
- Al hacer push: “Cambios subidos a GitHub”

### Automatización activa (ya instalada)

- **Hook post-commit**: al crear un commit, ATLAS lo anuncia por Audio/Telegram.
- **Repo monitor after-fix**: si hay demasiados archivos, separa el commit por grupos y notifica.

