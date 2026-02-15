# Repo Hygiene Automática (ATLAS) — Commit/Pull/Push sin acumulación

Este documento define el **proceso obligatorio** para que ATLAS haga commit y “actualice repo” sin volver a meter artefactos locales (`snapshots/`, `logs/`, capturas) en Git.

## Objetivo

- **Prohibido** versionar evidencia/local.
- **Permitido** versionar solo: código, docs, config (sin secretos), assets de producto.
- **Automático**: ATLAS debe ejecutar higiene **antes** de cada commit y **antes** de cada push.

## 1) Reglas de oro (lo que NUNCA se sube)

- `snapshots/**` (evidencia, reportes, bundles)
- `logs/**` (logs y bitácoras runtime)
- capturas sueltas: `camera_*.jpg/png`, `direct_camera_*.jpg/png`, `proxy_camera_*.jpg/png`, `vision_camera_*.jpg/png`

Estas reglas están respaldadas por `.gitignore`. Si alguien las trackea por accidente, **Repo Hygiene lo corrige**.

## 2) Comando estándar (manual)

Desde la raíz del repo:

- **Escaneo (seguro, no modifica nada)**:

```bash
python scripts/repo_hygiene.py --scan
```

- **Arreglar (des-trackea + refuerza `.gitignore`)**:

```bash
python scripts/repo_hygiene.py --fix
```

- **Arreglar + commit**:

```bash
python scripts/repo_hygiene.py --fix --commit
```

## 3) Modo automático (ATLAS) — recomendado

ATLAS debe ejecutar el modo automático (fix + commit + push) **solo si está explícitamente permitido**:

- habilita:
  - `REPO_HYGIENE_AUTO=true`
  - `REPO_HYGIENE_ENABLED=true` (default)
  - `REPO_HYGIENE_INTERVAL_SECONDS=21600` (6h; ajustable)

Luego ATLAS puede correr:

```bash
python scripts/repo_hygiene.py --auto
```

**Seguridad**: si `REPO_HYGIENE_AUTO` no está habilitado, `--auto` se degrada a `--scan` y emite alerta en OPS.

## 4) Ciclo obligatorio “Commit & Update Repo” (algoritmo)

ATLAS debe seguir este orden, SIEMPRE:

1. `repo_hygiene --scan`
2. Si detecta tracked forbidden (`logs/`, `snapshots/`, capturas):
   - ejecutar `repo_hygiene --fix`
3. `git status --porcelain` debe estar limpio de artefactos prohibidos
4. `git add -A` (solo cambios permitidos)
5. `git commit -m "<mensaje>"` (sin incluir logs/snapshots)
6. `git push`
7. Verificación:
   - `git status -sb` debe quedar “up to date”

## 5) Integración por Scheduler (para no acumular)

Al iniciar ATLAS (API), se registra el job:

- `repo_hygiene_cycle` (kind: `repo_hygiene_cycle`)

Comportamiento:
- Por defecto ejecuta `scripts/repo_hygiene.py --scan`
- Si `REPO_HYGIENE_AUTO=true`, ejecuta `--auto`

Este job evita acumulación **aunque el operador no se acuerde**.

## 6) Nota sobre archivos grandes (ej. modelos `.pt`)

Ejemplo: `nexus/atlas_nexus_robot/backend/yolov8n.pt`.

Opciones:
- Mantener en repo si es aceptable (offline-first).
- Migrar a Git LFS.
- Descargar en build/primer arranque (y **no versionar**).

## 7) Señalización / Evidencia

`scripts/repo_hygiene.py` emite eventos al **OPS Bus** (si está disponible) con:
- conteos de archivos prohibidos trackeados
- reglas agregadas a `.gitignore`
- resultado de commit/push automático

