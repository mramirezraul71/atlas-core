# tests/smoke/

Comprobaciones ligeras de humo asociadas al PR A2 (archivado +
simplificación del script de arranque). No son tests unitarios ni
están integradas en un framework; son scripts autocontenidos que
se ejecutan a mano.

## Scripts

### `a2_parity_check.sh`

Captura la superficie HTTP viva (`atlas_adapter.atlas_http_api:app`
en `127.0.0.1:8791`) antes y después de A2, normaliza campos
volátiles (timestamps, `ms`, `uptime`) y compara.

Uso típico:

```bash
# 1) En main (pre-A2), con la API arrancada:
bash tests/smoke/a2_parity_check.sh before

# 2) En la rama A2, con la API arrancada:
bash tests/smoke/a2_parity_check.sh after

# 3) Comparar:
bash tests/smoke/a2_parity_check.sh diff
```

Salidas en `/tmp/atlas_a2_parity/`. El diff produce `.patch` por
cada endpoint que difiera.

Requisitos: `bash`, `curl`. Recomendado: `jq` (para normalización).

### `b_parity_check.sh`

Captura sólo el endpoint `/intent` (el único que B modifica
internamente) para una batería de 14 casos que cubren los 12
`kind`s + `empty` + un caso `inbox.fallback` explícito. Normaliza
`ms`, timestamps y rutas de snapshot con formato
`YYYYMMDD_HHMMSS` (que dependen del momento de ejecución) antes
de comparar.

Uso típico:

```bash
# 1) En main (pre-B), con la API arrancada:
bash tests/smoke/b_parity_check.sh before

# 2) En la rama B, con la API arrancada:
bash tests/smoke/b_parity_check.sh after

# 3) Comparar:
bash tests/smoke/b_parity_check.sh diff
```

Salidas en `/tmp/atlas_b_parity/`.

**Nota sobre `empty`**: `command_router.handle` usa `if not text`
(truthy), no `text.strip()`. Por tanto `text=""` es `empty`
(`"ATLAS: vacío."`), pero `text="   "` cae en el fallback de
inbox. El smoke refleja el caso estricto `text=""` como
`14_empty.json`.

**Importante sobre el vault**: varios casos de la batería tienen
efectos de filesystem (`/note create`, `/note append`,
`/snapshot`, inbox fallback). Para que la comparación before/after
sea limpia, **el vault debe estar en el mismo estado inicial en
ambas corridas**. Recomendación: apuntar `ATLAS_VAULT_DIR`,
`ATLAS_NOTES_DIR`, `ATLAS_SNAPS_DIR` a un directorio temporal y
borrarlo antes de cada captura. Si se ejecuta sobre el vault real
sin limpiar, casos como `08_note_view` acumularán líneas entre
ejecuciones y el diff marcará falsos positivos que no reflejan
cambios en `/intent`.

Requisitos: `bash`, `curl`. Recomendado: `jq`.

### `ps1_parse_check.ps1`

Parseo estático del script `03_run_atlas_api.ps1` tras la
simplificación de A2. No ejecuta el script; solo verifica:

- Que PowerShell puede parsearlo sin errores.
- Que existe exactamente una asignación
  `$LiveApp = "atlas_adapter.atlas_http_api:app"`.
- Que la rama de override manual `-AppImport` sigue presente.

Uso:

```powershell
pwsh -NoProfile -File tests/smoke/ps1_parse_check.ps1
```

## Alcance

Estos scripts cubren únicamente el perímetro modificado por A2. No
ejercitan lógica de negocio (estrategia, riesgo, etc.) y no sustituyen
a las pruebas que vendrán con los pasos B–E del plan de Atlas Push.
