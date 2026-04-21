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
