#!/usr/bin/env bash
# tests/smoke/b_parity_check.sh
#
# Smoke de paridad funcional del endpoint /intent ANTES/DESPUÉS
# del PR B (IntentRouter wrapper sobre command_router.handle).
#
# A diferencia del smoke de A2, aquí sólo se ejercita /intent, ya que
# B únicamente cambia la implementación interna de ese endpoint. Los
# demás endpoints (/status, /tools, /modules, /execute) no se tocan
# en B y se validan con tests/smoke/a2_parity_check.sh.
#
# Uso:
#   1. Arranca la API viva contra main (pre-B) en 127.0.0.1:8791.
#        ./03_run_atlas_api.ps1 -RepoPath C:\ATLAS
#      En otra shell:
#        bash tests/smoke/b_parity_check.sh before
#   2. Checkout de la rama B, vuelve a arrancar la API, y:
#        bash tests/smoke/b_parity_check.sh after
#   3. Compara:
#        bash tests/smoke/b_parity_check.sh diff
#
# Requisitos: bash, curl. Recomendado: jq (para normalización).
#
# Nota sobre el caso "empty": command_router.handle utiliza
# `if not text` (truthy), no `text.strip()`. Por tanto:
#   - text=""          -> "ATLAS: vacío."   (kind empty)
#   - text="   \n\t "  -> mensaje de inbox  (kind inbox.fallback)
# El smoke refleja ambos casos por separado.

set -u

HOST="${ATLAS_HOST:-127.0.0.1}"
PORT="${ATLAS_PORT:-8791}"
BASE="http://${HOST}:${PORT}"
OUT_DIR="${OUT_DIR:-/tmp/atlas_b_parity}"

mode="${1:-before}"

case "$mode" in
  before|after|diff) ;;
  *)
    echo "Uso: $0 {before|after|diff}" >&2
    exit 2
    ;;
esac

mkdir -p "${OUT_DIR}/${mode}" 2>/dev/null || true

# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

# Normaliza campos volátiles (timings, timestamps, rutas de snapshot
# que incluyen fecha/hora) para que el diff no marque como regresión
# lo que solo cambia entre invocaciones.
normalize() {
  if command -v jq >/dev/null 2>&1; then
    jq 'walk(
      if type == "object" then
        with_entries(
          if (.key | ascii_downcase) as $k
             | ($k == "ms" or $k == "elapsed_ms" or $k == "ts"
                or $k == "timestamp" or $k == "now" or $k == "started_at"
                or $k == "uptime" or $k == "uptime_s")
          then .value = "<normalized>"
          else . end
        )
      else . end
    )
    | if (.output | type) == "string" then
        # Normalizar rutas de snapshot con timestamp
        #   snapshots/20260421_153301_hito-1
        # a:
        #   snapshots/<STAMP>_hito-1
        .output |= gsub("\\d{8}_\\d{6}"; "<STAMP>")
      else . end' 2>/dev/null || cat
  else
    cat
  fi
}

post_intent() {
  local name="$1" ; local text="$2"
  # Construir JSON con jq si está, si no con printf escapando
  # manualmente (los textos de la batería no llevan comillas dobles).
  local body
  if command -v jq >/dev/null 2>&1; then
    body=$(jq -n --arg t "$text" '{user:"raul", text:$t, meta:null}')
  else
    # Escape mínimo: backslash y comilla doble.
    local esc
    esc=${text//\\/\\\\}
    esc=${esc//\"/\\\"}
    body="{\"user\":\"raul\",\"text\":\"${esc}\",\"meta\":null}"
  fi

  curl -sS -m 5 -X POST "${BASE}/intent" \
    -H "Content-Type: application/json" \
    -d "$body" 2>/dev/null \
    | normalize \
    > "${OUT_DIR}/${mode}/${name}.json"
  echo "  POST /intent text=$(printf '%q' "$text")  ->  ${OUT_DIR}/${mode}/${name}.json"
}

# ------------------------------------------------------------------
# Captura (before / after)
#
# Batería: 13 casos cubriendo los 12 kinds + empty.
# ------------------------------------------------------------------

capture() {
  echo "[b_parity] modo=${mode} base=${BASE}"
  echo "[b_parity] salida=${OUT_DIR}/${mode}/"

  post_intent "01_status"                 "/status"
  post_intent "02_doctor"                 "/doctor"
  post_intent "03_modules"                "/modules"
  post_intent "04_snapshot_sin_label"     "/snapshot"
  post_intent "05_snapshot_con_label"     "/snapshot smoke-B"
  post_intent "06_note_create"            "/note create Smoke-B-Foo"
  post_intent "07_note_append"            "/note append Smoke-B-Foo | linea desde smoke B"
  post_intent "08_note_view"              "/note view Smoke-B-Foo"
  post_intent "09_natural_note_create"    "Atlas, crea una nota llamada Smoke-B-Bar"
  post_intent "10_natural_note_append"    "Atlas, agrega a la nota Smoke-B-Bar que hola desde smoke B"
  post_intent "11_natural_snapshot"       "Atlas, crea un snapshot llamado smoke-b-hito"
  post_intent "12_natural_modules"        "Atlas, dime qué módulos tienes activos"
  post_intent "13_inbox_fallback"         "texto libre que no encaja en nada smoke B"
  post_intent "14_empty"                  ""

  echo "[b_parity] captura ${mode} completa."
}

if [[ "$mode" == "before" || "$mode" == "after" ]]; then
  capture
  exit 0
fi

# ------------------------------------------------------------------
# Diff
# ------------------------------------------------------------------

BEFORE="${OUT_DIR}/before"
AFTER="${OUT_DIR}/after"

if [[ ! -d "$BEFORE" || ! -d "$AFTER" ]]; then
  echo "[b_parity] faltan ${BEFORE} o ${AFTER}. Ejecuta before y after primero." >&2
  exit 1
fi

echo "[b_parity] diff before vs after en ${OUT_DIR}"
rc=0
for f in "$BEFORE"/*.json; do
  name="$(basename "$f")"
  if [[ ! -f "${AFTER}/${name}" ]]; then
    echo "  FALTA en after: ${name}"
    rc=1
    continue
  fi
  if ! diff -u "$f" "${AFTER}/${name}" > "${OUT_DIR}/diff_${name%.json}.patch"; then
    echo "  DIFERENCIA: ${name}  (ver ${OUT_DIR}/diff_${name%.json}.patch)"
    rc=1
  else
    rm -f "${OUT_DIR}/diff_${name%.json}.patch"
    echo "  ok: ${name}"
  fi
done

if [[ "$rc" -eq 0 ]]; then
  echo "[b_parity] PARIDAD OK: /intent no cambia entre before y after."
else
  echo "[b_parity] PARIDAD ROTA: revisar los .patch listados arriba." >&2
fi
exit "$rc"
