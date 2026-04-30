#!/usr/bin/env bash
# tests/smoke/a2_parity_check.sh
#
# Smoke de paridad funcional ANTES/DESPUÉS del PR A2.
#
# Uso:
#   1. Arranca la API viva contra main (o pre-A2) en 127.0.0.1:8791.
#        ./03_run_atlas_api.ps1 -RepoPath C:\ATLAS
#      En otra shell:
#        bash tests/smoke/a2_parity_check.sh before
#   2. Checkout de la rama A2, vuelve a arrancar la API, y:
#        bash tests/smoke/a2_parity_check.sh after
#   3. Compara:
#        bash tests/smoke/a2_parity_check.sh diff
#
# Objetivo: verificar que A2 (archivado + simplificación del .ps1) no
# altera la superficie HTTP viva (atlas_adapter.atlas_http_api:app).
# No intenta validar semántica de negocio, solo contratos de respuesta.
#
# Requisitos: bash, curl, jq.

set -u

HOST="${ATLAS_HOST:-127.0.0.1}"
PORT="${ATLAS_PORT:-8791}"
BASE="http://${HOST}:${PORT}"
OUT_DIR="${OUT_DIR:-/tmp/atlas_a2_parity}"

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

# Normaliza campos volátiles (timings, timestamps) para que el diff
# no marque como regresión lo que solo cambia entre invocaciones.
normalize() {
  # jq si está disponible, si no, passthrough.
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
    )' 2>/dev/null || cat
  else
    cat
  fi
}

hit_get() {
  local name="$1" ; local path="$2"
  curl -sS -m 5 "${BASE}${path}" 2>/dev/null \
    | normalize \
    > "${OUT_DIR}/${mode}/${name}.json"
  echo "  GET  ${path}  ->  ${OUT_DIR}/${mode}/${name}.json"
}

hit_post() {
  local name="$1" ; local path="$2" ; local body="$3"
  curl -sS -m 5 -X POST "${BASE}${path}" \
    -H "Content-Type: application/json" \
    -d "${body}" 2>/dev/null \
    | normalize \
    > "${OUT_DIR}/${mode}/${name}.json"
  echo "  POST ${path}  ->  ${OUT_DIR}/${mode}/${name}.json"
}

# ------------------------------------------------------------------
# Captura (before / after)
# ------------------------------------------------------------------

capture() {
  echo "[a2_parity] modo=${mode} base=${BASE}"
  echo "[a2_parity] salida=${OUT_DIR}/${mode}/"

  hit_get  "01_status"   "/status"
  hit_get  "02_tools"    "/tools"
  hit_get  "03_modules"  "/modules"

  # 3 ejecuciones de /execute (una por tool típica).
  hit_post "04_execute_ping"   "/execute" '{"tool":"ping","args":{}}'
  hit_post "05_execute_status" "/execute" '{"tool":"status","args":{}}'
  hit_post "06_execute_noop"   "/execute" '{"tool":"noop","args":{}}'

  # 3 intents típicos.
  hit_post "07_intent_hola"    "/intent" '{"text":"hola"}'
  hit_post "08_intent_status"  "/intent" '{"text":"status"}'
  hit_post "09_intent_ayuda"   "/intent" '{"text":"ayuda"}'

  echo "[a2_parity] captura ${mode} completa."
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
  echo "[a2_parity] faltan ${BEFORE} o ${AFTER}. Ejecuta before y after primero." >&2
  exit 1
fi

echo "[a2_parity] diff before vs after en ${OUT_DIR}"
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
  echo "[a2_parity] PARIDAD OK: la superficie HTTP no cambia entre before y after."
else
  echo "[a2_parity] PARIDAD ROTA: revisar los .patch listados arriba." >&2
fi
exit "$rc"
