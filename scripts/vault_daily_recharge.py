#!/usr/bin/env python3
"""
ATLAS — Vault Daily Recharge
=============================
Descarga contenido automáticamente desde YouTube y lo sube a RAULI Bóveda.
Corre diariamente vía Windows Task Scheduler o desde el sentinel de ATLAS.

Uso:
    python scripts/vault_daily_recharge.py [--dry-run] [--slot cami/musica] [--force]

Env vars requeridas:
    VAULT_ADMIN_TOKEN   — token admin del servidor RAULI-VISION

Dependencias:
    pip install yt-dlp requests
"""

import argparse
import json
import logging
import os
import random
import shutil
import subprocess
import sys
import tempfile
import time
from datetime import datetime, date
from pathlib import Path

import requests

# ─── Paths ────────────────────────────────────────────────────────────────────
BASE_DIR   = Path(__file__).resolve().parent.parent
LOG_DIR    = BASE_DIR / "logs"
LOG_PATH   = LOG_DIR / "vault_recharge.log"
CONFIG_PATH = Path(__file__).resolve().parent / "vault_recharge_config.json"
STATE_PATH  = BASE_DIR / "state" / "vault_recharge_state.json"

LOG_DIR.mkdir(exist_ok=True)
(BASE_DIR / "state").mkdir(exist_ok=True)

# ─── Logging ──────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [VAULT-RECHARGE] %(levelname)s %(message)s",
    handlers=[
        logging.FileHandler(LOG_PATH, encoding="utf-8"),
        logging.StreamHandler(sys.stdout),
    ],
)
log = logging.getLogger("vault_recharge")


# ─── Helpers ──────────────────────────────────────────────────────────────────

def _load_config() -> dict:
    if not CONFIG_PATH.exists():
        log.error(f"Config no encontrada: {CONFIG_PATH}")
        sys.exit(1)
    with open(CONFIG_PATH, encoding="utf-8") as f:
        return json.load(f)


def _load_state() -> dict:
    if STATE_PATH.exists():
        try:
            with open(STATE_PATH, encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return {"last_run": None, "titles_seen": {}}


def _save_state(state: dict) -> None:
    with open(STATE_PATH, "w", encoding="utf-8") as f:
        json.dump(state, f, ensure_ascii=False, indent=2)


def _get_admin_token(cfg: dict) -> str:
    """Lee el token desde env o desde el vault de credenciales."""
    token_env = cfg.get("admin_token_env", "ADMIN_TOKEN")
    tok = os.environ.get(token_env, "").strip()
    if tok:
        return tok
    # Intentar leer del vault maestro
    vault_path = Path(r"C:\dev\credenciales.txt")
    if vault_path.exists():
        for line in vault_path.read_text(encoding="utf-8").splitlines():
            if "=" in line and ("ADMIN_TOKEN" in line or "VAULT_ADMIN" in line):
                return line.split("=", 1)[1].strip()
    # Fallback al token por defecto del docker-compose de RAULI-VISION
    env_file = Path(__file__).resolve().parent.parent / "_external" / "RAULI-VISION" / ".env"
    if env_file.exists():
        for line in env_file.read_text(encoding="utf-8").splitlines():
            if "=" in line and "ADMIN_TOKEN" in line and not line.strip().startswith("#"):
                return line.split("=", 1)[1].strip()
    # Fallback: leer del bat que arranca el servidor (fuente de verdad)
    bat_file = Path(__file__).resolve().parent.parent / "_external" / "RAULI-VISION" / "espejo" / "run_espejo.bat"
    if bat_file.exists():
        for line in bat_file.read_text(encoding="utf-8", errors="replace").splitlines():
            if "ADMIN_TOKEN=" in line and not line.strip().startswith("rem"):
                return line.split("ADMIN_TOKEN=", 1)[1].strip()
    log.warning("ADMIN_TOKEN no encontrado en ninguna fuente")
    return ""


def _check_yt_dlp() -> bool:
    """Verifica que yt-dlp está disponible."""
    try:
        r = subprocess.run(["yt-dlp", "--version"], capture_output=True, timeout=10)
        if r.returncode == 0:
            log.info(f"yt-dlp disponible: {r.stdout.decode().strip()}")
            return True
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass
    # Intentar con python -m yt_dlp
    try:
        r = subprocess.run(
            [sys.executable, "-m", "yt_dlp", "--version"],
            capture_output=True, timeout=10
        )
        if r.returncode == 0:
            log.info(f"yt-dlp (módulo) disponible: {r.stdout.decode().strip()}")
            return True
    except Exception:
        pass
    return False


def _yt_dlp_cmd() -> list[str]:
    """Retorna el comando base de yt-dlp."""
    try:
        r = subprocess.run(["yt-dlp", "--version"], capture_output=True, timeout=5)
        if r.returncode == 0:
            return ["yt-dlp"]
    except FileNotFoundError:
        pass
    return [sys.executable, "-m", "yt_dlp"]


def _free_disk_mb(path: Path) -> float:
    """Espacio libre en disco en MB."""
    try:
        usage = shutil.disk_usage(str(path))
        return usage.free / 1024 / 1024
    except Exception:
        return 99999.0


def _get_existing_titles(vault_url: str, channel: str, category: str) -> set[str]:
    """Descarga el catálogo actual para evitar duplicados."""
    try:
        resp = requests.get(
            f"{vault_url}/api/vault/catalog",
            params={"channel": channel, "category": category},
            timeout=10,
        )
        if resp.status_code == 200:
            data = resp.json()
            return {item["title"].lower() for item in data.get("items", [])}
    except Exception as e:
        log.warning(f"No se pudo obtener catálogo: {e}")
    return set()


def _get_vault_size_mb(vault_url: str, admin_token: str) -> float:
    """Obtiene el tamaño total del vault en MB."""
    try:
        resp = requests.get(
            f"{vault_url}/api/vault/admin/status",
            headers={"X-Admin-Token": admin_token},
            timeout=10,
        )
        if resp.status_code == 200:
            data = resp.json()
            return data.get("total_size_gb", 0) * 1024
    except Exception:
        pass
    return 0.0


def _read_bat_admin_token() -> str:
    """Lee ADMIN_TOKEN desde _external/RAULI-VISION/espejo/run_espejo.bat."""
    bat_file = Path(__file__).resolve().parent.parent / "_external" / "RAULI-VISION" / "espejo" / "run_espejo.bat"
    if not bat_file.exists():
        return ""
    for line in bat_file.read_text(encoding="utf-8", errors="replace").splitlines():
        if "ADMIN_TOKEN=" in line and not line.strip().lower().startswith("rem"):
            return line.split("ADMIN_TOKEN=", 1)[1].strip()
    return ""


def _admin_token_valid(vault_url: str, admin_token: str) -> bool:
    """Valida token admin contra /api/vault/admin/status."""
    tok = (admin_token or "").strip()
    if not tok:
        return False
    try:
        resp = requests.get(
            f"{vault_url}/api/vault/admin/status",
            headers={"X-Admin-Token": tok},
            timeout=10,
        )
        return resp.status_code == 200
    except Exception:
        return False


def _vault_is_empty(vault_url: str) -> bool:
    """Retorna True si el vault no tiene ningún item activo en ningún canal."""
    try:
        resp = requests.get(f"{vault_url}/api/vault/catalog", timeout=8)
        if resp.status_code == 200:
            return len(resp.json().get("items", [])) == 0
    except Exception:
        pass
    return False


def _search_youtube(query: str, max_results: int, max_duration: int, ytdlp: list[str]) -> list[dict]:
    """
    Busca en YouTube y retorna metadatos de los resultados.
    Retorna lista de {id, title, uploader, duration, url}
    """
    search_query = f"ytsearch{max_results * 3}:{query}"  # pedir más para filtrar
    cmd = ytdlp + [
        search_query,
        "--dump-json",
        "--no-playlist",
        "--quiet",
        "--no-warnings",
        "--ignore-errors",
    ]
    try:
        r = subprocess.run(cmd, capture_output=True, timeout=60)
        results = []
        for line in r.stdout.decode("utf-8", errors="replace").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                info = json.loads(line)
                duration = info.get("duration", 0) or 0
                if max_duration > 0 and duration > max_duration:
                    continue
                results.append({
                    "id":       info.get("id", ""),
                    "title":    info.get("title", "Sin título"),
                    "uploader": info.get("uploader", info.get("channel", "")),
                    "duration": duration,
                    "url":      info.get("webpage_url", f"https://www.youtube.com/watch?v={info.get('id','')}"),
                })
                if len(results) >= max_results:
                    break
            except json.JSONDecodeError:
                continue
        return results
    except subprocess.TimeoutExpired:
        log.warning(f"Timeout al buscar: {query}")
        return []
    except Exception as e:
        log.warning(f"Error al buscar '{query}': {e}")
        return []


def _download_item(
    url: str,
    audio_only: bool,
    output_dir: str,
    ytdlp: list[str],
    timeout: int = 300,
) -> Path | None:
    """
    Descarga un item de YouTube.
    Retorna la ruta del archivo descargado, o None si falla.
    """
    out_tmpl = os.path.join(output_dir, "%(title).80s.%(ext)s")

    if audio_only:
        fmt_args = [
            "--extract-audio",
            "--audio-format", "mp3",
            "--audio-quality", "5",  # calidad media para ahorrar espacio
        ]
    else:
        fmt_args = [
            "-f", "bestvideo[ext=mp4][height<=720]+bestaudio[ext=m4a]/best[ext=mp4][height<=720]/best[height<=720]/best",
            "--merge-output-format", "mp4",
        ]

    cmd = ytdlp + [
        url,
        "-o", out_tmpl,
        "--no-playlist",
        "--no-warnings",
        "--quiet",
        "--no-progress",
        "--no-part",
        "--retries", "2",
        "--fragment-retries", "2",
    ] + fmt_args

    before = set(Path(output_dir).iterdir())
    try:
        r = subprocess.run(cmd, capture_output=True, timeout=timeout)
        after = set(Path(output_dir).iterdir())
        new_files = after - before
        if new_files:
            # Tomar el archivo más reciente
            return max(new_files, key=lambda p: p.stat().st_mtime)
        if r.returncode != 0:
            err = r.stderr.decode("utf-8", errors="replace")[:300]
            log.warning(f"yt-dlp error (rc={r.returncode}): {err}")
        return None
    except subprocess.TimeoutExpired:
        log.warning(f"Timeout descargando: {url}")
        return None
    except Exception as e:
        log.warning(f"Error descargando {url}: {e}")
        return None


def _upload_to_vault(
    vault_url: str,
    admin_token: str,
    file_path: Path,
    title: str,
    artist: str,
    category: str,
    channel: str,
    genre: str,
    duration_secs: int,
    dry_run: bool = False,
) -> bool:
    """Sube un archivo al vault vía multipart/form-data."""
    if dry_run:
        log.info(f"[DRY-RUN] Subiría: {title} → {channel}/{category}")
        return True

    try:
        with open(file_path, "rb") as fh:
            resp = requests.post(
                f"{vault_url}/api/vault/admin/upload",
                headers={"X-Admin-Token": admin_token},
                data={
                    "title":         title,
                    "artist":        artist,
                    "category":      category,
                    "channel":       channel,
                    "genre":         genre,
                    "duration_secs": str(duration_secs),
                },
                files={"file": (file_path.name, fh)},
                timeout=120,
            )
        if resp.status_code == 200:
            data = resp.json()
            if data.get("ok"):
                log.info(f"Subido OK: [{channel}/{category}] «{title}» → id={data.get('id','?')}")
                return True
            else:
                log.warning(f"Vault rechazó: {data.get('error','?')} — {title}")
        else:
            log.warning(f"HTTP {resp.status_code} al subir «{title}»: {resp.text[:200]}")
    except Exception as e:
        log.error(f"Error subiendo «{title}»: {e}")
    return False


# ─── Core logic ───────────────────────────────────────────────────────────────

def process_slot(
    slot: dict,
    cfg: dict,
    vault_url: str,
    admin_token: str,
    state: dict,
    ytdlp: list[str],
    dry_run: bool = False,
    force_slot: str | None = None,
) -> int:
    """Procesa un slot (channel+category), descarga y sube N items. Retorna items subidos."""
    channel  = slot["channel"]
    category = slot["category"]
    slot_key = f"{channel}/{category}"

    if force_slot and slot_key != force_slot:
        return 0

    log.info(f"── Procesando slot: {slot_key} ──")

    max_items   = cfg.get("max_items_per_slot", 3)
    max_size_mb = cfg.get("max_total_size_mb", 4096)
    min_free_mb = cfg.get("min_free_disk_mb", 2048)
    dl_timeout  = cfg.get("download_timeout_secs", 300)
    audio_only  = slot.get("audio_only", False)
    genre       = slot.get("genre", "")
    max_dur     = slot.get("max_duration_secs", 600)
    queries     = slot["queries"]

    # Guardar títulos vistos (deduplicación entre días)
    seen_key = f"seen_{slot_key}"
    titles_seen: set = set(state.get("titles_seen", {}).get(seen_key, []))

    # Títulos que ya están en el vault (evitar duplicados en el servidor)
    existing = _get_existing_titles(vault_url, channel, category)
    titles_seen |= existing

    # Límite de almacenamiento total
    current_size = _get_vault_size_mb(vault_url, admin_token)
    if current_size >= max_size_mb:
        log.warning(f"Vault lleno ({current_size:.0f} MB / {max_size_mb} MB) — saltando {slot_key}")
        return 0

    # Espacio libre en disco
    free_mb = _free_disk_mb(BASE_DIR)
    if free_mb < min_free_mb:
        log.warning(f"Disco lleno ({free_mb:.0f} MB libres) — saltando {slot_key}")
        return 0

    # Barajar queries para variar el contenido cada día
    shuffled_queries = queries.copy()
    random.shuffle(shuffled_queries)

    uploaded = 0
    with tempfile.TemporaryDirectory(prefix="vault_recharge_") as tmpdir:
        for query in shuffled_queries:
            if uploaded >= max_items:
                break

            log.info(f"Buscando: «{query}»")
            results = _search_youtube(query, max_items * 2, max_dur, ytdlp)
            random.shuffle(results)  # variar selección

            for item in results:
                if uploaded >= max_items:
                    break

                title = item["title"]
                if title.lower() in titles_seen:
                    log.debug(f"Duplicado, saltando: {title}")
                    continue

                log.info(f"Descargando: «{title}» ({item['duration']}s)")
                file_path = _download_item(
                    item["url"],
                    audio_only=audio_only,
                    output_dir=tmpdir,
                    ytdlp=ytdlp,
                    timeout=dl_timeout,
                )
                if file_path is None:
                    continue

                ok = _upload_to_vault(
                    vault_url=vault_url,
                    admin_token=admin_token,
                    file_path=file_path,
                    title=title,
                    artist=item.get("uploader", ""),
                    category=category,
                    channel=channel,
                    genre=genre,
                    duration_secs=item["duration"],
                    dry_run=dry_run,
                )
                if ok:
                    titles_seen.add(title.lower())
                    uploaded += 1
                    # Eliminar el archivo temporal inmediatamente para liberar espacio
                    try:
                        file_path.unlink()
                    except Exception:
                        pass

                # Pequeña pausa entre descargas para no saturar
                time.sleep(2)

    # Actualizar estado persistente (conservar últimos 500 títulos vistos)
    seen_list = list(titles_seen)[-500:]
    if "titles_seen" not in state:
        state["titles_seen"] = {}
    state["titles_seen"][seen_key] = seen_list

    log.info(f"Slot {slot_key}: {uploaded}/{max_items} items subidos")
    return uploaded


def run(args: argparse.Namespace) -> None:
    cfg        = _load_config()
    state      = _load_state()
    admin_token = _get_admin_token(cfg)
    dry_run    = args.dry_run
    force_slot = args.slot  # e.g. "cami/musica"
    vault_url = (args.vault_url or os.environ.get("VAULT_URL") or cfg.get("vault_url", "http://127.0.0.1:3000")).strip().rstrip("/")

    today = date.today().isoformat()

    if force_slot and "/" not in force_slot:
        log.error("Formato de --slot invalido. Usa CHANNEL/CATEGORY, por ejemplo: cami/musica")
        sys.exit(1)

    if not args.force and not dry_run:
        if state.get("last_run") == today:
            # Aún así correr si el vault está vacío (primer arranque o limpieza)
            if not _vault_is_empty(vault_url):
                log.info(f"Ya se ejecutó hoy ({today}) y el vault tiene contenido. Usa --force para forzar.")
                return
            log.info("Guard omitido: vault está vacío — cargando contenido inicial.")

    if not _check_yt_dlp():
        log.error(
            "yt-dlp no está instalado. Ejecuta:\n"
            "  pip install yt-dlp\n"
            "o descarga desde https://github.com/yt-dlp/yt-dlp"
        )
        sys.exit(1)

    ytdlp = _yt_dlp_cmd()

    # Verificar que el servidor está levantado
    try:
        r = requests.get(f"{vault_url}/api/health", timeout=5)
        log.info(f"RAULI-VISION health: HTTP {r.status_code}")
    except Exception as e:
        log.error(f"RAULI-VISION no responde en {vault_url}: {e}")
        log.error("Asegúrate de que el servidor está corriendo antes de ejecutar el recharge.")
        sys.exit(1)

    # Validar token admin contra el backend objetivo antes de descargar contenido.
    if not _admin_token_valid(vault_url, admin_token):
        fallback_bat = _read_bat_admin_token()
        if fallback_bat and fallback_bat != admin_token and _admin_token_valid(vault_url, fallback_bat):
            log.warning("ADMIN token rechazado por backend objetivo; usando fallback de run_espejo.bat")
            admin_token = fallback_bat
        else:
            log.error(f"ADMIN token invalido para {vault_url}. Revisa ATLAS_VISION_ADMIN_TOKEN/credenciales.")
            sys.exit(1)

    log.info(f"{'='*55}")
    log.info(f"ATLAS Vault Daily Recharge — {today}")
    log.info(f"Target vault_url: {vault_url}")
    log.info(f"Dry-run: {dry_run} | Force: {args.force}")
    log.info(f"{'='*55}")

    total_uploaded = 0
    for slot in cfg.get("slots", []):
        try:
            n = process_slot(
                slot, cfg, vault_url, admin_token, state,
                ytdlp, dry_run, force_slot
            )
            total_uploaded += n
        except Exception as e:
            log.error(f"Error en slot {slot.get('channel')}/{slot.get('category')}: {e}", exc_info=True)

    if not dry_run:
        state["last_run"] = today
        _save_state(state)

    log.info(f"{'='*55}")
    log.info(f"Recharge completado: {total_uploaded} items subidos hoy")
    log.info(f"{'='*55}")

    if force_slot and "/" in force_slot:
        channel, category = force_slot.split("/", 1)
        try:
            check = requests.get(
                f"{vault_url}/api/vault/catalog",
                params={"channel": channel, "category": category},
                timeout=10,
            )
            if check.status_code == 200:
                items = check.json().get("items", [])
                log.info(f"Verificacion catalogo [{force_slot}]: {len(items)} items visibles en app/backend objetivo")
            else:
                log.warning(f"Verificacion catalogo fallo HTTP {check.status_code}")
        except Exception as e:
            log.warning(f"No se pudo verificar catalogo final: {e}")


# ─── Entry point ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="ATLAS Vault Daily Recharge — carga automática de contenido"
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Simula sin descargar ni subir nada"
    )
    parser.add_argument(
        "--slot", default=None, metavar="CHANNEL/CATEGORY",
        help="Procesar solo un slot específico, e.g. cami/musica"
    )
    parser.add_argument(
        "--force", action="store_true",
        help="Ignorar el guard de 'ya corrió hoy'"
    )
    parser.add_argument(
        "--vault-url", default=None, metavar="URL",
        help="Override del backend objetivo, e.g. https://vision.rauliatlasapp.com"
    )
    args = parser.parse_args()
    run(args)
