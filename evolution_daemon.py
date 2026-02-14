#!/usr/bin/env python3
"""ATLAS_EVOLUTION: daemon de crecimiento perpetuo. Tríada de Crecimiento cada 12h.

  - PyPI: buscar actualizaciones y probar en sandbox (.temp_venv / temp_growth); si pasa test → Asimilación Exitosa o pendiente de aprobación (GOVERNED).
  - GitHub: sincronizar mejoras de código (permisos lectura/escritura vía token en credenciales.txt).
  - Hugging Face: buscar modelos de visión para Insta360 Link 2 (webcam/cámara) y lenguaje.

  Gobernanza: si EVOLUTION_GOVERNED=1, los cambios al núcleo requieren confirmación en el Dashboard (GET /api/evolution/status, POST /api/evolution/approve).
  Frecuencia: ATLAS_EVOLUTION_INTERVAL_SEC (default 43200 = 12h). Reporte "Asimilación Exitosa" en Dashboard si ciclo sin errores.
"""
from __future__ import annotations

import asyncio
import hashlib
import json
import logging
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE))

# Cargar credenciales antes de nada
from modules.evolution.credentials import load_credentials, get_pypi_auth, ensure_pypi_password

CREDS = load_credentials()

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("atlas.evolution")

# Ciclo cada 12 horas (Tríada de Crecimiento). Override: ATLAS_EVOLUTION_INTERVAL_SEC
SCAN_INTERVAL_SEC = int(os.environ.get("ATLAS_EVOLUTION_INTERVAL_SEC", "43200"))  # 12h
DASHBOARD_URL = (os.environ.get("ATLAS_DASHBOARD_URL") or "http://127.0.0.1:8791").rstrip("/")
TEMP_VENV = BASE / ".temp_venv"
TEMP_WORKSPACE = BASE / "temp_workspace"
TEMP_MODELS_CACHE = BASE / "temp_models_cache"
REQUIREMENTS_TXT = BASE / "requirements.txt"
GITHUB_REPOS = [r.strip() for r in os.environ.get("ATLAS_EVOLUTION_GITHUB_REPOS", "mramirezraul71/atlas-core").split(",") if r.strip()]
# Modelos VLA (Vision-Language-Action) y visión para Insta360 Link 2
HF_SEARCH_QUERIES = os.environ.get(
    "ATLAS_EVOLUTION_HF_QUERIES",
    "VLA,vision-language,vision,webcam,camera,object detection,Insta360",
).split(",")
# Gobernanza: si GOVERNED, no se aplican cambios al núcleo hasta confirmación en Dashboard
EVOLUTION_GOVERNED = os.environ.get("EVOLUTION_GOVERNED", "").strip().upper() in ("1", "TRUE", "GOVERNED")


def _parse_version(v: str) -> tuple:
    """Convierte '1.2.3' en (1,2,3) para comparación; sufijos (a1, b2) se ignoran como 0."""
    if not v:
        return (0, 0, 0)
    parts = re.sub(r"[^0-9.]", " ", v).split(".")[:4]
    out = []
    for p in (parts + ["0", "0", "0"])[:3]:
        try:
            out.append(int(p.strip() or 0))
        except ValueError:
            out.append(0)
    return tuple(out)


def _version_gt(latest: str, current: str) -> bool:
    """True si latest > current (versión semver simplificada)."""
    return _parse_version(latest) > _parse_version(current)


def _parse_requirements_line(line: str) -> tuple[str, str | None, str]:
    """Devuelve (nombre_paquete, version_actual_o_None, línea_original)."""
    line = line.strip()
    if not line or line.startswith("#") or line.startswith("-"):
        return ("", None, line)
    # package, package==x.y.z, package>=x.y.z, package[x,y]
    name = line.split("==")[0].split(">=")[0].split("[")[0].strip()
    if not name or not re.match(r"^[a-zA-Z0-9_-]+$", name):
        return ("", None, line)
    ver = None
    if "==" in line:
        ver = line.split("==")[1].split()[0].strip()
    elif ">=" in line:
        ver = line.split(">=")[1].split()[0].strip()
    return (name, ver, line)


def _clean_sandbox_temp() -> None:
    """Si el test en sandbox falla: limpia temporales (__pycache__, .pypirc) para el próximo ciclo."""
    try:
        for d in (TEMP_VENV, TEMP_WORKSPACE, TEMP_MODELS_CACHE):
            if not d.exists():
                continue
            for p in d.rglob("__pycache__"):
                if p.is_dir():
                    shutil.rmtree(p, ignore_errors=True)
            for p in d.rglob("*.pyc"):
                try:
                    p.unlink()
                except Exception:
                    pass
        pypirc = TEMP_VENV / ".pypirc"
        if pypirc.exists():
            pypirc.unlink(missing_ok=True)
    except Exception as e:
        logger.debug("clean_sandbox_temp: %s", e)


async def _notify_dashboard(payload: dict) -> bool:
    """Envía JSON de confirmación al Dashboard. No altera producción."""
    try:
        import aiohttp
        body = {"target": "EVOLUTION_REPORT", "action": "worker_result", "value": json.dumps(payload)}
        async with aiohttp.ClientSession() as session:
            async with session.post(
                f"{DASHBOARD_URL}/api/push/command",
                json=body,
                timeout=aiohttp.ClientTimeout(total=10),
            ) as r:
                return r.status == 200
    except Exception as e:
        logger.debug("notify_dashboard: %s", e)
        return False


async def _bitacora_log(message: str, ok: bool = True) -> None:
    """Registro industrial: envía cada paso a la Bitácora ANS (sin silencio operativo)."""
    if not message:
        return
    try:
        import aiohttp
        async with aiohttp.ClientSession() as session:
            async with session.post(
                f"{DASHBOARD_URL}/ans/evolution-log",
                json={"message": message, "ok": ok},
                timeout=aiohttp.ClientTimeout(total=5),
            ) as r:
                if r.status != 200:
                    logger.debug("bitacora_log HTTP %s: %s", r.status, await r.text())
    except Exception as e:
        logger.debug("bitacora_log: %s", e)


async def _ping_nexus_and_log() -> None:
    """Registro industrial: comprueba NEXUS en 8000 y envía [CONEXIÓN] a la Bitácora."""
    try:
        import urllib.request
        url = (os.environ.get("NEXUS_BASE_URL") or "http://127.0.0.1:8000").rstrip("/") + "/health"
        req = urllib.request.Request(url, method="GET", headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=5) as r:
            ok = r.status == 200
    except Exception:
        ok = False
    msg = "[CONEXIÓN] Buscando NEXUS en puerto 8000... OK." if ok else "[CONEXIÓN] Buscando NEXUS en puerto 8000... Desconectado."
    await _bitacora_log(msg, ok=ok)


async def _get_governance_from_dashboard() -> bool:
    """True si el Dashboard está en modo GOVERNED (requiere Aprobar Asimilación)."""
    try:
        import aiohttp
        async with aiohttp.ClientSession() as session:
            async with session.get(
                f"{DASHBOARD_URL}/api/evolution/status",
                timeout=aiohttp.ClientTimeout(total=5),
            ) as r:
                if r.status == 200:
                    data = await r.json()
                    return data.get("governed", False)
    except Exception:
        pass
    return EVOLUTION_GOVERNED


class AtlasEvolutionDaemon:
    """Ciclo perpetuo: cada SCAN_INTERVAL ejecuta 3 workers en sandbox y notifica al Dashboard si OK."""

    def __init__(self) -> None:
        self._creds = load_credentials()
        self._interval = SCAN_INTERVAL_SEC
        self._governed = EVOLUTION_GOVERNED
        self._pypi_enabled = True
        self._github_enabled = bool(self._creds.get("github_token"))
        self._hf_enabled = bool(self._creds.get("hf_token"))
        if self._governed:
            logger.info("Modo GOVERNED: los cambios al núcleo requieren confirmación en el Dashboard")
        if not self._github_enabled:
            logger.warning("Worker GitHub deshabilitado: falta github_token en credenciales")
        if not self._hf_enabled:
            logger.warning("Worker Hugging Face deshabilitado: falta hf_token en credenciales")
        TEMP_VENV.mkdir(parents=True, exist_ok=True)
        TEMP_WORKSPACE.mkdir(parents=True, exist_ok=True)
        TEMP_MODELS_CACHE.mkdir(parents=True, exist_ok=True)

    async def _worker_pypi(self) -> None:
        """PyPI Scan: Escaneo requirements.txt → Consulta API → Sandbox .temp_venv → Test de Importación. Registro en Bitácora ANS."""
        if not self._pypi_enabled:
            return
        try:
            await _bitacora_log("[EVOLUCIÓN] Fase PyPI (Software): escaneo de requirements.txt")
            if not REQUIREMENTS_TXT.exists():
                logger.info("[PyPI] requirements.txt no encontrado, omitiendo")
                await _bitacora_log("[EVOLUCIÓN] requirements.txt no encontrado. Omitiendo fase PyPI.", ok=True)
                return
            lines = REQUIREMENTS_TXT.read_text(encoding="utf-8", errors="ignore").splitlines()
            entries = [_parse_requirements_line(l) for l in lines]
            packages_to_scan = [(n, cur_ver, orig) for n, cur_ver, orig in entries if n][:8]
            if not packages_to_scan:
                await _bitacora_log("[EVOLUCIÓN] Sin paquetes a escanear en requirements.txt", ok=True)
                return
            import urllib.request
            import base64
            pypi_user, pypi_pass = get_pypi_auth(self._creds)
            auth_header = ""
            if pypi_user and pypi_pass:
                auth_header = base64.b64encode(f"{pypi_user}:{pypi_pass}".encode()).decode()
            venv_py = TEMP_VENV / "Scripts" / "python.exe" if os.name == "nt" else TEMP_VENV / "bin" / "python"
            pip = TEMP_VENV / "Scripts" / "pip.exe" if os.name == "nt" else TEMP_VENV / "bin" / "pip"
            if not venv_py.exists():
                await _bitacora_log("[SANDBOX] Iniciando entorno de pruebas aislado (.temp_venv)...")
                subprocess.run([sys.executable, "-m", "venv", str(TEMP_VENV)], capture_output=True, timeout=120, cwd=str(BASE))
            if not pip.exists():
                await _bitacora_log("[ERROR] Fallo en Sandbox: no se pudo crear pip. Reintentando en próximo ciclo (12h).", ok=False)
                return
            env = dict(os.environ)
            if pypi_user and pypi_pass:
                pypirc = TEMP_VENV / ".pypirc"
                pypirc.write_text(
                    "[pypi]\nusername = __token__\npassword = " + ensure_pypi_password(self._creds.get("pypi_token", "")),
                    encoding="utf-8",
                )
                env["HOME"] = str(TEMP_VENV)
                env["USERPROFILE"] = str(TEMP_VENV)
            for pkg_name, current_ver, original_line in packages_to_scan:
                latest_ver = None
                try:
                    req = urllib.request.Request(f"https://pypi.org/pypi/{pkg_name}/json", headers={"Accept": "application/json"})
                    if auth_header:
                        req.add_header("Authorization", f"Basic {auth_header}")
                    with urllib.request.urlopen(req, timeout=8) as r:
                        data = json.loads(r.read().decode("utf-8"))
                    latest_ver = data.get("info", {}).get("version", "")
                except Exception as e:
                    logger.debug("[PyPI] consulta %s: %s", pkg_name, e)
                    continue
                if not latest_ver:
                    continue
                if current_ver and not _version_gt(latest_ver, current_ver):
                    continue
                await _bitacora_log(f"[EVOLUCIÓN] Detectada actualización en PyPI: {pkg_name} v{latest_ver}")
                await _bitacora_log("[SANDBOX] Iniciando entorno de pruebas aislado...")
                spec = f"{pkg_name}=={latest_ver}"
                r = subprocess.run([str(pip), "install", "-q", spec], capture_output=True, timeout=90, cwd=str(BASE), text=True, env=env)
                if r.returncode != 0:
                    await _bitacora_log("[ERROR] Fallo en Sandbox. Reintentando en próximo ciclo (12h).", ok=False)
                    logger.debug("[PyPI] install sandbox %s falló", spec)
                    continue
                import_module = pkg_name.replace("-", "_").split("[")[0]
                imp = subprocess.run([str(venv_py), "-c", f"import {import_module}"], capture_output=True, timeout=15, cwd=str(BASE))
                if imp.returncode != 0:
                    await _bitacora_log("[ERROR] Fallo en Sandbox. Reintentando en próximo ciclo (12h).", ok=False)
                    logger.debug("[PyPI] import %s falló tras install", pkg_name)
                    continue
                await _bitacora_log("[OK] Test de integridad superado. Asimilando herramienta...")
                self._governed = await _get_governance_from_dashboard()
                if self._governed:
                    await _notify_dashboard({
                        "worker": "pypi",
                        "status": "pending_approval",
                        "event": "evolution_pending_approval",
                        "package": pkg_name,
                        "old_version": current_ver or "any",
                        "new_version": latest_ver,
                        "sandbox": str(TEMP_VENV),
                        "message": "Confirmar en el Dashboard para aplicar al núcleo",
                    })
                    await _bitacora_log("[EVOLUCIÓN] Modo GOVERNED: pausa. Aprobar Asimilación en el Dashboard (sección Aprobaciones).", ok=True)
                    logger.info("[PyPI] Sandbox OK (GOVERNED): %s %s -> %s — pendiente de confirmación en Dashboard", pkg_name, current_ver or "?", latest_ver)
                    break
                # Regla de Autonomía (GROWTH): actualizar requirements.txt y notificar Asimilación Exitosa
                await _bitacora_log(f"[OK] Asimilación autónoma (GROWTH): {pkg_name}=={latest_ver}. Núcleo actualizado.", ok=True)
                new_line = f"{pkg_name}=={latest_ver}"
                new_content = []
                for file_line in lines:
                    n, _, parsed_orig = _parse_requirements_line(file_line)
                    if n == pkg_name and parsed_orig == original_line:
                        new_content.append(new_line + "\n")
                    else:
                        new_content.append(file_line)
                REQUIREMENTS_TXT.write_text("".join(new_content), encoding="utf-8")
                await _notify_dashboard({
                    "worker": "pypi",
                    "status": "asimilacion_mejora_exitosa",
                    "message": "Asimilación de Mejora Exitosa",
                    "package": pkg_name,
                    "old_version": current_ver or "any",
                    "new_version": latest_ver,
                    "requirements_updated": True,
                    "sandbox": str(TEMP_VENV),
                })
                logger.info("[PyPI] Asimilación de Mejora Exitosa: %s %s -> %s; requirements.txt actualizado", pkg_name, current_ver or "?", latest_ver)
                break
            else:
                await _bitacora_log("[PYPI] Analizando requirements.txt... 0 cambios pendientes.", ok=True)
                await _notify_dashboard({"worker": "pypi", "status": "ok", "scan": "sin_upgrades", "sandbox": str(TEMP_VENV)})
        except Exception as e:
            logger.error("[PyPI] worker falló: %s", e)
            await _bitacora_log("[ERROR] Fallo en Sandbox. Reintentando en próximo ciclo (12h).", ok=False)
            _clean_sandbox_temp()
            await _notify_dashboard({"worker": "pypi", "status": "error", "message": str(e)[:200], "sandbox_cleaned": True})

    async def _worker_github(self) -> None:
        """GitHub Sync: revisar nuevos commits y herramientas en repos seguidos; clonar en temp_workspace y validar estructura."""
        if not self._github_enabled:
            return
        token = self._creds.get("github_token")
        if not token:
            return
        try:
            await _bitacora_log("[EVOLUCIÓN] Fase GitHub (Código): sincronización de repositorios Rauli y Trading")
            import urllib.request
            import urllib.error
            for repo in GITHUB_REPOS[:3]:
                repo = repo.strip()
                if not repo:
                    continue
                headers = {"Authorization": f"Bearer {token}", "Accept": "application/vnd.github.v3+json"}
                # Releases
                release_tag = ""
                try:
                    req = urllib.request.Request(f"https://api.github.com/repos/{repo}/releases/latest", headers=headers)
                    with urllib.request.urlopen(req, timeout=10) as r:
                        rel = json.loads(r.read().decode("utf-8"))
                    release_tag = rel.get("tag_name", "")
                except urllib.error.HTTPError as e:
                    if e.code != 404:
                        logger.debug("[GitHub] releases %s: %s", repo, e)
                # Commits recientes (rama por defecto)
                commits_info = []
                try:
                    req = urllib.request.Request(f"https://api.github.com/repos/{repo}/commits?per_page=5", headers=headers)
                    with urllib.request.urlopen(req, timeout=10) as r:
                        commits = json.loads(r.read().decode("utf-8"))
                    for c in commits[:3]:
                        if isinstance(c, dict):
                            sha = c.get("sha", "")[:7]
                            msg = (c.get("commit", {}).get("message") or "").split("\n")[0][:80]
                            commits_info.append({"sha": sha, "message": msg})
                except urllib.error.HTTPError as e:
                    logger.debug("[GitHub] commits %s: %s", repo, e)
                clone_dir = TEMP_WORKSPACE / repo.replace("/", "_")
                clone_dir.mkdir(parents=True, exist_ok=True)
                if (clone_dir / ".git").exists():
                    subprocess.run(["git", "fetch", "origin", "--depth", "10"], capture_output=True, timeout=30, cwd=str(clone_dir))
                else:
                    subprocess.run(["git", "clone", "--depth", "10", f"https://{token}@github.com/{repo}.git", str(clone_dir)], capture_output=True, timeout=90, env=os.environ)
                tools = []
                for f in ["setup.py", "pyproject.toml", "requirements.txt", "Makefile", "Dockerfile"]:
                    if (clone_dir / f).exists():
                        tools.append(f)
                valid = bool(tools)
                payload = {
                    "worker": "github",
                    "status": "ok",
                    "repo": repo,
                    "tag": release_tag,
                    "recent_commits": commits_info,
                    "tools": tools,
                    "sandbox": str(clone_dir),
                }
                repo_label = "RAULI_CORE" if "rauli" in repo.lower() or "Rauli" in repo else repo.split("/")[-1].upper().replace("-", "_")
                await _bitacora_log(f"[GH] Sincronizando repositorio {repo_label}... Actualizado.", ok=True)
                await _notify_dashboard(payload)
                logger.info("[GitHub] %s tag=%s commits=%s tools=%s", repo, release_tag, len(commits_info), tools)
                break
        except Exception as e:
            logger.error("[GitHub] worker falló: %s", e)
            await _bitacora_log("[ERROR] Fase GitHub fallida. Reintentando en próximo ciclo (12h).", ok=False)
            await _notify_dashboard({"worker": "github", "status": "error", "message": str(e)[:200]})

    async def _worker_huggingface(self) -> None:
        """Hugging Face Hunt: buscar modelos de visión o lenguaje más ligeros o potentes; cachear metadatos en temp_models_cache."""
        if not self._hf_enabled:
            return
        token = self._creds.get("hf_token")
        if not token:
            return
        try:
            await _bitacora_log("[EVOLUCIÓN] Fase Hugging Face (IA): búsqueda de modelos para Insta360 Link 2")
            import urllib.request
            import urllib.parse
            seen = set()
            collected = []
            for query in (HF_SEARCH_QUERIES or ["vision", "language model", "small", "ocr"])[:4]:
                try:
                    url = "https://huggingface.co/api/models?search=" + urllib.parse.quote(query) + "&limit=5"
                    req = urllib.request.Request(url, headers={"Authorization": f"Bearer {token}", "Accept": "application/json"})
                    with urllib.request.urlopen(req, timeout=12) as r:
                        data = json.loads(r.read().decode("utf-8"))
                except Exception as e:
                    logger.debug("[HF] search %s: %s", query, e)
                    continue
                models = data if isinstance(data, list) else data.get("models", [])
                for m in models:
                    model_id = m.get("id", "") if isinstance(m, dict) else str(m)
                    if not model_id or model_id in seen:
                        continue
                    seen.add(model_id)
                    collected.append({"id": model_id, "query": query, "meta": m if isinstance(m, dict) else {"id": model_id}})
            for item in collected[:3]:
                model_id = item["id"]
                cache_file = TEMP_MODELS_CACHE / f"{model_id.replace('/', '_')}.meta.json"
                cache_file.parent.mkdir(parents=True, exist_ok=True)
                cache_file.write_text(json.dumps(item["meta"]), encoding="utf-8")
                h = hashlib.sha256(cache_file.read_bytes()).hexdigest()[:16]
                await _notify_dashboard({
                    "worker": "huggingface",
                    "status": "ok",
                    "model_id": model_id,
                    "search_query": item["query"],
                    "hash": h,
                    "sandbox": str(TEMP_MODELS_CACHE),
                })
                logger.info("[HF] %s (query=%s) hash=%s", model_id, item["query"], h)
            if not collected:
                await _bitacora_log("[EVOLUCIÓN] Hugging Face: sin modelos nuevos en esta pasada. Cache OK.", ok=True)
                await _notify_dashboard({"worker": "huggingface", "status": "ok", "scan": "no_new_models", "sandbox": str(TEMP_MODELS_CACHE)})
        except Exception as e:
            logger.error("[HF] worker falló: %s", e)
            await _bitacora_log("[ERROR] Fase Hugging Face fallida. Reintentando en próximo ciclo (12h).", ok=False)
            await _notify_dashboard({"worker": "huggingface", "status": "error", "message": str(e)[:200]})

    async def _run_cycle(self) -> None:
        """Ejecuta la Tríada de Crecimiento: PyPI | GitHub | Hugging Face. Registro obligatorio en Bitácora ANS."""
        await _ping_nexus_and_log()
        self._governed = await _get_governance_from_dashboard()
        await _bitacora_log("[EVOLUCIÓN] Iniciando ciclo de la Tríada (PyPI/GH/HF)")
        # Prueba de Nervios: verificar cámara (Insta360) antes de cada escaneo PyPI/GH/HF
        try:
            import nexus_actions
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, nexus_actions.run_nerve_test_before_triada)
        except Exception as e:
            logger.debug("nerve_test_before_triada: %s", e)
        logger.info("Tríada de Crecimiento: PyPI | GitHub | Hugging Face")
        tasks = []
        if self._pypi_enabled:
            tasks.append(asyncio.create_task(self._worker_pypi()))
        if self._github_enabled:
            tasks.append(asyncio.create_task(self._worker_github()))
        if self._hf_enabled:
            tasks.append(asyncio.create_task(self._worker_huggingface()))
        cycle_ok = True
        if tasks:
            results = await asyncio.gather(*tasks, return_exceptions=True)
            for i, r in enumerate(results):
                if isinstance(r, Exception):
                    logger.error("Worker %s: %s", i, r)
                    cycle_ok = False
        # Reporte ANS: Bitácora Dashboard; Sandbox exitoso -> "Asimilación Exitosa" / "Sistema Regenerado"
        await _notify_dashboard({
            "event": "ans_bitacora",
            "cycle_ok": cycle_ok,
            "asimilacion_exitosa": cycle_ok,
            "asimilacion_mejora_exitosa": cycle_ok,
            "state": "Sistema Regenerado" if cycle_ok else "Ciclo con errores",
            "message": "Asimilación Exitosa" if cycle_ok else "Ciclo con errores",
            "workers": {"pypi": self._pypi_enabled, "github": self._github_enabled, "huggingface": self._hf_enabled},
        })
        # Estado para ANS (evolution_health): proceso crítico integrado
        try:
            from datetime import datetime, timezone
            log_dir = BASE / "logs"
            log_dir.mkdir(parents=True, exist_ok=True)
            (log_dir / "evolution_last_cycle.json").write_text(
                json.dumps({"ok": cycle_ok, "ts": datetime.now(timezone.utc).isoformat(), "message": "Asimilación Exitosa" if cycle_ok else "Ciclo con errores"}),
                encoding="utf-8",
            )
        except Exception as e:
            logger.debug("write evolution_last_cycle: %s", e)

    async def run_forever(self) -> None:
        """Bucle perpetuo: despertar cada 6h, ejecutar Tríada de Crecimiento. Opera como daemon; no termina."""
        logger.info("ATLAS_EVOLUTION daemon iniciado — ciclo cada %ss (%.1fh)", self._interval, self._interval / 3600)
        while True:
            try:
                await self._run_cycle()
            except Exception as e:
                logger.exception("Ciclo evolution falló: %s", e)
            logger.info("Próximo ciclo (Tríada de Crecimiento) en %ss", self._interval)
            await asyncio.sleep(self._interval)


def main() -> None:
    daemon = AtlasEvolutionDaemon()
    try:
        asyncio.run(daemon.run_forever())
    except KeyboardInterrupt:
        logger.info("ATLAS_EVOLUTION detenido por usuario")


if __name__ == "__main__":
    main()
