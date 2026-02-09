import os
import re
import json
from datetime import datetime
from pathlib import Path

# =========================
# Paths / Config
# =========================
def _env(name: str, default: str) -> str:
    v = os.getenv(name)
    return v.strip() if v and v.strip() else default

ATLAS_ROOT = Path(_env("ATLAS_ROOT", r"C:\ATLAS"))
VAULT_DIR  = Path(_env("ATLAS_VAULT_DIR", str(ATLAS_ROOT / "ATLAS_VAULT")))
NOTES_DIR  = Path(_env("ATLAS_NOTES_DIR", str(VAULT_DIR / "NOTES")))
LOGS_DIR   = Path(_env("ATLAS_LOGS_DIR", str(ATLAS_ROOT / "logs")))
SNAPS_DIR  = Path(_env("ATLAS_SNAPS_DIR", str(ATLAS_ROOT / "snapshots")))

LOG_FILE   = Path(_env("ATLAS_LOG_FILE", str(LOGS_DIR / "atlas.log")))

def _ensure_dirs():
    NOTES_DIR.mkdir(parents=True, exist_ok=True)
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    SNAPS_DIR.mkdir(parents=True, exist_ok=True)

def _now_stamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")

def _safe_name(s: str) -> str:
    s = s.strip().replace('"', "").replace("'", "")
    s = re.sub(r"[^\w\- áéíóúÁÉÍÓÚñÑ]", "", s)
    s = re.sub(r"\s+", " ", s).strip()
    if not s:
        s = "nota"
    return s

def _note_path(title: str) -> Path:
    title = _safe_name(title)
    # archivo amigable (mantiene espacios). Si prefieres guiones, cambia aquí.
    return NOTES_DIR / f"{title}.md"

def _write_log(line: str):
    _ensure_dirs()
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(f"[{ts}] {line}\n")

# =========================
# Core Actions
# =========================
def status() -> str:
    _ensure_dirs()
    return f"ATLAS: OK | logs={LOG_FILE} | snapshots={SNAPS_DIR}"

def doctor() -> str:
    _ensure_dirs()
    parts = []
    parts.append(("VAULT", VAULT_DIR))
    parts.append(("NOTES", NOTES_DIR))
    parts.append(("LOGS", LOGS_DIR))
    parts.append(("SNAPS", SNAPS_DIR))

    ok = True
    lines = ["ATLAS DOCTOR:"]
    for name, p in parts:
        try:
            p.mkdir(parents=True, exist_ok=True)
            test = p / ".atlas_write_test"
            with open(test, "w", encoding="utf-8") as f:
                f.write("ok")
            test.unlink(missing_ok=True)
            lines.append(f"- {name}: OK")
        except Exception as e:
            ok = False
            lines.append(f"- {name}: FAIL ({e})")

    if ok:
        _write_log("doctor: OK")
    else:
        _write_log("doctor: FAIL")

    return "\n".join(lines)

def modules_report() -> str:
    return (
        " Módulos activos:\n"
        "- Telegram Bot\n"
        "- Command Router\n"
        "- Notes (Vault)\n"
        "- Snapshots\n"
        "- Logs\n"
        "- Doctor\n"
        "- Inbox\n"
        "\nComandos:\n"
        "/status\n"
        "/doctor\n"
        "/modules\n"
        "/snapshot etiqueta\n"
        "/note create Titulo\n"
        "/note append Titulo | texto\n"
        "/note view Titulo\n"
    )

def note_create(title: str) -> str:
    _ensure_dirs()
    p = _note_path(title)
    if not p.exists():
        with open(p, "w", encoding="utf-8") as f:
            f.write(f"# {title.strip()}\n\n")
    _write_log(f"note_create: {p}")
    return f"Nota OK\n{p}"

def note_append(title: str, text: str) -> str:
    _ensure_dirs()
    p = _note_path(title)
    if not p.exists():
        with open(p, "w", encoding="utf-8") as f:
            f.write(f"# {title.strip()}\n\n")
    with open(p, "a", encoding="utf-8") as f:
        f.write(text.rstrip() + "\n")
    _write_log(f"note_append: {p}")
    return f"Nota OK\n{p}"

def note_view(title: str, limit: int = 3500) -> str:
    _ensure_dirs()
    p = _note_path(title)
    if not p.exists():
        return f" No existe la nota: {p}\nUsa: /note create {title}"
    content = p.read_text(encoding="utf-8", errors="ignore")
    content = content.strip()
    if len(content) > limit:
        content = content[:limit] + "\n\n(cortado)"
    return f" {p.name}\n\n{content}"

def inbox(text: str) -> str:
    # Guarda cualquier cosa como "bandeja"
    _ensure_dirs()
    p = NOTES_DIR / "Inbox.md"
    with open(p, "a", encoding="utf-8") as f:
        f.write(text.rstrip() + "\n")
    _write_log("inbox: +1")
    return f"Nota OK\n{p}"

def snapshot(label: str = "snapshot") -> str:
    _ensure_dirs()
    label = _safe_name(label).replace(" ", "-")
    snap_name = f"{_now_stamp()}_{label}"
    snap_dir = SNAPS_DIR / snap_name
    snap_dir.mkdir(parents=True, exist_ok=True)

    meta = {
        "name": snap_name,
        "created_at": datetime.now().isoformat(),
        "vault_dir": str(VAULT_DIR),
        "notes_dir": str(NOTES_DIR),
        "logs_file": str(LOG_FILE),
    }
    (snap_dir / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")

    _write_log(f"snapshot: {snap_dir}")
    return f"Snapshot creado: {snap_dir}"

# =========================
# Text Router (Spanish friendly)
# =========================
def handle(text: str) -> str:
    if not text:
        return "ATLAS: vacío."

    t = text.strip()

    # ----- Telegram style commands -----
    low = t.lower()

    if low.startswith("/status"):
        return status()
    if low.startswith("/doctor"):
        return doctor()
    if low.startswith("/modules"):
        return modules_report()
    if low.startswith("/snapshot"):
        parts = t.split(maxsplit=1)
        label = parts[1].strip() if len(parts) > 1 else "snapshot"
        return snapshot(label)

    # Notes commands:
    # /note create Titulo
    # /note append Titulo | texto
    # /note view Titulo
    if low.startswith("/note"):
        rest = t[5:].strip()
        if not rest:
            return "Uso:\n/note create Titulo\n/note append Titulo | texto\n/note view Titulo"
        if rest.lower().startswith("create"):
            title = rest[6:].strip()
            return note_create(title or "Nota")
        if rest.lower().startswith("append"):
            payload = rest[6:].strip()
            if "|" in payload:
                title, msg = payload.split("|", 1)
                return note_append(title.strip(), msg.strip())
            return "Uso: /note append Titulo | texto"
        if rest.lower().startswith("view"):
            title = rest[4:].strip()
            return note_view(title or "Inbox")
        return "Uso:\n/note create Titulo\n/note append Titulo | texto\n/note view Titulo"

    # ----- Natural Spanish (your current way) -----
    # "Atlas, crea una nota llamada X"
    m = re.search(r"crea\s+una\s+nota\s+(llamada|con\s+el\s+t[ií]tulo)\s+(.+)$", low, re.IGNORECASE)
    if m:
        title = t.split(m.group(1), 1)[-1].strip()
        return note_create(title)

    # "Atlas, agrega a la nota X que ...."
    m2 = re.search(r"agrega\s+a\s+la\s+nota\s+(.+?)\s+que\s+(.+)$", t, re.IGNORECASE)
    if m2:
        title = m2.group(1).strip().strip('"').strip("'")
        msg = m2.group(2).strip()
        return note_append(title, msg)

    # "Atlas, crea un snapshot llamado X"
    m3 = re.search(r"crea\s+un\s+snapshot\s+llamado\s+(.+)$", t, re.IGNORECASE)
    if m3:
        label = m3.group(1).strip()
        return snapshot(label)

    # "Atlas, dime qué módulos tienes activos"
    if "módulos" in low and ("activos" in low or "tienes" in low):
        return modules_report()

    # fallback to inbox
    return inbox(t)
