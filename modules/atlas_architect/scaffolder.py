from __future__ import annotations

import re
import difflib
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

from .ans_logger import ANSBitacoraLogger
from .fs_tools import FilesystemTools
from .governance import ChangePlan, PlannedChange
from .project_indexer import ProjectIndexer


def _slug(name: str) -> str:
    s = (name or "").strip().lower()
    s = re.sub(r"[^a-z0-9_-]+", "_", s)
    s = re.sub(r"_+", "_", s).strip("_")
    return s or "app"


@dataclass
class ScaffoldResult:
    ok: bool
    project_root: str
    index_path: str
    notes: str = ""


class AppScaffolder:
    """Crea estructura completa de apps (Frontend/Backend/DB) desde cero."""

    def __init__(self, repo_root: Path, fs: FilesystemTools, ans: Optional[ANSBitacoraLogger] = None) -> None:
        self.repo_root = Path(repo_root).resolve()
        self.fs = fs
        self.ans = ans
        self.indexer = ProjectIndexer(repo_root=self.repo_root)

    def plan_inventory_app(self, org_name: str, app_name: str = "inventario", *, mode: str = "governed") -> ChangePlan:
        slug = _slug(f"{org_name}_{app_name}")
        root = (self.repo_root / "apps" / slug).resolve()
        backend = root / "backend"
        frontend = root / "frontend"
        db = root / "db"

        files: Dict[str, str] = {
            str(backend / "requirements.txt"): "\n".join(
                [
                    "fastapi>=0.129.0",
                    "uvicorn>=0.40.0",
                    "pydantic>=2.12.5",
                    "pytest>=8.0.0",
                    "httpx>=0.25.0",
                ]
            )
            + "\n",
            str(backend / "db.py"): _TEMPLATE_DB,
            str(backend / "main.py"): _template_main(slug),
            str(backend / "test_smoke.py"): _TEMPLATE_TEST,
            str(frontend / "index.html"): _template_frontend_html(slug),
            str(frontend / "app.js"): _TEMPLATE_FRONTEND_JS,
            str(root / "README.md"): _template_readme(slug),
        }

        changes: List[PlannedChange] = []
        for path, content in files.items():
            p = Path(path)
            before = p.read_text(encoding="utf-8", errors="ignore") if p.exists() else ""
            after = content
            diff = "\n".join(
                difflib.unified_diff(
                    before.splitlines(),
                    after.splitlines(),
                    fromfile=str(p).replace("\\", "/") + " (before)",
                    tofile=str(p).replace("\\", "/") + " (after)",
                    lineterm="",
                )
            )
            changes.append(
                PlannedChange(
                    kind="write_file",
                    path=path,
                    justification="Scaffold: app inventario (backend/frontend/db)",
                    content=content,
                    ops=None,
                    diff_preview=diff[:12000],
                )
            )

        # Directorios (sin diff): se crean al aplicar, pero se muestran como intención.
        for d in [str(root), str(backend), str(frontend), str(db)]:
            changes.insert(
                0,
                PlannedChange(
                    kind="write_file",
                    path=str(Path(d) / ".gitkeep"),
                    justification="Scaffold: asegurar directorio",
                    content="",
                    ops=None,
                    diff_preview="",
                ),
            )

        return ChangePlan(goal=f"Scaffold inventario {slug}", mode=mode, changes=changes)

    def scaffold_inventory_app(self, org_name: str, app_name: str = "inventario") -> ScaffoldResult:
        slug = _slug(f"{org_name}_{app_name}")
        root = (self.repo_root / "apps" / slug).resolve()
        backend = root / "backend"
        frontend = root / "frontend"
        db = root / "db"

        if self.ans:
            self.ans.log_suggestion(f"Diseñando arquitectura para nueva App: {slug} (inventario panadería).")

        # Directorios
        self.fs.create_directory(str(backend))
        self.fs.create_directory(str(frontend))
        self.fs.create_directory(str(db))

        # Backend (FastAPI + SQLite)
        self.fs.write_file(
            str(backend / "requirements.txt"),
            "\n".join(
                [
                    "fastapi>=0.129.0",
                    "uvicorn>=0.40.0",
                    "pydantic>=2.12.5",
                    "pytest>=8.0.0",
                    "httpx>=0.25.0",
                ]
            )
            + "\n",
            justification="Scaffold: dependencias backend inventario",
        )

        self.fs.write_file(
            str(backend / "db.py"),
            _TEMPLATE_DB,
            justification="Scaffold: SQLite repo inventario",
        )
        self.fs.write_file(
            str(backend / "main.py"),
            _template_main(slug),
            justification="Scaffold: FastAPI app inventario",
        )
        self.fs.write_file(
            str(backend / "test_smoke.py"),
            _TEMPLATE_TEST,
            justification="Scaffold: smoke tests inventario",
        )

        # Frontend (simple)
        self.fs.write_file(
            str(frontend / "index.html"),
            _template_frontend_html(slug),
            justification="Scaffold: frontend simple inventario",
        )
        self.fs.write_file(
            str(frontend / "app.js"),
            _TEMPLATE_FRONTEND_JS,
            justification="Scaffold: JS simple inventario",
        )

        # README
        self.fs.write_file(
            str(root / "README.md"),
            _template_readme(slug),
            justification="Scaffold: README app inventario",
        )

        # Index del proyecto creado
        idx = self.indexer.index(root)
        idx_path = self.indexer.persist(idx, name=f"{slug}_index")
        if self.ans:
            self.ans.log_suggestion(f"Scaffold completo. Index guardado en: {idx_path}")

        return ScaffoldResult(ok=True, project_root=str(root), index_path=idx_path, notes="scaffold_inventory_app")


_TEMPLATE_DB = """\
from __future__ import annotations

import sqlite3
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional


SCHEMA = \"\"\"
CREATE TABLE IF NOT EXISTS items (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  sku TEXT UNIQUE NOT NULL,
  name TEXT NOT NULL,
  qty INTEGER NOT NULL DEFAULT 0,
  unit TEXT NOT NULL DEFAULT 'u',
  updated_ts TEXT NOT NULL
);
\"\"\"


def _db_path() -> Path:
    return Path(__file__).resolve().parent.parent / "db" / "inventory.sqlite"


def _connect() -> sqlite3.Connection:
    p = _db_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(p))
    conn.execute("PRAGMA journal_mode=WAL;")
    conn.execute("PRAGMA foreign_keys=ON;")
    conn.executescript(SCHEMA)
    conn.commit()
    return conn


@dataclass(frozen=True)
class Item:
    id: int
    sku: str
    name: str
    qty: int
    unit: str
    updated_ts: str


class InventoryRepo:
    def __init__(self) -> None:
        self._conn = _connect()

    def list_items(self, limit: int = 200) -> List[Item]:
        rows = self._conn.execute(
            "SELECT id, sku, name, qty, unit, updated_ts FROM items ORDER BY name LIMIT ?",
            (int(limit),),
        ).fetchall()
        return [Item(*r) for r in rows]

    def upsert(self, sku: str, name: str, qty: int, unit: str, updated_ts: str) -> Item:
        sku = (sku or "").strip()
        if not sku:
            raise ValueError("sku requerido")
        name = (name or "").strip()
        if not name:
            raise ValueError("name requerido")
        unit = (unit or "u").strip() or "u"
        qty = int(qty or 0)
        self._conn.execute(
            \"\"\"INSERT INTO items (sku, name, qty, unit, updated_ts)
                 VALUES (?, ?, ?, ?, ?)
                 ON CONFLICT(sku) DO UPDATE SET
                   name=excluded.name, qty=excluded.qty, unit=excluded.unit, updated_ts=excluded.updated_ts
            \"\"\",
            (sku, name, qty, unit, updated_ts),
        )
        self._conn.commit()
        row = self._conn.execute(
            "SELECT id, sku, name, qty, unit, updated_ts FROM items WHERE sku=?",
            (sku,),
        ).fetchone()
        assert row
        return Item(*row)
"""


def _template_main(slug: str) -> str:
    return f"""\
from __future__ import annotations

from datetime import datetime, timezone
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field

from db import InventoryRepo

app = FastAPI(title="Inventario - {slug}")
repo = InventoryRepo()


class ItemUpsert(BaseModel):
    sku: str = Field(..., min_length=1, max_length=64)
    name: str = Field(..., min_length=1, max_length=120)
    qty: int = 0
    unit: str = "u"


@app.get("/health")
def health():
    return {{"ok": True, "service": "inventory_backend"}}


@app.get("/items")
def list_items(limit: int = 200):
    return {{"ok": True, "items": [i.__dict__ for i in repo.list_items(limit=limit)]}}


@app.post("/items/upsert")
def upsert_item(body: ItemUpsert):
    try:
        now = datetime.now(timezone.utc).isoformat()
        it = repo.upsert(body.sku, body.name, body.qty, body.unit, now)
        return {{"ok": True, "item": it.__dict__}}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
"""


_TEMPLATE_TEST = """\
from __future__ import annotations

from fastapi.testclient import TestClient


def test_health_and_upsert():
    from main import app
    c = TestClient(app)
    r = c.get("/health")
    assert r.status_code == 200
    r = c.post("/items/upsert", json={"sku": "PAN-001", "name": "Pan", "qty": 10, "unit": "u"})
    assert r.status_code == 200
    data = r.json()
    assert data["ok"] is True
    r = c.get("/items")
    assert r.status_code == 200
    assert r.json()["ok"] is True
"""


def _template_frontend_html(slug: str) -> str:
    return f"""\
<!doctype html>
<html lang="es">
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>Inventario - {slug}</title>
    <style>
      body {{ font-family: system-ui, Arial; background: #0b1220; color: #e8eefc; margin: 0; }}
      header {{ padding: 16px; background: rgba(255,255,255,0.06); backdrop-filter: blur(10px); }}
      main {{ padding: 16px; max-width: 980px; margin: 0 auto; }}
      .card {{ background: rgba(255,255,255,0.06); border: 1px solid rgba(255,255,255,0.12); border-radius: 14px; padding: 14px; margin: 12px 0; }}
      input, button {{ padding: 10px; border-radius: 10px; border: 1px solid rgba(255,255,255,0.18); background: rgba(0,0,0,0.25); color: #e8eefc; }}
      button {{ cursor: pointer; }}
      table {{ width: 100%; border-collapse: collapse; }}
      th, td {{ text-align: left; padding: 8px; border-bottom: 1px solid rgba(255,255,255,0.1); }}
      .row {{ display: flex; gap: 10px; flex-wrap: wrap; }}
      .row > * {{ flex: 1; min-width: 160px; }}
    </style>
  </head>
  <body>
    <header>
      <strong>Inventario (Rauli)</strong>
      <span style="opacity:0.75">— frontend simple</span>
    </header>
    <main>
      <div class="card">
        <div class="row">
          <input id="api" placeholder="API Base URL" value="http://127.0.0.1:8005" />
          <input id="sku" placeholder="SKU" value="PAN-001" />
          <input id="name" placeholder="Nombre" value="Pan" />
          <input id="qty" placeholder="Cantidad" value="10" />
          <input id="unit" placeholder="Unidad" value="u" />
          <button id="upsert">Guardar</button>
          <button id="refresh">Refrescar</button>
        </div>
      </div>
      <div class="card">
        <table>
          <thead>
            <tr><th>SKU</th><th>Nombre</th><th>Cantidad</th><th>Unidad</th><th>Actualizado</th></tr>
          </thead>
          <tbody id="rows"></tbody>
        </table>
      </div>
    </main>
    <script src="./app.js"></script>
  </body>
</html>
"""


_TEMPLATE_FRONTEND_JS = """\
async function apiBase() {
  return document.getElementById("api").value.trim().replace(/\\/+$/, "");
}

async function refresh() {
  const base = await apiBase();
  const r = await fetch(`${base}/items`);
  const data = await r.json();
  const rows = document.getElementById("rows");
  rows.innerHTML = "";
  (data.items || []).forEach((it) => {
    const tr = document.createElement("tr");
    tr.innerHTML = `<td>${it.sku}</td><td>${it.name}</td><td>${it.qty}</td><td>${it.unit}</td><td>${it.updated_ts}</td>`;
    rows.appendChild(tr);
  });
}

async function upsert() {
  const base = await apiBase();
  const body = {
    sku: document.getElementById("sku").value.trim(),
    name: document.getElementById("name").value.trim(),
    qty: parseInt(document.getElementById("qty").value, 10) || 0,
    unit: document.getElementById("unit").value.trim() || "u",
  };
  const r = await fetch(`${base}/items/upsert`, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify(body),
  });
  const data = await r.json();
  if (!data.ok) alert(JSON.stringify(data));
  await refresh();
}

document.getElementById("refresh").addEventListener("click", refresh);
document.getElementById("upsert").addEventListener("click", upsert);
refresh();
"""


def _template_readme(slug: str) -> str:
    return f"""\
## {slug}

App de inventario offline-first (panadería Rauli).

### Backend

Desde `backend/`:

```powershell
python -m uvicorn main:app --host 127.0.0.1 --port 8005 --reload
```

### Frontend

Abrir `frontend/index.html` en el navegador y apuntar a `http://127.0.0.1:8005`.

### Tests

Desde `backend/`:

```powershell
python -m pytest -q
```
"""

