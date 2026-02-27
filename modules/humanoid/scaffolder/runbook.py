"""Generate RUNBOOK.md with execution steps and smoke tests."""
from __future__ import annotations

from typing import Any, Dict, List


def runbook_fastapi(name: str, base_path: str) -> str:
    return f"""# RUNBOOK: {name}

## Setup
```bash
cd {base_path}
python -m venv .venv
.venv\\Scripts\\activate   # Windows
pip install -r requirements.txt
```

## Run
```bash
uvicorn main:app --reload --host 127.0.0.1 --port 8000
```

## Smoke tests
- GET http://127.0.0.1:8000/ -> {{"ok": true}}
- GET http://127.0.0.1:8000/health -> {{"ok": true}}
"""


def runbook_flutter(name: str, base_path: str) -> str:
    return f"""# RUNBOOK: {name}

## Prerequisites
- Flutter SDK installed

## Setup & Run
```bash
cd {base_path}
flutter pub get
flutter run
```

## Smoke tests
- App launches without error
- Home screen shows "Hello"
"""


def runbook_node(name: str, base_path: str) -> str:
    return f"""# RUNBOOK: {name}

## Setup & Run
```bash
cd {base_path}
npm install
npm start
```

## Smoke tests
- GET http://127.0.0.1:3000/ -> {{"ok": true, "app": "{name}"}}
"""


def generate_runbook(app_type: str, name: str, base_path: str) -> str:
    t = (app_type or "fastapi").lower().strip()
    if t in ("flutter",):
        return runbook_flutter(name, base_path)
    if t in ("node", "pwa"):
        return runbook_node(name, base_path)
    return runbook_fastapi(name, base_path)
