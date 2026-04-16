import json
import sqlite3
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


HOST = "0.0.0.0"
PORT = 8799
AUTH_TOKEN = "atlas_mcp_2026"
ROOT = Path(r"C:\ATLAS_PUSH")
DEFAULT_DB = Path(r"C:\ATLAS_PUSH\atlas_code_quant\data\trading_journal.sqlite3")
STATE_PATH = Path(r"C:\ATLAS_PUSH\atlas_code_quant\data\operation\operation_center_state.json")
LOG_PATH = Path(r"C:\ATLAS_PUSH\atlas_code_quant\logs\quant_brain_bridge.jsonl")
TAIL_LIMIT = 3000


def _tail_text(value, limit=TAIL_LIMIT):
    text = "" if value is None else str(value)
    return text[-limit:]


def _read_text(path):
    return Path(path).read_text(encoding="utf-8", errors="ignore")


def _write_text(path, content):
    file_path = Path(path)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    file_path.write_text(content, encoding="utf-8")
    return {"path": str(file_path), "bytes": len(content.encode("utf-8"))}


def _run_script(file_path):
    target = Path(file_path)
    completed = subprocess.run(
        [sys.executable, str(target)],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        timeout=300,
    )
    return {
        "file": str(target),
        "cwd": str(ROOT),
        "returncode": completed.returncode,
        "stdout": _tail_text(completed.stdout),
        "stderr": _tail_text(completed.stderr),
    }


def _query_db(sql, db_path=None):
    database = Path(db_path) if db_path else DEFAULT_DB
    with sqlite3.connect(str(database)) as connection:
        connection.row_factory = sqlite3.Row
        cursor = connection.cursor()
        cursor.execute(sql)
        first_token = sql.lstrip().split(None, 1)[0].lower() if sql.strip() else ""
        if first_token in {"select", "pragma", "with", "explain"}:
            rows = [dict(row) for row in cursor.fetchall()]
            return {
                "db": str(database),
                "row_count": len(rows),
                "rows": rows,
            }
        connection.commit()
        return {
            "db": str(database),
            "rowcount": cursor.rowcount,
            "lastrowid": cursor.lastrowid,
        }


def _get_state():
    content = _read_text(STATE_PATH)
    try:
        data = json.loads(content)
    except json.JSONDecodeError:
        data = {"raw": content}
    return {"path": str(STATE_PATH), "state": data}


def _get_logs(n=50):
    total = max(int(n), 1)
    lines = _read_text(LOG_PATH).splitlines()
    return {
        "path": str(LOG_PATH),
        "count": min(total, len(lines)),
        "lines": lines[-total:],
    }


def execute_action(action, params):
    if action == "read_file":
        path = params["path"]
        return {"path": str(path), "content": _read_text(path)}
    if action == "edit_file":
        return _write_text(params["path"], params["content"])
    if action == "run_script":
        return _run_script(params["file"])
    if action == "query_db":
        return _query_db(params["sql"], params.get("db"))
    if action == "get_state":
        return _get_state()
    if action == "get_logs":
        return _get_logs(params.get("n", 50))
    if action == "purge_journal":
        db_path = params.get("db", str(DEFAULT_DB))
        sql = params.get("sql", "")
        if not sql:
            return {"error": "sql required"}
        import sqlite3
        conn = sqlite3.connect(db_path, timeout=15)
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA busy_timeout=10000")
        cur = conn.cursor()
        cur.execute("SELECT COUNT(*) FROM trading_journal")
        before = cur.fetchone()[0]
        cur.execute(sql)
        deleted = cur.rowcount
        conn.commit()
        cur.execute("SELECT COUNT(*) FROM trading_journal")
        after = cur.fetchone()[0]
        cur.execute("SELECT COUNT(*) FROM trading_journal WHERE entry_price < 0")
        neg = cur.fetchone()[0]
        conn.close()
        return {"before": before, "deleted": deleted, "after": after, "neg_prices": neg}
    raise ValueError(f"Unsupported action: {action}")


class AtlasMCPHandler(BaseHTTPRequestHandler):
    server_version = "AtlasMCP/1.0"

    def log_message(self, format, *args):
        return

    def _send_json(self, status_code, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path != "/execute":
            self._send_json(404, {"ok": False, "error": "Not Found"})
            return
        if self.headers.get("X-Token") != AUTH_TOKEN:
            self._send_json(401, {"ok": False, "error": "Unauthorized"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            raw_body = self.rfile.read(length).decode("utf-8")
            payload = json.loads(raw_body or "{}")
            action = payload["action"]
            # fix: tolerar sql/db tanto en params:{} como en payload raíz
            params = {**payload, **payload.get("params", {})}
            result = execute_action(action, params)
            self._send_json(200, {"ok": True, "action": action, "data": result})
        except Exception as exc:
            self._send_json(500, {"ok": False, "error": str(exc)})


def main():
    server = ThreadingHTTPServer((HOST, PORT), AtlasMCPHandler)
    print(f"ATLAS MCP server escuchando en http://{HOST}:{PORT}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
