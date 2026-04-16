import json, urllib.request

MCP = "https://activities-person-off-discussions.trycloudflare.com/execute"
TOKEN = "atlas_mcp_2026"

def mcp_post(payload):
    data = json.dumps(payload).encode()
    req = urllib.request.Request(MCP, data=data, headers={"X-Token": TOKEN, "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=15) as resp:
        return json.loads(resp.read())

path = r"C:\ATLAS_PUSH\atlas_code_quant\journal\service.py"

# Leer contenido
r = mcp_post({"action": "read_file", "params": {"path": path}})
content = r["data"]["content"]

OLD = "        strategy_id, signature = _stable_strategy_id(strategy)\n        incoming_strategy_type = str(strategy.get(\"strategy_type\") or \"unknown\")"

NEW = (
    "        strategy_id, signature = _stable_strategy_id(strategy)\n"
    "        # PATCH: skip sandbox-restricted symbols (OS, AA) — phantom prevention\n"
    "        _SYNC_BLOCKED: frozenset = frozenset({\"OS\", \"AA\"})\n"
    "        if str(strategy.get(\"underlying\") or \"\") in _SYNC_BLOCKED:\n"
    "            return strategy_id, False, True  # skip, not created, skipped=True\n"
    "        incoming_strategy_type = str(strategy.get(\"strategy_type\") or \"unknown\")"
)

if OLD in content:
    new_content = content.replace(OLD, NEW, 1)
    r2 = mcp_post({"action": "edit_file", "params": {"path": path, "content": new_content}})
    print("PATCH APLICADO:", r2.get("ok"), r2.get("data", {}).get("bytes"))
    # Verificar
    r3 = mcp_post({"action": "read_file", "params": {"path": path}})
    if "_SYNC_BLOCKED" in r3["data"]["content"]:
        print("VERIFICADO: patch en disco OK")
    else:
        print("ERROR: patch no encontrado en disco")
else:
    print("ERROR: texto a reemplazar no encontrado")
    # Mostrar contexto real
    idx = content.find("_stable_strategy_id")
    print("Contexto real:", repr(content[idx:idx+200]))

