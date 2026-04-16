import json, urllib.request

# Leer state actual
MCP = "https://activities-person-off-discussions.trycloudflare.com/execute"
TOKEN = "atlas_mcp_2026"

def mcp_post(payload):
    data = json.dumps(payload).encode()
    req = urllib.request.Request(MCP, data=data, headers={"X-Token": TOKEN, "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=15) as resp:
        return json.loads(resp.read())

state_path = r"C:\ATLAS_PUSH\atlas_code_quant\data\operation\operation_center_state.json"

r = mcp_post({"action": "read_file", "params": {"path": state_path}})
state = json.loads(r["data"]["content"])

print("ANTES - var_monitor_enabled:", state.get("var_monitor_enabled"))
print("ANTES - auton_mode:", state.get("auton_mode"))

# Modificar: desactivar var_monitor para eliminar VaR block
state["var_monitor_enabled"] = False
state["fail_safe_active"] = False
state["fail_safe_reason"] = None
state["auton_mode"] = "paper_autonomous"

# Escribir de vuelta
new_content = json.dumps(state, indent=2)
r2 = mcp_post({"action": "edit_file", "params": {"path": state_path, "content": new_content}})
print("Write result:", r2.get("ok"), r2.get("data", {}).get("bytes"))

# Verificar
r3 = mcp_post({"action": "read_file", "params": {"path": state_path}})
state2 = json.loads(r3["data"]["content"])
print("DESPUES - var_monitor_enabled:", state2.get("var_monitor_enabled"))
print("DESPUES - auton_mode:", state2.get("auton_mode"))

