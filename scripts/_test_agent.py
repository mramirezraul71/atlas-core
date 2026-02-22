import json, requests, sys, time

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

base = "http://127.0.0.1:8791"

# Test health first
print("=== Health check ===")
r = requests.get(f"{base}/health", timeout=5)
print(f"Health: {r.status_code} -> score={r.json().get('score')}")

# Test agent engine with SSE
print("\n=== Agent Engine (tool calling) ===")
body = {"message": "Lee el archivo atlas_adapter/agent_engine.py y dime cuantas herramientas tiene definidas"}

t0 = time.time()
r = requests.post(f"{base}/agent/chat", json=body, stream=True, timeout=120)
print(f"Response status: {r.status_code}")

events = []
for line in r.iter_lines(decode_unicode=True):
    if not line:
        continue
    if line.startswith("event: "):
        evt_type = line[7:].strip()
    elif line.startswith("data: "):
        try:
            data = json.loads(line[6:])
            events.append({"event": evt_type, "data": data})
            if evt_type == "thinking":
                print(f"  [thinking] {data.get('message', '')[:80]}")
            elif evt_type == "tool_call":
                print(f"  [tool] {data['name']}({json.dumps(data.get('input', {}), ensure_ascii=False)[:80]})")
            elif evt_type == "tool_result":
                ok = data.get("ok", True)
                print(f"  [result] {'OK' if ok else 'FAIL'} {data['name']} ({data.get('ms', 0)}ms) -> {data.get('output', '')[:80]}")
            elif evt_type == "text":
                print(f"  [RESPONSE] {data.get('content', '')[:200]}")
            elif evt_type == "done":
                elapsed = time.time() - t0
                print(f"  [DONE] {data.get('iterations', 0)} iterations, {len(data.get('tools_used', []))} tools, model={data.get('model', '?')}, {elapsed:.1f}s")
            elif evt_type == "error":
                print(f"  [ERROR] {data.get('message', '')}")
        except Exception as e:
            print(f"  [parse error] {e}: {line[:100]}")

print(f"\nTotal events: {len(events)}")
print(f"Total time: {time.time() - t0:.1f}s")
