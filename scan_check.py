
import json, urllib.request
B="http://localhost:8795"
K="atlas-quant-local"
def g(p):
    try:
        r=urllib.request.Request(f"{B}{p}",headers={"X-API-Key":K})
        with urllib.request.urlopen(r,timeout=8) as x: return json.loads(x.read())
    except Exception as e: return {"e":str(e)}
def p(path,body):
    try:
        d=json.dumps(body).encode()
        r=urllib.request.Request(f"{B}{path}",data=d,headers={"X-API-Key":K,"Content-Type":"application/json"},method="POST")
        with urllib.request.urlopen(r,timeout=8) as x: return json.loads(x.read())
    except Exception as e: return {"e":str(e)}

h=g("/health")
ss=g("/scanner/status")
sd=ss.get("data",{}) if "e" not in ss else {}
# Arrancar si detenido
if sd.get("running")==False:
    p("/scanner/control",{"action":"start"})
sr=g("/scanner/report?activity_limit=10")
srd=sr.get("data",{}) if "e" not in sr else {}
sm=srd.get("summary",{})
cands=srd.get("candidates",[])
rejs=srd.get("rejections",[])
rc={}
for r in rejs:
    reasons=r.get("reasons") or [r.get("reason","?")]
    for reason in (reasons if isinstance(reasons,list) else [reasons]):
        rc[str(reason)]=rc.get(str(reason),0)+1
top_r=sorted(rc.items(),key=lambda x:x[1],reverse=True)[:5]
rd=g("/operation/readiness")
rdd=rd.get("data",{}) if "e" not in rd else {}
jd=g("/journal/stats")
today=jd.get("data",{}).get("today",{}) if "e" not in jd else {}
out={
  "health": h.get("ok") if "e" not in h else h,
  "scanner":{
    "running":sd.get("running"),
    "cycles":sd.get("cycle_count"),
    "last_at":sd.get("last_cycle_at"),
    "ms":sd.get("last_cycle_ms"),
    "step":sd.get("current_step"),
    "symbol":sd.get("current_symbol"),
    "error":sd.get("last_error"),
  },
  "universe":{
    "total":sm.get("universe_total"),
    "batch":sm.get("batch_size"),
    "prefilter":sm.get("prefilter_selected"),
    "deep":sm.get("deep_scan_symbols"),
    "provisional":sm.get("provisional_candidates"),
    "accepted":sm.get("accepted"),
    "rejected":sm.get("rejected"),
    "threshold":sm.get("dynamic_min_selection_score"),
    "timeframes":sm.get("timeframes"),
  },
  "candidates":[{"sym":c.get("symbol"),"tf":c.get("timeframe"),"dir":c.get("direction"),"score":c.get("selection_score"),"wr":c.get("local_win_rate_pct"),"pf":c.get("local_profit_factor"),"strat":c.get("strategy_label")} for c in cands[:5]],
  "top_rejections":top_r,
  "readiness":{"ready":rdd.get("ready"),"score":rdd.get("score"),"reason":str(rdd.get("reason",""))[:60]},
  "journal_today":{"trades":today.get("total_trades"),"wins":today.get("winning_trades"),"wr":today.get("win_rate_pct"),"pnl":today.get("total_pnl")},
}
print(json.dumps(out,indent=2))
