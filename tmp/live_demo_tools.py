import json, urllib.request
base='http://127.0.0.1:8791'

def post(path,p,timeout=90):
    d=json.dumps(p).encode('utf-8')
    req=urllib.request.Request(base+path,data=d,headers={'Content-Type':'application/json'},method='POST')
    with urllib.request.urlopen(req,timeout=timeout) as r:
        return json.loads(r.read().decode('utf-8','replace'))

out={}
out['hands']=post('/api/workspace/terminal/execute', {'command':'python -c "print(\"ATLAS_HANDS_OK\")"','timeout_sec':30})
out['open']=post('/api/workspace/navigate', {'action':'open_url','payload':{'url':'http://127.0.0.1:8791/ui','driver':'digital','show_browser':True}}, timeout=120)
out['text']=post('/api/workspace/navigate', {'action':'extract_text','payload':{'driver':'digital'}}, timeout=120)
out['shot']=post('/api/workspace/navigate', {'action':'screenshot','payload':{'driver':'digital','path':'C:/ATLAS_PUSH/snapshots/digital_feet/live_watch.png'}}, timeout=120)
out['close']=post('/api/workspace/navigate', {'action':'close','payload':{'driver':'digital'}}, timeout=60)
print(json.dumps(out,ensure_ascii=False,indent=2))
