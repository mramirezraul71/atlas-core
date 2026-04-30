import os,sys
sys.stdout.reconfigure(encoding='utf-8')
base=r'C:\ATLAS_PUSH\atlas_code_quant'
py_files=[]
total_lines=0
modules=set()
for root,dirs,files in os.walk(base):
    if '__pycache__' in root: continue
    for f in files:
        if f.endswith('.py'):
            p=os.path.join(root,f)
            py_files.append(p)
            rel=root.replace(base,'').strip(os.sep)
            top=rel.split(os.sep)[0] if rel else 'root'
            modules.add(top)
            try:
                lines=len(open(p,encoding='utf-8',errors='ignore').readlines())
                total_lines+=lines
            except: pass
print(f'Total archivos .py: {len(py_files)}')
print(f'Total lineas de codigo: {total_lines:,}')
print(f'Modulos principales ({len(modules)}): {sorted(modules)}')
# Modulo mas grande
by_size=sorted(py_files,key=lambda x: os.path.getsize(x),reverse=True)[:5]
print('Top 5 archivos mas grandes:')
for f in by_size:
    kb=os.path.getsize(f)//1024
    print(f'  {kb}KB {os.path.basename(f)}')
