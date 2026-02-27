
import time, multiprocessing, os
import cv2

def probe_one(i, out):
 backends=[]
 if hasattr(cv2,'CAP_DSHOW'): backends.append(cv2.CAP_DSHOW)
 if hasattr(cv2,'CAP_MSMF'): backends.append(cv2.CAP_MSMF)
 backends.append(cv2.CAP_ANY)
 for b in backends:
  try:
   cap=cv2.VideoCapture(i,b)
   if not cap.isOpened():
    try: cap.release()
    except: pass
    continue
   t=time.perf_counter(); ok,frame=cap.read(); dt=int((time.perf_counter()-t)*1000)
   try: cap.release()
   except: pass
   if ok and frame is not None:
    out.put((i, True, dt, str(b)))
    return
  except Exception:
   pass
 out.put((i, False, -1, ''))

def main():
 q=multiprocessing.Queue()
 for i in range(0,4):
  p=multiprocessing.Process(target=probe_one,args=(i,q))
  p.daemon=True
  p.start(); p.join(0.9)
  if p.is_alive():
   try: p.terminate()
   except: pass
   p.join(0.2)
   print(i, 'HUNG')
   continue
  try:
   r=q.get_nowait(); print(r)
  except Exception:
   print(i,'NORESULT')
if __name__=='__main__':
 main()
