"""Isolated study IO and separate, enclosing-operation resource accounting."""
from pathlib import Path
import os, sys, json, time, hashlib
os.environ.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
D=Path(__file__).resolve().parent
G=D.parent; H=G.parent.parent.parent; V3=H.parent
ALLOWANCE=10800.0
def read(p): return json.loads(Path(p).read_text(encoding='utf-8'))
def write(p,v):
 p=Path(p); assert p.resolve().is_relative_to(D)
 p.parent.mkdir(parents=True,exist_ok=True)
 p.write_text(json.dumps(v,sort_keys=True,indent=2,allow_nan=False)+'\n',encoding='utf-8')
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(8<<20),b''):h.update(b)
 return h.hexdigest()
def ah(a):
 import numpy as np
 return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
def local(p):
 s=str(p).replace('\\','/')
 if s.startswith('//?/'):s=s[4:]
 if os.name=='nt' and s.startswith('/mnt/c/'):s='C:/'+s[7:]
 if os.name!='nt' and s[1:2]==':':s='/mnt/'+s[0].lower()+s[2:]
 return Path(s)
def charge(name,seconds,**extra):
 p=D/'OPERATIONS.json'; rows=read(p) if p.exists() else []
 assert name not in [r['name'] for r in rows]
 rows.append(dict(name=name,seconds=seconds,**extra));write(p,rows)
 used=sum(r['seconds'] for r in rows)
 write(D/'RESOURCE_LEDGER.json',dict(name='Component-Owned Prism Planner Architecture',allowance_seconds=ALLOWANCE,consumed_seconds=used,remaining_seconds=ALLOWANCE-used,operations=rows,prior_ledgers_separate=True))
 assert used<=ALLOWANCE,'ArchitectureResourceAllowanceExhausted'
def stats(a):
 import numpy as np
 a=np.asarray(a)
 return dict(count=a.size,**dict(zip(['min','p10','median','p90','max'],map(float,np.quantile(a,[0,.1,.5,.9,1]))))) if a.size else dict(count=0)
