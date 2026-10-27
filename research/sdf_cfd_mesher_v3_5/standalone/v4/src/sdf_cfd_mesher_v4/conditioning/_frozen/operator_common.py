"""Fixed-geometry reconstruction calibration. No planner or solver execution."""
from pathlib import Path
import os,sys,json,time,hashlib,importlib.util
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
import numpy as np
D=Path(__file__).resolve().parent; G=D.parent; F=G.parent; H=G.parents[2]
OLD=G/'prism_numerical_health_constraint_calibration_20260915'
FORE=G/'prism_column_geometry_forensics_20260912'; Q=F/'weak_mode_provenance_20260912'
def read(p):return json.loads(Path(p).read_text())
def write(name,x):
 p=(D/name).resolve();assert p.is_relative_to(D)
 p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(x,sort_keys=True,indent=2,allow_nan=False)+'\n')
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(8<<20),b''):h.update(b)
 return h.hexdigest()
def local(p):
 s=str(p);return Path('C:/'+s[7:]) if s.startswith('/mnt/c/') else Path(s)
def raw(p,dim=1,dtype='<f8'):
 x=np.memmap(p,mode='r',dtype=dtype);return x.reshape(-1,dim) if dim>1 else x
def lp(level):
 root=F.parent/('sb_gradient_isolation_20260912' if level=='SB' else 'boundary_only_remaining_family_20260912')
 r=read(root/'results'/f'{level}_A/RESULT.json')
 g=local(r['geometry'])
 try:
  with (g/'owner.i64').open('rb') as fh:fh.read(1)
 except OSError:
  import subprocess
  g=local(subprocess.check_output(['wsl','-e','readlink','-f',r['geometry']],text=True).strip())
 return r,local(r['output']),g,H/level/'A/04_complete/foundation_audit'
def quant(x):
 x=np.asarray(x);assert np.isfinite(x).all()
 return dict(zip(['min','p1','p10','median','p90','p99','max'],map(float,np.quantile(x,[0,.01,.1,.5,.9,.99,1]))))|dict(count=x.size)
def bind(paths):
 prev=read(D/'INPUT_MANIFEST.json') if (D/'INPUT_MANIFEST.json').exists() else []
 rows={r['path']:r for r in prev}
 for p in sorted(set(map(Path,paths)),key=str):
  p=p.resolve();h=sha(p)
  if str(p)in rows:assert rows[str(p)]['sha256']==h
  rows[str(p)]=dict(path=str(p),bytes=p.stat().st_size,sha256=h)
 write('INPUT_MANIFEST.json',sorted(rows.values(),key=lambda x:x['path']))
def basis():
 aa=np.zeros((11,3));aa[:3]=np.eye(3);hh=np.zeros((11,3,3))
 hh[3]=np.diag([1,-1,0])/np.sqrt(2);hh[4]=np.diag([1,1,-2])/np.sqrt(6)
 for k,(i,j)in enumerate([(0,1),(0,2),(1,2)]):
  hh[5+k,i,j]=hh[5+k,j,i]=1/np.sqrt(2)
  hh[8+k,i,j]=1/np.sqrt(2);hh[8+k,j,i]=-1/np.sqrt(2)
 quads=[(i,j)for i in range(3)for j in range(3)if i!=j]
 return aa,hh,quads
