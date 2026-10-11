"""Run one isolated production-equivalent control with explicit qualified settings."""
import os,sys,subprocess
from pathlib import Path
settings={'CRITICAL':'1','QUALITY_BATCH':'1','QUALITY_CODEC':'4','INDEX_OVERLAP':'1','INDEX_PROCESS':'1','FACE_KEYS':'1','DEGREE_OVERLAP':'1','DEGREE_PROCESS':'1','DEGREE_WORKERS':'2','PROFILE':'0','PERSISTENT':'1','NUMERIC':'1','LOOKUP':'1'}
if __name__=='__main__':
    for k,v in settings.items():os.environ['V4_'+k]=v
    os.environ['V4_DIRECT']=sys.argv[2] if len(sys.argv)>2 else '0'
    os.environ['V4_PROFILE']=sys.argv[3] if len(sys.argv)>3 else '0'
    raise SystemExit(subprocess.call([sys.executable,str(Path(__file__).with_name('measure_direct_job.py')),sys.argv[1]]))
