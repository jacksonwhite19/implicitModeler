"""Reserve bounded audit time for a stopped task worker; no mesh decisions."""
import json,time,sys,subprocess
from pathlib import Path
import psutil
from run_production_scale_resume import OUT,native
from production_compute_control import checkpoint_stop_record

name,label=sys.argv[1:3];folder=OUT/name/label
reserve=int(sys.argv[3]) if len(sys.argv)>3 else 600
assert 300<=reserve<=600
while not (folder/'SESSION.json').exists():
    workers=[]
    for p in psutil.process_iter(['pid','cmdline']):
        args=p.info['cmdline'] or []
        if 'worker' in args and name in args and '--label' in args and args[args.index('--label')+1]==label:
            workers.append(p)
    rows=[]
    if (folder/'PROGRESS.jsonl').exists():
        for line in (folder/'PROGRESS.jsonl').read_text().splitlines():
            try:rows.append(json.loads(line))
            except json.JSONDecodeError:pass
    published=any(r.get('event')=='checkpoint_published' and r.get('layer',0)>2 for r in rows)
    used=sum(json.loads(p.read_text())['seconds'] for p in (OUT/name).glob('*/SESSION.json'))
    used+=sum(json.loads(p.read_text())['replay_seconds'] for p in (OUT/name).glob('*/COMMITTED_RESULT.json'))
    seed=json.loads((folder/'SEED_AUTHORITY.json').read_text())['seconds']
    elapsed=max((time.time()-p.create_time() for p in workers),default=0)
    reason='DurableCheckpointReserveForFinalAudit' if published else (f'CaseBudgetReserve{reserve}SecondsForAudit' if used+seed+elapsed>=4500-reserve else None)
    if reason and workers:
        for p in workers:p.terminate();p.wait()
        native.g.save(folder/'TERMINATION.json',checkpoint_stop_record(folder,reason,published or any(r.get('event')=='certificate_complete' for r in rows)))
        break
    time.sleep(2)
while not (folder/'SESSION.json').exists():time.sleep(1)
subprocess.run([sys.executable,'-B',str(Path(__file__).with_name('collect_production_resume.py')),name,label],check=True)
