"""Read-only transfer of a pre-event native checkpoint into a fresh run."""
import sys,json,os,shutil
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from run_geometry_family import OUT,run,g
from sdf_cfd_mesher_v4.common.authority import file_hash


def fork(name,source_label,dest_label,current=False):
    source=OUT/name/source_label;dest=OUT/name/dest_label;dest.mkdir(exist_ok=False)
    trace=json.loads((source/'TRACE.json').read_text())
    if current:chosen=trace[-1]
    else:
        first=next(i for i,r in enumerate(trace) if r['operation']=='RollbackRemesh')
        if first==0:raise ValueError('InitialCheckpointSelectionRequired')
        chosen=trace[first-1]
    path=next(p for p in (source/'checkpoints').glob('NATIVE_*.json') if json.loads(p.read_text())['scientific']==chosen['state'])
    envelope=json.loads(path.read_text());checkpoints=dest/'checkpoints';objects=checkpoints/'objects';objects.mkdir(parents=True)
    for row in envelope['components'].values():
        src=source/'checkpoints/objects'/row['file'];assert file_hash(src)==row['sha256']
        target=objects/src.name
        if not target.exists():os.link(src,target)
    shutil.copy2(path,checkpoints/path.name)
    g.save(checkpoints/'CURRENT.json',dict(format=envelope['format'],checkpoint=path.name,scientific=envelope['scientific'],sha256=file_hash(path)))
    g.save(dest/'RESTART_INPUT.json',dict(source=str(path),sha256=file_hash(path),scientific=envelope['scientific'],layer=chosen['layer'],scope='Exact last committed state' if current else 'Exact committed state before first topology transaction'))
    run(name,dest_label,resume=True)

if __name__=='__main__':fork(*sys.argv[1:4],current='--current' in sys.argv[4:])
