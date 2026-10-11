"""Read-only checkpoint comparison; no expanded proof population in RAM."""
import json,sqlite3,sys
from pathlib import Path
from contextlib import closing
from run_geometry_family import OUT,g
from sdf_cfd_mesher_v4.common.authority import file_hash


def component(envelope_path,envelope,name):
    row=envelope['components'][name];p=envelope_path.parent/'objects'/row['file']
    assert file_hash(p)==row['sha256']
    return p


def retained_tables(a,b):
    with closing(sqlite3.connect(':memory:',uri=True)) as db:
        db.execute('ATTACH DATABASE ? AS old',(a.resolve().as_uri()+'?mode=ro',))
        db.execute('ATTACH DATABASE ? AS new',(b.resolve().as_uri()+'?mode=ro',))
        old=db.execute('SELECT count(*) FROM old.receipts').fetchone()[0]
        new=db.execute('SELECT count(*) FROM new.receipts').fetchone()[0]
        retained=db.execute('SELECT count(*) FROM old.receipts a JOIN new.receipts b ON a.key=b.key WHERE a.value=b.value').fetchone()[0]
    return dict(before=old,after=new,exact_retained=retained,retired_or_changed=old-retained,new_or_changed=new-retained)


def run(name,label):
    folder=OUT/name/label;trace=json.loads((folder/'TRACE.json').read_text());paths={}
    for p in (folder/'checkpoints').glob('NATIVE_*.json'):
        e=json.loads(p.read_text());paths[e['scientific']]=(p,e)
    start=folder/'RESTART_INPUT.json'
    if not start.exists():start=folder/'REPLAY_INPUT.json'
    before=None
    if start.exists():
        r=json.loads(start.read_text());p=Path(r.get('source',r.get('source_checkpoint')));before=(p,json.loads(p.read_text()))
    rows=[]
    for step in trace:
        after=paths[step['state']]
        if before and step['operation']=='RollbackRemesh':
            p,a=before;q,b=after;row=dict(state_before=a['scientific'],state_after=b['scientific'],layer=step['layer'])
            for name in ('atlas','forecast'):
                old=json.loads(component(p,a,name).read_text());new=json.loads(component(q,b,name).read_text())
                hits=sum(new.get(k)==v for k,v in old.items())
                row[name]=dict(before=len(old),after=len(new),exact_retained=hits,retired_or_changed=len(old)-hits,new_or_changed=len(new)-hits)
            for name in ('quality','certificate'):row[name]=retained_tables(component(p,a,name),component(q,b,name))
            rows.append(row)
        before=after
    g.save(folder/'LOCALITY.json',dict(scope='Exact scientific receipt/dependency retention, not inferred kernel execution counts',events=rows))


if __name__=='__main__':run(*sys.argv[1:])
