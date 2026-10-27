"""Seal retained evidence and an exact task-local cleanup plan before deletion."""
import sys
import capsule_steady_models as s
from capsule_family_seal import allocation
h=s.h


def plan():
    folders=[s.ROOT/n for n in ('GC2-S1','GC2-S2','GC2F-S1','GC2F-S2')]
    folders += [s.ROOT/'model_runs'/n for n in ('SA-GW1-S1','SA-GW1-S2')]
    remove=[];retained={};evidence={}
    for folder in folders:
        m=h.read(folder/'MEASUREMENTS.json');h.verify_record(m)
        assert (folder/'pilot.exit').read_text().strip()=='0'
        for file,key in (('HISTORY.json','history'),('pilot.log','log')):
            assert h.file_hash(folder/file)==m[key]
            evidence[str((folder/file).relative_to(s.ROOT))]=m[key]
        keep={0,20,50,100,200,300,400,500}
        if folder.name=='GC2F-S2':keep.update((30,40,60))
        for rank in range(6):
            proc=folder/'pilot'/f'processor{rank}'
            for p in proc.iterdir():
                if not p.is_dir() or not p.name.isdigit():continue
                assert p.resolve().is_relative_to(s.ROOT.resolve())
                if int(p.name) in keep:
                    for f in p.rglob('*'):
                        if f.is_file():retained[str(f.relative_to(s.ROOT))]=h.file_hash(f)
                else:
                    assert 0<int(p.name)<500
                    remove.append(dict(path=str(p.resolve()),**allocation(p)))
    for file in ('FINE_COMPARISON.json','FINE_DENSE_WAKE.json','MODEL_COMPARISON.json','LOAD_DECOMPOSITION.json'):
        r=h.read(s.ROOT/file);h.verify_record(r);evidence[file]=h.file_hash(s.ROOT/file)
    result=h.seal(dict(schema='TaskOwnedRedundantFieldRetentionPlan/1',root=str(s.ROOT.resolve()),
        retained_files=retained,evidence=evidence,remove=remove,
        protected='All initial, terminal and diagnostic checkpoint fields; unique fine S2 excursion 30/40/50/60; every prior study remains untouched',
        reclaim_logical=sum(r['logical_bytes'] for r in remove),reclaim_physical=sum(r['physical_bytes'] for r in remove)))
    h.durable_json(s.ROOT/'RETENTION_AUTHORITY.json',result);print(result['identity'])


def verify():
    p=h.read(s.ROOT/'RETENTION_AUTHORITY.json');h.verify_record(p)
    for rel,digest in {**p['retained_files'],**p['evidence']}.items():assert h.file_hash(s.ROOT/rel)==digest
    from pathlib import Path
    assert all(not Path(r['path']).exists() for r in p['remove'])
    r=h.seal(dict(schema='TaskOwnedFieldCleanup/1',plan=p['identity'],removed_directories=len(p['remove']),
        reclaimed_logical_bytes=p['reclaim_logical'],reclaimed_physical_bytes=p['reclaim_physical'],retained_hashes_verified=True,prior_evidence_deleted=False))
    h.durable_json(s.ROOT/'CLEANUP.json',r);print(r['identity'])


if __name__=='__main__':globals()[sys.argv[1]]()
