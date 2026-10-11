"""Evidence closure for this steady-only study; no production promotion."""
import json,shutil,subprocess,sys
from pathlib import Path
import capsule_steady_models as s
from capsule_family_seal import allocation

def storage(label):
    r=dict(free_bytes=shutil.disk_usage(s.ROOT).free,total=allocation(s.ROOT),
        variants={p.name:allocation(p) for p in s.ROOT.iterdir() if p.is_dir() and (p/'CANDIDATE.json').exists()},
        method='NTFS FileStandardInfo AllocationSize; every task file must have one link')
    s.h.durable_json(s.ROOT/f'STORAGE_{label}.json',r);print(json.dumps(r))

def verify_prior():
    r=s.h.read(s.OLD/'RESULT.json');s.h.verify_record(r)
    for rel,digest in r['evidence'].items():assert s.h.file_hash(s.OLD/rel)==digest,rel
    for rel,digest in r['implementation'].items():
        if rel!='identity':assert s.h.file_hash(s.f.prior.control.V4/rel)==digest,rel
    pre=s.h.read(s.ROOT/'PREFLIGHT.json')
    for rel,ident in pre['authorities'].items():
        path=s.f.prior.control.V4/rel;row=s.h.read(path);s.h.verify_record(row);assert row['identity']==ident
        for file,digest in row.get('evidence',{}).items():
            if isinstance(digest,str) and len(digest)==64:assert s.h.file_hash(path.parent/file)==digest,file
    assert s.f.prior.frozen()==pre['frozen']
    status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
    assert [x for x in status if 'capsule_steady_models' not in x]==pre['unrelated']
    return dict(prior_result=r['identity'],authorities=pre['authorities'],frozen=pre['frozen'],unrelated_count=len(pre['unrelated']))

def seal():
    prior=verify_prior();decision=s.h.read(s.ROOT/'DECISION.json');s.h.verify_record(decision)
    assert decision['terminal'] and decision['URANS_runs']==decision['SAS_runs']==decision['DES_runs']==decision['LES_runs']==0
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==s.HEAD
    source=s.h.read(s.ROOT/'MODEL_SOURCE_RECOVERY.json');s.h.verify_record(source)
    for path,digest in source['sources'].items():
        assert s.h.file_hash(s.ROOT/'model_sources'/path.removeprefix('/opt/openfoam13/'))==digest
    context=s.h.read(s.ROOT/'SA_FREESTREAM_CONTEXT.json');s.h.verify_record(context)
    assert s.h.file_hash(s.ROOT/context['snapshot'])==context['snapshot_sha256']
    wall=s.h.read(s.ROOT/'WALL_STRESS_COMPARISON.json');s.h.verify_record(wall)
    assert s.h.file_hash(s.ROOT/'model_sources/wallShearStress.C')==wall['source_sha256']
    for name,r in wall['runs'].items():
        model,mesh=name.split('-',1)
        folder=s.ROOT/'wall_reference'/mesh if model=='SST' else s.ROOT/'model_runs'/name
        assert s.h.file_hash(folder/'WALL_STRESS.npz')==r['archive_sha256']
        assert r['new_CFD_iterations']==0
    for folder in s.ROOT.iterdir():
        if not folder.is_dir() or not (folder/'CANDIDATE.json').exists():continue
        c=s.h.read(folder/'CANDIDATE.json');s.h.verify_record(c)
        for rel,digest in c['evidence'].items():assert s.h.file_hash(folder/rel)==digest
        con=s.h.read(folder/'CFD_CONTRACT.json');s.h.verify_record(con)
        for rel,digest in con['controls'].items():assert s.h.file_hash(folder/'pilot'/rel)==digest
        assert int((folder/'pilot.exit').read_text())==0
        m=s.h.read(folder/'MEASUREMENTS.json');s.h.verify_record(m)
        assert s.h.file_hash(folder/'HISTORY.json')==m['history']
        assert s.h.file_hash(folder/'pilot.log')==m['log']
    for folder in (s.ROOT/'model_runs').glob('*'):
        if not folder.is_dir() or not (folder/'CFD_CONTRACT.json').exists():continue
        con=s.h.read(folder/'CFD_CONTRACT.json');s.h.verify_record(con)
        for rel,digest in con['controls'].items():assert s.h.file_hash(folder/'pilot'/rel)==digest
        assert int((folder/'pilot.exit').read_text())==0
        m=s.h.read(folder/'MEASUREMENTS.json');s.h.verify_record(m)
        assert m['contract']==con['identity'] and m['iterations']==500
        assert s.h.file_hash(folder/'HISTORY.json')==m['history']
        assert s.h.file_hash(folder/'pilot.log')==m['log']
    files={str(p.relative_to(s.ROOT)):s.h.file_hash(p) for p in s.ROOT.rglob('*') if p.is_file() and p.suffix in ('.json','.txt','.md') and p!=s.ROOT/'RESULT.json' and not any(part in ('pilot','foundation','W1','assembly','model_sources') for part in p.relative_to(s.ROOT).parts)}
    code={str(p.relative_to(s.f.prior.control.V4)):s.h.file_hash(p) for top in ('tools','tests') for p in (s.f.prior.control.V4/top).glob('*capsule_steady_models*') if p.is_file()}
    r=s.h.seal(dict(schema='CapsuleSteadyMeshModelSensitivityResult/1',classification=decision['classification'],starting_head=s.HEAD,decision=decision['identity'],prior=prior,evidence=files,implementation=code,production_authority_replaced=False,Run_A=False,Run_A2=False))
    s.h.durable_json(s.ROOT/'RESULT.json',r);print(r['identity'])

if __name__=='__main__':
    if sys.argv[1]=='storage':storage(sys.argv[2])
    elif sys.argv[1]=='verify':print(json.dumps(verify_prior()))
    else:seal()
