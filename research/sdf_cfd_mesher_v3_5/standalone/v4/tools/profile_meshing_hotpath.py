"""Isolated exact performance controls. Never switches production authority."""
import cProfile
import json
import subprocess
import shutil
import time
from pathlib import Path
import numpy as np
import capsule_sizing_control as c

OUT=c.V4/'results/production_hotpath_optimization'
SEALED=c.V4/'results/capsule_c1_core_qualification'
START='54ad959209ebee398a4bb1e73c992f429228168e'

def preflight():
    import capsule_user_domain_handoff as h
    from incremental_transaction import Store
    from audit_sealed_l11_core_domain import v3_inventory
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==START
    assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
    OUT.mkdir(exist_ok=False)
    c.write(OUT/'PREFLIGHT.json',dict(head=START,status_hex=subprocess.check_output(['git','status','--porcelain=v1','-z']).hex(),free=shutil.disk_usage(OUT).free,v3=v3_inventory()))
    a=h.read(SEALED/'AUTHORITY.json');h.verify_record(a)
    assert a['identity']=='ed9075a03ccc08315c25a7a35f6d99f184703bd7283e52da00d562d53b3f9fed'
    for name,sha in a['reports'].items():assert h.file_hash(SEALED/name)==sha
    r=h.read(SEALED/'COMPLETE_AUTHORITY.json');h.verify_record(r)
    assert r['scientific']=='a1e9d5906a7a723ec9906c938a52a25921e23d867aa9f22a489162cba9770b4f'
    assert r['content']=='bbc503c3a04dd5d4f5083d9237d20f51dafbc4578b437b1364c01ff4de5f5c81'
    assert r['access']=='a2d8e3d3722fa3305cc5b23e87cd68120a70d79cdd732ef0d6c02a1dc7b566ba'
    assert h.file_hash(SEALED/'complete_mesh'/('ROOT_'+r['access']+'.json'))=='cb2061add4c1e4150126d15c48135161f66fe651c62c3bc8bd42d8cf104c01ac'
    with Store(SEALED/'complete_mesh').open(r['access']),h.CommittedState(r['parent']) as state:state.check()
    c.write(OUT/'VERIFIED.json',dict(passed=True,authority=a['identity'],complete=r))

def summary(prof,path):
    import pstats
    prof.dump_stats(str(path)+'.prof')
    s=pstats.Stats(prof);rows=[]
    for (file,line,name),(cc,nc,tt,ct,_) in s.stats.items():
        rows.append(dict(file=file,line=line,function=name,primitive_calls=cc,calls=nc,self_seconds=tt,cumulative_seconds=ct))
    c.write(str(path)+'.json',sorted(rows,key=lambda r:r['self_seconds'],reverse=True)[:100])

def core(mode):
    folder=OUT/(mode+'_core');folder.mkdir()
    shutil.copyfile(SEALED/'CANDIDATES.json',folder/'CANDIDATES.json')
    c.OUT=folder
    queries=[];base=c.MeasuredCallback
    class Recorded(base):
        def __call__(self,dim,tag,x,y,z,lc):
            value=super().__call__(dim,tag,x,y,z,lc);queries.append((x,y,z,value));return value
    c.MeasuredCallback=Recorded
    if mode.endswith('optimized'):
        from optimized_core_sizing import ExactCachedSizingField
        c.CoreSizingField=ExactCachedSizingField
    prof=cProfile.Profile()
    if not mode.startswith('raw_'):prof.enable()
    r=c.run('C1');prof.disable()
    if not mode.startswith('raw_'):summary(prof,folder/'PROFILE')
    np.save(folder/'QUERIES.npy',np.array(queries))
    if r['status']!='generated_interface_passed':raise ValueError(r)
    import capsule_control_optimize as opt
    opt.OUT=folder;opt.run('C1')
    equality={}
    for name in ('OUTER.npz','NETGEN.npz'):
        a=np.load(folder/'C1'/name);b=np.load(SEALED/'C1'/name)
        equality[name]=all(a[k].tobytes()==b[k].tobytes() for k in a.files)
    c.write(folder/'EQUALITY.json',equality)
    assert all(equality.values())

def certificate(mode):
    import gzip
    import capsule_user_domain_handoff as h
    from certify_repaired_complete import unique_faces
    if mode in ('baseline','sample'):
        from native_mixed_face_certificate import certify
    else:
        from optimized_mixed_certificate import certify
    folder=OUT/(mode+'_certificate');folder.mkdir()
    tick=h.measured();source=SEALED/'assembly'
    x=np.load(source/'coordinates.npy');p=np.load(source/'prisms.npy');t=np.load(source/'tets.npy')
    quads=unique_faces(p,[[0,1,4,3],[1,2,5,4],[2,0,3,5]])
    caps=unique_faces(p,[[0,2,1],[3,4,5]])
    core_faces=unique_faces(t,[[0,2,1],[0,1,3],[1,2,3],[2,0,3]])
    oldcaps={tuple(sorted(f)) for f in caps}
    new=[tuple(map(int,f)) for f in core_faces if tuple(sorted(f)) not in oldcaps]
    faces=[tuple(map(int,f)) for f in quads]+[tuple(map(int,f)) for f in caps]+new
    old=len(quads)+len(caps)
    c.write(folder/'PREPARATION.json',h.metrics(*tick))
    prof=cProfile.Profile()
    if not mode.startswith('raw_'):prof.enable()
    tick=h.measured()
    result=certify(x,faces,range(old,min(old+1000,len(faces))) if mode=='sample' else range(old,len(faces)))
    rows=result.pop('proofs');result['certification_metrics']=h.metrics(*tick)
    tick=h.measured()
    if hasattr(rows,'write_jsonl'):rows.write_jsonl(folder/'PROOFS.jsonl.gz')
    else:
        with gzip.open(folder/'PROOFS.jsonl.gz','wt',encoding='utf8') as stream:
            for row in rows:stream.write(json.dumps(row,sort_keys=True,separators=(',',':'))+'\n')
    result['serialization_metrics']=h.metrics(*tick)
    prof.disable()
    if not mode.startswith('raw_'):summary(prof,folder/'PROFILE')
    oracle=h.read(SEALED/'CONTINUOUS.json')
    result['exact_oracle_equal']=all(result[k]==oracle[k] for k in ('passed','required_pairs','methods','failures','proof_hash','coverage'))
    c.write(folder/'RESULT.json',result)
    assert result['passed'] and (mode=='sample' or result['exact_oracle_equal'])

def scaling(families_only=False):
    import hashlib
    import capsule_user_domain_handoff as h
    from certify_repaired_complete import unique_faces
    from optimized_mixed_certificate import Native
    source=SEALED/'assembly'
    x=np.load(source/'coordinates.npy');p=np.load(source/'prisms.npy');t=np.load(source/'tets.npy')
    quads=unique_faces(p,[[0,1,4,3],[1,2,5,4],[2,0,3,5]])
    caps=unique_faces(p,[[0,2,1],[3,4,5]])
    core_faces=unique_faces(t,[[0,2,1],[0,1,3],[1,2,3],[2,0,3]])
    oldcaps={tuple(sorted(f)) for f in caps}
    new=[tuple(map(int,f)) for f in core_faces if tuple(sorted(f)) not in oldcaps]
    faces=[tuple(map(int,f)) for f in quads]+[tuple(map(int,f)) for f in caps]+new
    old=len(quads)+len(caps)
    ids=np.asarray([list(f) if len(f)==4 else list(f)+[f[2]] for f in faces],dtype=np.uint64)
    mask=np.asarray([len(f)==4 for f in faces]);ids[mask]=ids[mask][:,[0,1,3,2]]
    q=np.ascontiguousarray(x[ids]);native=Native();records=[];expected=None
    for workers in ((8,) if families_only else (1,2,4,8,12,16,24)):
        tick=h.measured();pairs=native.pairs(q,range(old,len(faces)),workers);broad=h.metrics(*tick)
        tick=h.measured();result=native.screen(q,ids,pairs,workers);screen=h.metrics(*tick)
        hashes=[hashlib.sha256(a.tobytes()).hexdigest() for a in (pairs,result)]
        if expected is None:expected=hashes
        assert hashes==expected
        records.append(dict(workers=workers,broad=broad,predicates=screen,pairs=len(pairs),hashes=hashes))
        if not families_only:c.write(OUT/'THREAD_SCALING.json',records)
    if families_only:
        from sdf_cfd_mesher_v4.certificate.efficient_proof import PROOF_CLASSES
        import termination_geometry_certificate as exact
        groups=[]
        for code in np.unique(result[:,0]):
            selected=np.flatnonzero(result[:,0]==code);batch=pairs[selected]
            tick=h.measured();again=native.screen(q,ids,batch,8)
            assert np.array_equal(again,result[selected])
            groups.append(dict(method=PROOF_CLASSES.get(int(code),'ExactFallbackRequired'),count=len(selected),metrics=h.metrics(*tick)))
            if code==0:
                totals={}
                for i,j in batch:
                    w=time.perf_counter();cpu=time.process_time();r=exact.pair(x,faces[int(i)],faces[int(j)])
                    v=totals.setdefault(r['method'],dict(count=0,wall=0.,cpu=0.))
                    v['count']+=1;v['wall']+=time.perf_counter()-w;v['cpu']+=time.process_time()-cpu
                groups[-1]['exact_methods']=totals
        c.write(OUT/'PREDICATE_FAMILY_DIAGNOSTICS.json',dict(scope='Separate grouped full-population rerun; group times are not an additive production profile',groups=groups))

def degree_control():
    import capsule_user_domain_handoff as h
    from incremental_layer_validation import ExactVolume
    from optimized_global_degree import degree, PERFORMANCE
    from sdf_cfd_mesher_v4.common.authority import identity
    x=np.load(SEALED/'assembly/coordinates.npy')[:202740]
    prisms=np.load(SEALED/'assembly/prisms.npy')
    _,tri,_=c.field_inputs();n=16895
    faces=[tuple(reversed(f)) for f in tri]+[tuple(f+11*n) for f in tri]
    tick=h.measured();samples=[ExactVolume.sample(x,dict(vertices=p)) for p in prisms]
    c.write(OUT/'DEGREE_PREPARATION.json',h.metrics(*tick))
    expected=h.read(c.V4/'results/recursive_layers9to11/L11/VOLUME_DELTA.json')['global_degree']
    records=[]
    for workers in (1,4,8):
        tick=h.measured();result=degree(x,faces,samples,workers=workers)
        record=dict(workers=workers,metrics=h.metrics(*tick),parallel=dict(PERFORMANCE),identity=identity(result),exact_equal=result==expected)
        records.append(record);c.write(OUT/'DEGREE_SCALING.json',records)
        assert record['exact_equal']

def prism(mode, stage):
    import os
    from unittest.mock import patch
    import qualify_recursive_layers9to11 as production
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
    folder=OUT/(mode+'_prism');folder.mkdir(exist_ok=True)
    import hashlib
    sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('optimized_*') if p.is_file()}
    c.write(folder/(stage+'_IMPLEMENTATION.json'),sources)
    reference,path,root=production.parent(11)
    if mode.endswith('optimized'):
        from optimized_admission import install
        install()
    prof=cProfile.Profile()
    if not mode.startswith('raw_'):prof.enable()
    tick=production.measured()
    with patch.object(os,'link',production.denied),patch.object(NativeCheckpointStore,'restart',production.denied),production.CommittedState(reference) as base:
        adapter,spec=production.inputs(base)
        with production.PairIncidence(base,path,root) as incidence:
            if stage=='admission':
                from composable_admission import admit
                admit(base,adapter,spec,folder)
            elif stage=='generation':
                from composable_transition import produce
                produce(base,adapter,spec,folder)
            elif stage=='assembly':
                from composable_assembly import assemble
                assemble(base,adapter,folder,pair_index=incidence)
            elif stage=='publication':
                from recursive_production_publication import publish
                publish(base,adapter,folder,incidence)
            else:raise ValueError(stage)
            result=dict(metrics=production.metrics(*tick),stats=dict(base.stats),incidence=dict(incidence.stats))
            if mode.endswith('optimized'):
                from optimized_global_degree import PERFORMANCE
                result['parallel_degree']=dict(PERFORMANCE)
    prof.disable()
    if not mode.startswith('raw_'):summary(prof,folder/(stage+'_PROFILE'))
    c.write(folder/(stage+'_MEASURED.json'),result)

if __name__=='__main__':
    import sys
    if sys.argv[1]=='preflight':preflight()
    elif sys.argv[1]=='certificate':certificate(sys.argv[2])
    elif sys.argv[1]=='prism':prism(sys.argv[2],sys.argv[3])
    elif sys.argv[1]=='scaling':scaling()
    elif sys.argv[1]=='families':scaling(True)
    elif sys.argv[1]=='degree':degree_control()
    else:core(sys.argv[1])
