"""Fresh bounded transaction exercise; not a full Task-4 runner qualification.

No accepted remesh geometry is read during construction. Qualified reference
files are opened only after remesh, exact validation and continuation finish.
"""
import json, sys, time
from pathlib import Path
import generalize_local_remesh as g
from sdf_cfd_mesher_v4.advancing_front.remesh_transaction import (
    MixedTransactionStore, identity, flattened_inputs, retain_receipts, preservation)
from sdf_cfd_mesher_v4.advancing_front.termination_policy import REMESH_CHECKS
from sdf_cfd_mesher_v4.common.authority import file_hash
q=g.q; np=g.np; ROOT=g.s.ROOT
OUT=ROOT/'results/production_automatic_termination_remesh'


def mesh(x,tri,fronts,levels):
    p,c,o=g.s.construct(x,tri,fronts,levels)
    return dict(points=p.tolist(),cells=g.s.r.w.clean(c),outer=g.s.r.w.clean(o),
                wall=dict(coordinates=x.tolist(),triangles=tri.tolist()),first_layer=fronts[0].tolist(),
                vertex_depth=list(map(int,levels)),last_prism=[int(min(levels[v] for v in f)) for f in tri])


def receipts(m,check,authority):
    inputs=flattened_inputs(m,authority); cells={str(c['id']):c for c in m['cells']}
    quality={}; proof={}
    for row in check['quality']['cells']:
        c=cells[str(row['id'])]; deps=['cells:'+str(row['id']),'authority:quality']
        deps += ['faces:'+','.join(map(str,sorted(f))) for f in c['faces']]
        quality['cell:'+str(row['id'])]=dict(inputs={k:inputs[k] for k in deps},value=row)
    for row in check['quality']['faces']:
        key=','.join(map(str,sorted(row['vertices'])))
        deps=['faces:'+key,'authority:quality']+['cells:'+str(i) for i in row['owners']]
        quality['face:'+key]=dict(inputs={k:inputs[k] for k in deps},value=row)
    owners,_=g.t.ownership(m['cells'],m['outer'],[tuple(reversed(f)) for f in m['wall']['triangles']])
    keys=[','.join(map(str,k)) for k in sorted(owners)]
    for row in check['certificate']['proofs']:
        pair=[keys[i] for i in row['pair']]; deps=['faces:'+k for k in pair]+['authority:certificate']
        proof[identity(pair)]=dict(inputs={k:inputs[k] for k in deps},value={k:v for k,v in row.items() if k!='pair'})
    return g.s.r.w.clean(dict(quality=quality,certificate=proof))


def run(name, suffix='primary'):
    started=time.perf_counter(); folder=OUT/name/suffix; folder.mkdir(parents=True,exist_ok=True)
    spec=next(s for s in g.fixtures.definitions() if s['name']==name)
    x,tri=g.fixtures.build(spec); schedule=q.load_schedule(); fronts=[]; scan=[]
    # Fresh rolling nominal proposals. This harness uses the qualified fixture
    # generator; production NativeSdf/Task-4 planner adapters are not claimed.
    for k,target in enumerate(schedule.targets,1):
        fronts.append(g.fixtures.front(spec,target))
        p,c,o,check,_=g.validation(x,tri,fronts,[k]*len(x),False)
        sr=g.fixtures.sdf_record(spec,fronts[-1],target)
        scan.append(dict(layer=k,quality=check['quality']['passed'],sdf=sr))
        if not check['quality']['passed'] or not sr['admitted']:
            oc=g.s.cert.certify(p,o,range(len(o)))
            seed,columns=g.failure_vertices(tri,check['quality'],oc)
            capacity = bool(spec.get('cavity') and any(g.t.F(float(r))-g.t.F(float(target))<=0 for r in spec['radii']))
            if not seed or not (oc['failures'] or capacity): raise ValueError('NoQualifiedCurrentDirectEvidence')
            event=dict(layer=k,quality=check['quality'],outer=oc,sdf=sr,seed_vertices=sorted(seed),columns=columns)
            break
    pp,cc,oo,support,support_times=g.validation(x,tri,fronts[:-1],[k-1]*len(x))
    if not support['passed']: raise ValueError('ContaminatedSupport')
    before=mesh(x,tri,fronts[:-1],[k-1]*len(x))
    authority=dict(schedule=q.array_hash(schedule.targets),certificate=file_hash(Path(g.s.cert.__file__)),
                   quality=file_hash(Path(g.t.__file__)))
    old_receipts=receipts(before,support,authority)
    local=dict(receipts=old_receipts)
    store=MixedTransactionStore(folder/'checkpoints'); stable=store.initialize(before,local,authority)
    context=dict(region=identity(dict(wall=before['wall'],seed=sorted(seed))),layer=k,
                 base=identity(stable),topology=q.array_hash(tri),schedule=authority['schedule'])
    ref=dict(context=context,proof=identity(g.s.r.w.clean(event)))
    seeds=dict(vertices=sorted(seed),columns=columns,cells=[k*1000000+c for c in columns],faces=sorted({v for row in oc['failures'] for v in row['pair']}))
    evidence=dict(context=context,support=dict(context=context,proof=identity(g.s.r.w.clean(support)),status='clean'),
                  immediate=dict(ref,status='invalid',seed=seeds),warnings=[ref['proof']],attempts=[],accepted=None,
                  witnesses=[dict(ref,kind='QualifiedCapacityLimit' if capacity else 'ExactCollision',scope='immediate',qualified=True,seed=seeds)])
    timings={}; artifact={}; calls=0
    def backend(request,private):
        nonlocal calls
        calls+=1; tick=time.perf_counter()
        levels,expansion=g.discover(x,tri,fronts,set(request['seed']['vertices']),k)
        timings['cavity_discovery']=time.perf_counter()-tick
        if levels is None: raise ValueError('CavityDiscoveryFailure')
        tick=time.perf_counter(); replacement=mesh(x,tri,fronts,levels);timings['generation']=time.perf_counter()-tick
        removed=g.s.removal_manifest(private['cells'],replacement['cells'])
        details=dict(removed_cells=[r['cell']['id'] for r in removed['removed']],manifest=removed,
                     expansion=expansion,seed=request['seed'],levels=levels.tolist())
        return replacement,details
    def rebuild(old,new,details):
        tick=time.perf_counter(); a=flattened_inputs(old['mesh'],authority);b=flattened_inputs(new,authority)
        kept={}; invalid={}
        for family,r in old['local']['receipts'].items():
            kept[family],invalid[family]=retain_receipts(r,a,b)
        timings['receipt_invalidation']=time.perf_counter()-tick
        artifact['retained']=kept
        return dict(receipts=kept),dict(invalidated={k:len(v) for k,v in invalid.items()},
                                       retained={k:len(v) for k,v in kept.items()},keys=invalid)
    def validate(request,new,local,details):
        tick=time.perf_counter();levels=np.array(new['vertex_depth'])
        vp,vc,vo,checked,vt=g.validation(x,tri,fronts,levels)
        if not checked['passed']:raise ValueError('ExactRemeshValidationFailed')
        timings['exact_validation']=time.perf_counter()-tick;timings['validation_stages']=vt
        fresh=receipts(new,checked,authority)
        for family,old in artifact['retained'].items():
            for key,value in old.items():
                if fresh[family].get(key)!=value:
                    g.save(folder/'REUSE_MISMATCH.json',dict(family=family,key=key,old=value,new=fresh[family].get(key)))
                    raise ValueError('ReceiptReuseInputIncomplete')
        local['receipts']=fresh
        continuation=g.continue_local(spec,x,tri,fronts,levels,seed)
        timings['continuation']=continuation.pop('runtime')
        ids=sorted({v for f in vo for v in f});lookup={v:i+1 for i,v in enumerate(ids)}
        ingest=dict(coordinates=vp[ids].tolist(),triangles=[[lookup[v] for v in f] for f in vo],outer_hash=identity(vo))
        g.save(folder/'CORE_INGEST_INPUT.json',ingest)
        import subprocess
        def wsl(p):return '/mnt/'+p.drive[0].lower()+str(p)[2:].replace('\\','/')
        tick=time.perf_counter()
        subprocess.run(['wsl','--','python3',wsl(ROOT/'tools/cavity_core_ingestion_smoke.py'),wsl(folder/'CORE_INGEST_INPUT.json')],check=True,capture_output=True)
        timings['core_ingestion']=time.perf_counter()-tick
        core=json.loads((folder/'CORE_INGESTION.json').read_text())
        artifact.update(check=checked,continuation=continuation,core=core,mesh=new,details=details)
        # The ingestion tool aborts before writing success if identity differs.
        assigned=[]
        used=sorted({v for cell in vc for v in cell['vertices']})
        for layer in range(1,len(fronts)+1):
            vertices=[v for v in used if v//len(x)==layer]
            if vertices:
                assigned.append(g.fixtures.sdf_record(spec,vp[vertices],schedule.targets[layer-1]))
        audit=preservation(before,new)
        checks=dict(hard_quality=checked['quality']['passed'],exact_validity=all(
            r['jacobian_sign']>0 and r['volume_sign']>0 for r in checked['quality']['exact']),
            ownership=checked['ownership']['passed'],continuous=checked['certificate']['passed'],
            sdf_target=all(r['admitted'] for r in assigned),volume_conformity=checked['volume']['exact_volume_balance']
            and checked['volume']['positive_cells'] and checked['volume']['interior_degree']['passed'],
            outer_interface=checked['vertex_links']['passed'] and checked['ownership']['passed'],
            core_compatibility=core['passed'],neighbor_continuation=continuation['passed'],
            wall_first_layer=True,outside_preserved=audit==preservation(before,new))
        artifact['assigned_sdf']=assigned
        return dict(context=request['context'],request_id=request['request_id'],proof=identity(g.s.r.w.clean(checked)),checks=checks)
    def diagnostic(fn):
        def call(*args):
            try:return fn(*args)
            except Exception:
                import traceback
                traceback.print_exc();raise
        return call
    tick=time.perf_counter();result=store.dispatch(evidence,diagnostic(backend),diagnostic(rebuild),diagnostic(validate))
    timings['event']=time.perf_counter()-tick
    if not result['committed']:
        g.save(folder/'FAILED.json',result);raise ValueError('MixedTransactionRejected:'+str(result['invocation']['receipt']))
    # This checks persisted state, not an in-memory backend result.
    final=store.restart(); assert final['mesh']==artifact['mesh']
    assert store.dispatch(evidence,backend,rebuild,validate)==result and calls==1
    # Comparison-only sealed reference, opened after construction finishes.
    reference=json.loads((ROOT/'results/local_termination_remesh_generalization'/name/'result.json').read_text())
    equivalent=dict(levels=artifact['mesh']['vertex_depth']==reference['vertex_depth'],
        cells=artifact['mesh']['cells']==reference['cells'],outer=artifact['mesh']['outer']==reference['outer'],
        quality=g.s.r.w.clean(artifact['check']['quality'])==reference['result']['quality'],
        certificate=g.s.r.w.clean(artifact['check']['certificate'])==reference['result']['certificate'],
        continuation=g.s.r.w.clean(artifact['continuation'])==reference['continuation'])
    record=dict(name=name,event=event,scan=scan,evidence=evidence,transaction=result,equivalence=equivalent,
        final_mesh_hash=identity(final['mesh']),final_state_hash=identity(final),
        continued=artifact['continuation'],validation=artifact['check'],
        layer_count_histogram=dict(g.t.Counter(artifact['mesh']['last_prism'])),
        first_layer_preserved=True,wall_preserved=True,
        production_runner_integrated=False,qualification='BoundedMixedTransactionExercisePassed')
    g.save(folder/'result.json',record)
    timings.update(total=time.perf_counter()-started,peak_rss=q.psutil.Process().memory_info().peak_wset,support=support_times)
    g.save(folder/'runtime.json',timings)
    print(name,record['qualification'],equivalent,flush=True)


if __name__=='__main__':run(sys.argv[1],sys.argv[2] if len(sys.argv)>2 else 'primary')
