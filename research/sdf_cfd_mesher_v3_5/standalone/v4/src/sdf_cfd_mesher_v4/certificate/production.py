"""Task-4 runner adapter for the Task-4B qualified exact-safe hierarchy.

Only the newest transaction is proved. Pair ownership is max(layer_a,layer_b).
Unchanged earlier proof payloads are immutable. Final verification enumerates
the complete population again and checks every bound proof, without re-solving.
"""
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import gzip,json,time
import numpy as np
from .efficient import Filter
from .efficient_proof import prove
from .compact_storage import write_compact
from .incremental import kernel
from ..common.authority import array_hash,file_hash
from ..forecast.detailed.scientific import canonical,scientific_hash

VERSION='Task4BQualifiedProductionAdapter/1'

def surface_arrays(wall,edges,prefix):
    # One output owner. Fancy indexing is bounded to one edge plane instead
    # of four complete shell arrays plus a concatenated coordinate history.
    layers=len(prefix);E=len(edges);nv=len(wall)
    q=np.empty((layers,E,4,3),dtype=np.result_type(wall,*prefix))
    labels=np.empty((layers,E,4),dtype=np.uint64)
    lower=wall
    for k,upper in enumerate(prefix):
        for corner,(points,endpoint,level) in enumerate(((lower,0,k),(lower,1,k),(upper,0,k+1),(upper,1,k+1))):
            q[k,:,corner]=points[edges[:,endpoint]]
            labels[k,:,corner]=level*nv+edges[:,endpoint]
        lower=upper
    return q.reshape(-1,4,3),labels.reshape(-1,4)


def source_identity():
    here=Path(__file__).parent
    names=('efficient.py','efficient_native.rs','efficient_proof.py','efficient_root_keys.py','production.py','exact_preparation.py','parallel_screen.py','compact_storage.py','cad_algebra_cache.py')
    return scientific_hash({n:file_hash(here/n) for n in names})

def certify_immediate(front,top,prefix,path,*,batch=512,cache=None,progress=None,workers=1):
    # The old batch/cache arguments remain API-compatible. Proof ownership and
    # persistence are per immutable transaction; no previous proof is invalidated.
    if workers<1:raise ValueError('InvalidCertificateWorkerCount')
    started=time.perf_counter();path=Path(path);path.mkdir(parents=True,exist_ok=True)
    q,labels=surface_arrays(front.coordinates,top.edges,prefix);layer=len(prefix);native=Filter();E=len(top.edges)
    geometry_preparation=time.perf_counter()-started
    s=time.perf_counter();forest=native.forest(q,E);packed=forest.pairs(layer);forest.close();enumeration=time.perf_counter()-s
    s=time.perf_counter();valid=native.valid(q[-E:]);invalid=[];fallback_valid=[]
    for i in np.flatnonzero(valid==0):
        p=kernel.Patch.binary(q[-E+i],tuple(map(int,labels[-E+i])));ok,why=p.valid()
        record=dict(side=[layer,int(i)],passed=ok,reason=why);fallback_valid.append(record)
        if not ok:invalid.append(record)
    validity=time.perf_counter()-s
    # Invalid surfaces fail closed before pair acceptance.
    if invalid:
        return dict(adapter=VERSION,complete=False,layer=layer,candidate_pairs=len(packed),completed_pairs=0,
            forbidden=[],unresolved=[],invalid_surfaces=invalid),[dict(seconds=time.perf_counter()-started)]
    if workers==1:result,perf=prove(q,labels,packed,native)
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:result,perf=prove(q,labels,packed,native,executor=pool,screen_workers=workers)
    forbidden=[];unresolved=[]
    for r in result['fallback']:
        c=r['result']['classification']
        if c not in ('CertifiedSeparated','CertifiedAllowedIncidentContact'):
            (forbidden if 'Forbidden' in c else unresolved).append(r)
    binding_started=time.perf_counter()
    record=dict(adapter=VERSION,layer=layer,complete=True,candidate_pairs=len(packed),completed_pairs=len(packed),
        forbidden=forbidden,unresolved=unresolved,invalid_surfaces=invalid,topology=top.sha256,
        geometry_hash=array_hash(q),labels_hash=array_hash(labels),source_identity=source_identity(),kernel=kernel.VERSION,
        pair_hash=array_hash(packed),code_hash=array_hash(result['codes']),witness_hash=array_hash(result['witness']),
        outcome_hash=array_hash(result['outcomes']),fallback_hash=scientific_hash(result['fallback']),
        validity_hash=array_hash(valid),validity_fallback=fallback_valid,
        class_counts={str(c):int(n) for c,n in zip(*np.unique(result['codes'],return_counts=True))},
        exact_calls=perf['exact_calls'],ownership='max(layer_a,layer_b)',
        complement='Strict corner-AABB separation; bilinear convex hull contains the entire surface.')
    binding=time.perf_counter()-binding_started
    s=time.perf_counter()
    write_compact(path/'COMPACT.npz',dict(packed=packed,codes=result['codes'],witness=result['witness'],outcomes=result['outcomes'],validity=valid))
    compact_seconds=time.perf_counter()-s;rich_started=time.perf_counter()
    with (path/'EXACT.json.gz').open('wb') as f:
        with gzip.GzipFile(filename='',mode='wb',fileobj=f,mtime=0) as z:z.write(canonical(result['fallback']))
    rich_seconds=time.perf_counter()-rich_started;manifest_started=time.perf_counter()
    (path/'MANIFEST.json').write_bytes(canonical(record))
    perf.update(geometry_preparation_seconds=geometry_preparation,binding_seconds=binding,compact_serialization_seconds=compact_seconds,rich_serialization_seconds=rich_seconds,manifest_serialization_seconds=time.perf_counter()-manifest_started,enumeration_seconds=enumeration,validity_seconds=validity,serialization_seconds=time.perf_counter()-s,seconds=time.perf_counter()-started)
    if progress:progress(layer,0,E,record,perf)
    return record,[perf]

def verify_committed_layer(front,top,prefix,path,ledger,*,workers=1):
    if ledger.get('adapter')!=VERSION:
        # Historical transactions retain their historical representation.
        from .rolling import verify_committed_layer as legacy
        return legacy(front,top,prefix,path,ledger,workers=workers)
    started=time.perf_counter();path=Path(path);q,labels=surface_arrays(front.coordinates,top.edges,prefix)
    assert ledger['layer']==len(prefix) and ledger['complete']
    assert ledger['topology']==top.sha256 and ledger['source_identity']==source_identity() and ledger['kernel']==kernel.VERSION
    assert not any(ledger[k] for k in ('forbidden','unresolved','invalid_surfaces'))
    assert array_hash(q)==ledger['geometry_hash'] and array_hash(labels)==ledger['labels_hash']
    assert json.loads((path/'MANIFEST.json').read_text())==ledger
    native=Filter();forest=native.forest(q,len(top.edges));pairs=forest.pairs(len(prefix));forest.close()
    with np.load(path/'COMPACT.npz') as z:stored={k:z[k] for k in z.files}
    assert np.array_equal(pairs,stored['packed']) and (len(pairs)<2 or np.all(pairs[1:]>pairs[:-1]))
    for key,field in [('packed','pair_hash'),('codes','code_hash'),('witness','witness_hash'),('outcomes','outcome_hash'),('validity','validity_hash')]:assert array_hash(stored[key])==ledger[field]
    with gzip.open(path/'EXACT.json.gz','rb') as f:exact=json.load(f)
    assert scientific_hash(exact)==ledger['fallback_hash']
    codes=stored['codes'];outcomes=stored['outcomes'];unknown=np.flatnonzero(codes==0)
    assert len(exact)==len(unknown)==ledger['exact_calls']
    assert np.all((codes>=0)&(codes<=6)) and np.all(np.isin(outcomes,[1,2]))
    for i,r in zip(unknown,exact):
        assert r['pair']==[int(pairs[i])>>32,int(pairs[i])&0xffffffff]
        c=r['result']['classification'];assert c in ('CertifiedSeparated','CertifiedAllowedIncidentContact')
        assert outcomes[i]==(1 if c=='CertifiedSeparated' else 2)
    assert np.all(outcomes[(codes==1)|(codes==2)]==1) and np.all(outcomes[codes>=3]==2)
    assert ledger['candidate_pairs']==ledger['completed_pairs']==len(pairs)
    assert len(ledger['validity_fallback'])==int(np.count_nonzero(stored['validity']==0))
    assert all(r['passed'] for r in ledger['validity_fallback'])
    return len(pairs),[dict(seconds=time.perf_counter()-started,reused_proofs=len(pairs),new_exact_calls=0)]
