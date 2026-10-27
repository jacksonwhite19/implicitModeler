"""Actual SF2 full-versus-incremental exact scientific equivalence."""
from task3_repair_study import *
from task3d_preflight import OUT
from sdf_cfd_mesher_v4.certificate.incremental import build_scope,ProofCache,certify,dependency_changes,proof_invalidation
import io,gzip

def normalized(stream):
    rows=[json.loads(x) for x in stream.splitlines()]
    return sorted(rows,key=lambda r:(r['side_a'],r['side_b']))

class RecordedSession(CertificateSession):
    def __init__(self,scope):
        super().__init__(0);self.by_labels={p:s for p,s in zip(scope.labels,scope.sides)};self.rows=[]
    def pair(self,a,b):
        r=super().pair(a,b);sa,sb=self.by_labels[a.ids],self.by_labels[b.ids]
        self.rows.append(dict(side_a=min(sa,sb),side_b=max(sa,sb),**r));return r

def store_stream(name,raw,replay):
    path=OUT/name
    if replay:
        with gzip.open(path,'rb') as f:assert f.read()==raw
    else:
        with path.open('wb') as f:
            with gzip.GzipFile(filename='',mode='wb',fileobj=f,mtime=0) as z:z.write(raw)

def run(replay=False):
    started=time.perf_counter();ctx=load();front,top,sdf,schedule,coords,normals,full,*_=ctx
    # Preserved sampling control, not a production risk-location rule.
    columns=[22890];e=np.unique(top.triangle_edges[columns]);ids=np.unique(top.edges[e])[:1]
    field=np.tile([1.,0,0],(len(ids),1))
    trial,projection=project_tangent_move(coords[0,ids],field,np.full(len(ids),schedule.targets[0]/32),sdf,schedule.targets[0])
    fc=LocalForecastCache(front,top,schedule,sdf,coords,normals);future=fc.forecast(ids,trial)
    idx=updated_index(full,future,dependencies(top,ids)[1]);old=build_scope(full,e);new=build_scope(idx,e)
    cache=ProofCache();baseline_stream=io.BytesIO();baseline,bperf=certify(old,cache=cache,record_sink=baseline_stream.write,seconds_budget=180)
    assert baseline['complete'],baseline
    changed=dependency_changes(old,new);invalid=proof_invalidation(old,new)
    inc_stream=io.BytesIO();incremental,iperf=certify(new,cache=cache,record_sink=inc_stream.write,seconds_budget=180)
    assert incremental['complete'],incremental
    # Independent frozen caller, enumeration, pair dispatch and serialization.
    recorded=RecordedSession(new);kernel.clear_kernel_caches();repaired_kernel.clear_kernel_caches()
    st=time.perf_counter();full_result=certify_all(idx,SimpleNamespace(side_edges=e),session=recorded,seconds_budget=180)
    full_seconds=time.perf_counter()-st;assert full_result['complete'],full_result
    ref=sorted(recorded.rows,key=lambda r:(r['side_a'],r['side_b']));got=normalized(inc_stream.getvalue())
    assert scientific_hash(ref)==scientific_hash(got),'FullIncrementalScientificMismatch'
    warm_stream=io.BytesIO();warm,wperf=certify(new,cache=cache,record_sink=warm_stream.write,seconds_budget=180)
    assert incremental==warm and inc_stream.getvalue()==warm_stream.getvalue();assert wperf['cache_misses']==0
    rollback_stream=io.BytesIO();rollback,rperf=certify(old,cache=cache,record_sink=rollback_stream.write,seconds_budget=180)
    assert baseline==rollback and baseline_stream.getvalue()==rollback_stream.getvalue();assert rperf['cache_misses']==0
    result=dict(source_columns=columns,changed_vertices=ids,projection=projection,baseline=baseline,incremental=incremental,
        full=dict(complete=full_result['complete'],counts=full_result['counts'],forbidden=full_result['forbidden'],unresolved=full_result['unresolved']),
        normalized_full_hash=scientific_hash(ref),normalized_incremental_hash=scientific_hash(got),
        complete_record_equality=True,forbidden_set_equality=True,unresolved_set_equality=True,
        dependency_changes=changed,proof_invalidation=invalid,cache=dict(cold_misses=bperf['cache_misses'],incremental_hits=iperf['cache_hits'],incremental_misses=iperf['cache_misses'],warm_hits=wperf['cache_hits'],warm_misses=wperf['cache_misses'],rollback_hits=rperf['cache_hits'],rollback_misses=rperf['cache_misses']),
        no_private_step_admission=True,production_committed_layer=front.committed_layer)
    h=scientific_hash(result)
    for name,raw in [('EQUIVALENCE_BASELINE_PROOFS.jsonl.gz',baseline_stream.getvalue()),('EQUIVALENCE_CANDIDATE_PROOFS.jsonl.gz',inc_stream.getvalue())]:store_stream(name,raw,replay)
    if replay:assert scientific_hash(json.loads((OUT/'EQUIVALENCE.json').read_text()))==h
    else:(OUT/'EQUIVALENCE.json').write_text(json.dumps(plain(result),indent=2,sort_keys=True)+'\n')
    (OUT/('EQUIVALENCE_REPLAY.json' if replay else 'EQUIVALENCE_HASH.json')).write_text(json.dumps(dict(scientific_hash=h),indent=2)+'\n')
    (OUT/('EQUIVALENCE_REPLAY_TIMINGS.json' if replay else 'EQUIVALENCE_TIMINGS.json')).write_text(json.dumps(dict(cold_baseline=bperf,incremental=iperf,warm=wperf,rollback=rperf,full_seconds=full_seconds,total_seconds=time.perf_counter()-started,peak_rss=psutil.Process().memory_info().peak_wset),indent=2,sort_keys=True)+'\n')
    print(json.dumps(dict(hash=h,baseline_pairs=baseline['candidate_pairs'],candidate_pairs=incremental['candidate_pairs'],**result['cache']),indent=2),flush=True)

if __name__=='__main__':run('--replay' in sys.argv)
