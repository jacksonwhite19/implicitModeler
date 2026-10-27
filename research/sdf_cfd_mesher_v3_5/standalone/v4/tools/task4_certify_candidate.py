"""Complete immediate exact signs, side certificate, then atomic publication."""
from task4_initial_scan import *
import gzip,hashlib,threading
from sdf_cfd_mesher_v4.advancing_front.rolling_state import CommitStore
from sdf_cfd_mesher_v4.advancing_front.rolling_metrics import evaluate_cells,validate_admission_history
from sdf_cfd_mesher_v4.advancing_front.rolling_cache import load_candidate
from sdf_cfd_mesher_v4.forecast.exact_dyadic import exact_wedge
from sdf_cfd_mesher_v4.certificate.production import certify_immediate
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate.runtime import canonical_bytes
from task3c_io import read
from sdf_cfd_mesher_v4.advancing_front.validation_evidence import input_identity,make_receipt

class MemorySampler:
    """Observed aggregate RSS of this exact-certification process tree."""
    def __init__(self):self.stop=threading.Event();self.peak=0;self.samples=0;self.process=psutil.Process()
    def sample(self):
        while not self.stop.is_set():
            processes=[self.process]+self.process.children(recursive=True);rss=0
            for process in processes:
                try:rss+=process.memory_info().rss
                except (psutil.NoSuchProcess,psutil.AccessDenied):pass
            self.peak=max(self.peak,rss);self.samples+=1;self.stop.wait(.1)
    def __enter__(self):self.thread=threading.Thread(target=self.sample,daemon=True);self.thread.start();return self
    def __exit__(self,*_):self.stop.set();self.thread.join()

def exact_all(top,low,high,path):
    path=Path(path);h=hashlib.sha256();bad=[];started=time.perf_counter();count=0
    with path.open('wb') as file:
        with gzip.GzipFile(filename='',mode='wb',fileobj=file,mtime=0) as sink:
            for c,ids in enumerate(top.triangles):
                record=dict(column=c,**exact_wedge(np.concatenate((low[ids],high[ids]))));encoded=canonical_bytes(record);sink.write(encoded);h.update(encoded);count+=1
                if record['jacobian_sign']<=0 or record['volume_sign']<=0:bad.append(record)
                if c%10000==0:print('EXACT_SIGNS',c,len(top.triangles),time.perf_counter()-started,flush=True)
    return dict(passed=not bad,cells=count,failures=bad,records_sha256=h.hexdigest()),time.perf_counter()-started

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--layer',type=int,required=True);args=parser.parse_args();layer=args.layer
    started=time.perf_counter();front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));schedule=load_schedule();sdf=NativeSdf()
    workers=int(os.environ.get('V4_CERTIFICATE_WORKERS','12'))
    if not 1<=workers<=12:raise ValueError('CertificateWorkerComputationalBound')
    store=CommitStore(OUT/'build_A',front,top,schedule);state=store.initialize();assert state.layer==layer-1
    candidate,forecast,_=load_candidate(OUT/f'LAYER{layer}_CANDIDATE.npz',OUT/'INITIAL_FORECAST.npz')
    decisions=read(OUT/('CORRECTION_HISTORY.json' if layer==1 else f'LAYER{layer}_SEARCH.json'))
    validate_admission_history(decisions['history'])
    previous=None
    if layer>1:
        low=front.coordinates if layer==2 else np.load(store.path/f'FRONT_{layer-2:02}.npy')
        previous=evaluate_cells(top,low,state.coordinates,layer-1)
    numeric=evaluate_layer(top,state.coordinates,candidate,layer,previous)
    # The prior cell centre/volume primitives are independent of its old cap
    # faces. Only actual new side faces and the new shared cap are admitted.
    write(OUT/f'LAYER_{layer:02}_NUMERIC.json',dict(gate=numeric['gate'],events=numeric['events']))
    if not numeric['gate']['passed']:raise RuntimeError('ImmediateNumericFailure')
    sdf_started=time.perf_counter();q=sdf.evaluate(candidate);assert np.all(abs(q.phi-schedule.targets[layer-1])<=q.source_error_bound)
    sdf_seconds=time.perf_counter()-sdf_started
    write(OUT/f'LAYER_{layer:02}_SDF.json',dict(target=float(schedule.targets[layer-1]),maximum_residual=float(abs(q.phi-schedule.targets[layer-1]).max()),
        phi_hash=array_hash(q.phi),gradient_hash=array_hash(q.gradient),gradient_norm_min=float(q.gradient_norm.min()),gradient_norm_max=float(q.gradient_norm.max())))
    signs,sign_seconds=exact_all(top,state.coordinates,candidate,OUT/f'LAYER_{layer:02}_EXACT_SIGNS.jsonl.gz')
    write(OUT/f'LAYER_{layer:02}_EXACT_SIGNS.json',signs)
    if not signs['passed']:raise RuntimeError('ImmediateExactSignFailure')
    prefix=[np.load(store.path/f'FRONT_{k:02}.npy') for k in range(1,layer)]+[candidate]
    certificate_started=time.perf_counter()
    with MemorySampler() as memory:
        proof,timings=certify_immediate(front,top,prefix,OUT/f'certificate_layer_{layer:02}',workers=workers,
            progress=lambda k,b,e,p,t:print('CERTIFICATE_BATCH',k,b,e,p['completed_pairs'],len(p['forbidden']),len(p['unresolved']),t['seconds'],flush=True))
    write(OUT/f'LAYER_{layer:02}_CERTIFICATE.json',proof)
    performance=dict(exact_sign_seconds=sign_seconds,numeric_seconds=numeric['seconds'],sdf_validation_seconds=sdf_seconds,certificate_wall_seconds=time.perf_counter()-certificate_started,certificate_stages=timings,total_seconds=time.perf_counter()-started,peak_rss=psutil.Process().memory_info().peak_wset)
    performance.update(certificate_workers=workers,observed_tree_peak_rss=memory.peak,memory_samples=memory.samples,memory_sample_interval_seconds=.1)
    (OUT/f'LAYER_{layer:02}_TIMINGS.json').write_text(json.dumps(performance,indent=2)+'\n')
    if not proof['complete'] or any(proof[k] for k in ('forbidden','unresolved','invalid_surfaces')):raise RuntimeError('ImmediateContinuousFailure')
    warnings=read(OUT/f'LAYER{layer}_WARNINGS.json')
    corrections=read(OUT/'CORRECTION_HISTORY.json')['outcomes'] if layer==1 else read(OUT/f'LAYER{layer}_CORRECTIONS.json')
    checkpoint_started=time.perf_counter();next_state=store.commit(state,candidate,numeric['gate'],signs,proof,array_hash(forecast),warnings,corrections)
    performance.update(checkpoint_seconds=time.perf_counter()-checkpoint_started,total_seconds=time.perf_counter()-started)
    (OUT/f'LAYER_{layer:02}_TIMINGS.json').write_text(json.dumps(performance,indent=2)+'\n')
    np.savez_compressed(OUT/f'LAYER_{layer:02}_CONDITIONING.npz',sigma=numeric['cells']['sigma'],G_bound=numeric['cells']['G_bound'],G_bound_available=numeric['cells']['G_bound_available'])
    prior_low=None if layer==1 else front.coordinates if layer==2 else np.load(store.path/f'FRONT_{layer-2:02}.npy')
    inputs=input_identity(top,state.coordinates,candidate,prior_low,layer,schedule.targets,sdf.source_identity)
    conditioning={k:numeric['cells'][k] for k in ('sigma','G_bound','G_bound_available')}
    receipt=make_receipt(top,inputs,numeric['gate'],read(OUT/f'LAYER_{layer:02}_SDF.json'),signs,proof,conditioning)
    # Auxiliary receipt may be rebuilt after interruption. It cannot publish a
    # layer and is written only after the atomic commit has succeeded.
    (OUT/f'LAYER_{layer:02}_VALIDATION_RECEIPT.json').write_bytes((json.dumps(receipt,sort_keys=True,indent=2)+'\n').encode())
    print('COMMITTED',next_state.layer,next_state.state_hash,flush=True)
