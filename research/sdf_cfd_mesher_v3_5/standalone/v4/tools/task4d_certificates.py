"""Bind unchanged Build-A proofs to reproduced geometry; one fresh adapter check."""
from task4d_common import *
from task4c_archive import verify_reference
from sdf_cfd_mesher_v4.certificate.production import surface_arrays,source_identity
from task4_certify_candidate import certify_immediate,MemorySampler
import psutil

def run(fresh=8):
    assert certify_immediate.__module__=='sdf_cfd_mesher_v4.certificate.production'
    f,t,x=geometry();rows=[];started=time.perf_counter()
    for layer in range(1,21):
        tick=time.perf_counter();proof=read(C/f'certificate_layer_{layer:02}/MANIFEST.json')
        q,labels=surface_arrays(f.coordinates,t.edges,x[1:layer+1]);assert array_hash(q)==proof['geometry_hash'];assert array_hash(labels)==proof['labels_hash']
        assert source_identity()==proof['source_identity'];assert not any(proof[k] for k in ('forbidden','unresolved','invalid_surfaces'))
        count=verify_reference(C/f'certificate_layer_{layer:02}')
        rows.append(dict(layer=layer,pairs=count,geometry_hash=proof['geometry_hash'],pair_hash=proof['pair_hash'],outcome_hash=proof['outcome_hash'],reused_exact_proofs=count,seconds=time.perf_counter()-tick))
        del q,labels
    reuse=time.perf_counter()-started
    save('CERTIFICATE_BINDINGS.json',dict(rows=rows,pairs=sum(r['pairs'] for r in rows),reuse_verification_seconds=reuse,adapter=certify_immediate.__module__,forbidden=0,unresolved=0,scope='Authority and geometry binding of sealed complete proofs; not a new exact solve'))
    path=OUT/f'certificate_{fresh:02}';path.mkdir(exist_ok=True)
    proof,perf=certify_immediate(f,t,x[1:fresh+1],path,workers=8)
    old=read(C/f'certificate_layer_{fresh:02}/MANIFEST.json')
    assert scientific_hash(proof)==scientific_hash(old)
    save('FRESH_CERTIFICATE.json',dict(layer=fresh,manifest=proof,performance=perf,identical_scientific_hash=scientific_hash(proof),peak_rss=psutil.Process().memory_info().peak_wset))
    print('CERTIFICATE BINDINGS AND FRESH CHECK PASSED',reuse,perf[0]['seconds'],flush=True)

if __name__=='__main__':
    with MemorySampler() as memory:run()
    save('CERTIFICATE_MEMORY.json',dict(peak_tree_rss=memory.peak,samples=memory.samples))
