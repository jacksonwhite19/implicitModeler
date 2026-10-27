from task4h_common import *
import gzip,io,cProfile,pstats,psutil
import task4_prepare_next as prod
from sdf_cfd_mesher_v4.forecast.forensics import exact_wedge as old
from sdf_cfd_mesher_v4.forecast.exact_dyadic import exact_wedge,exact_signs
from sdf_cfd_mesher_v4.certificate.continuous_prism_certificate.runtime import canonical_bytes

def run():
    require_time();start=time.perf_counter();front=prod.load_sf2_wall();tri=front.triangles;low=front.coordinates;rows=[]
    for layer in range(1,21):
        require_time();high=np.load(BASE/'build_A'/f'FRONT_{layer:02}.npy');p=np.concatenate((low[tri],high[tri]),axis=1)
        t=time.perf_counter();new=[exact_wedge(x) for x in p];new_seconds=time.perf_counter()-t
        baseline_seconds=None
        if layer in (1,16):
            t=time.perf_counter();reference=[old(x) for x in p];baseline_seconds=time.perf_counter()-t
            assert new==reference,(layer,'ExactValueMismatch');del reference
        t=time.perf_counter();signs=[exact_signs(x) for x in p];sign_seconds=time.perf_counter()-t
        assert all(s['jacobian_sign']==r['jacobian_sign'] and s['volume_sign']==r['volume_sign'] for s,r in zip(signs,new))
        h=hashlib.sha256();buffer=io.BytesIO();t=time.perf_counter()
        with gzip.GzipFile(filename='',mode='wb',fileobj=buffer,mtime=0) as sink:
            for c,r in enumerate(new):
                encoded=canonical_bytes(dict(column=c,**r));sink.write(encoded);h.update(encoded)
        serialization_seconds=time.perf_counter()-t
        receipt=read(BASE/f'LAYER_{layer:02}_EXACT_SIGNS.json');assert h.hexdigest()==receipt['records_sha256'],(layer,'FrozenRecordHashMismatch')
        stored_gzip=BASE/f'LAYER_{layer:02}_EXACT_SIGNS.jsonl.gz'
        compressed_hash=hashlib.sha256(buffer.getbuffer()).hexdigest();assert compressed_hash==file_hash(stored_gzip),(layer,'StoredBytesMismatch')
        # Independent replay of all values; same canonical hash, no timing fields.
        t=time.perf_counter();h2=hashlib.sha256()
        for c,x in enumerate(p):h2.update(canonical_bytes(dict(column=c,**exact_wedge(x))))
        assert h2.hexdigest()==h.hexdigest()
        timing=read(BASE/f'LAYER_{layer:02}_TIMINGS.json')
        rows.append(dict(layer=layer,cells=len(p),old_kernel_seconds=baseline_seconds,new_kernel_seconds=new_seconds,sign_only_seconds=sign_seconds,serialization_compression_seconds=serialization_seconds,stored_complete_stage_seconds=timing['exact_sign_seconds'],records_hash=h.hexdigest(),replay_hash=h2.hexdigest(),gzip_hash=compressed_hash,exact_values_equal=True,exact_signs_equal=True,canonical_bytes_equal=True,replay_seconds=time.perf_counter()-t))
        save('JACOBIAN_EQUIVALENCE.json',dict(rows=rows,completed_layers=layer,peak_rss=psutil.Process().memory_info().peak_wset,total_seconds=time.perf_counter()-start))
        print('JACOBIAN_LAYER',layer,new_seconds,baseline_seconds,flush=True);low=high
    pr=cProfile.Profile();pr.enable()
    for x in p[:4096]:exact_wedge(x)
    pr.disable();profile=[]
    for (f,line,name),(cc,nc,tt,ct,callers) in pstats.Stats(pr).stats.items():profile.append(dict(file=Path(f).name,line=line,name=name,calls=nc,self_seconds=tt,total_seconds=ct))
    save('JACOBIAN_NEW_PROFILE.json',dict(cells=4096,call_tree=sorted(profile,key=lambda x:-x['self_seconds'])))
if __name__=='__main__':run()
