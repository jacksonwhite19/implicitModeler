"""Production exact-record writer over immutable fronts; never mesh creation."""
from task4h_common import *
import task4_certify_candidate as runner
import psutil
def run():
    require_time();start=time.perf_counter();front=runner.load_sf2_wall();top=runner.build_topology(front.triangles,len(front.coordinates));low=front.coordinates;rows=[]
    temporary=OUT/'EXACT_RECORD_WRITER_FIXTURE.jsonl.gz'
    for layer in range(1,21):
        require_time();high=np.load(BASE/'build_A'/f'FRONT_{layer:02}.npy');result,seconds=runner.exact_all(top,low,high,temporary)
        assert result==read(BASE/f'LAYER_{layer:02}_EXACT_SIGNS.json')
        assert file_hash(temporary)==file_hash(BASE/f'LAYER_{layer:02}_EXACT_SIGNS.jsonl.gz')
        rows.append(dict(layer=layer,cells=len(top.triangles),seconds=seconds,old_seconds=read(BASE/f'LAYER_{layer:02}_TIMINGS.json')['exact_sign_seconds'],record_hash=result['records_sha256'],byte_identical=True));low=high
        save('PRODUCTION_JACOBIANS.json',dict(rows=rows,peak_rss=psutil.Process().memory_info().peak_wset,total_seconds=time.perf_counter()-start))
    # Delete only the named, task-owned temporary output after every hash passes.
    assert temporary.resolve().parent==OUT.resolve();temporary.unlink()
if __name__=='__main__':run()
