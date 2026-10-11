"""Measured allocation reproducer using preserved real proof values."""
import sys
sys.dont_write_bytecode=True
import gc,hashlib,json,sqlite3,time,tracemalloc
from pathlib import Path
from run_production_scale_resume import OUT,native
from validation_row_stream import rows


def run():
    path=native.ROOT/'results/first_layer_determinant_applicability/capsule_production_control/A/checkpoints'
    pointer=json.loads((path/'CURRENT.json').read_text())
    manifest=json.loads((path/pointer['checkpoint']).read_text())
    source=path/'objects'/manifest['components']['certificate']['file']
    with sqlite3.connect(source.resolve().as_uri()+'?mode=ro',uri=True) as db:
        values=[json.loads(v)['value'] for (v,) in db.execute('SELECT value FROM receipts ORDER BY key LIMIT 10000')]
    results=[]
    for name in ('whole_population_json','row_stream'):
        gc.collect();tracemalloc.start();tick=time.perf_counter()
        normalized=json.loads(json.dumps(native.g.s.r.w.clean(values))) if name=='whole_population_json' else rows(values)
        digest=hashlib.sha256()
        for value in normalized:digest.update(json.dumps(value,sort_keys=True,separators=(',',':')).encode())
        current,peak=tracemalloc.get_traced_memory();tracemalloc.stop()
        results.append(dict(mode=name,rows=len(values),seconds=time.perf_counter()-tick,
            additional_python_peak_bytes=peak,remaining_python_bytes=current,scientific=digest.hexdigest()))
        del normalized
    assert results[0]['scientific']==results[1]['scientific']
    native.g.save(OUT/'ROW_NORMALIZATION_PROFILE.json',dict(results=results,exact=True,
        scope='Actual preserved proof values; Python normalization allocations only, not a full-run RSS claim'))


if __name__=='__main__':run()
