import sys,json,sqlite3,time,tracemalloc
from pathlib import Path
from validation_row_stream import validation_digest
from run_production_scale_resume import OUT,native
p=OUT/'capsule_production_control/B/checkpoints';pointer=json.loads((p/'CURRENT.json').read_text());env=json.loads((p/pointer['checkpoint']).read_text());dbpath=p/'objects'/env['components']['certificate']['file']
with sqlite3.connect(dbpath.resolve().as_uri()+'?mode=ro',uri=True) as db:
 rows=[json.loads(v)['value'] for (v,) in db.execute('SELECT value FROM receipts ORDER BY key LIMIT 10000')]
checked=dict(passed=True,certificate=dict(passed=True,required_pairs=len(rows),proofs=rows),quality=dict(cells=[],faces=[],exact=[]))
results=[]
for method in ('old_full_clean_json','streamed_validation_digest'):
 tracemalloc.start();tick=time.perf_counter()
 digest=native.g.q.identity(native.g.s.r.w.clean(checked)) if method.startswith('old') else validation_digest(checked)
 current,peak=tracemalloc.get_traced_memory();tracemalloc.stop()
 results.append(dict(method=method,seconds=time.perf_counter()-tick,additional_peak_python_bytes=peak,scientific=digest))
assert results[0]['scientific']==results[1]['scientific']
native.g.save(OUT/'VALIDATION_DIGEST_PROFILE.json',dict(results=results,rows=10000,exact=True,scope='10000 actual preserved proof values; input graph excluded; hash algorithm and canonical bytes unchanged'))
print(json.dumps(results))
