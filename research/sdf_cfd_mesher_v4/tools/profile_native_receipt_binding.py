"""Bounded exact proof-binding allocation reproducer using preserved receipts."""
import sys
sys.dont_write_bytecode=True
import json,itertools,sqlite3,time,tracemalloc,tempfile
from pathlib import Path
from run_production_scale_resume import OUT
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import receipt_table_from_rows,scientific_digest


def run():
    source=OUT/'capsule_production_control/A/checkpoints'
    pointer=json.loads((source/'CURRENT.json').read_text());envelope=json.loads((source/pointer['checkpoint']).read_text())
    dbpath=source/'objects'/envelope['components']['certificate']['file']
    def rows():
        with sqlite3.connect(dbpath.resolve().as_uri()+'?mode=ro',uri=True) as db:
            for key,value in db.execute('SELECT key,value FROM receipts ORDER BY key LIMIT 10000'):
                yield key,json.loads(value)
    records=[]
    with tempfile.TemporaryDirectory() as tmp:
        for mode in ('expanded','native_stream'):
            tracemalloc.start();tick=time.perf_counter()
            value=dict(rows()) if mode=='expanded' else receipt_table_from_rows(Path(tmp),rows())
            current,peak=tracemalloc.get_traced_memory();seconds=time.perf_counter()-tick;tracemalloc.stop()
            digest=scientific_digest(value)
            records.append(dict(mode=mode,records=len(value),seconds=seconds,retained_python_bytes=current,peak_python_bytes=peak,scientific=digest))
            del value
    assert records[0]['scientific']==records[1]['scientific']
    record=dict(source_checkpoint=pointer,rows=records,passed=True,scope='Measured Python allocations for binding 10000 actual native proof receipts; excludes source disk and full-run RSS')
    (OUT/'NATIVE_BIND_PROFILE.json').write_text(json.dumps(record,indent=2));print(json.dumps(record))

if __name__=='__main__':run()
