"""Compact deterministic scientific storage; JSON pointers stay inspectable."""
from pathlib import Path
import gzip,json
from sdf_cfd_mesher_v4.common.scientific_stream import canonical_chunks,scientific_hash

def read(path):
    path=Path(path);value=json.loads(path.read_text())
    if isinstance(value,dict) and value.get('format') in ('Task3CExactJsonGzip/1','Task3CExactJsonGzip/2'):
        with gzip.open(path.parent/value['payload'],'rb') as f:result=json.load(f)
        for target,source in value.get('references',{}).items():
            result['final'][target]=result['final'][source]
        assert scientific_hash(result)==value['scientific_sha256']
        return result
    return value

def write(path,value):
    path=Path(path);payload=path.with_suffix(path.suffix+'.gz');refs={};encoded=value
    if isinstance(value,dict) and 'final' in value:
        encoded=dict(value);encoded['final']=dict(value['final'])
        for target,source in [('candidate','baseline'),('final_tails','baseline_tails')]:
            if target in value['final'] and source in value['final'] and scientific_hash(value['final'][target])==scientific_hash(value['final'][source]):
                refs[target]=source;encoded['final'].pop(target)
    with payload.open('wb') as f:
        with gzip.GzipFile(filename='',mode='wb',fileobj=f,mtime=0) as z:
            for block in canonical_chunks(encoded):z.write(block)
    path.write_text(json.dumps(dict(format='Task3CExactJsonGzip/2',payload=payload.name,references=refs,scientific_sha256=scientific_hash(value)),indent=2,sort_keys=True)+'\n')

if __name__=='__main__':
    import time,psutil
    started=time.perf_counter()
    root=Path(__file__).resolve().parents[1]/'results/task3c'
    for pattern in ('component_*_ring*.json','component_*_closed_support.json','SENSITIVITY.json'):
        for path in sorted(root.glob(pattern)):
            assert path.resolve().is_relative_to(root.resolve())
            write(path,read(path))
    (root/'ARCHIVE_TIMINGS.json').write_text(json.dumps(dict(seconds=time.perf_counter()-started,peak_rss=psutil.Process().memory_info().peak_wset),indent=2)+'\n')
