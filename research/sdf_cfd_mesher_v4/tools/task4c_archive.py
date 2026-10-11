"""Reference identical sealed payloads after full fresh proof qualification.

Only this study's redundant files are removed, with resolved path checks.
The fresh manifests, timings, identities and native validity data remain here.
"""
from task4c_common import *

def verify_reference(path):
    ref=read(path/'REFERENCE.json');proof=read(path/'MANIFEST.json')
    for key in ('pairs','exact'):
        source=ROOT/ref[key]['path'];assert file_hash(source)==ref[key]['sha256']
    with np.load(ROOT/ref['pairs']['path']) as z:
        for key,field in [('packed','pair_hash'),('codes','code_hash'),('witness','witness_hash'),('outcomes','outcome_hash')]:assert array_hash(z[key])==proof[field]
    assert scientific_hash(read(ROOT/ref['exact']['path']))==proof['fallback_hash']
    with np.load(path/'VALIDITY.npz') as z:assert array_hash(z['validity'])==proof['validity_hash']
    return proof['candidate_pairs']

def run():
    removed=0;rows=[]
    for layer in range(1,21):
        path=OUT/f'certificate_layer_{layer:02}';proof=read(path/'MANIFEST.json');pairs=B/f'optimized_2/PAIRS_{layer:02}.npz';exact=B/f'optimized_2/EXACT_{layer:02}.json'
        if not (path/'REFERENCE.json').exists():
            with np.load(path/'COMPACT.npz') as a,np.load(pairs) as b:
                for key in ('packed','codes','witness','outcomes'):assert np.array_equal(a[key],b[key])
                np.savez_compressed(path/'VALIDITY.npz',validity=a['validity'])
            import gzip
            with gzip.open(path/'EXACT.json.gz','rb') as f:own=json.load(f)
            assert scientific_hash(own)==scientific_hash(read(exact))==proof['fallback_hash']
            ref={key:dict(path=p.relative_to(ROOT).as_posix(),sha256=file_hash(p)) for key,p in [('pairs',pairs),('exact',exact)]}
            ref['reason']='Fresh production-selected proof arrays and exact fallback records matched the sealed Task-4B payload exactly.'
            (path/'REFERENCE.json').write_text(json.dumps(ref,sort_keys=True,indent=2)+'\n')
        verify_reference(path)
        for filename in ('COMPACT.npz','EXACT.json.gz'):
            target=(path/filename).resolve();assert target.is_relative_to(OUT.resolve())
            if target.exists():removed+=target.stat().st_size;target.unlink()
        rows.append(dict(layer=layer,pairs=proof['candidate_pairs'],reference=file_hash(path/'REFERENCE.json')))
    save('STORAGE_REFERENCES.json',dict(rows=rows,redundant_bytes_removed=removed,scientific_payloads_identical=True))
    print('Referenced qualified identical payloads; redundant bytes removed',removed)

if __name__=='__main__':run()
