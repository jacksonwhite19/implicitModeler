"""Remove only proven unpublished replay duplicates after final qualification."""
import subprocess
import sys
from qualify_recursive_layers9to11 import *
from path_preserving_deduplication import metadata
from recursive_production_verification import inventory
from retain_recursive_layer6 import check_duplicate, recover


def run():
    assert read(OUT/'FINAL_DEEP.json')['passed'] and read(OUT/'FINAL_REPLAY.json')['passed']
    if (OUT/'FINAL_RETAIN.json').exists():
        raise ValueError('PreserveCompletedRetention')
    tick = measured(); results = {}
    for layer in (9,10,11):
        out = OUT/f'L{layer}'; replay = (out/'replay').resolve()
        before = inventory(out)
        bound = set(read(out/'CANDIDATE_PLAN.json')['references'])
        rows = []
        for path in sorted(replay.rglob('*')):
            if not path.is_file():
                continue
            path = path.resolve(); canonical = (out/path.relative_to(replay)).resolve()
            if not canonical.is_file():
                continue
            digest = file_hash(path)
            if file_hash(canonical) != digest:
                continue
            check_duplicate(path,canonical,replay,bound,digest)
            record = metadata(path)
            assert record['links'] == 1
            rows.append(dict(path=str(path),canonical=str(canonical),sha256=digest,metadata=record))
        durable_json(out/'FINAL_CLEANUP_MANIFEST.json',dict(rows=rows,policy='Exact unpublished replay duplicates only'))
        removed = []
        try:
            for row in rows:
                check_duplicate(row['path'],row['canonical'],replay,bound,row['sha256'])
                Path(row['path']).unlink(); removed.append(row)
            subprocess.run([sys.executable,'-c',
                "import sys; from verify_recursive_layers9to11 import reopen; from qualify_recursive_layers9to11 import OUT,durable_json; n=int(sys.argv[1]); p=OUT/f'L{n}'; durable_json(p/'FINAL_POST_REOPEN.json',reopen(p,n))",
                str(layer)],check=True)
        except BaseException:
            recover(removed,replay,bound)
            raise
        results[str(layer)] = dict(passed=True,before=before,after=inventory(out),removed_paths=len(removed),
            physical_reclaimed=sum(r['metadata']['cluster_bytes'] for r in removed),checkpoint_retirement=False)
    result = dict(passed=True,layers=results,metrics=metrics(*tick))
    durable_json(OUT/'FINAL_RETAIN.json',result)
    print({k:{n:v for n,v in r.items() if n not in ('before','after')} for k,r in results.items()},flush=True)


if __name__ == '__main__':
    run()
