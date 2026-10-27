"""Retain bounded restart state and reference identical preserved transactions."""
from task4d_common import *

def remove(path,records,reason):
    path=path.resolve();assert path.is_relative_to(OUT.resolve()) and path.is_file()
    records.append(dict(path=path.relative_to(ROOT).as_posix(),sha256=file_hash(path),bytes=path.stat().st_size,reason=reason));path.unlink()

def run():
    removed=[];checkpoints=[];references=[]
    for directory in ('sequence_1','sequence_2','probe','cold_profile/runner_08','restart_10','cold_native_9','first_runner'):
        path=OUT/directory
        if not path.exists():continue
        for p in sorted(path.glob('LAYER*_PLANNER.npz')):
            layer=int(p.stem[5:].split('_')[0])
            with np.load(p,allow_pickle=False) as z:checkpoint=json.loads(str(z['metadata']))['checkpoint']
            checkpoints.append(dict(directory=directory,layer=layer,checkpoint=checkpoint,original_file_sha256=file_hash(p)))
            if directory=='sequence_2' and layer in (9,20):continue
            remove(p,removed,'Advisory metadata retained; redundant hot cache not needed after equality/restart checks.')
        for p in sorted(path.glob('LAYER*_CANDIDATE.npz')):
            source=BASE/p.name
            from sdf_cfd_mesher_v4.advancing_front.rolling_cache import load_candidate
            own=load_candidate(p,BASE/'INITIAL_FORECAST.npz');original=load_candidate(source,BASE/'INITIAL_FORECAST.npz')
            assert all(np.array_equal(a.view(np.uint64),b.view(np.uint64)) for a,b in zip(own,original))
            references.append(dict(path=p.relative_to(ROOT).as_posix(),source=source.relative_to(ROOT).as_posix(),source_sha256=file_hash(source),scientific_hashes=[array_hash(v) for v in own]))
            remove(p,removed,'Complete coordinate/forecast/normal arrays equal preserved Build A.')
        for p in sorted(path.glob('LAYER*_SEARCH.json')):
            layer=int(p.stem[5:].split('_')[0]);source=BASE/('CORRECTION_HISTORY.json' if layer==1 else p.name)
            assert scientific_hash(read(p))==scientific_hash(read(source))
            references.append(dict(path=p.relative_to(ROOT).as_posix(),source=source.relative_to(ROOT).as_posix(),source_sha256=file_hash(source),scientific_hash=scientific_hash(read(p))))
            payload=p.with_suffix('.json.gz')
            if payload.exists():remove(payload,removed,'Full canonical history equals sealed source; reference retained.')
            remove(p,removed,'Full canonical history equals sealed source; reference retained.')
        if directory=='first_runner':
            for p in sorted(path.glob('REGION_*_CANDIDATE.npz')):
                with np.load(p) as own,np.load(BASE/p.name) as old:
                    assert own.files==old.files
                    assert all(np.array_equal(own[k].view(np.uint64),old[k].view(np.uint64)) for k in own.files)
                references.append(dict(path=p.relative_to(ROOT).as_posix(),source=(BASE/p.name).relative_to(ROOT).as_posix(),source_sha256=file_hash(BASE/p.name)))
                remove(p,removed,'Actual first-runner local arrays equal preserved Build A.')
    save('PLANNER_CHECKPOINTS.json',checkpoints);save('STORAGE_REFERENCES.json',dict(references=references,removed=removed,redundant_bytes_removed=sum(r['bytes'] for r in removed)))
    print('COMPACTED',sum(r['bytes'] for r in removed),flush=True)

if __name__=='__main__':run()
