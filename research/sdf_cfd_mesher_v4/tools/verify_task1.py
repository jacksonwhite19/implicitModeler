"""One bounded read-only evidence pass. Queries only 17 selected trajectories."""
from pathlib import Path
import sys,json,time,dataclasses
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.dont_write_bytecode=True
import numpy as np
from sdf_cfd_mesher_v4.common.authority import REPO,file_hash,array_hash
from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.sdf.native import NativeSdf
from sdf_cfd_mesher_v4.advancing_front.predict import predict_next_front
from sdf_cfd_mesher_v4.forecast.engine import cache_key
from sdf_cfd_mesher_v4.runtime.regions import dependencies
from sdf_cfd_mesher_v4.runtime.scaling import v3_scaling_case

def main():
    t=time.perf_counter();out=ROOT/'results/task1';b=json.loads((out/'BENCHMARK.json').read_text());checks={}
    expected={'V3_SOURCE_FORK_MANIFEST.json':'949faff4837568fd67449758d34dd15667ab9c32a92ca80c8439934ad4eac5cb','DATA_AUTHORITIES.json':'20f1aeba70dc21dd3e4d8fe19bdb8999d75d5a73ce3a0231f255fe5595838241','V4_SETUP_AUTHORITY.json':'190d04c817c6164f7a004adfa8f257f0d61cf55c19d9b0c970a85493b868d87e'}
    assert all(file_hash(ROOT/'authorities'/n)==h for n,h in expected.items());checks['setup_authorities']=True
    m=json.loads((ROOT/'authorities/V3_SOURCE_FORK_MANIFEST.json').read_text())
    assert all(file_hash(REPO/r['original_path'])==r['original_sha256']==file_hash(ROOT/r['destination']) for r in m['files']);checks['copied_source_and_v3_preservation']=len(m['files'])
    rows=json.loads((ROOT/'authorities/TASK1_SOURCE_BINDING.json').read_text())['files']
    assert all(file_hash(REPO/r['path'])==r['sha256'] for r in rows)
    assert all(file_hash(ROOT/r['destination'])==r['sha256'] for r in rows if r['copied']);checks['task1_source_bindings']=len(rows)
    front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));schedule=load_schedule();sdf=NativeSdf()
    assert front.coordinate_hash==b['initial_front']['coordinate_hash'];assert top.sha256==b['initial_front']['connectivity_hash'];assert array_hash(top.edges)==b['initial_front']['edge_hash'];checks['initial_front']=True
    s=json.loads((out/'SCHEDULE.json').read_text());assert np.array_equal(schedule.targets,[r['target_m'] for r in s['layers']]);checks['schedule']=True
    key=cache_key(front,schedule,sdf);assert dataclasses.asdict(key)==b['cache_key'] and key.sha256==b['cache_key_sha256'];checks['cache_identity']=True
    v=np.load(out/'selected_verifier_samples.npz');x=front.coordinates[v['vertices']];q=sdf.evaluate(x)
    assert np.array_equal(q.phi,v['wall_phi']);assert np.array_equal(q.gradient,v['wall_gradient'])
    for k,target in enumerate(schedule.targets):
        p=predict_next_front(x,target,sdf);assert np.array_equal(p.coordinates,v['coordinates'][k]);assert np.array_equal(p.normals,v['normal'][k]);assert np.array_equal(p.final_phi,v['phi'][k]);assert np.array_equal(p.iterations,v['iterations'][k]);x=p.coordinates
    checks['selected_scalar_gradient_projection_exact']=dict(vertices=len(v['vertices']),projected_states=len(v['vertices'])*20)
    counts=dependencies(top,b['invalidation']['requested_vertices'])
    for name,values in zip(('vertices','edges','triangles'),counts):assert values.tolist()==b['invalidation'][name]
    checks['local_invalidation_counts']=dict(zip(('vertices','edges','triangles'),map(len,counts)))
    assert dataclasses.asdict(v3_scaling_case())==b['scaling'];checks['scaling_arithmetic']=True
    assert b['deterministic_exact'] and b['scientific_hash_run1']==b['scientific_hash_run2'];checks['recorded_complete_replay_hashes_equal']=True
    artifacts={p.relative_to(ROOT).as_posix():dict(sha256=file_hash(p),bytes=p.stat().st_size) for p in sorted(out.glob('*.npz'))}
    # Verify archived coordinates, normals and residuals at the same selected IDs.
    for first in range(0,20,5):
        with np.load(out/f'forecast_layers_{first+1:02}_{first+5:02}.npz') as z:
            assert np.array_equal(z['coordinates'][:,v['vertices']],v['coordinates'][first:first+5])
            assert np.array_equal(z['normals'][:,v['vertices']],v['normal'][first:first+5])
            assert np.array_equal(z['residual'][:,v['vertices']],v['residual'][first:first+5])
    checks['archive_selected_scientific_values']=True
    result=dict(passed=True,seconds=time.perf_counter()-t,checks=checks,artifacts=artifacts,full_forecast_rerun=False,scientific_hash_run1=b['scientific_hash_run1'],scientific_hash_run2=b['scientific_hash_run2'])
    (out/'VERIFIER.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n');print(json.dumps(result,indent=2))

if __name__=='__main__':main()
