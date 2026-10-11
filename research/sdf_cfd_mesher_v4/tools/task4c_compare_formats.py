"""Read-only exact comparison against the sealed Task-4B format bridge.

Called by a future completed A2 comparison, never launches a build or replay.
"""
from task4c_common import ROOT,BASE,B,read,file_hash,array_hash,scientific_hash,np,json
from sdf_cfd_mesher_v4.geometry.front import load_sf2_wall,build_topology
from sdf_cfd_mesher_v4.forecast.schedule import load_schedule
from sdf_cfd_mesher_v4.advancing_front.rolling_state import CommitStore
from sdf_cfd_mesher_v4.certificate.production import VERSION,surface_arrays,verify_committed_layer
from sdf_cfd_mesher_v4.certificate.replay_equivalence import compare_checkpoint,compare_final_values

def compare_formats(a_path,b_path,a,b):
    assert file_hash(a_path/'AUTHORITY.json')=='be5098ec2885cf1a2d0f1480b356a3d25bca3abd8cf0d984952c781cab393ce9'
    old_authority=json.loads((a_path/'AUTHORITY.json').read_text())
    assert file_hash(a_path/'FINAL_VALIDATION.json')==old_authority['evidence_sha256']['results/task4/FINAL_VALIDATION.json']
    assert scientific_hash(a)==scientific_hash(read(a_path/'FINAL_VALIDATION.json'))
    assert file_hash(B/'AUTHORITY.json')=='8fe4af86b569705ac41399dcdbd0a02ddb7ff41b62a75fc254cd7998bd754aac'
    authority=json.loads((B/'AUTHORITY.json').read_text())
    for path,h in authority['source_sha256'].items():assert file_hash(ROOT/path)==h
    for name in ('FULL_BASELINE_EQUIVALENCE.json','optimized_2/COMPLETENESS.json'):
        rel='results/task4b/'+name;assert file_hash(B/name)==authority['evidence_sha256'][rel]
    bridge=read(B/'FULL_BASELINE_EQUIVALENCE.json');assert len(bridge)==20
    assert all(r['all_pairs_identical'] and r['all_outcomes_identical'] for r in bridge)
    front=load_sf2_wall();top=build_topology(front.triangles,len(front.coordinates));schedule=load_schedule()
    assert CommitStore(a_path/'build_A',front,top,schedule).restart().layer==20
    assert CommitStore(b_path/'build_A',front,top,schedule).restart().layer==20
    coverage=[];prefix=[]
    for layer in range(1,21):
        ac=json.loads((a_path/f'build_A/LAYER_{layer:02}.json').read_text());bc=json.loads((b_path/f'build_A/LAYER_{layer:02}.json').read_text())
        xa=np.load(a_path/f'build_A/FRONT_{layer:02}.npy');xb=np.load(b_path/f'build_A/FRONT_{layer:02}.npy')
        assert np.array_equal(xa.view(np.uint64),xb.view(np.uint64));prefix.append(xb)
        name=f'optimized_2/LAYER_{layer:02}.json';assert file_hash(B/name)==authority['evidence_sha256']['results/task4b/'+name]
        ref=read(B/name)
        if bc['certificate'].get('adapter')==VERSION:
            verify_committed_layer(front,top,prefix,b_path/f'certificate_layer_{layer:02}',bc['certificate'])
            coverage.append(compare_checkpoint(ac,bc,ref))
        else:
            # A partially completed old-format A2 prefix remains byte-exact.
            assert scientific_hash(ac)==scientific_hash(bc)
            # Convert only the proof representation for the semantic comparator.
            view=dict(bc,certificate=dict(bc['certificate'],pair_hash=ref['pair_hash'],outcome_hash=ref['outcome_hash']))
            coverage.append(compare_checkpoint(ac,view,ref))
    aa,bb=compare_final_values(a,b,coverage)
    return dict(A=a['shell_scientific_hash'],A2=b['shell_scientific_hash'],raw_storage_hashes_equal=a['shell_scientific_hash']==b['shell_scientific_hash'],
        passed=aa==bb,semantic_A=aa,semantic_A2=bb,complete_pair_outcome_comparison=True,coverage=coverage,
        representation_change='Task-4B qualified proof hierarchy; geometry, connectivity, quality, decisions, SDF and conditioning compared exactly.',
        independent_exact_proof_cache=True)
