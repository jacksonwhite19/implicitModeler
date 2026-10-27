"""Reuse first-layer coverage and sealing checks on the live source binding.

NativeCheckpointStore remains the only mesh-state publisher. The final JSON
is an evidence index over its verified checkpoints, not a second publisher.
Scientific replay fields follow seal_production_first_layer.py.
"""
import argparse,json
from run_live_sf2_first_layer import OUT,ROOT,SURFACE,REQUEST,preflight,read,save,sha,identity

FIELDS=['front_coordinate_hash','front_topology_hash','prism_connectivity_hash','ownership_hash','quality_hash','certificate_hash','committed']

def coverage(label):
    from production_first_layer_coverage import run
    import production_first_layer_coverage as module
    if not read(OUT/label/'RESULT.json')['passed']:raise ValueError('CoverageRequiresPassedCandidate')
    original=module.OUT
    try:
        module.OUT=OUT
        run(label)
    finally:module.OUT=original

def compare_realizations(left_folder,right_folder,left_preflight,right_preflight):
    """Existing exact replay checks with explicit run-owned evidence paths."""
    a=read(left_folder/'RESULT.json');b=read(right_folder/'RESULT.json')
    if not a['passed'] or not b['passed'] or not a['committed']:raise ValueError('FirstLayerNotQualified')
    if any(a[k]!=b[k] for k in FIELDS):raise ValueError('FirstLayerScientificReplayMismatch')
    files={}
    # Same authoritative comparisons as the existing capsule seal, plus
    # complete trajectory/reference and volume evidence from this binding.
    for name in ['CANDIDATE.npz','QUALITY.json','DIAGNOSTICS.json','TRAJECTORY.json','TOPOLOGY.json','VOLUME.json','CERTIFICATE_SUMMARY.json','CERTIFICATE.jsonl']:
        left=sha(left_folder/name);right=sha(right_folder/name)
        if left!=right:raise ValueError('FirstLayerFileReplayMismatch:'+name)
        files[name]=left
    for folder in (left_folder,right_folder):
        cov=read(folder/'COVERAGE_RECHECK.json')
        if any(cov[k]!=0 for k in ['forbidden','unresolved','missing','duplicate']):raise ValueError('FirstLayerCoverageFailure')
        checkpoint=folder/'published_checkpoints/CURRENT.json'
        if not checkpoint.exists():checkpoint=folder/'checkpoints/CURRENT.json'
        if read(checkpoint)['scientific']!=a['committed']:raise ValueError('NativeCheckpointMismatch')
    for key in ['dependencies','certificate_binary','surface','request_sha256','height_hex']:
        if left_preflight[key]!=right_preflight[key]:raise ValueError('ReplaySourceIdentityMismatch:'+key)
    replay=dict(exact=True,fields={k:a[k] for k in FIELDS},files=files,
        semantics='Exact stored binary64 trajectory, connectivity, reference, quality, continuous proof and native scientific-state identities; process timings and storage layout excluded.')
    return dict(replay,passed=True)


def replay_and_index():
    current=preflight();pa=read(OUT/'PREFLIGHT_A.json');pb=read(OUT/'PREFLIGHT_B.json')
    for key in ['dependencies','certificate_binary','surface','request_sha256','height_hex']:
        if not current[key]==pa[key]==pb[key]:raise ValueError('ReplaySourceIdentityMismatch:'+key)
    replay=compare_realizations(OUT/'A',OUT/'B',pa,pb)
    a=read(OUT/'A/RESULT.json')
    save(OUT/'REPLAY.json',replay)
    from first_layer_metric_state import VERSION
    scientific=dict(surface=current['surface']['surface_identity'],request=identity(read(REQUEST)),metric_semantics=VERSION,**{k:a[k] for k in FIELDS})
    implementation=dict(current['dependencies'])
    implementation.update(current['certificate_binary'])
    wall_implementation=read(SURFACE/'IMPLEMENTATION.json')
    implementation.update(wall_implementation['dependencies'])
    implementation.update(wall_implementation['binaries'])
    from pathlib import Path
    for name in ['verify_live_sf2_first_layer.py','execute_live_sf2_first_layer.py','test_live_sf2_first_layer.py','production_first_layer_coverage.py','seal_production_first_layer.py']:
        p=Path(__file__).with_name(name);implementation[p.relative_to(ROOT).as_posix()]=sha(p)
    policy_source=ROOT/'research/sdf_cfd_mesher_v3/sdf_guided_advancing_front_qualification/accepted_front_state.py'
    implementation[policy_source.relative_to(ROOT).as_posix()]=sha(policy_source)
    save(OUT/'IMPLEMENTATION.json',implementation)
    index=dict(classification='V4LiveSmoothSF2FirstLayerQualified',first_layer_identity=identity(scientific),
        committed_front_identity=a['committed'],scientific=scientific,generated_layers_per_run=1,realizations=2,
        first_layer_qualified=True,later_layer_qualified=False,core_generated=False,
        geometry=current['surface']['geometry_identity'],implementation_sha256=sha(OUT/'IMPLEMENTATION.json'),
        kernel_source=current['source'],surface_implementation_sha256=sha(SURFACE/'IMPLEMENTATION.json'),
        request_sha256=sha(REQUEST),preflight_sha256={label:sha(OUT/('PREFLIGHT_'+label+'.json')) for label in ['A','B']},
        replay_sha256=sha(OUT/'REPLAY.json'),native_publisher='MixedFrontRunner.initialize -> NativeCheckpointStore.publish/restart',
        canonical_checkpoint='A/checkpoints',verification_checkpoint='B/checkpoints',
        top_role='ProspectiveFirstPrismDeterminant; internal top obligation, complete_neighbor_metrics=false; not permanent terminal cap',
        scope='Exactly this sealed smooth surface, full normal height and applicable first-layer gates; no later-layer, core or CFD qualification.')
    save(OUT/'AUTHORITY.json',index);print(json.dumps(index,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--coverage',choices=['A','B']);parser.add_argument('--seal',action='store_true');args=parser.parse_args()
    if args.coverage:coverage(args.coverage)
    elif args.seal:replay_and_index()
    else:raise ValueError('ExplicitVerificationRequired')
