"""Read-only L5 integration preflight. Never invokes admission or generation."""
from pathlib import Path
import json
import shutil
import sqlite3
import subprocess
import time
from unittest.mock import patch

import psutil
from clean_sheet_candidate import Candidate
from clean_sheet_resume import require_l5_scope
from clean_sheet_l4_bootstrap import SURFACE, REQUEST, metrics
from geometry_family_authority import Source
from path_preserving_deduplication import durable_json, file_hash, metadata, physical_key
from production_surface_authority import load_production_wall
from production_forecast_admission import ProductionForecastAdapter, BoundProductionForecastCache
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.common.authority import identity

V4 = Path(__file__).resolve().parents[1]
REPO = V4.parents[1]
PRIOR = V4/'results/clean_sheet_incremental_l4_to_l5/resume'
OUT = V4/'results/layer6_incremental_readiness'
L5 = '686c220a55965e0ee2868f2ab8502c6cda3ef6b6db8e97fca4806cf04afc0ec7'
ROOT = '126b582745596b58a7f191f936e3a7906170c20e665ed6c9d226d52cc4566e51'
CONTENT = '7c8ad64863ecd707279f4f9420c9fc543eeaa9f01bfd213cf11d3cc0e755e062'


def read(p):
    return json.loads(Path(p).read_text())


def denied(*args, **kwargs):
    raise AssertionError('PreflightForbidsSQLLegacyRestartAndForecast')


def rejection(call, expected):
    try:
        call()
    except ValueError as exc:
        assert str(exc) == expected
        return str(exc)
    raise AssertionError('ExpectedIntegrationRejection')


def run():
    start = time.perf_counter()
    free = shutil.disk_usage(REPO.anchor).free
    head = subprocess.check_output(['git','rev-parse','HEAD'], text=True).strip()
    assert head == 'd168512162945d1d57f1dae9cfe9913589394609'
    branch = subprocess.check_output(['git','branch','--show-current'], text=True).strip()
    assert branch == 'feature/sdf-cfd-mesher-v4-advancing-front'
    raw = subprocess.check_output(['git','status','--porcelain=v1','-z'])
    unrelated = [r for r in raw.split(b'\0') if r and b'layer6_incremental_readiness' not in r]
    previous = read(PRIOR.parent/'BASELINE.json')
    assert b'\0'.join(unrelated).hex() == previous['status_hex']
    assert len(unrelated) == 155
    protected = {}
    for folder in (PRIOR, V4/'results/incremental_immutable_state_access', V4/'results/dependency_graph_scaling_audit'):
        authority = read(folder/'AUTHORITY.json')
        if 'scientific' in authority:
            assert identity(authority['scientific']) == authority['identity']
            expected = {'incremental_immutable_state_access': '7744d0bc88a60f75e6048edb105c983bcc351e9946b3710b32faa9e9df76e5a8',
                        'dependency_graph_scaling_audit': '612ae1d9d3663d29a47cdca5b01c347c4bf691637355848e7ddac7b6df8e88df'}
            assert authority['identity'] == expected[folder.name]
            refs = authority['scientific']['evidence']
        else:
            assert authority['qualification_identity'] == '1cb74365c156d207ed93260eb9d16ee0988bb6dd6381ac6e990a3c98f4c83d87'
            refs = authority['files']
        for name, digest in refs.items():
            p = folder/name
            assert file_hash(p) == digest, str(p)
            protected[str(p)] = digest
        protected[str(folder/'AUTHORITY.json')] = file_hash(folder/'AUTHORITY.json')
    native = V4/'results/production_adaptive_capsule_layers4_to5/L5/A/checkpoints'
    current = read(native/'CURRENT.json')
    checkpoint = native/current['checkpoint']
    assert file_hash(checkpoint) == 'c623ccf811899964fb9e8e537339730bfb2c037f9dad99b5be1ce39bfd60f98b'
    assert read(checkpoint)['scientific'] == L5
    protected[str(checkpoint)] = file_hash(checkpoint)
    protected[str(native/'CURRENT.json')] = file_hash(native/'CURRENT.json')
    assert read(PRIOR/'CANDIDATE_SEAL.json')['legacy_scientific'] == L5
    with patch.object(sqlite3, 'connect', denied), patch.object(NativeCheckpointStore, 'restart', denied), patch.object(BoundProductionForecastCache, 'forecast', denied):
        tick = time.perf_counter(); cpu = time.process_time(); io = psutil.Process().io_counters()
        with Candidate(PRIOR/'candidate', ROOT) as state:
            assert state.plan['scientific'] == CONTENT
            wall, tri, surface = load_production_wall(SURFACE)
            assert surface['surface_identity'] == '14770350129d217662b9e936b2b2bd39d9ea3e14b47ad7fedc9e47425da1aafd'
            assert surface['geometry_identity'] == 'a52676253aa7996b6ac4d453f40ae838aa9d569d883b8fa64fe6799cd22cbc65'
            source = Source(dict(components=[read(SURFACE.parent/'GEOMETRY.json')['definition']]))
            adapter = state.initialize(wall, tri, surface, REQUEST, source)
            assert adapter.binding['request'] == 'ec28d584b04afb8bd4679d6719d0f600140b63dcff26a0cc50c2607537d33b91'
            assert adapter.binding['schedule'] == 'ae2cfd505b0cb52218c4f000688df150e8f2c65f2d2d28de78785f1de39f0597'
            assert adapter.binding['layer'] == 5 and adapter.binding['next_layer'] == 6
            increment = float(adapter.remaining_increments[0])
            assert increment == 0.00035506009452908446
            assert increment.hex() == '0x1.744eb7e7e7516p-12'
            measured = metrics(tick, cpu, io)
            failures = {
                'qualified_admission_harness': rejection(lambda: require_l5_scope(adapter), 'CleanSheetRequiresL4ToL5'),
                'public_immutable_adapter': rejection(lambda: ProductionForecastAdapter.from_immutable_access(state, wall, tri, surface, REQUEST, source, state.plan['expected']), 'UnqualifiedImmutableAccessType'),
            }
            missing_body = [k for k in ('components','content_root','files','expected') if k not in state.body]
            missing_api = [k for k in ('verified_legacy_identity','directory','_root','store','_object_path','_verify_read') if not hasattr(state,k)]
            assert missing_body and missing_api
            refs = {**state.plan['references'], **state.body['produced']}
            stats = dict(state.stats); base_stats = dict(state.base.stats)
            binding = adapter.binding
    # Allocation metadata only; no historical receipt/quality content is opened.
    physical = [metadata(p) for p in refs]
    unique = {physical_key(r):r for r in physical}
    for p, digest in protected.items():
        assert file_hash(p) == digest
    result = dict(
        classification='V4CapsuleLayer6IncrementalEndToEndBlockedNonComposableBase',
        claim='L5 trusted open and exact L6 increment binding pass; the qualified L4-to-L5 assembly cannot consume its own incremental output as the next base.',
        starting_head=head, branch=branch, authoritative_layer=5, committed=L5,
        content_root=CONTENT, access_root=ROOT, increment=increment, increment_hex=increment.hex(),
        binding=binding, protected_files=protected, protected_files_checked=len(protected),
        rejections=failures, missing_base_body=missing_body, missing_base_api=missing_api,
        normal_open_init=measured, candidate_stats=stats, base_stats=base_stats,
        sql_rows=0, historical_proof_payloads=0, historical_quality_rows=0,
        full_prefix_scientific_hashes=0, full_prefix_deep_copies=0,
        prior_reference_path_count=len(physical), prior_unique_allocated_bytes=sum(r['cluster_bytes'] for r in unique.values()),
        starting_free_bytes=free, ending_free_bytes=shutil.disk_usage(REPO.anchor).free,
        L6_admission=False, L6_forecast=False, L6_generation=False, L6_publication=False,
        L7_admission=False, L7_generation=False, core=False,
        unrelated_dirty_count=len(unrelated), unrelated_status_hex=b'\0'.join(unrelated).hex(),
        total_probe_seconds=time.perf_counter()-start,
        limitations='No L6 geometry, scientific certification, publication, recovery, replay or retention was attempted. No claim of L6 storage capacity or speedup. Allocation is the referenced immutable closure, not all historical evidence.',
        next_task='QualifyV4ComposableIncrementalBaseAccessAndTransitionFromSealedLayer5')
    OUT.mkdir(parents=True, exist_ok=True)
    durable_json(OUT/'PREFLIGHT.json', result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('protected_files','unrelated_status_hex','binding')}, indent=2))


if __name__ == '__main__':
    run()
