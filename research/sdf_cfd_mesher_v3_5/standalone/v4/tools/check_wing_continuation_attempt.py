"""Record the authorized campaign and probe the existing admission boundary.

This is a fail-closed execution preflight, not an alternate layer runner.
It never changes diagnostic authorization, source locks, or scientific inputs.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
import time

V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
sys.path[:0] = [str(V4/'src'), str(V4/'tools')]
HEAD = '3d32521cff07fdc16eec5061f31b5008a91d21b2'


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def run(out, expected_head=HEAD):
    import numpy as np
    import psutil
    from sdf_cfd_mesher_v4.common.authority import identity, file_hash
    from path_preserving_deduplication import durable_json
    from check_sf2_continuation import existing_state_class, exact_vertex_map, STUDY, PRIOR
    from accepted_trajectory_contract import AcceptedTrajectoryOwner, bind_forecast, plan_transition
    from sealed_front_connection import load_case, qualified_dispatch, inspect_front, indexed_session
    from composable_state import CommittedState
    from run_live_sf2_surface import CanonicalKernel, bind_canonical, bind_sf2_policy, verify_kernel_sources

    started = time.perf_counter()
    git = lambda *args: subprocess.check_output(['git', *args], cwd=ROOT).decode('utf-8').strip()
    actual_head = git('rev-parse', 'HEAD')
    if actual_head != expected_head:
        raise ValueError('UnexpectedStartingHead')
    branch = git('branch', '--show-current')
    if branch != 'feature/sdf-cfd-mesher-v4-advancing-front':
        raise ValueError('UnexpectedBranch')
    # Preserve exact case-sensitive status; ignore only this new checker.
    status = subprocess.check_output(['git','-c','core.quotepath=false','status','--porcelain=v1'],cwd=ROOT).decode('utf-8').splitlines()
    status = [s for s in status if s != '?? research/sdf_cfd_mesher_v4/tools/check_wing_continuation_attempt.py']
    old = read(V4/'results/accepted_front_consumers/BASELINE.json')
    if status != old['status'] or len(status) != 155:
        raise ValueError('UnexpectedInitialWorktree')
    paths = {r['path'] for r in old['files']}
    paths.update(str(p.relative_to(ROOT)).replace('\\','/') for p in [
        V4/'tools/accepted_trajectory_contract.py', V4/'tools/production_readonly_admission.py',
        V4/'results/accepted_front_consumers/EVIDENCE.json',
        V4/'results/accepted_front_consumers/qualified_consumers/CHECK.json',
        V4/'results/sf2_continuation_binding/target_check/DIAGNOSTIC_REQUEST.json',
        V4/'results/sf2_continuation_binding/target_check/SCALAR_COUNTEREXAMPLE.json'])
    protected = [dict(path=p, bytes=(ROOT/p).stat().st_size, sha256=file_hash(ROOT/p)) for p in sorted(paths)]
    out.mkdir(parents=True, exist_ok=False)
    save = lambda name, value: durable_json(out/name, value)
    save('BASELINE.json', dict(head=actual_head, branch=branch, status=status, files=protected,
         captured_utc=datetime.now(timezone.utc).isoformat(), available_c=psutil.disk_usage('C:/').free))
    phases = {}
    record = read(V4/'results/sf2_continuation_binding/target_check/DIAGNOSTIC_REQUEST.json')
    expected = 'b2cb76a6c7384d2e60cc1dddf9682474ee72822cc6e1945611613a77b1b29265'
    if identity(record) != expected:
        raise ValueError('DiagnosticIdentityMismatch')
    case = read(PRIOR/'CASES.json')['wing']
    wall, tri, surface, request = load_case(case)
    old_state, selected = existing_state_class().load(STUDY/'planner/selected_state')
    mapping = exact_vertex_map(old_state.wall, wall)
    increments = []
    for layer in range(2, 6):
        encoded = record['nominal_increment_hex'][layer-1]
        value = float.fromhex(encoded)
        if not np.all(selected[:,layer-1].view('u8') == np.float64(value).view('u8')):
            raise ValueError('NonmatchingAuthorizedIncrement')
        increments.append(dict(layer=layer, value_m=value, binary64_hex=encoded, quantity='path_length_m'))
    authorization = dict(schema='V4WingContinuationUserAuthorization/1',
        authorization='Explicit user request: qualify admission, then continue layers 2 through 5; stop at first required failure.',
        user_authorized=True, production_admission_passed=False,
        diagnostic_contract=expected, diagnostic_generation_authorized=record['generation_authorized'],
        original_request_sha256=request['request_file_sha256'],
        original_request_identity=identity(request['declaration']),
        first_layer_authority='e8cdfcad633d1bb4ab0af8ae680319348581ba8c675d4b73e812247ddfdbe6ec',
        initial_committed_parent=case['committed'], surface=surface['surface_identity'],
        geometry=case['geometry_identity'], units='m', first_full_normal_height_m=float.fromhex(record['nominal_increment_hex'][0]),
        trajectory_owner='4e06020e395804614bf747e8543e7e277c5f361fab5efda5570ceca38835561a',
        recovered_schedule='3d53dbb6d333873a7c91a5642ef2514b98602d4eedc4fdf7f25e0b8b95584304',
        authorized_layers=[2,3,4,5], increments=increments,
        semantics='Existing AcceptedFrontState binary64 update order and declared unit directions; later path lengths, not scalar offsets.',
        restrictions=['No layer 6','No tuning or scientific retry','No bypass of diagnostic or production guards',
                      'Each actual committed parent needs verified binding; this record is not an admission receipt'])
    save('AUTHORIZATION.json', authorization)
    with np.load(V4/'results/live_sf2_first_layer/A/CANDIDATE.npz', allow_pickle=False) as data:
        first, refs = data['front'], data['face_refs']
    kernel, _ = verify_kernel_sources()
    phases['input_verification'] = time.perf_counter()-started
    stop = None
    with CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')], timeout=60) as host:
        obj = host.load_rhai((V4/'results/smooth_wing_recovery/recovered_sf2.rhai').read_text(encoding='utf-8'), 1.)
        authority = bind_sf2_policy(bind_canonical(obj)).authority
        tick = time.perf_counter()
        owner = AcceptedTrajectoryOwner(authority, record, expected, wall, first, tri, refs, mapping, selected, parent=case['committed'])
        if owner.source_identity != authorization['trajectory_owner'] or owner.schedule.sha256 != authorization['recovered_schedule']:
            raise ValueError('ContinuationOwnerOrScheduleMismatch')
        phases['live_trajectory_owner'] = time.perf_counter()-tick
        print('verified live owner and exact L2-L5 increments', flush=True)
        with qualified_dispatch(), CommittedState(read(PRIOR/'WING_REFERENCE.json')) as base:
            tick = time.perf_counter()
            adapter = bind_forecast(base, case, owner)
            plan = plan_transition(base, adapter)
            save('BOUND_PLAN.json', plan)
            inspected = inspect_front(base, wall, tri, surface, request, base.expected)
            with indexed_session(base, inspected, read(PRIOR/'WING_INDICES.json')) as (numeric, pairs):
                face = base['prism_front']['0']
                access = dict(scientific=base.scientific, content=base.content_root, root=base.root,
                    layer=base.layer, persistent_faces=len(numeric),
                    pair_probe=pairs.affected({'face:'+face:'unchanged_probe'}), deep_restart=False)
                save('ACCESS.json', access)
                phases['authenticated_indexed_access'] = time.perf_counter()-tick
                import composable_admission
                tick = time.perf_counter()
                try:
                    composable_admission.admit(base, adapter, adapter.trajectory_spec, out/'layer2')
                except ValueError as error:
                    stop = dict(code=str(error), stage='production_admission_entry', attempted_layer=2,
                        user_authorization=identity(authorization), last_qualified_layer=1,
                        input_state=base.scientific, geometry=obj.identity, kernel_source=kernel,
                        owner=owner.source_identity, schedule=owner.schedule.sha256,
                        explanation='User authorization is present. The existing AcceptedFrontState entry remains diagnostic-only and has no production authorization consumer. Guard preserved.',
                        scientific_screening_run=False, cells_generated=0, layers_advanced=0,
                        replay=False, publication=False, later_layers='Not started',
                        output_directory_created=(out/'layer2').exists())
                    save('STOP.json', stop)
                else:
                    raise RuntimeError('UnexpectedAdmissionPass: this bounded preflight has no alternate generation route')
                phases['admission_entry'] = time.perf_counter()-tick
                base.check()
    if stop is None:
        raise RuntimeError('MissingStopEvidence')
    changed = [r['path'] for r in protected if file_hash(ROOT/r['path']) != r['sha256'] or (ROOT/r['path']).stat().st_size != r['bytes']]
    save('RESULT.json', dict(classification='V4AuthorizedWingContinuationBlockedAtProductionAdmissionConnection',
        stop=stop, authorization_identity=identity(authorization), increments=increments,
        phases=phases, seconds=time.perf_counter()-started,
        peak_main_working_set_bytes=psutil.Process().memory_info().peak_wset,
        protected_files=len(protected), changed_protected_files=changed,
        available_c=psutil.disk_usage('C:/').free))
    if changed:
        raise ValueError('ProtectedContentChanged')
    print(json.dumps(stop, indent=2), flush=True)


def finalize(out):
    """Read-only preservation and task-artifact accounting, not a run retry."""
    out = out.resolve()
    from sdf_cfd_mesher_v4.common.authority import file_hash
    from path_preserving_deduplication import durable_json
    from path_preserving_deduplication import metadata
    import psutil
    baseline = read(out/'BASELINE.json')
    changed = [r['path'] for r in baseline['files'] if file_hash(ROOT/r['path']) != r['sha256'] or (ROOT/r['path']).stat().st_size != r['bytes']]
    owned = [str(Path(__file__).relative_to(ROOT)).replace('\\','/'),
             'research/sdf_cfd_mesher_v4/WING_CONTINUATION_L2_L5_REPORT.md',
             str(out.relative_to(ROOT)).replace('\\','/')+'/']
    status = subprocess.check_output(['git','-c','core.quotepath=false','status','--porcelain=v1'],cwd=ROOT).decode('utf-8').splitlines()
    unrelated = [s for s in status if not any(s[3:] == p or (p.endswith('/') and s[3:].startswith(p)) for p in owned)]
    if changed or unrelated != baseline['status']:
        raise ValueError('PreservationCheckFailed')
    paths = [Path(__file__), V4/'WING_CONTINUATION_L2_L5_REPORT.md']
    paths += sorted(p for p in out.rglob('*') if p.is_file() and p.name != 'MANIFEST.json')
    artifacts = []
    for p in paths:
        row = metadata(p, content=True)
        artifacts.append(dict(path=str(p.relative_to(ROOT)).replace('\\','/'),
            bytes=row['bytes'], allocated=row['allocated'], sha256=file_hash(p)))
    record = dict(protected_files=len(baseline['files']), changed_protected_files=changed,
        original_status_entries=len(unrelated), exact_original_status_preserved=True,
        artifacts=artifacts, logical_bytes_excluding_manifest=sum(r['bytes'] for r in artifacts),
        allocated_bytes_excluding_manifest=sum(r['allocated'] for r in artifacts),
        final_c_free_bytes=psutil.disk_usage('C:/').free,
        seconds_since_baseline=(datetime.now(timezone.utc)-datetime.fromisoformat(baseline['captured_utc'])).total_seconds(),
        cleanup='None', coverage='Selected file content plus exact Git status, not full proof payload or repository inventory.')
    durable_json(out/'MANIFEST.json', record)
    print(json.dumps({k:v for k,v in record.items() if k!='artifacts'}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--expected-head', default=HEAD)
    parser.add_argument('--finalize', action='store_true')
    args = parser.parse_args()
    if args.finalize:
        finalize(args.output)
    else:
        run(args.output, args.expected_head)
