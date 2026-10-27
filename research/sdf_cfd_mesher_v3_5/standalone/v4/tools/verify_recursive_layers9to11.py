"""Normal milestones and separately invoked final-shell qualification."""
import json
import os
from pathlib import Path
import sqlite3
import sys
from datetime import datetime, timezone
from unittest.mock import patch
import numpy as np
from qualify_recursive_layers9to11 import (OUT, V4, parent, read, inputs, measured,
    metrics, denied, CommittedState, plan_transition, PairIncidence, durable_json,
    file_hash, identity, INCREMENTS)
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.common.authority import array_hash
from recursive_production_verification import inventory


def completion(state):
    """Inspect the sealed final front without requesting a pending increment."""
    from clean_sheet_l4_bootstrap import SURFACE, REQUEST, load_production_wall
    from production_prism_sizing import read_request
    wall, triangles, surface = load_production_wall(SURFACE)
    bound = read_request(REQUEST, surface)
    assert identity(bound['declaration']) == state.expected['request']
    assert bound['schedule_identity'] == state.expected['schedule']
    assert bound['request_file_sha256'] == state.expected['request_file']
    assert state.layer == len(bound['increments']) == 11
    assert float(bound['targets'][-1]) == .004563781431330088
    assert {int(c['last_prism_layer']) for c in state['columns'].values()} == {11}
    n = len(wall)
    coordinates = np.array([state['vertices'][f'layer:11:source:{i}'] for i in range(n)])
    assert array_hash(coordinates) == state.expected['front_coordinates']
    assert set(state['prism_front']) == {str(i) for i in range(len(triangles))}
    for i, tri in enumerate(triangles):
        face = state['faces'][state['prism_front'][str(i)]]
        assert set(face['vertices']) == {f'layer:11:source:{int(v)}' for v in tri}
    state.check()
    return dict(passed=True, layer=11, requested_layers=11, next_layer=None,
        thickness=float(bound['targets'][-1]), thickness_hex=float(bound['targets'][-1]).hex(),
        vertices=n, triangles=len(triangles), coordinates=array_hash(coordinates),
        request=state.expected['request'], schedule=bound['schedule_identity'],
        core=False, forecast=False)


def reopen(out, layer):
    from production_forecast_admission import BoundProductionForecastCache
    tick = measured()
    with patch.object(os, 'link', denied), patch.object(sqlite3, 'connect', denied), \
            patch.object(NativeCheckpointStore, 'restart', denied), \
            patch.object(BoundProductionForecastCache, 'forecast', denied), \
            CommittedState(read(out/'REFERENCE.json')) as state:
        assert state.layer == layer
        if layer == 11:
            next_state = completion(state)
        else:
            adapter, _ = inputs(state)
            next_state = plan_transition(state, adapter, layer+1)
            assert next_state['increment'] == INCREMENTS[layer+1]
        with PairIncidence(state, out/'INCIDENCE_MANIFEST.json', state.body['plan']['pair_incidence_root']) as index:
            assert index.body['parent_count'] == len(state['certificate'])
        return dict(passed=True, next_state=next_state, layer=layer, reference=state.reference,
            content=state.content_root, stats=dict(state.stats), metrics=metrics(*tick),
            proof_rows=0, quality_rows=0, sql_rows=0, full_prefix_hashes=0, deep_copies=0,
            admission=False, forecast=False, generation=False)


def verify(mode, layer):
    out = OUT/f'L{layer}'
    if layer not in INCREMENTS or (OUT/'FAILURE.json').exists():
        raise ValueError('BoundedFailureStop')
    tick = measured()
    if mode == 'reopen':
        result = reopen(out, layer)
    elif mode == 'diagnostics':
        import recursive_production_diagnostics as diagnostics
        with patch.object(diagnostics, 'parent', parent):
            diagnostics.run(out, layer)
        return
    elif mode == 'safety':
        import recursive_production_verification as existing
        with patch.object(existing, 'parent', parent):
            existing.verify('safety', layer, out)
        return
    elif mode == 'retain':
        assert read(out/'REOPEN.json')['passed'] and read(out/'SAFETY.json')['passed']
        # No routine replay trees exist at these production milestones.
        # All scientific deltas remain active or needed for the final audit.
        assert not (out/'replay').exists()
        before = inventory(out)
        result = dict(passed=True, removed_paths=0, physical_reclaimed=0,
            reason='No duplicate replay bulk; preserve authenticated closure and final qualification inputs',
            inventory=before, post_cleanup_reopen=reopen(out,layer), metrics=metrics(*tick))
        durable_json(out/'CLEANUP_MANIFEST.json',dict(rows=[],removed_bytes=0,checkpoint_retirement=False))
    elif mode == 'seal':
        if (out/'AUTHORITY.json').exists():
            raise ValueError('PreserveSealedAuthority')
        for name in ('REOPEN','RETAIN','SAFETY'):
            assert read(out/(name+'.json'))['passed']
        admission = read(out/'ADMISSION.json')
        generation = read(out/'GENERATION.json')
        certificate = read(out/'FRESH_CERTIFICATE.json')
        volume = read(out/'VOLUME_DELTA.json')
        diagnostic = read(out/'DIAGNOSTICS.json')
        assembly = read(out/'ASSEMBLY.json')
        assert admission['decision']['generation_attempt_authorized']
        assert not generation['hard_failures'] and generation['topology_audit']['passed']
        assert certificate['passed'] and not certificate['failures']
        assert volume['exact_balance'] and volume['global_degree']['passed']
        assert not volume['global_degree']['unresolved']
        assert all(x['failures'] == 0 for x in diagnostic['quality'].values())
        assert assembly['pair_accounting']['prior_pair_rows_scanned'] == 0
        assert assembly['prior_proof_payloads_read'] == assembly['prior_proof_payloads_recomputed'] == 0
        writer = read(out/'INDEX_MEASURED.json')['writer']
        assert writer['final']['journal_mode'] == 'delete' and writer['final']['synchronous'] == 2
        assert writer['rows'] == certificate['required_pairs']
        plan = read(out/'CANDIDATE_PLAN.json')
        reference = read(out/'REFERENCE.json')
        names = ('ADMISSION','GENERATION','ASSEMBLY','PUBLICATION','REFERENCE','REOPEN','RETAIN','SAFETY','DIAGNOSTICS')
        files = {n+'.json':file_hash(out/(n+'.json')) for n in names}
        result = dict(passed=True,layer=layer,scientific=plan['scientific'],
            scientific_identity_kind='Authenticated component scientific root; legacy digest deferred to final shell audit',
            content=plan['scientific'],reference=reference,incidence_root=plan['pair_incidence_root'],files=files,
            sealed_at=datetime.now(timezone.utc).isoformat(),post_retention_reopen=reopen(out,layer),
            checkpoint_sha256=file_hash(out/'candidate'/('ROOT_'+reference['root']+'.json')),
            deep_replay_policy='One final shell qualification after L11',metrics=metrics(*tick))
        compact = dict(schema='V4NormalProductionLayerCertificate/1',layer=layer,
            content=result['content'],reference=reference,components=plan['components'],
            admission=admission['decision'],quality=diagnostic['quality'],geometry=generation,
            continuous=certificate,assembly=assembly,checkpoint_sha256=result['checkpoint_sha256'],
            retention=file_hash(out/'CLEANUP_MANIFEST.json'),
            implementation={p.name:file_hash(p) for p in (Path(__file__),V4/'tools/qualify_recursive_layers9to11.py')},
            reproduction='Replay changed transition from authenticated parent using exact bound schedule; compare component mapping',
            final_deep_replay_pending=True)
        compact['identity'] = identity(compact)
        durable_json(out/'COMPACT_CERTIFICATE.json',compact)
        result['compact_certificate_sha256'] = file_hash(out/'COMPACT_CERTIFICATE.json')
        result['identity'] = identity(result)
        durable_json(out/'AUTHORITY.json',result)
        durable_json(OUT/'PRODUCTION.json',result)
    else:
        raise ValueError(mode)
    durable_json(out/(mode.upper()+'.json'),result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('inventory','post_retention_reopen','post_cleanup_reopen')},indent=2),flush=True)


if __name__ == '__main__':
    try:
        verify(sys.argv[1],int(sys.argv[2]))
    except BaseException as exc:
        import traceback
        durable_json(OUT/'FAILURE.json',dict(phase=sys.argv[1],layer=int(sys.argv[2]),
            error=repr(exc),traceback=traceback.format_exc(),core=False))
        raise
