"""Production-critical milestones; final deep/replay is a separate shell audit."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import traceback
from unittest.mock import patch

from qualify_recursive_layer6 import inputs, measured, metrics, denied
from path_preserving_deduplication import durable_json, file_hash
from composable_state import CommittedState, plan_transition
from incremental_pair_storage import PairIncidence
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.common.authority import identity

V4 = Path(__file__).resolve().parents[1]
OUT = V4/'results/recursive_layers9to11'
PREVIOUS = V4/'results/recursive_layers7to8'
HEAD = 'ce0f0a5e23d59d8e31c4fa275cb3d6c54636820c'
INCREMENTS = {9: .0006079352484572211, 10: .0007272925600482035, 11: .0008700835643990311}


def read(path):
    return json.loads(Path(path).read_text())


def parent(layer):
    if layer not in INCREMENTS:
        raise ValueError('OutsideRequestedSchedule')
    directory = PREVIOUS/'L8' if layer == 9 else OUT/f'L{layer-1}'
    authority = read(directory/'AUTHORITY.json')
    if not authority['passed'] or authority['layer'] != layer-1:
        raise ValueError('UnqualifiedParent')
    if identity({k:v for k,v in authority.items() if k != 'identity'}) != authority['identity']:
        raise ValueError('ChangedParentAuthority')
    for name, digest in authority['files'].items():
        if file_hash(directory/name) != digest:
            raise ValueError('ChangedParentEvidence:'+name)
    return authority['reference'], directory/'INCIDENCE_MANIFEST.json', authority['incidence_root']


def run(mode, layer):
    OUT.mkdir(exist_ok=True)
    out = OUT/f'L{layer}'
    out.mkdir(exist_ok=True)
    if (OUT/'FAILURE.json').exists() or (out/'AUTHORITY.json').exists():
        raise ValueError('PreserveFailureOrSealedMilestone')
    tick = measured()
    try:
        reference, incidence_path, incidence_root = parent(layer)
        with patch.object(os, 'link', denied), patch.object(NativeCheckpointStore, 'restart', denied), CommittedState(reference) as base:
            adapter, spec = inputs(base)
            plan = plan_transition(base, adapter, layer)
            assert plan['increment'] == INCREMENTS[layer]
            with PairIncidence(base, incidence_path, incidence_root) as incidence:
                if mode == 'preflight':
                    assert layer == 9
                    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip() == HEAD
                    assert subprocess.check_output(['git','branch','--show-current'],text=True).strip() == 'feature/sdf-cfd-mesher-v4-advancing-front'
                    a = read(PREVIOUS/'L8/AUTHORITY.json')
                    assert a['scientific'] == '483b00f955f51fe07987f839bc9441c8b081a262b41d7b41dc60fd4831757699'
                    assert a['checkpoint_sha256'] == '6efbed2489d1f0bcf44ac9774df31200a74b3077c3f322a16df3859b692eb23c'
                    assert base.root == 'a4c3560a5d58ee9ac6deb09ee5815a216359b0cc92f01bb2fcbf8dd77aeafb28'
                    assert base.content_root == 'c59a2976f44247a53e648eb50f8af7080039506d8e8976f6a969c92df082f256'
                    status = subprocess.check_output(['git','status','--porcelain=v1','-z'])
                    baseline = bytes.fromhex(read(PREVIOUS/'PREFLIGHT.json')['status_hex'])
                    unrelated = [r for r in status.split(b'\0') if r and b'recursive_layers9to11' not in r]
                    assert unrelated == [r for r in baseline.split(b'\0') if r]
                    assert len(unrelated) == 155
                    result = dict(passed=True, head=HEAD, status_hex=baseline.hex(), plan=plan,
                        free=shutil.disk_usage(V4).free, current_epoch_access=True, epoch_renewal=False,
                        incidence_root=incidence_root, parent_count=incidence.body['parent_count'])
                    durable_json(OUT/'PREFLIGHT.json', result)
                elif mode == 'admission':
                    import composable_admission
                    durable_json(out/'START_STORAGE.json',dict(free=shutil.disk_usage(V4).free,parent=reference))
                    composable_admission.admit(base, adapter, spec, out)
                    result = dict(passed=True)
                elif mode == 'generation':
                    import composable_transition
                    composable_transition.produce(base, adapter, spec, out)
                    result = dict(passed=True)
                elif mode == 'assembly':
                    import composable_assembly
                    composable_assembly.assemble(base, adapter, out, pair_index=incidence)
                    result = dict(passed=True, incidence=dict(incidence.stats))
                elif mode == 'publication':
                    from recursive_production_publication import publish
                    result = dict(passed=True, reference=publish(base, adapter, out, incidence))
                else:
                    raise ValueError('UnsupportedPhase')
                result.update(metrics=metrics(*tick), stats=dict(base.stats))
                durable_json(out/(mode.upper()+'_MEASURED.json'),result)
                print(json.dumps(result,indent=2),flush=True)
    except BaseException as exc:
        durable_json(OUT/'FAILURE.json',dict(layer=layer,phase=mode,error=repr(exc),
            traceback=traceback.format_exc(),metrics=metrics(*tick),authoritative_layer=layer-1,
            legacy_fallbacks=0,hardlinks_created=0,core=False))
        raise


if __name__ == '__main__':
    run(sys.argv[1], int(sys.argv[2]))
