"""Bounded production milestones; never advance beyond an unqualified parent."""
from pathlib import Path
import json
import os
import shutil
import sqlite3
import subprocess
import sys
import traceback
from unittest.mock import patch

from qualify_recursive_layer6 import inputs, measured, metrics, denied
from path_preserving_deduplication import durable_json, file_hash, metadata, physical_key
from composable_state import CommittedState, plan_transition
from incremental_pair_storage import PairIncidence
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.common.authority import identity
import recursive_epoch_recovery as recovery

V4 = Path(__file__).resolve().parents[1]
OUT = V4 / 'results/recursive_layers7to8'
CONTROL = V4 / 'results/optimized_l6_publication_control_copy'
HEAD = '7a52ab2c144b156e82bbc32feee2aba61c9d45f3'
ROOT = '0adb8d3c87527076f9e132d0406383c52e1b99d4e56795a0f5c86ca60a0c75c9'
CONTENT = '15259f4e506a7ae574666283b40d33428f788de8f9d5f3e29314d15ef27178f0'


def read(path):
    return json.loads(Path(path).read_text())


def parent(layer):
    if layer == 7:
        reference = recovery.discover(V4/'results/recursive_epoch_recovery/access')['reference']
        if reference['root'] != ROOT:
            raise ValueError('UnexpectedProductionAuthority')
        return reference, CONTROL/'INCIDENCE_MANIFEST.json', read(CONTROL/'AUTHORITY.json')['incidence_root']
    if layer == 8:
        qualified = read(OUT/'L7/AUTHORITY.json')
        if not qualified['passed']:
            raise ValueError('L7NotQualified')
        if identity({k:v for k,v in qualified.items() if k != 'identity'}) != qualified['identity']:
            raise ValueError('ChangedL7Authority')
        for name, digest in qualified['files'].items():
            if file_hash(OUT/'L7'/name) != digest:
                raise ValueError('ChangedL7QualificationEvidence')
        return qualified['reference'], OUT/'L7/INCIDENCE_MANIFEST.json', qualified['incidence_root']
    raise ValueError('OutsideBoundedLayers')


def run(mode, layer=7):
    OUT.mkdir(exist_ok=True)
    out = OUT/f'L{layer}'
    out.mkdir(exist_ok=True)
    tick = measured()
    if (OUT/'FAILURE.json').exists():
        raise ValueError('StopAfterRecordedFailure')
    if (out/'AUTHORITY.json').exists():
        raise ValueError('PreserveSealedMilestone')
    try:
        if mode in ('reopen', 'deep', 'replay', 'diagnostics', 'retain', 'seal', 'safety'):
            from recursive_production_verification import verify
            verify(mode, layer, out)
            return
        reference, incidence_path, incidence_root = parent(layer)
        with patch.object(os, 'link', denied), patch.object(NativeCheckpointStore, 'restart', denied), CommittedState(reference) as base:
            adapter, spec = inputs(base)
            plan = plan_transition(base, adapter, layer)
            if plan['increment'] != {7: .0004247698513556187, 8: .0005081658834681988}[layer]:
                raise ValueError('WrongScheduleIncrement')
            with PairIncidence(base, incidence_path, incidence_root) as incidence:
                if mode == 'preflight':
                    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip() == HEAD
                    assert subprocess.check_output(['git','branch','--show-current'],text=True).strip() == 'feature/sdf-cfd-mesher-v4-advancing-front'
                    authority = read(CONTROL/'AUTHORITY.json')
                    assert authority['qualification'] == '3d0e18d4ee6b946ee5178906c199de537692bc77381a6eb3a38048b35d221774'
                    assert authority['architecture'] == '7876913f1e0c07a6ec40e1729c5f526771aee093e5016556186f30f5c44f0c49'
                    protected = {}
                    for name, digest in authority['files'].items():
                        p = CONTROL/name
                        assert file_hash(p) == digest, name
                        protected[str(p)] = digest
                    assert base.content_root == CONTENT and base.layer == 6
                    status = subprocess.check_output(['git','status','--porcelain=v1','-z'])
                    baseline = read(V4/'results/l6_write_pair_scaling_resume/PREFLIGHT.json')['status_hex']
                    unrelated = [r for r in status.split(b'\0') if r and b'qualify_recursive_layers7to8.py' not in r and b'results/recursive_layers7to8' not in r]
                    assert len(unrelated) == 155
                    assert unrelated == [r for r in bytes.fromhex(baseline).split(b'\0') if r]
                    files = {}
                    for segment in incidence.body['segments']:
                        for key in ('offsets','ordinals','keys','mapping','faces'):
                            p = Path(segment[key]); files[str(p)] = metadata(p)
                    physical = {physical_key(r): r for r in files.values()}
                    result = dict(passed=True, head=HEAD, reference=reference, plan=plan, protected=protected,
                        status_hex=baseline, epoch=recovery.access.epoch(), incidence_root=incidence_root,
                        incidence_population=incidence.body['parent_count'], incidence_segments=len(incidence.body['segments']),
                        incidence_physical=sum(r['cluster_bytes'] for r in physical.values()),
                        free=shutil.disk_usage(V4).free, metrics=metrics(*tick), stats=dict(base.stats),
                        hardlinks_created=0, epoch_renewal=False, legacy_fallbacks=0)
                    durable_json(OUT/'PREFLIGHT.json',result)
                elif mode == 'admission':
                    import composable_admission
                    durable_json(out/'START_STORAGE.json', dict(free=shutil.disk_usage(V4).free,
                        parent=reference, parent_incidence_root=incidence_root))
                    composable_admission.admit(base, adapter, spec, out)
                    result = dict(passed=True, metrics=metrics(*tick), stats=dict(base.stats))
                elif mode == 'generation':
                    import composable_transition
                    composable_transition.produce(base, adapter, spec, out)
                    result = dict(passed=True, metrics=metrics(*tick), stats=dict(base.stats))
                elif mode == 'assembly':
                    import composable_assembly
                    composable_assembly.assemble(base, adapter, out, pair_index=incidence)
                    result = dict(passed=True, metrics=metrics(*tick), stats=dict(base.stats), incidence=dict(incidence.stats))
                elif mode == 'publication':
                    from recursive_production_publication import publish
                    published = publish(base, adapter, out, incidence)
                    result = dict(passed=True, reference=published, metrics=metrics(*tick), stats=dict(base.stats))
                else:
                    raise ValueError('UnsupportedPhase')
                durable_json(out/(mode.upper()+'_MEASURED.json'),result)
                print(json.dumps(result,indent=2),flush=True)
    except BaseException as exc:
        failure = dict(layer=layer, phase=mode, error=repr(exc), traceback=traceback.format_exc(),
            metrics=metrics(*tick), authoritative_layer=layer-1, legacy_fallbacks=0,
            hardlinks_created=0, L9_admission=False, L9_generation=False, core=False)
        durable_json(OUT/'FAILURE.json',failure)
        raise


if __name__ == '__main__':
    run(sys.argv[1], int(sys.argv[2]) if len(sys.argv)>2 else 7)
