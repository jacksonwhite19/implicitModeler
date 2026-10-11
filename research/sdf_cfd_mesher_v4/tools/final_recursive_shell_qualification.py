"""Explicit final-shell audit and one replay campaign, outside normal startup."""
from collections import Counter
from fractions import Fraction
import math
import os
import shutil
import sys
from unittest.mock import patch
from qualify_recursive_layers9to11 import *
from verify_recursive_layers9to11 import completion
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import scientific_digest
from sdf_cfd_mesher_v4.advancing_front.mixed_front import geometry_inputs, face_key, cycle


def deep():
    tick = measured()
    assert read(OUT/'L11/AUTHORITY.json')['passed']
    with CommittedState(read(OUT/'L11/REFERENCE.json')) as state:
        refs = {}; ancestor = state; depth = 0
        while ancestor is not None:
            depth += 1
            if ancestor.kind == 'native':
                refs.update(ancestor.wrapped.body['files'])
            elif ancestor.kind == 'clean_sheet':
                refs.update(ancestor.wrapped.plan['references']); refs.update(ancestor.wrapped.body['produced'])
            else:
                refs.update(ancestor.body['plan']['references']); refs.update(ancestor.body['produced'])
            ancestor = getattr(ancestor,'parent',None)
        total = 0
        for path,record in refs.items():
            assert file_hash(path) == record['sha256'],path
            total += Path(path).stat().st_size
        print('closure authenticated',len(refs),total,flush=True)
        durable_json(OUT/'FINAL_DEEP_PROGRESS.json',dict(stage='closure',files=len(refs),bytes=total,metrics=metrics(*tick)))
        import verify_optimized_l6_control as audit
        with patch.object(audit,'OUT',OUT/'L11'):
            incidence = audit.incidence_deep(state)
        completed = completion(state)
        layer_counts = Counter(); source_counts = Counter(); owner_visits = 0; used_vertices = set()
        for key,cell in state['cells'].items():
            layer_counts[int(cell['layer'])] += 1
            source_counts[cell['source']] += 1
            assert len(cell['vertices']) == 6 and len(set(cell['vertices'])) == 6
            used_vertices.update(cell['vertices'])
            assert {int(v.split(':')[1]) for v in cell['vertices']} == {int(cell['layer'])-1,int(cell['layer'])}
            for perimeter in cell['faces']:
                face = state['faces'][face_key(perimeter)]
                assert face['owners'][key] == perimeter
                owner_visits += 1
        assert layer_counts == Counter({i:33786 for i in range(1,12)})
        assert len(source_counts) == 33786 and set(source_counts.values()) == {11}
        assert used_vertices == set(state['vertices']) and len(used_vertices) == 12*16895
        face_counts = Counter()
        for key,face in state['faces'].items():
            owners = list(face['owners'].values())
            assert len(owners) in (1,2) and face_key(face['vertices']) == key
            if len(owners) == 2:
                assert cycle(owners[0]) == cycle(tuple(reversed(owners[1])))
            else:
                assert {int(v.split(':')[1]) for v in face['vertices']} in ({0},{11})
            face_counts[len(owners)] += 1
        assert owner_visits == sum(n*c for n,c in face_counts.items())
        print('deep topology and source continuity',dict(layer_counts),dict(face_counts),flush=True)
        durable_json(OUT/'FINAL_DEEP_PROGRESS.json',dict(stage='topology',layers=dict(layer_counts),face_incidence=dict(face_counts),metrics=metrics(*tick)))
        # This is explicitly deep: recompute current geometric dependency hashes.
        bindings = geometry_inputs(state)
        print('deep geometric bindings',len(bindings),flush=True)
        durable_json(OUT/'FINAL_DEEP_PROGRESS.json',dict(stage='bindings',bindings=len(bindings),metrics=metrics(*tick)))
        populations = Counter(); methods = Counter(); extrema = {}; total_volume = Fraction()

        def bound(row):
            for key,value in row['inputs'].items():
                if bindings.get(key) != value:
                    raise ValueError('FinalDependencyMismatch:'+key)

        def metric(name,value,key,limit,upper=False,strict=False):
            value = float(value)
            assert math.isfinite(value),(key,name)
            assert value <= limit if upper else value > limit if strict else value >= limit,(key,name,value)
            if name not in extrema or (value > extrema[name]['value'] if upper else value < extrema[name]['value']):
                extrema[name] = dict(value=value,key=key,margin=limit-value if upper else value-limit)

        original_quality = state.rows['quality'].items_deep
        def quality():
            nonlocal total_volume
            previous = None
            for key,row in original_quality():
                assert previous is None or key > previous
                previous = key; bound(row)
                kind = key.split(':',1)[0]; populations[kind] += 1
                v = row.get('value',{})
                if kind == 'cell':
                    metric('volume',v['volume'],key,0,strict=True)
                    metric('determinant',v['determinant'],key,.0011)
                    metric('aspect',v['aspect'],key,1000,upper=True)
                    exact = row['exact']
                    assert exact['jacobian_sign'] > 0 and Fraction(exact['signed_volume']) > 0
                    total_volume += Fraction(exact['signed_volume'])
                    metric('exact_jacobian',Fraction(exact['jacobian']),key,0,strict=True)
                elif kind == 'face':
                    metric('weight',v['weight'],key,.055)
                    metric('skewness',v['skewness'],key,3.63636,upper=True)
                    metric('symmetric_volume_ratio',v['symmetric_volume_ratio'],key,.01)
                elif kind == 'final_incidence':
                    metric('completed_determinant',v['determinant'],key,.0011)
                yield key,row

        original_proofs = state.rows['certificate'].items_deep
        def proofs():
            previous = None
            for key,row in original_proofs():
                assert previous is None or key > previous
                previous = key; bound(row)
                assert key == identity(sorted(row['faces']))
                assert row['value']['classification'] in ('CertifiedSeparated','CertifiedAllowedIncidentContact')
                methods[row['value']['method']] += 1
                populations['proof'] += 1
                if populations['proof'] % 1_000_000 == 0:
                    print('deep proof rows',populations['proof'],flush=True)
                    durable_json(OUT/'FINAL_DEEP_PROGRESS.json',dict(stage='proofs',proofs=populations['proof'],metrics=metrics(*tick)))
                yield key,row
        for row in state.rows.values():
            row.deep = True
        try:
            with patch.object(state.rows['quality'],'items_deep',quality), patch.object(state.rows['certificate'],'items_deep',proofs):
                scientific = scientific_digest(state)
        finally:
            for row in state.rows.values():
                row.deep = False
        assert populations['proof'] == len(state['certificate']) == incidence['pairs']
        assert populations['cell'] == 11*33786 == len(state['cells'])
        assert populations['final_incidence'] == populations['cell']
        assert populations['face'] == face_counts[2]
        assert sum(v for k,v in populations.items() if k != 'proof') == len(state['quality'])
        from clean_sheet_l4_bootstrap import SURFACE, load_production_wall
        from incremental_layer_validation import ExactVolume
        import numpy as np
        wall,tri,_ = load_production_wall(SURFACE)
        n = len(wall)
        points = np.array([state['vertices'][f'layer:{k}:source:{i}'] for k in range(12) for i in range(n)])
        boundary = sum((ExactVolume.contribution(points,tuple(reversed(f))) for f in tri),Fraction())
        boundary += sum((ExactVolume.contribution(points,tuple(f+11*n)) for f in tri),Fraction())
        assert total_volume == boundary
        state.check()
        result = dict(passed=True,scientific=scientific,content=state.content_root,reference=state.reference,
            depth=depth,files_authenticated=len(refs),bytes_authenticated=total,
            incidence=incidence,completion=completed,populations=dict(populations),proof_methods=dict(methods),
            quality_extrema=extrema,hard_failures=0,missing=0,duplicate=0,forbidden=0,unresolved=0,
            geometric_bindings=len(bindings),metrics=metrics(*tick),normal_path=False)
        degree = read(OUT/'L11/VOLUME_DELTA.json')['global_degree']
        assert degree['passed'] and not degree['unresolved']
        result.update(layer_cell_counts=dict(layer_counts),source_columns=len(source_counts),face_incidence=dict(face_counts),
            exact_volume=str(total_volume),exact_boundary_volume=str(boundary),exact_volume_balance=True,
            topology_ownership=True,source_continuity=True,unused_vertices=0,vertices=len(used_vertices),
            global_degree=dict(passed=True,witnesses=len(degree['values']),degrees=sorted({r['degree'] for r in degree['values']}),
                unresolved=degree['unresolved'],evidence_sha256=file_hash(OUT/'L11/VOLUME_DELTA.json')))
    durable_json(OUT/'FINAL_DEEP.json',result)
    print(json.dumps(result,indent=2),flush=True)


def replay():
    """One final campaign. Exact transitions compose by induction over sealed parents."""
    assert read(OUT/'FINAL_DEEP.json')['passed']
    import composable_transition
    import composable_assembly
    import composable_publication
    tick = measured(); results = {}
    for layer in (9,10,11):
        phase = measured(); out = OUT/f'L{layer}'; replay = out/'replay'
        replay.mkdir(exist_ok=False)
        # Admission is sealed production evidence. No new forecast is run here.
        for name in ('ADMISSION.json','DIRECT.json','LOCALIZATION.json','SCREEN_ARRAYS.npz'):
            shutil.copyfile(out/name,replay/name)
        reference,path,root = parent(layer)
        with patch.object(os,'link',denied), patch.object(NativeCheckpointStore,'restart',denied), CommittedState(reference) as base:
            adapter,spec = inputs(base)
            with PairIncidence(base,path,root) as incidence:
                composable_transition.produce(base,adapter,spec,replay)
                composable_assembly.assemble(base,adapter,replay,incidence)
                plan = composable_publication.prepare(base,adapter,replay)
                # Seal the independent result before consulting the target mapping.
                durable_json(replay/'REPLAY_SEAL.json',dict(content=plan['scientific'],components=plan['components']))
                expected = read(out/'CANDIDATE_PLAN.json')
                equal = {k:v == expected['components'][k] for k,v in plan['components'].items()}
                assert all(equal.values()) and plan['scientific'] == expected['scientific']
                results[str(layer)] = dict(passed=True,content=plan['scientific'],components=equal,metrics=metrics(*phase))
                durable_json(out/'FINAL_REPLAY_LAYER.json',results[str(layer)])
        print('final replay layer',layer,'passed',flush=True)
    result = dict(passed=True,layers=results,scientific=read(OUT/'FINAL_DEEP.json')['scientific'],
        argument='Exact component equality for every new transition, with identical authenticated parent roots, establishes complete final equivalence by composition.',
        admission='Sealed admission evidence reused; no new forecast',metrics=metrics(*tick),normal_path=False)
    durable_json(OUT/'FINAL_REPLAY.json',result)
    print(json.dumps(result,indent=2),flush=True)


if __name__ == '__main__':
    if (OUT/'FAILURE.json').exists():
        raise ValueError('StopAfterFailure')
    try:
        {'deep':deep,'replay':replay}[sys.argv[1]]()
    except BaseException as exc:
        import traceback
        durable_json(OUT/'FAILURE.json',dict(phase='final_'+sys.argv[1],error=repr(exc),
            traceback=traceback.format_exc(),core=False,complete_qualification=False))
        raise
