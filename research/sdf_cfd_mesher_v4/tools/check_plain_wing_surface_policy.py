"""Review-only admission and metric plumbing. No meshing or acceptance test."""
import argparse
import copy
import dataclasses
import json
import math
from pathlib import Path
import sys
import time
from unittest.mock import patch

import numpy as np
import psutil

sys.dont_write_bytecode = True
V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
sys.path[:0] = [str(V4 / 'src'), str(V4 / 'tools')]
from historical_capsule_surface import SurfaceSizing, SeedVertex, metric, save, sha
from surface_binding import bind_canonical
from production_adaptive_surface import ProductionAdaptiveSdfSurfaceMesher
from development_surface_policy import bind_development_policy, load_case
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel, GeometryFailure
from replay_surface_dispatch import verify_kernel_sources


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--case', type=Path, default=V4 / 'cases/plain_wing_surface_development_proposal.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('OutputMustBeNew')
    start = time.perf_counter()
    case = load_case(args.case, require_approval=False)
    checks = []

    def reject(label, operation, fragment):
        try:
            operation()
        except (ValueError, GeometryFailure) as e:
            assert fragment in str(e), (label, str(e))
            checks.append(dict(check=label, result='expected_rejection', diagnostic=str(e)))
        else:
            raise AssertionError(label + ': unexpectedly succeeded')

    reject('unapproved execution configuration', lambda: load_case(args.case), 'CasePolicyApprovalRequired')
    source, direct = verify_kernel_sources()
    assert source['source_sha256'] == case['geometry']['kernel_source_digest']
    original = next(c for c in direct['cases'] if c['name'] == case['geometry']['script_case'])
    policy = SurfaceSizing(**case['sizing_policy'])
    policy.validate()
    assert dataclasses.asdict(policy) == dataclasses.asdict(SurfaceSizing())
    assert case['accuracy_limits'] == dict(position_m=policy.position_limit_m,
                                         normal_rad=math.radians(policy.normal_limit_deg))
    assert original['metres_per_model_unit'] == case['geometry']['metres_per_model_unit']
    host = ROOT / 'target/debug/canonical_kernel_host.exe'
    # These guards forbid entry into all potentially expensive surface stages.
    with patch('surface_execution.execute_surface', side_effect=AssertionError('MeshingForbidden')) as execution, \
            CanonicalKernel([str(host)]) as owner:
        t = time.perf_counter()
        g = owner.load_rhai(original['script'], original['metres_per_model_unit'])
        load_seconds = time.perf_counter() - t
        assert g.identity == case['geometry']['canonical_identity']
        assert g.stored_topology()['topology_identity'] == case['geometry']['topology_identity']
        entry = ProductionAdaptiveSdfSurfaceMesher()
        t = time.perf_counter()
        b = bind_canonical(g)
        before = entry.admission(b)
        assert {r['operation'] for r in before['issues']} == {'metric', 'sizing_policy', 'caller_accuracy_limits'}
        b.qualification.accuracy_limits = copy.deepcopy(case['accuracy_limits'])
        bind_development_policy(b, policy)
        after = entry.admission(b)
        assert after['executable'] and after['issues'] == []
        assert after['seed_route'] == case['seed_route']
        assert b.qualification.projection_tolerance_m == 2e-12
        admission_seconds = time.perf_counter() - t
        checks.append(dict(check='candidate admission; not execution authorization', result='passed'))

        a = b.authority
        samples = []
        # Four source vertices on actual incident curve/sheet/corner sets, not
        # a repeated midpoint/projection study or a mesh realization.
        sheets = sorted(a.sheet_names)
        for sref in [sheets[0], sheets[1], sheets[len(sheets)//2], sheets[-1]]:
            loop = a.unit.brep.loops[a.unit.sheet_names[sref]]
            cref = next(r for r, ends in a.unit.curve_vertices.items()
                        if all(a.unit.point_names[j] in loop for j in ends))
            i = a.unit.curve_vertices[cref][0]
            key = a.unit.point_names[i]
            point = a.unit.points[i] * a.scale
            vertex = SeedVertex(point, sref, 0, a.unit.corner_ref[key])
            curve = a.metric(vertex, {cref}, policy.metric_multiplier)
            surface = metric(a, point, policy, sheet_ref=sref)
            carrier = metric(a, point, policy, differential=True, sheet_ref=sref)
            legacy_si = a.unit.metric(SeedVertex(point/a.scale, sref, 0, vertex.corner_ref), {cref}, 1.)
            assert np.array_equal(curve, surface)
            assert np.isfinite(curve).all() and np.array_equal(curve, curve.T)
            eigenvalues = np.linalg.eigvalsh(curve)
            assert (eigenvalues > 0).all()
            samples.append(dict(sheet=sref, curve=cref, corner=vertex.corner_ref,
                eigenvalues_per_m2=eigenvalues.tolist(), directional_lengths_m=(1/np.sqrt(eigenvalues)).tolist(),
                curve_surface_max_difference=float(np.max(abs(curve-surface))),
                differential_pointwise_max_difference=float(np.max(abs(carrier-surface))),
                legacy_HMAX_numeric_max_difference=float(np.max(abs(legacy_si-surface)))))
        checks.append(dict(check='four shared curve/surface SPD metrics', result='passed'))
        reject('policy overwrite', lambda: bind_development_policy(b, policy), 'ExistingMetricBindingMustNotBeOverwritten')
        reject('metric multiplier drift', lambda: a.metric(vertex, {cref}, .64), 'DevelopmentMetricMultiplierMismatch')
        reject('unknown metric reference', lambda: a.metric(vertex, {-1}, 1.), 'DevelopmentMetricReferenceMismatch')
        b.qualification.accuracy_limits = None
        reject('uncoupled accuracy', lambda: a.metric(vertex, {cref}, 1.), 'DevelopmentPackageAccuracyMismatch')
        b.qualification.accuracy_limits = copy.deepcopy(case['accuracy_limits'])
        b.sizing_policy = dataclasses.replace(policy, maximum_size_m=.08)
        reject('unreviewed size ceiling', lambda: a.metric(vertex, {cref}, 1.), 'DevelopmentPackagePolicyMismatch')
        b.sizing_policy = policy
        b.geometry_identity = 'altered'
        reject('identity mismatch', lambda: entry.admission(b), 'GeometryIdentityMismatch')
        b.geometry_identity = g.identity
        assert entry.admission(b)['executable']
        assert b.seed_provider == a.labelled_seed
        assert execution.call_count == 0
        host_peak = psutil.Process(owner.transport.process.pid).memory_info().peak_wset

    result = dict(classification='V4PlainWingDevelopmentSurfacePolicyProposedAdmissionVerified',
        production_execution_authorized=False, mesh_generated=False, accuracy_acceptance='not_evaluated',
        case_sha256=sha(args.case), geometry_identity=g.identity,
        topology_identity=case['geometry']['topology_identity'], kernel_source=source,
        host_sha256=sha(host), before_issues=[r['operation'] for r in before['issues']],
        candidate_issues=after['issues'], candidate_callable_admission=True,
        seed_route=after['seed_route'], projection_gate_m=2e-12,
        sizing_policy=dataclasses.asdict(policy), accuracy_limits=case['accuracy_limits'],
        source_counts={k:len(getattr(a,k)) for k in ('sheet_names','curve_names','corner_names')},
        metric_samples=samples, checks=checks, source_files=case['source_files'],
        geometry_load_seconds=load_seconds, admission_seconds=admission_seconds,
        total_seconds=time.perf_counter()-start,
        checker_peak_working_set_bytes=psutil.Process().memory_info().peak_wset,
        host_peak_working_set_bytes=host_peak,
        not_run=['charts','carrier refinement','MMGS','projection study','surface qualification','capsule replay','CFD'])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False)+'\n', newline='\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
