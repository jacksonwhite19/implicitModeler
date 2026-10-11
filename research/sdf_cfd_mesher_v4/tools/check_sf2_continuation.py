"""Recover a retained SF2 continuation for diagnostics, never advancement.

The accepted arrays are comparison fixtures. This tool does not generate a new
trajectory field or convert path length to an SDF offset. Production dispatch
remains fail-closed when its existing target contract cannot represent SF2.
"""
import argparse
import ast
import copy
import importlib.util
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch

import numpy as np
import psutil

V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
V3 = V4.parent / 'sdf_cfd_mesher_v3'
SF2 = V3 / 'fresh_supported_refinement_family_qualification_2026-09-09'
STUDY = SF2 / 'SF2/A/wing'
PRIOR = V4 / 'results/sealed_front_connection'
sys.path[:0] = [str(V4/'src'), str(V4/'tools')]
from sdf_cfd_mesher_v4.common.authority import identity, array_hash, file_hash
from path_preserving_deduplication import durable_json
from sealed_front_connection import read, load_case
from production_prism_sizing import decode_schedule


def existing_state_class():
    path = V3/'sdf_guided_advancing_front_qualification/accepted_front_state.py'
    preserved = V4/'src/sdf_cfd_mesher_v4/legacy_full_stack_planner/_support/sdf_guided_advancing_front_qualification/accepted_front_state.py'
    if path.read_bytes() != preserved.read_bytes():
        raise ValueError('AcceptedStatePreservedSourceMismatch')
    spec = importlib.util.spec_from_file_location('sf2_existing_accepted_state', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.AcceptedFrontState


def exact_vertex_map(historical, current):
    """Identity bookkeeping only; no nearest-point correspondence."""
    if historical.shape != current.shape:
        raise ValueError('WallInventoryMismatch')
    a = np.lexsort(historical.T[::-1]); b = np.lexsort(current.T[::-1])
    if np.any(np.all(historical[a][1:] == historical[a][:-1], axis=1)):
        raise ValueError('AmbiguousSourceCoordinateMapping')
    if historical[a].tobytes() != current[b].tobytes():
        raise ValueError('WallSourceBitsMismatch')
    mapping = np.empty(len(b), dtype=np.int64); mapping[b] = a
    return mapping


def validate_diagnostic(record, expected, *, layer, units, geometry, parent):
    # This is not a production request reader or a geometry predictor.
    if identity(record) != identity(expected):
        raise ValueError('AlteredContinuationDiagnostic')
    if units != 'm': raise ValueError('ContinuationUnitsMismatch')
    if geometry != record['geometry']: raise ValueError('ContinuationGeometryMismatch')
    if parent != record['committed_parent']: raise ValueError('StaleContinuationParent')
    if layer >= record['historical_layer_count']: raise ValueError('NoPendingIncrement')
    if layer != record['committed_layer']: raise ValueError('ContinuationSchedulePositionMismatch')
    if record['later_scalar_quantity'] != 'path_length_m':
        raise ValueError('UnsupportedContinuationScalarMeaning')
    for source in record['sources']:
        if file_hash(ROOT/source['path']) != source['sha256']:
            raise ValueError('ContinuationSourceMismatch:'+source['path'])
    return record['next_increment_hex']


def rejected(call, expected):
    try: call()
    except ValueError as e:
        if str(e) != expected: raise
        return str(e)
    raise AssertionError('ExpectedRejection:'+expected)


def live_normal_check(state, mapping, wall, tri, refs, corners, curves, geometry):
    from run_live_sf2_surface import CanonicalKernel, bind_canonical, bind_sf2_policy, verify_kernel_sources
    source, _ = verify_kernel_sources()
    ids = set(np.linspace(0,len(wall)-1,64,dtype=int)) | set(np.flatnonzero(corners))
    for ref in np.unique(refs): ids.update(np.unique(tri[refs==ref])[::max(1,len(np.unique(tri[refs==ref]))//8)])
    for ref in np.unique(curves[:,2]): ids.update(curves[curves[:,2]==ref][::max(1,len(curves[curves[:,2]==ref])//8),:2].ravel().astype(int))
    incident = {int(v):set() for v in ids}
    for face,ref in zip(tri,refs):
        for v in face:
            if int(v) in incident: incident[int(v)].add(int(ref))
    rows=[]
    with CanonicalKernel([str(ROOT/'target/debug/canonical_kernel_host.exe')],timeout=60) as host:
        obj=host.load_rhai((V4/'results/smooth_wing_recovery/recovered_sf2.rhai').read_text(encoding='utf-8'),1.)
        if obj.identity != geometry: raise ValueError('LiveGeometryIdentityMismatch')
        authority=bind_sf2_policy(bind_canonical(obj)).authority
        for v in sorted(ids):
            live=np.array([authority.exact_normal(r,wall[v]) for r in sorted(incident[int(v)])])
            old=state.normals[list(state.incidents[mapping[v]])]
            if live.tobytes()!=old.tobytes(): raise ValueError('LiveIncidentNormalMismatch:'+str(v))
            rows.append(dict(vertex=int(v),sheets=sorted(incident[int(v)]),normals_hash=array_hash(live)))
    return dict(geometry=geometry,kernel_source=source,samples=len(rows),incident_normals_bit_equal=True,
        source_sheets=sorted(map(int,np.unique(refs))),curve_references=sorted(map(int,np.unique(curves[:,2]))),rows=rows)


def recover(out):
    State = existing_state_class()
    state, selected = State.load(STUDY/'planner/selected_state')
    nominal = np.load(STUDY/'planner/input_state/increments.npy', allow_pickle=False)
    if not np.array_equal(nominal, np.broadcast_to(nominal[0], nominal.shape)):
        raise ValueError('NonuniformNominalAuthority')
    planner_source=V3/'sdf_guided_advancing_front_qualification/run_naca_staged_planner.py'
    nodes=[n for n in ast.parse(planner_source.read_text(encoding='utf-8')).body
        if (isinstance(n,ast.FunctionDef) and n.name=='nominal_increments') or
        (isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id in ('H1','LAYERS','R_NOMINAL') for t in n.targets))]
    namespace={'np':np}
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(planner_source),'exec'),namespace)
    if namespace['nominal_increments']().tobytes()!=nominal[0].tobytes():
        raise ValueError('NominalScheduleImplementationMismatch')
    staged = read(STUDY/'planner/staged/CURRENT_SCHEDULE.json')
    if file_hash(STUDY/'planner/staged/current_schedule.npy') != staged['schedule_file_sha256']:
        raise ValueError('SelectedScheduleSourceMismatch')
    if not np.array_equal(selected, np.load(STUDY/'planner/staged/current_schedule.npy')):
        raise ValueError('SelectedScheduleHandoffMismatch')
    planning = read(STUDY/'planner/PLANNING_RESULT.json')
    if not planning['canonical_numerical_pass'] or not read(STUDY/'planner/COMPLETE_GEOMETRY_GATE.json')['pass_gate']:
        raise ValueError('HistoricalPlanNotAccepted')
    shell = read(STUDY/'shell/FINAL_INDEPENDENT_AUDIT.json')
    if not shell['pass_gate'] or not shell['all_20_layers']:
        raise ValueError('HistoricalShellNotAccepted')
    case = read(PRIOR/'CASES.json')['wing']
    wall, tri, surface, original = load_case(case)
    with np.load(V4/'results/live_sf2_first_layer/A/CANDIDATE.npz') as z:
        first = z['front']; refs = z['face_refs']; corners = z['corner_refs']; curves = z['curve_edges']
    mapping = exact_vertex_map(state.wall, wall)
    if state.first[mapping].tobytes() != first.tobytes():
        raise ValueError('AcceptedFirstFrontMismatch')
    inverse = np.argsort(mapping)
    def oriented_key(f):
        t = tuple(map(int, f)); return min(t, t[1:]+t[:1], t[2:]+t[:2])
    historical_faces = {oriented_key(inverse[f]): i for i, f in enumerate(state.faces)}
    face_map = np.array([historical_faces[oriented_key(f)] for f in tri])
    if len(set(face_map)) != len(tri) or len(historical_faces) != len(tri):
        raise ValueError('SourceFaceMappingMismatch')
    href = np.load(STUDY/'planner/refs.npy')
    if not np.array_equal(href[face_map], refs): raise ValueError('SourceSheetMappingMismatch')
    live=live_normal_check(state,mapping,wall,tri,refs,corners,curves,case['geometry_identity'])
    durable_json(out/'LIVE_NORMALS.json',live)
    # Check the stored vectors against the exact expression in bind_planner.bind.
    # This corroborates provenance. It is not a new live trajectory implementation.
    vector_differences = 0
    for v, ss in enumerate(state.incidents):
        n = state.normals[list(ss)]
        direct = n.sum(axis=0)/np.linalg.norm(n.sum(axis=0))
        vector_differences += direct.tobytes() != state.vectors[0, v].tobytes()
    if vector_differences or not np.array_equal(state.vectors, np.broadcast_to(state.vectors[0], state.vectors.shape)):
        raise ValueError('RetainedLaterVectorProvenanceMismatch')
    src = [SF2/n for n in ('bind_planner.py','staged_dispatch.py','finish_planning.py',
        'check_complete_geometry.py','materialize.py','verify_shell.py','SF2/FROZEN_CONTRACT.json')]
    src += [STUDY/n for n in ('planner/selected_state/state.json','planner/input_state/increments.npy',
        'planner/staged/CURRENT_SCHEDULE.json','planner/staged/current_schedule.npy',
        'planner/PLANNING_RESULT.json','planner/COMPLETE_GEOMETRY_GATE.json','shell/FINAL_INDEPENDENT_AUDIT.json')]
    src += [STUDY/'planner/selected_state'/n for n in ('wall.npy','first_front.npy','faces.npy','source_normals.npy','later_vectors.npy','increments.npy','positions.npy','delta.npy')]
    src += [V3/'sdf_guided_advancing_front_qualification'/n for n in ('accepted_front_state.py','run_naca_staged_planner.py')]
    src += [V4/'results/live_sf2_first_layer'/n for n in ('REQUEST.json','AUTHORITY.json','A/CANDIDATE.npz')]
    policy_path=V4/'cases/live_spline_sf2_coupled_development.json'
    metric_floor=read(policy_path)['policy']['minimum_owned_layer_thickness_m']
    src += [policy_path,STUDY/'shell/displacement_record/record.json']
    record = dict(schema='V4RetainedSF2ContinuationDiagnostic/1', units='m',
        geometry=case['geometry_identity'], surface=case['surface_identity'], committed_parent=case['committed'],
        committed_layer=1, original_request_file_sha256=original['request_file_sha256'],
        original_request_identity=identity(original['declaration']), original_schedule_identity=original['schedule_identity'],
        authorization='User-authorized recovery and binding for diagnostics only; no generation or publication.',
        first_scalar_quantity='full_normal_height_m', later_scalar_quantity=state.scalar_quantity,
        ownership=state.ownership, historical_layer_count=selected.shape[1],
        nominal_increment_hex=[float(x).hex() for x in nominal[0]],
        nominal_cumulative_path_hex=[float(x).hex() for x in np.cumsum(nominal[0],dtype=np.float64)],
        selected_schedule_array_hash=array_hash(selected),
        selected_schedule_per_vertex=True, next_increment_hex=float(selected[0,1]).hex(),
        current_to_historical_vertex_map=array_hash(mapping), current_to_historical_face_map=array_hash(face_map),
        positions_are='Retained planner comparison fixtures. Not live general-purpose trajectory generation.',
        generation_authorized=False, sources=[dict(path=str(p.relative_to(ROOT)).replace('\\','/'),sha256=file_hash(p)) for p in src])
    durable_json(out/'DIAGNOSTIC_REQUEST.json',record)
    arguments=dict(layer=1, units='m', geometry=case['geometry_identity'], parent=case['committed'])
    validate_diagnostic(record,record,**arguments)
    negatives={}
    for key,value,error in [('units','mm','ContinuationUnitsMismatch'),('geometry','wrong','ContinuationGeometryMismatch'),
                            ('parent','stale','StaleContinuationParent'),('layer',2,'ContinuationSchedulePositionMismatch'),
                            ('layer',20,'NoPendingIncrement')]:
        negatives[key+str(value)]=rejected(lambda:validate_diagnostic(record,record,**dict(arguments,**{key:value})),error)
    changed=copy.deepcopy(record);changed['next_increment_hex']=float(selected[0,1]*2).hex()
    negatives['altered_record']=rejected(lambda:validate_diagnostic(changed,record,**arguments),'AlteredContinuationDiagnostic')
    # Exercise the actual existing production reader, without disguising units.
    path_request=copy.deepcopy(original['declaration'])
    path_request.update(scalar_quantity='path_length_m',requested_layer_count=20,
        increments_m_hex=record['nominal_increment_hex'],schedule_total_m_hex=record['nominal_cumulative_path_hex'][-1])
    negatives['production_path_length']=rejected(lambda:decode_schedule(path_request),'AmbiguousHeightMeaning')
    local=copy.deepcopy(path_request);local['height_scope']='source_vertex'
    negatives['production_selected_local_schedule']=rejected(lambda:decode_schedule(local),'LocalScheduleRequiresQualifiedMapping')
    wrong=selected.copy();wrong[0,0]=np.nextafter(wrong[0,0],np.inf)
    negatives['changed_first_increment']=rejected(lambda:state.positions(wrong),'ImmutableFirstIncrementChanged')
    # Reconstruct only retained, already materialized historical fronts.
    positions=state.positions(selected)
    shell_positions=np.load(STUDY/'shell/displacement_record/positions.npy',mmap_mode='r')
    if positions.tobytes()!=shell_positions.tobytes(): raise ValueError('NativeDisplacementHandoffMismatch')
    pairs=state.pairs
    delta=positions[2]-positions[1]
    height=np.einsum('ij,ij->i',state.normals[pairs[:,1]],delta[pairs[:,0]])
    vi,si=map(int,pairs[int(np.argmin(height))])
    counterexample=dict(historical_vertex=vi,current_vertex=int(inverse[vi]),source_normal_index=si,
        source_normal=state.normals[si].tolist(),actual_delta=delta[vi].tolist(),
        scalar_increment_m=float(selected[vi,1]),normal_increment_m=float(height.min()),
        difference_m=float(selected[vi,1]-height.min()))
    # Read-only counterexample against the established V4 scalar predictor's
    # own source/error contract. This historical query is NOT a replacement for
    # the live canonical owner or an input to any new front generation.
    from sdf_cfd_mesher_v4.sdf.native import NativeSdf
    from sdf_cfd_mesher_v4.common.authority import reference
    geometry_path,geometry_hash=reference('common geometry')
    if geometry_path.resolve()!=(STUDY/'WING_AUTHORITY_A.json').resolve():
        raise ValueError('ScalarCounterexampleGeometryMismatch')
    native=NativeSdf();sample=native.evaluate(positions[2,[vi]])
    target=float(np.cumsum(nominal[0],dtype=np.float64)[1])
    scalar_counterexample=dict(geometry_file=str(geometry_path.relative_to(ROOT)),geometry_file_sha256=geometry_hash,
        source_identity=native.source_identity,historical_vertex=vi,current_vertex=int(inverse[vi]),
        point=positions[2,vi].tolist(),phi_m=float(sample.phi[0]),nominal_cumulative_target_m=target,
        residual_m=float(sample.phi[0]-target),source_error_bound_m=float(sample.source_error_bound[0]),
        outside_existing_projection_stop=bool(abs(sample.phi[0]-target)>sample.source_error_bound[0]),
        prediction_called=False,scope='Historical NativeSdf read-only scalar semantics check; not live geometry replacement.')
    durable_json(out/'SCALAR_COUNTEREXAMPLE.json',scalar_counterexample)
    region={}
    for ref in sorted(set(refs)):
        vertices=np.unique(tri[refs==ref]); old=mapping[vertices]
        region[str(int(ref))]=dict(vertices=len(vertices),path_min=float(np.linalg.norm(delta[old],axis=1).min()),
            path_max=float(np.linalg.norm(delta[old],axis=1).max()))
    rows=[]
    for k in range(selected.shape[1]):
        vals=selected[:,k]
        rows.append(dict(layer=k+1,nominal=float(nominal[0,k]),nominal_hex=float(nominal[0,k]).hex(),
            distinct_values=len(np.unique(vals)),changed_vertices=int(np.count_nonzero(vals!=nominal[:,k])),
            min=float(vals.min()),max=float(vals.max())))
    return dict(classification='V4SF2ContinuationRecoveredDiagnosticBindingProductionTargetGap',
        diagnostic_request_identity=identity(record),diagnostic_request_file_sha256=file_hash(out/'DIAGNOSTIC_REQUEST.json'),
        vertices=len(wall),triangles=len(tri),wall_and_first_front_bit_equal_after_exact_permutation=True,
        all_oriented_faces_and_sheet_refs_equal=True,vertex_map=array_hash(mapping),face_map=array_hash(face_map),
        later_vector_expression_bit_differences=vector_differences,original_request_unchanged=True,
        first_height_hex=state.first_height.hex(),second_increment_hex=float(selected[0,1]).hex(),
        second_increment_m=float(selected[0,1]),metric_floor_m=metric_floor,
        metric_floor_hex=float(metric_floor).hex(),second_normal_min=float(height.min()),
        second_normal_max=float(height.max()),second_path_min=float(np.linalg.norm(delta,axis=1).min()),
        second_path_max=float(np.linalg.norm(delta,axis=1).max()),rows=rows,regions=region,
        corners=int(np.count_nonzero(corners)),curve_vertices=len(np.unique(curves[:,:2])),
        accepted_positions_hash=array_hash(positions),accepted_state_load_verified_all_saved_arrays=True,
        nominal_schedule_bit_equal_to_existing_function=True,path_is_not_normal_height_witness=counterexample,
        scalar_offset_counterexample=scalar_counterexample,
        native_displacement_positions_bit_equal=True,live_normals= {k:v for k,v in live.items() if k!='rows'},
        historical_late_components=staged['late_components'],historical_compressed_vertices=planning['compressed_vertices'],
        historical_shell_pass=shell['pass_gate'],historical_shell_prisms=shell['physical_prisms'],
        rejections=negatives,no_layer_advanced=True,production_forecast_generation_connected=False,
        blocker='Existing optimized forecast requires scalar-offset Prediction fields and global normal-height schedule. Accepted SF2 uses immutable X1, path-length later vectors, and selected per-vertex future increments. No supported declared-vector target/forecast contract is installed. Do not relabel these as SDF distances.',
        frozen_contract_conflict='FROZEN_CONTRACT.wing_candidate says compatible-offset later vectors; actual bind_planner, state.scalar_quantity, saved vectors, selected positions and shell displacement record use unit averaged incident normals with path-length scalars. Metadata is not used to override executable and retained array evidence.')


def access_controls(out):
    import check_sealed_front_connection as prior
    from sealed_front_connection import qualified_dispatch
    # Reuse guarded current-epoch access. Never invoke recovery or enrollment.
    with patch.object(prior,'save',lambda name,value:durable_json(out/name,value)):
        with qualified_dispatch():
            prior.wing_open('WING_ACCESS.json')
            prior.capsule()
            prior.dispatch()


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--access-controls',action='store_true');args=p.parse_args()
    if args.output.exists():raise ValueError('PreserveExistingDiagnostic')
    args.output.mkdir(parents=True)
    started=time.perf_counter()
    result=recover(args.output)
    if args.access_controls:access_controls(args.output)
    result['seconds']=time.perf_counter()-started
    result['peak_working_set_bytes']=psutil.Process().memory_info().peak_wset
    result['access_controls_executed']=args.access_controls
    durable_json(args.output/'CHECK.json',result)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
