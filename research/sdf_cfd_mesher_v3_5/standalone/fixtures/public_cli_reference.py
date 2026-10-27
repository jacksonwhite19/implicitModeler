"""Experimental file-input launcher for the released isolated direct-complete CLI.

The mesher, owners, admission, publication and consumer remain in the release.
This adapter supplies ordinary CLI inputs and its required owned process context.
"""
from pathlib import Path
from datetime import datetime, timezone
from types import SimpleNamespace
import argparse
import copy
import hashlib
import json
import math
import os
import subprocess
import sys
import time

sys.dont_write_bytecode = True
SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[3]
SOURCE = REPO / 'research/finite_cli_runs/166/research/sdf_cfd_mesher_v4'
PAYLOAD, OWNER, TRANSITION, NATIVE = 3480, 3600, 3000, 450
RELEASE = '980b044e55fffaa6d0e0c99991bafe3f81a9e268164625864c8a90247faa9727'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def write(path, value):
    with Path(path).open('x', encoding='utf8') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)


def surface_configuration(case, size=None):
    """Map a nominal SI edge-size request to the existing generic manual metric."""
    if size is None:
        return case, None  # Preserve omitted-option case bytes and provenance.
    if type(size) not in (int, float) or not math.isfinite(size) or size <= 0:
        raise ValueError('FinitePositiveSurfaceSizeRequired')
    squared = size * size
    if squared == 0 or not math.isfinite(squared) or not math.isfinite(1 / squared):
        raise ValueError('SurfaceSizeNotRepresentableMetric')
    try:
        manual = case['general_sdf']['manual']
        surface = case['surface']
        lower, upper = manual['minimum_size_m'], manual['maximum_size_m']
        original_size = manual['size_m']
    except (KeyError, TypeError) as error:
        raise ValueError('SurfaceSizeRequiresGenericManualSIProfile') from error
    if (any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in (lower, upper, original_size))
            or not lower <= original_size <= upper or surface.get('minimum_size_m') != lower
            or surface.get('maximum_size_m') != upper
            or surface.get('length_scale') != 1 or surface.get('metric_multiplier') != 1):
        raise ValueError('SurfaceSizeProfileBoundsConflict')
    if not lower <= size <= upper:
        raise ValueError('SurfaceSizeOutsideExistingBounds', lower, upper)
    effective = copy.deepcopy(case)
    effective['general_sdf']['manual']['size_m'] = size
    summary = dict(schema='V4NominalSurfaceSizeRequest/1', source='explicit_cli',
        flag='--surface-size', quantity='nominal_triangle_edge_length', units='m', requested=size,
        original_manual_size_m=original_size, effective_manual=copy.deepcopy(effective['general_sdf']['manual']),
        effective_surface=copy.deepcopy(surface),
        mapping='general_sdf.manual.size_m only; existing matching min/max bounds retained',
        further_refinement_possible=True,
        remaining_constraints=['geometry normal variation and position/normal accuracy',
            'source features and projection policy', 'Prism-host tangent metric and shape conditioning',
            'MMGS adaptation and independent final acceptance'],
        limitation='Nominal metric edge-size request, not an exact output edge length or triangle-count guarantee; seed, Prism and bulk sizing are separate.')
    return effective, summary


def identity(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
        allow_nan=False).encode()).hexdigest()


def surface_policy_configuration(case, policy, size, angle_degrees=None, position_error=None):
    """Resolve one declared engineering policy before any geometry execution."""
    if policy != 'size-led':
        raise ValueError('UnsupportedSurfacePolicy')
    if size is None:
        raise ValueError('SizeLedRequiresExplicitSurfaceSize')
    # Use the same size/bounds/metric-representability checks as the legacy CLI.
    surface_configuration(case, size)
    if case['general_sdf']['manual']['size_m'] != size:
        raise ValueError('SizeLedNominalSizeNotApplied')
    default_position = size / 4
    normal = 15. if angle_degrees is None else angle_degrees
    position = default_position if position_error is None else position_error
    from surface_requirements195 import configure as accuracy_configuration
    effective, accuracy = accuracy_configuration(case, normal, position)
    # Derived inputs are not flags the user supplied. Keep those origins explicit.
    supplied = {k:v for k,v in (('--surface-normal-angle-deg', angle_degrees),
        ('--surface-position-error', position_error)) if v is not None}
    accuracy.update(source='declared_surface_policy', supplied_flags=supplied,
        requested_normal_angle_deg=angle_degrees, requested_position_error_m=position_error,
        resolved_normal_angle_deg=normal, resolved_position_error_m=position)
    manual = effective['general_sdf']['manual']
    summary = dict(schema='V4SizeLedEngineeringPolicy/1', selection='size-led',
        selected_by='--surface-policy', fixed_before_generation=True,
        requested=dict(nominal_edge_m=size, normal_angle_deg=angle_degrees,
            position_error_m=position_error),
        preset=dict(position_formula='nominal_edge_m / 4', position_fraction=.25,
            position_units='m', default_position_m=default_position,
            normal_angle_deg=15., normal_units='degrees'),
        basis='h/4 is a dimensionally scaled positional budget. 15 degrees is an engineering choice anchored by Task196 at h=200um; neither is a theorem, curvature inference or universal best value.',
        precedence='Each explicit advanced error flag overrides only its corresponding preset default. No clamp or observed-error adjustment.',
        field_origins=dict(nominal_edge_m='explicit_cli',
            position_limit_m='preset' if position_error is None else 'explicit_cli',
            normal_limit_rad='preset' if angle_degrees is None else 'explicit_cli'),
        supplied_advanced_flags=supplied, effective=dict(copy.deepcopy(accuracy['effective']),
            nominal_edge_m=size,nominal_edge_m_hex=float(size).hex()),
        effective_profile=effective['general_sdf']['profile'],
        remaining_controls=dict(manual=copy.deepcopy(manual),
            surface={k:copy.deepcopy(v) for k,v in effective['surface'].items()
                if k not in ('position_limit_m','normal_limit_rad')},
            source_normal_map_step_m=1e-5, curvature_reserve=2.5),
        projection_relationship='Projection tolerance and displacement remain independent, unchanged controls. A tighter declared position allowance is not clamped to them; final numerical acceptance can refuse.',
        interpretation='Nominal triangle edge target, not a hard uniform edge length or face-count guarantee. Geometry/accuracy, shape and Prism-host constraints can require finer faces; flat regions need not be inaccurate at coarse size.',
        assurance='sampled_numerical_only', achieved_error_prediction=False,
        limitation='No whole-source coverage, continuous accuracy, exact normals, repeatability, y-plus or CFD guarantee. Declared limits remain mandatory and are never chosen from observed maxima.')
    return effective, accuracy, summary


def surface_policy_outcome(root, settings, stop_after='surface'):
    """Report retained achieved values after the existing owner retires."""
    from finite_research.partial_stages import validate
    from surface_requirements195 import verify_numerical_evidence
    packet = Path(root)/'request'; output = packet/'live01/output'
    planned = read(packet/'LIVE_REQUEST.json'); stage = read(packet/'STAGE_REQUEST.json')
    if stop_after not in ('surface','prisms','complete'):raise ValueError('UnsupportedPolicyReportStage')
    if stop_after=='complete':
        if planned.get('stop_after')!='complete' or stage.get('stop_after')!='complete':raise ValueError('CompletePolicyRequiresCompleteRequest')
        from finite_research.direct_consumer import source_dependency
        from contextlib import ExitStack
        import numpy as np
        lifecycle=read(Path(root)/'LIFECYCLE.json');cold=read(Path(root)/'controller/COLD_READBACK.json')
        current=read(output/'CURRENT.json');parent=read(output/'background/PARENT_EXPECTATIONS.json')
        prepared=read(output/'DIRECT_PUBLICATION_PREPARED.json')
        if (lifecycle.get('passed') is not True or lifecycle.get('controller_returncode')!=0
            or lifecycle.get('publication_state')!='COMMITTED' or lifecycle.get('cold_readback_state')!='PASSED'
            or lifecycle.get('cold_reader_returncode')!=0 or cold.get('passed') is not True
            or lifecycle.get('cold_readback_sha256')!=sha(Path(root)/'controller/COLD_READBACK.json')
            or current.get('schema')!='V4DirectCompleteAcceptance/1'
            or current.get('source_release_sha256')!=planned['source_release_sha256']
            or current.get('parent_expectations_sha256')!=sha(output/'background/PARENT_EXPECTATIONS.json')
            or current.get('mesh')!=parent['binding']['parent_prism_mesh']):
            raise ValueError('CompletePolicyRequiresCommittedColdReadback')
        if current!=prepared.get('current'):raise ValueError('ChangedCompletePolicyCurrent')
        with np.load(output/'MESH.npz',allow_pickle=False) as saved,ExitStack() as held:
            dependency=source_dependency(output,parent,saved['points'],held)
        completion=dict(artifacts=dependency['files'])
        counts=accepted_prism_layers(output,stage['upstream_flags']['through_layer'])
        if (type(counts['accepted']) is not int or counts['accepted']!=stage['upstream_flags']['through_layer']
            or type(stage['upstream_flags']['through_layer']) is not int):raise ValueError('TypedCompleteLayerCountRequired')
    else:
        completion=validate(output, stop_after, planned['case_sha256'], stage['upstream_flags']['through_layer'],
            planned['source_release_sha256'], sha(packet/'STAGE_REQUEST.json'))
    fresh = output/'surface/A/fresh'; audit = read(fresh/'AUDIT.json')
    for name in ('AUDIT.json','SURFACE_AUTHORITY.json'):
        relative='surface/A/fresh/'+name
        if completion['artifacts'].get(relative)!=sha(fresh/name):
            raise ValueError('UnboundPolicySurfaceArtifact',name)
    record = read(fresh/'SURFACE_AUTHORITY.json')
    verify_numerical_evidence(read(Path(root)/'CASE.json'), audit,
        geometry_identity=record['geometry_identity'])
    geometry = audit['numerical_geometry']
    # The frozen public policy must still describe the actual effective CASE.
    case = read(Path(root)/'CASE.json')
    for key in ('position_limit_m','normal_limit_rad'):
        if float(case['surface'][key]).hex() != settings['effective'][key+'_hex']:
            raise ValueError('DetachedSurfacePolicyRequirements')
    report=dict(status='VERIFIED_SURFACE_STOP', assurance=audit['surface_assurance'],
        declared=copy.deepcopy(settings['effective']),
        achieved=dict(vertices=audit['vertices'], triangles=audit['triangles'],
            maximum_position_estimate_m=geometry['maximum_position_estimate_m'],
            maximum_normal_error_estimate_rad=geometry['maximum_normal_error_estimate_rad'],
            maximum_normal_error_estimate_deg=math.degrees(geometry['maximum_normal_error_estimate_rad']),
            sample_count=geometry['sample_count'], edge_quantiles_m=audit['edge'],
            composed_directional_request_quantiles_m=audit['sizing']['requested_directional_size_m']),
        numerical_audit_passed=audit['passed'], topology_passed=audit['topology']['passed'],
        forbidden_intersections=audit['self_intersection']['forbidden'],
        unresolved_intersections=audit['self_intersection']['unresolved'],
        tradeoff='Accuracy, geometry, shape and Prism-host constraints can refine below the nominal target. Larger declared allowances may permit lower geometric accuracy; no count or runtime gain is promised.')
    if stop_after=='prisms':
        current=read(output/'CURRENT.json');requested=stage['upstream_flags']['through_layer']
        if (type(completion.get('requested_prism_layers')) is not int
                or type(completion.get('accepted_prism_layers')) is not int
                or completion['requested_prism_layers']!=requested
                or completion['accepted_prism_layers']!=requested
                or current.get('schema')!='V4FastProdMeshOnly/1'
                or type(current.get('layer')) is not int or current['layer']!=requested
                or current.get('sha256')!=sha(output/'MESH.npz')
                or type(current.get('cells')) is not int or current['cells']<1):
            raise ValueError('IncompletePolicyPrismStop')
        report.update(status='VERIFIED_PRISMS_STOP',prism_stage=dict(requested_layers=requested,
            accepted_layers=completion['accepted_prism_layers'],accepted_cells=current['cells'],
            mesh_identity=current['mesh'],mesh_npz_sha256=current['sha256'],
            first_quantity='full_normal_height_m',later_quantity='path_length_m',
            scope='Accepted prism shell only. Later path lengths are not newly verified true-normal heights; no bulk or CFD qualification.'))
    if stop_after=='complete':
        report.update(status='VERIFIED_COMPLETE_WORKFLOW',prism_stage=dict(requested_layers=counts['requested'],accepted_layers=counts['accepted']),
            complete_consumer=cold['consumer'],publication_state='COMMITTED',cold_readback_state='PASSED',
            continuous_sdf_fluid='NOT_PERFORMED',normal_run_qualified=False,solver_ready=False)
    return report


def prism_configuration(case, layers=None, first_height=None, growth=None):
    """Configure the existing first-normal/later-path schema; never accept a mesh."""
    supplied = (layers is not None, first_height is not None, growth is not None)
    if any(supplied) and not all(supplied):
        raise ValueError('SupplyAllThreePrismControlsTogether')
    effective = copy.deepcopy(case)
    original = case['prisms']
    if 'later_path_lengths_m_hex' in original and 'later_normal_increments_m_hex' in original:
        raise ValueError('ConflictingPrismTrajectoryQuantities')
    host_changes = {}
    if all(supplied):
        if type(layers) is not int or not 1 <= layers <= sys.maxsize:
            raise ValueError('PositivePlatformRepresentablePrismLayerCountRequired')
        if (type(first_height) not in (int, float) or not math.isfinite(first_height)
                or first_height <= 0 or type(growth) not in (int, float)
                or not math.isfinite(growth) or growth < 1):
            raise ValueError('FinitePositiveFirstHeightAndGrowthAtLeastOneRequired')
        steps = []
        total = 0.
        for index in range(layers):
            try:
                value = float(first_height) * float(growth)**index
            except OverflowError as error:
                raise ValueError('PrismScheduleOverflow') from error
            if not math.isfinite(value) or value <= 0:
                raise ValueError('PrismStepOverflowOrUnderflow')
            following = total + value
            if not math.isfinite(following) or following <= total:
                raise ValueError('NonIncreasingFinitePrismStackRequired')
            steps.append(value)
            total = following
        p = effective['prisms']
        p.pop('later_normal_increments_m_hex', None)
        p.pop('growth_hex', None)
        p['first_full_normal_thickness_m_hex'] = steps[0].hex()
        p['later_path_lengths_m_hex'] = [v.hex() for v in steps[1:]]
        host = effective['surface']['prism_host']
        if host is not None:
            for key, value in dict(first_height_m=steps[0],
                    minimum_owned_layer_thickness_m=min(steps[1:], default=None)).items():
                if host[key] != value:
                    host_changes[key] = dict(original=host[key], effective=value)
                host[key] = value
    else:
        key = 'later_path_lengths_m_hex' if 'later_path_lengths_m_hex' in original else 'later_normal_increments_m_hex'
        encoded = [original['first_full_normal_thickness_m_hex'], *original[key]]
        steps = []
        total = 0.
        for value in encoded:
            step = float.fromhex(value)
            if not math.isfinite(step) or step <= 0 or step.hex() != value:
                raise ValueError('PositiveCanonicalBinary64Required')
            following = total + step
            if not math.isfinite(following) or following <= total:
                raise ValueError('NonIncreasingFinitePrismStackRequired')
            steps.append(step)
            total = following
    quantity = ('path_length_m' if 'later_path_lengths_m_hex' in effective['prisms']
        else 'normal_height_m')
    summary = dict(source='explicit_cli' if all(supplied) else 'case_profile',
        requested_layer_count=len(steps), units='m', first_quantity='full_normal_height_m',
        later_quantity=quantity, steps_m=steps, steps_m_hex=[v.hex() for v in steps],
        nominal_stack_length_m=total, nominal_stack_length_m_hex=total.hex(),
        explicit_controls=(dict(layers=layers, first_height_m=first_height, growth=growth)
            if all(supplied) else None),
        original_prism_schedule_identity=identity(original),
        effective_prism_schedule_identity=identity(effective['prisms']),
        derived_prism_host_hint_changes=host_changes,
        scope='Requested configuration, not acceptance. Generic later steps follow existing estimated original-wall-normal directions; no later true-normal verification.')
    return effective, summary


def accepted_prism_layers(output, requested):
    """Read the producer's accepted count, never substitute the requested count."""
    output = Path(output)
    rows = []
    for name, key in [('RESULT.json', 'last_qualified_layer'), ('STOP.json', 'last_accepted_layer')]:
        path = output/name
        if path.is_file():
            receipt = read(path)
            if type(receipt) is not dict:
                raise ValueError('InvalidAcceptedPrismLayerReceipt', name)
            value = receipt.get(key)
            if type(value) is not int or not 0 <= value <= requested:
                raise ValueError('InvalidAcceptedPrismLayerReceipt', name)
            rows.append(dict(path=str(path), sha256=sha(path), accepted=value, field=key))
    if rows and len({row['accepted'] for row in rows}) != 1:
        raise ValueError('ConflictingAcceptedPrismLayerReceipts')
    accepted = rows[0]['accepted'] if rows else None
    return dict(requested=requested, accepted=accepted,
        status=('NOT_REPORTED' if accepted is None else 'REQUESTED_COUNT_ACCEPTED'
            if accepted == requested else 'PARTIAL_OR_NOT_ACCEPTED'), receipts=rows,
        scope='Producer layer acceptance only; complete-mesh success still requires terminal lifecycle checks.')


def select_source(source, release, profile):
    global SOURCE, RELEASE, PAYLOAD, OWNER, TRANSITION, NATIVE
    source = Path(source).resolve()
    expected_source = REPO / ('research/finite_cli_runs/' + ('222' if profile=='prisms222' else '216' if profile=='prisms216' else '213' if profile=='prisms213' else '210' if profile=='complete210' else '208' if profile=='prisms208' else '207' if profile=='prisms207' else '196' if profile in ('size196','prisms198') else '195' if profile == 'size195' else '194' if profile == 'repair194' else '186' if profile == 'prism186' else '185' if profile == 'metadata185' else '182' if profile == 'measure182' else '181' if profile == 'measure181' else '175' if profile == 'discovery175' else '174' if profile == 'stages174' else '172' if profile == 'seed172' else '170' if profile == 'generous170' else '166') + '/research/sdf_cfd_mesher_v4')
    if source != expected_source or profile not in ('released166', 'generous170', 'seed172', 'stages174', 'discovery175', 'measure181','measure182','metadata185','prism186','repair194','size195','size196','prisms198','prisms207','prisms208','complete210','prisms213','prisms216','prisms222'):
        raise ValueError('ExplicitReviewedSourceProfileRequired')
    if sha(source/'runtime/RELEASE.json') != release:
        raise ValueError('ChangedReleasedMesher')
    if profile in ('generous170', 'seed172', 'stages174', 'discovery175', 'measure181','measure182','metadata185','prism186','repair194','size195','size196','prisms198','prisms207','prisms208','complete210','prisms213','prisms216','prisms222'):
        expected = dict(schema='ExplicitFiniteTimeProfile/1', payload_seconds=14400, owner_seconds=14700,
            transition_seconds=10800, native_seconds=7200, native_stage_seconds=3600, source_task=170)
        if read(source/'runtime/FINITE_TIME_PROFILE.json') != expected:
            raise ValueError('ChangedExplicitTimeProfile')
        PAYLOAD, OWNER, TRANSITION, NATIVE = 14400, 14700, 10800, 7200
    else:
        if release != '980b044e55fffaa6d0e0c99991bafe3f81a9e268164625864c8a90247faa9727':
            raise ValueError('ChangedReleased166')
        PAYLOAD, OWNER, TRANSITION, NATIVE = 3480, 3600, 3000, 450
    SOURCE, RELEASE = source, release


def owner_output_scope(root,profile):
    # Existing disk_check(root,label) visits root/label and root/(label+'_SCOPE').
    # The discovery command's entire request/output tree must be root/label.
    return (root.parent,root.name) if profile in ('stages174','discovery175','measure181','measure182','metadata185','prism186','repair194','size195','size196','prisms198','prisms207','prisms208','complete210','prisms213','prisms216','prisms222') else (root,'cli')

def setup():
    sys.path[:0] = [str(SOURCE/'tools'), str(SOURCE/'src'), str(SOURCE/'tools/finite_support')]
    from finite_research import inputs
    return inputs


def bounded_payload_deadline(spec, owner_deadline, absolute_remaining):
    now=time.monotonic()
    deadline=min(now+spec.get('payload_cap_seconds',PAYLOAD),owner_deadline,now+absolute_remaining)
    shared=spec.get('shared_payload_deadline_monotonic')
    if shared is not None:
        if type(shared) not in (int,float) or not math.isfinite(shared):raise ValueError('InvalidSharedPayloadDeadline')
        deadline=min(deadline,shared)
    return deadline


def measure(name,fn):
    obs=sys.modules.get('stage_observations')
    if obs is None:return fn()
    with obs.span(name,'public_phase'):return fn()

def measured_owner(fn,*args,**kwargs):
    return measure('public.owner_lifetime',lambda:fn(*args,**kwargs))

def worker(spec_path):
    spec=read(spec_path)
    if spec.get('stage_timing') is not None:
        select_source(spec['source_root'],spec['source_release_sha256'],spec['time_profile'])
        setup()
        import stage_observations as obs
        if obs.choice(spec) is not None:
            obs.activate(Path(spec['root'])/'telemetry/public_worker.json','public_worker',24,sha(spec_path))
    try:return _worker(spec_path)
    finally:
        obs=sys.modules.get('stage_observations')
        if obs is not None:obs.finish('raised' if sys.exc_info()[1] is not None else 'returned')

def _worker(spec_path):
    spec = read(spec_path)
    select_source(spec['source_root'], spec['source_release_sha256'], spec['time_profile'])
    root = Path(spec['root'])
    inputs = setup()
    pins = read(root/'OWNER_PINS.json')
    from immutable_state_access import guard
    with inputs.source_scope(RELEASE, pins):
        import native_scope
        from finite_research import process_entry, direct_lifecycle
        images = read(SOURCE/'runtime/FINITE_PYTHON_IDENTITY.json')
        descriptor = read(os.environ[native_scope.ENV])
        owner_deadline = descriptor['windows_job']['deadline']
        absolute_remaining = (datetime.fromisoformat(spec['candidate_deadline_utc'])-datetime.now(timezone.utc)).total_seconds()
        deadline = bounded_payload_deadline(spec,owner_deadline,absolute_remaining)
        def check():
            if time.monotonic() >= deadline:
                raise TimeoutError('SDFCLIInheritedDeadline')
        check.deadline = deadline
        check.remaining_seconds = lambda: max(0., deadline-time.monotonic())
        image = images['controller']
        base = [str(SOURCE/'tools/finite_cli.py'), 'run', '--request-dir', str(root/'request'), '--release-sha256', RELEASE]
        context = dict(schema='OwnedCLIProcessContext/1', evidence_dir=str(root/'controller'),
            manifest=str(root/'OWNER_PINS.json'), manifest_sha256=sha(root/'OWNER_PINS.json'),
            parent_pid=os.getpid(), argv=base, deadline_clock='monotonic', deadline=deadline,
            parent_deadline_monotonic=owner_deadline, python_executable=image['executable'],
            python_sha256=image['sha256'], python_file_identity=image['file_identity'],
            parent_python_executable=images['child']['executable'], parent_python_sha256=images['child']['sha256'],
            parent_python_file_identity=images['child']['file_identity'])
        context_path = root/'CONTEXT.json'
        process_entry.write(context_path, context)
        digest = sha(context_path)
        command = [image['executable'], '-B', *base, '--process-context', str(context_path),
            '--process-context-sha256', digest, '--process-evidence-dir', context['evidence_dir']]
        write(root/'COMMAND.json', dict(argv=command))
        with guard(context_path):
            check()
            with (root/'controller.stdout').open('x') as out, (root/'controller.stderr').open('x') as err:
                process = subprocess.Popen(command, stdout=out, stderr=err)
                try:
                    code = process.wait(timeout=check.remaining_seconds())
                except BaseException:
                    process.kill()
                    process.wait(timeout=5)
                    raise
            lifecycle = direct_lifecycle.terminal_readback(process, context_path, digest,
                Path(read(root/'request/LIVE_REQUEST.json')['output']), check, synthetic=False)
            write(root/'LIFECYCLE.json', lifecycle)
            if code != 0 or not lifecycle['passed']:
                raise ValueError('ScientificControllerOrColdConsumerFailed', code, lifecycle)
        check()
    return 0


def launch(args):
    started = time.monotonic()
    if args.maximum_faces <= 0:
        raise ValueError('ExplicitPositiveIntegerCapacityRequired')
    select_source(args.source_root, args.release_sha256, args.time_profile)
    if args.time_profile=='metadata185' and not args.prepare_only:raise ValueError('Metadata185PrepareOnlyRequired')
    stop_after=getattr(args,'stop_after','complete')
    if stop_after not in ('seed','surface','prisms','complete'):raise ValueError('InvalidRequestedStage')
    surface_policy=getattr(args,'surface_policy',None)
    if surface_policy is not None and not ((args.time_profile=='size196' and stop_after=='surface') or (args.time_profile in ('prisms198','prisms207','prisms208','prisms213','prisms216','prisms222') and stop_after=='prisms') or (args.time_profile=='complete210' and stop_after=='complete')):
        raise ValueError('SizeLedRequiresReviewedSurfaceOrPrismsProfile')
    if args.time_profile in ('prisms198','prisms207','prisms208','prisms213','prisms216','prisms222') and (stop_after!='prisms' or surface_policy!='size-led'):
        raise ValueError('Prisms198RequiresSizeLedPrismsStop')
    if args.time_profile=='complete210' and (stop_after!='complete' or surface_policy!='size-led'):raise ValueError('Complete210RequiresSizeLedCompleteRun')
    if args.time_profile in ('size195','size196') and stop_after!='surface':raise ValueError('Size195RequiresSurfaceOnly')
    if args.time_profile=='repair194' and stop_after!='surface':raise ValueError('Repair194RequiresSurfaceOnly')
    if args.time_profile=='prism186' and stop_after!='prisms':raise ValueError('Prism186RequiresPrismsStop')
    if stop_after!='complete' and args.time_profile not in ('stages174','discovery175','measure181','measure182','metadata185','prism186','repair194','size195','size196','prisms198','prisms207','prisms208','complete210','prisms213','prisms216','prisms222'):raise ValueError('StageStopsRequireReviewedStages174Source')
    payload_cap=getattr(args,'payload_cap_seconds',None)
    owner_cap=getattr(args,'owner_cap_seconds',None)
    if (payload_cap is None)!=(owner_cap is None):raise ValueError('SupplyBothDeadlineCaps')
    if payload_cap is not None and (args.time_profile not in ('stages174','discovery175','measure181','measure182','metadata185','prism186','repair194','size195','size196','prisms198','prisms207','prisms208','complete210','prisms213','prisms216','prisms222') or not 0<payload_cap<=PAYLOAD or not payload_cap+60<=owner_cap<=OWNER):
        raise ValueError('InvalidExplicitDeadlineReduction')
    discovery=getattr(args,'seed_discovery','dense')
    if discovery not in ('dense','sparse'):raise ValueError('InvalidSeedDiscovery')
    if discovery=='sparse' and (args.time_profile!='discovery175' or stop_after!='seed'):raise ValueError('SparseDiscoveryRequires175SeedStop')
    trace=getattr(args,'seed_trace_identities',False)
    if type(trace) is not bool or trace and (args.time_profile!='discovery175' or stop_after!='seed'):raise ValueError('SeedTraceRequires175SeedStop')
    group_deadline=getattr(args,'payload_deadline_monotonic',None)
    if group_deadline is not None and (args.time_profile!='discovery175' or not math.isfinite(group_deadline) or group_deadline<=time.monotonic()):raise ValueError('InvalidSharedGroupDeadline')
    timing=getattr(args,'stage_timing',None)
    if timing is not None and args.time_profile not in ('measure181','measure182','metadata185','prism186'):raise ValueError('TimingRequiresExplicit181')
    restart = read(args.restart)
    # Linux's released Python 3.10 requires +00:00 rather than a UTC Z suffix.
    for name in ('started_utc', 'preparation_deadline_utc', 'candidate_deadline_utc', 'total_deadline_utc'):
        restart[name] = datetime.fromisoformat(restart[name].replace('Z', '+00:00')).isoformat()
    prep_end = datetime.fromisoformat(restart['preparation_deadline_utc'])
    end = datetime.fromisoformat(restart['candidate_deadline_utc'])
    if datetime.now(timezone.utc) >= prep_end:
        raise TimeoutError('SDFCLIPreparationCutoff')
    if sha(SOURCE/'runtime/RELEASE.json') != RELEASE:
        raise ValueError('ChangedReleasedMesher')
    reviews = [read(path) for path in args.review]
    if (not args.prepare_only or reviews) and (len(reviews) != 2 or len({v['reviewer_id'] for v in reviews}) != 2):
        raise ValueError('TwoIndependentLauncherReviewsRequired')
    for review in reviews:
        if (review.get('approved') is not True or review.get('cli_sha256') != sha(SCRIPT)
                or (args.time_profile in ('generous170', 'seed172', 'stages174', 'discovery175', 'measure181','measure182','metadata185','prism186','repair194','size195','size196','prisms198','prisms207','prisms208','complete210','prisms213','prisms216','prisms222') and (review.get('source_release_sha256') != RELEASE
                    or review.get('time_profile') != args.time_profile))):
            raise ValueError('UnreviewedLauncher')
    # Snapshot the original profile once. Existing case/assignment hashes bind the
    # effective input; embedded provenance records the explicit override separately.
    profile_bytes = args.case.read_bytes()
    profile_sha256 = hashlib.sha256(profile_bytes).hexdigest()
    case, prism_settings = prism_configuration(json.loads(profile_bytes.decode('utf-8-sig')),
        args.prism_layers, args.prism_first_height, args.prism_growth)
    seed_flags = (args.seed_spacing, args.seed_cells_across, args.seed_query_budget)
    seed_settings = None
    if any(v is not None for v in seed_flags) and args.time_profile not in ('seed172','stages174','discovery175','measure181','measure182','metadata185','prism186','repair194','size195','size196','prisms198','prisms207','prisms208','complete210','prisms213','prisms216','prisms222'):
        raise ValueError('SeedControlsRequireExplicitReviewedSeed172Source')
    if args.time_profile in ('seed172','stages174','discovery175','measure181','measure182','metadata185','prism186','repair194','size195','size196','prisms198','prisms207','prisms208','complete210','prisms213','prisms216','prisms222'):
        sys.path[:0] = [str(SOURCE/'tools'), str(SOURCE/'src'), str(SOURCE/'tools/finite_support')]
        from seed_controls import configure, validate_settings
        case, seed_settings = configure(case, *seed_flags)
        validate_settings(case['general_sdf']['seed'])
        seed_settings.update(original_seed=copy.deepcopy(json.loads(profile_bytes.decode('utf-8-sig'))['general_sdf']['seed']),
            effective_seed=copy.deepcopy(case['general_sdf']['seed']))
    case, surface_settings = surface_configuration(case, getattr(args, 'surface_size', None))
    accuracy_flags=(getattr(args,'surface_normal_angle_deg',None),getattr(args,'surface_position_error',None))
    accuracy_settings=None
    policy_settings=None
    if surface_policy is not None:
        case,accuracy_settings,policy_settings=surface_policy_configuration(case,
            surface_policy,getattr(args,'surface_size',None),*accuracy_flags)
        surface_settings['effective_surface']=copy.deepcopy(case['surface'])
    elif any(v is not None for v in accuracy_flags):
        if args.time_profile not in ('size195','size196','prisms198','prisms207','prisms208','complete210','prisms213','prisms216','prisms222'):raise ValueError('SurfaceAccuracyRequiresExplicitSize195Source')
        from surface_requirements195 import configure as accuracy_configuration
        case,accuracy_settings=accuracy_configuration(case,*accuracy_flags)
        if surface_settings is not None:
            surface_settings['effective_surface']=copy.deepcopy(case['surface'])
    policy = read(args.policy)
    background = case.pop('background', None)
    if background is not None:
        for name in ('bounds_m', 'growth_per_m', 'maximum_size_m', 'interface_seed_policy'):
            if policy[name] != background[name]:
                raise ValueError('CaseProfileSizingMismatch', name)
    case['geometry'] = dict(rhai_file=str(args.sdf.resolve()), rhai_sha256=sha(args.sdf),
        metres_per_model_unit=case['geometry']['metres_per_model_unit'])
    root = args.output.resolve()
    root.mkdir()  # Exclusive one-command, one-candidate output directory.
    if timing is not None:
        setup()
        import stage_observations as obs
        (root/'telemetry').mkdir()
        obs.activate(root/'telemetry/public.json','public',24,sha(args.restart))
    write(root/'CASE.json', case)
    if args.time_profile == 'seed172':
        setup()
        from seed_preflight import run as seed_preflight
        metadata_pins = {str(SCRIPT):sha(SCRIPT),str(root/'CASE.json'):sha(root/'CASE.json'),
            str(args.case.resolve()):profile_sha256,str(args.sdf.resolve()):sha(args.sdf),
            str(args.policy.resolve()):sha(args.policy),str(args.restart.resolve()):sha(args.restart)}
        preflight = seed_preflight(root/'CASE.json', root, RELEASE, metadata_pins, prep_end)
        seed_settings['preflight'] = preflight
        print(json.dumps(dict(seed_preflight=preflight),allow_nan=False), flush=True)
        if sha(args.case) != profile_sha256:raise ValueError('ChangedOriginalCaseProfile')
    provenance = dict(original_case_path=str(args.case.resolve()), original_case_sha256=profile_sha256,
        effective_case_sha256=sha(root/'CASE.json'), prism_configuration=prism_settings,requested_stop_after=stop_after,seed_discovery=discovery,seed_trace=trace)
    if seed_settings is not None:provenance['seed_configuration'] = seed_settings
    if surface_settings is not None:provenance['surface_size_configuration'] = surface_settings
    if accuracy_settings is not None:provenance['surface_accuracy_configuration']=accuracy_settings
    if policy_settings is not None:
        provenance['surface_policy_configuration']=policy_settings
        write(root/'SURFACE_POLICY.json',policy_settings)
        print(json.dumps(dict(phase='BEFORE_EXECUTION',surface_policy=policy_settings,
            effective_case_sha256=sha(root/'CASE.json')),allow_nan=False),flush=True)
    write(root/'INVOCATION.json', dict(argv=sys.argv, restart=restart, cli_sha256=sha(SCRIPT),
        input_sha256=sha(args.sdf), case_profile_sha256=profile_sha256, policy_sha256=sha(args.policy),
        input_provenance=provenance, prepare_only=args.prepare_only,requested_stop_after=stop_after,seed_discovery=discovery,seed_trace=trace))
    contract = dict(case_path=str(root/'CASE.json'), case_sha256=sha(root/'CASE.json'),
        policy_path=str(args.policy.resolve()), policy_sha256=sha(args.policy),
        imported_source_path=str(args.sdf.resolve()), imported_source_sha256=sha(args.sdf))
    source_assignment = dict(schema='V4DirectSourceAssignment/1', execution_authorized=False,
        maximum_live_invocations=0, execution_target='direct_complete', prism_producer_mode='source_directions',
        proposed_payload_seconds=PAYLOAD, proposed_owner_seconds=OWNER, maximum_faces=args.maximum_faces,
        input_contracts=dict(input=contract), input_provenance=provenance)
    if args.time_profile in ('stages174','discovery175','measure181','measure182','metadata185','prism186','repair194','size195','size196','prisms198','prisms207','prisms208','complete210','prisms213','prisms216','prisms222'):source_assignment['stop_after']=stop_after
    if args.time_profile=='discovery175':
        source_assignment.update(seed_discovery=discovery,seed_trace=trace)
    if timing is not None:source_assignment['stage_timing']=timing
    write(root/'SOURCE_ASSIGNMENT.json', source_assignment)
    live = dict(schema='ReusableFiniteInputAssignment/2', assignment='FileInputDirectComplete',
        human_message=restart['authorization'], execution_target='direct_complete', prism_producer_mode='source_directions',
        upstream_forecast_diagnostics=False, transition_fluid_verification='backend_trusted', publication_enabled=True,
        execution_authorized=True, maximum_live_invocations=1, live_payload_seconds=PAYLOAD, live_owner_seconds=OWNER,
        live_transition_seconds=TRANSITION, proposed_live_payload_seconds=PAYLOAD, proposed_live_owner_seconds=OWNER,
        proposed_transition_seconds=TRANSITION,
        proposed_live_window_seconds=(end-datetime.fromisoformat(restart['started_utc'])).total_seconds(),
        proposed_live_launch_cutoff_seconds=(prep_end-datetime.fromisoformat(restart['started_utc'])).total_seconds(),
        start_utc=restart['started_utc'], deadline_utc=restart['candidate_deadline_utc'],
        live_cutoff_utc=restart['preparation_deadline_utc'], maximum_faces=args.maximum_faces,
        input_contracts=dict(input=contract), native_budget_scope='DOWNSTREAM_NATIVE_ONLY', native_seconds=NATIVE)
    if args.time_profile in ('stages174','discovery175','measure181','measure182','metadata185','prism186','repair194','size195','size196','prisms198','prisms207','prisms208','complete210','prisms213','prisms216','prisms222'):
        live['stop_after']=stop_after
        live['publication_enabled']=stop_after=='complete'
    if args.time_profile=='discovery175':live.update(seed_discovery=discovery,seed_trace=trace)
    if timing is not None:live['stage_timing']=timing
    if seed_settings is not None and 'preflight' in seed_settings and not seed_settings['preflight']['native_plan']['data']['admitted']:
        refusal = dict(passed=False,execution_authorized=False,mesh_generation=False,
            reason=seed_settings['preflight']['native_plan']['data']['reason'],input_provenance=provenance,
            available_controls=['--seed-spacing METRES','--seed-cells-across N','--seed-query-budget SAMPLES'],
            automatic_coarsening=False,accepted_layers=None)
        write(root/'PREFLIGHT_REFUSAL.json',refusal)
        print(json.dumps(refusal,allow_nan=False),flush=True)
        return 1
    images = read(SOURCE/'runtime/FINITE_PYTHON_IDENTITY.json')
    def ordinary(label, arguments):
        remaining = (prep_end-datetime.now(timezone.utc)).total_seconds()
        if group_deadline is not None:remaining=min(remaining,group_deadline-time.monotonic())
        if remaining <= 0:
            raise TimeoutError('SDFCLIPreparationCutoff')
        command = [images['controller']['executable'], '-B', str(SOURCE/'tools/finite_cli.py'),
            *arguments, '--release-sha256', RELEASE]
        if timing is not None and label=='prepare':command += ['--stage-timing',timing]
        write(root/(label+'_COMMAND.json'), dict(argv=command))
        with (root/(label+'.stdout')).open('x') as out, (root/(label+'.stderr')).open('x') as err:
            result = measure('public.'+label,lambda:subprocess.run(command, stdout=out, stderr=err, timeout=min(180, remaining)))
        if result.returncode:
            raise ValueError('OrdinaryCLIPreparationFailed', label, result.returncode)
    ordinary('prepare', ['prepare', '--request-dir', str(root/'prepared'), '--output', str(root/'prepared/live01/output'),
        '--assignment', str(root/'SOURCE_ASSIGNMENT.json'), '--assignment-sha256', sha(root/'SOURCE_ASSIGNMENT.json'),
        '--input-contract', 'input', '--execution-target', 'direct_complete', '--case', str(root/'CASE.json'),
        '--policy', str(args.policy.resolve()), '--maximum-faces', str(args.maximum_faces), '--payload-seconds', str(PAYLOAD), '--owner-seconds', str(OWNER)])
    if sha(args.case) != profile_sha256:
        raise ValueError('ChangedOriginalCaseProfile')
    if args.prepare_only:
        prepared = dict(passed=True, execution_authorized=False, mesh_generation=False,
            source_release_sha256=RELEASE, effective_case_sha256=sha(root/'CASE.json'),
            prepared_manifest_sha256=sha(root/'prepared/PINNED_SOURCES.json'),
            input_provenance=provenance, prism_layers=dict(requested=prism_settings['requested_layer_count'],
                accepted=None, status='NOT_RUN'))
        write(root/'PREPARED_RESULT.json', prepared)
        print(json.dumps(prepared, allow_nan=False), flush=True)
        return 0
    live['input_provenance'] = provenance
    write(root/'LIVE_ASSIGNMENT.json', live)
    ordinary('derive', ['derive', '--request-dir', str(root/'request'), '--output', str(root/'request/live01/output'),
        '--assignment', str(root/'LIVE_ASSIGNMENT.json'), '--assignment-sha256', sha(root/'LIVE_ASSIGNMENT.json'),
        '--input-contract', 'input', '--prepared', str(root/'prepared'), '--prepared-manifest-sha256', sha(root/'prepared/PINNED_SOURCES.json')])
    if sha(args.case) != profile_sha256:
        raise ValueError('ChangedOriginalCaseProfile')
    request = root/'request'
    task = read(request/'TASK.json')
    grant = dict(approved=True, mode='fresh_live_once', human_message=task['human_message'], assignment=task['assignment'],
        maximum_invocations=1, payload_seconds=PAYLOAD, owner_seconds=OWNER, transition_seconds=TRANSITION,
        activation_sha256=sha(request/'ACTIVATION.json'), task_sha256=sha(request/'TASK.json'),
        request_sha256=sha(request/'LIVE_REQUEST.json'), manifest_sha256=sha(request/'PINNED_SOURCES.json'),
        source_release_sha256=RELEASE, publication_enabled=live['publication_enabled'], native_scope='DOWNSTREAM_NATIVE_ONLY', native_seconds=NATIVE)
    write(request/'AUTHORIZATION.json', grant)
    write(request/'LAUNCH_REVIEW.json', dict(approved=True, mode='fresh_live_once',
        authorization_sha256=sha(request/'AUTHORIZATION.json'), task_sha256=grant['task_sha256'],
        manifest_sha256=grant['manifest_sha256'], source_release_sha256=RELEASE,
        independent_launcher_reviews={str(path.resolve()):sha(path) for path in args.review}))
    spec = dict(root=str(root), candidate_deadline_utc=restart['candidate_deadline_utc'],
        source_root=str(SOURCE), source_release_sha256=RELEASE, time_profile=args.time_profile,stage_timing=timing,
        payload_cap_seconds=payload_cap or PAYLOAD,owner_cap_seconds=owner_cap or OWNER,shared_payload_deadline_monotonic=group_deadline)
    write(root/'SPEC.json', spec)
    selected = read(request/'PINNED_SOURCES.json')
    for path in [SCRIPT, args.restart.resolve(), args.case.resolve(), args.policy.resolve(), args.sdf.resolve(),
        *[p.resolve() for p in args.review], *[p for p in root.glob('*.json')],
        request/'AUTHORIZATION.json', request/'LAUNCH_REVIEW.json', request/'PINNED_SOURCES.json']:
        selected[str(path)] = profile_sha256 if path == args.case.resolve() else sha(path)
    if args.time_profile=='prisms222':
        setup()
        from public_failure_seam import prepare as prepare_outer_failure
        from entry_resources import admit as resource_admission
        write(root/'ENTRY_RESOURCE_ADMISSION.json',resource_admission())
        selected[str((root/'ENTRY_RESOURCE_ADMISSION.json').resolve())]=sha(root/'ENTRY_RESOURCE_ADMISSION.json')
        selected=prepare_outer_failure(root,selected)
    write(root/'OWNER_PINS.json', selected)
    inputs = setup()
    if args.time_profile=='prisms222':
        # All contract/pin setup precedes the guarded canonical operation.
        from public_failure_seam import run as after_existing_owner
        owner_root,owner_label=owner_output_scope(root,args.time_profile)
        command=[images['child']['executable'], '-B', str(SCRIPT), '--worker', str(root/'SPEC.json')]
        owner_pins={Path(k):v for k,v in selected.items()}
        owner_pins[root/'OWNER_PINS.json']=sha(root/'OWNER_PINS.json')
        def existing_guarded_owner(protected):
            with inputs.source_scope(RELEASE,{str(k):v for k,v in protected.items()}) as (d,unused):
                import native_scope,resource_limits
                d.supervisor.HERE=owner_root
                requested_owner=owner_cap or OWNER
                remaining=(end-datetime.now(timezone.utc)).total_seconds()
                if datetime.now(timezone.utc)>=prep_end or remaining<18000 or remaining<requested_owner:
                    raise TimeoutError('InitialTopology222EntryReserveOrCutoff')
                entry_resources=resource_admission()
                write(root/'CANDIDATE_CONSUMED.json',dict(utc=datetime.now(timezone.utc).isoformat(),maximum=1,actual_preowner_resource_admission=entry_resources))
                return measured_owner(native_scope.bounded,owner_label,command,requested_owner,protected,d.supervisor,resources=resource_limits.APPROVED)
        result=after_existing_owner(owner_root,owner_label,root,command,owner_pins,existing_guarded_owner)
    else:
        with inputs.source_scope(RELEASE, selected) as (d, unused):
            import native_scope, resource_limits
            owner_root,owner_label=owner_output_scope(root,args.time_profile)
            d.supervisor.HERE = owner_root
            requested_owner=owner_cap or OWNER
            seconds = min(requested_owner, (end-datetime.now(timezone.utc)).total_seconds())
            if datetime.now(timezone.utc) >= prep_end or seconds < requested_owner:
                raise TimeoutError('SDFCLIScientificEntryWindow')
            write(root/'CANDIDATE_CONSUMED.json', dict(utc=datetime.now(timezone.utc).isoformat(), maximum=1))
            owner_pins = {Path(k):v for k,v in selected.items()}
            owner_pins[root/'OWNER_PINS.json'] = sha(root/'OWNER_PINS.json')
            result = measured_owner(native_scope.bounded, owner_label, [images['child']['executable'], '-B', str(SCRIPT), '--worker', str(root/'SPEC.json')],
                seconds, owner_pins, d.supervisor, resources=resource_limits.APPROVED)
    terminal = dict(returncode=result['returncode'], elapsed_seconds=time.monotonic()-started,
        scientific_acceptance=False, normal_run_qualified=False, solver_ready=False,
        requested_stop_after=stop_after,completed_stage=None,full_mesh_complete=False)
    if policy_settings is not None:
        terminal.update(producer_returncode=result['returncode'],
            surface_policy_configuration=policy_settings,
            surface_policy_outcome=dict(status='NOT_VERIFIED',achieved=None))
    # This reporting happens only after the enclosing owner and custody close.
    try:
        if stop_after!='complete':
            stage_path=root/'request/live01/output/STAGE_COMPLETION.json'
            stage=read(stage_path)
            if type(stage) is not dict:raise ValueError('InvalidTerminalStageReceipt')
            if stage.get('requested_stop_after')!=stop_after or stage.get('completed_stage')!=stop_after or stage.get('passed') is not True:
                raise ValueError('ContradictoryTerminalStageReceipt')
            if args.time_profile in ('prism186','prisms198','prisms207','prisms208','complete210','prisms213','prisms216','prisms222') and (stage.get('requested_prism_layers')!=prism_settings['requested_layer_count']
                    or type(stage.get('accepted_prism_layers')) is not int or stage['accepted_prism_layers']!=prism_settings['requested_layer_count']):
                raise ValueError('RequestedPrismCountNotAccepted')
            if args.time_profile=='prisms198' and type(stage.get('requested_prism_layers')) is not int:raise ValueError('TypedRequestedPrismCountRequired')
            if args.time_profile in ('prisms207','prisms208','complete210','prisms213','prisms216','prisms222') and (type(stage.get('requested_prism_layers')) is not int or type(stage.get('accepted_prism_layers')) is not int):raise ValueError('TypedRequestedPrismCountRequired')
            if args.time_profile=='discovery175' and (stage.get('seed_discovery')!=discovery or stage.get('seed_trace') is not trace):raise ValueError('ChangedTerminalSeedDiscovery')
            terminal.update(requested_stop_after=stop_after,completed_stage=stage['completed_stage'],full_mesh_complete=False,
                downstream=stage['downstream'],stage_completion_sha256=sha(stage_path),
                prism_layers=(dict(requested=prism_settings['requested_layer_count'],accepted=stage['accepted_prism_layers'])
                    if stop_after=='prisms' else dict(requested=prism_settings['requested_layer_count'],accepted=None,status='NOT_RUN')))
        else:terminal['prism_layers'] = accepted_prism_layers(root/'request/live01/output',
            prism_settings['requested_layer_count'])
    except (ValueError, OSError, KeyError) as error:
        if terminal['returncode']==0:terminal['returncode']=1
        terminal.update(full_mesh_complete=False,completed_stage=None)
        terminal['prism_layers'] = dict(requested=prism_settings['requested_layer_count'], accepted=None,
            status='REPORTING_ERROR', error=repr(error))
        if args.time_profile in ('prisms207','prisms208','complete210','prisms213','prisms216','prisms222') and result['returncode']!=0:
            terminal['prism_layers']['status']='PRODUCER_FAILED'
            terminal['secondary_reporting_error']=repr(error)
            terminal['primary_failure']='Actual producer/owner returncode; retained STOP/error receipts remain authoritative'
    if stop_after=='complete' and terminal['returncode']==0 and terminal['prism_layers'].get('accepted')!=prism_settings['requested_layer_count']:
        terminal.update(returncode=1,full_mesh_complete=False,completed_stage=None,reporting_error='MissingOrIncompleteAcceptedLayerReceipt')
    if stop_after=='complete' and terminal['returncode']==0:
        terminal.update(full_mesh_complete=True,completed_stage='complete')
    if policy_settings is not None and terminal['returncode']==0:
        try:terminal['surface_policy_outcome']=surface_policy_outcome(root,policy_settings,stop_after)
        except (ValueError,OSError,KeyError,TypeError) as error:
            terminal.update(returncode=1,surface_policy_reporting_error=repr(error))
            if stop_after=='complete':terminal.update(full_mesh_complete=False,completed_stage=None)
    write(root/'TERMINAL.json', terminal)
    print(json.dumps(terminal, allow_nan=False), flush=True)
    return terminal['returncode']


def main():
    if len(sys.argv) == 3 and sys.argv[1] == '--worker':
        return worker(Path(sys.argv[2]))
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('sdf', 'case', 'policy', 'output', 'restart'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--seed-trace-identities',action='store_true',help='175 seed-only identity audit; includes interval classification and dense trace overhead.')
    parser.add_argument('--seed-discovery',choices=('dense','sparse'),default='dense',help='Opt-in175 interval discovery; sparse is seed-only. Dense remains default.')
    parser.add_argument('--payload-deadline-monotonic',type=float,help=argparse.SUPPRESS)
    parser.add_argument('--stop-after',choices=('seed','surface','prisms','complete'),default='complete',help='Stop in the real pipeline. Partial stages require explicitly reviewed stages174 source; complete preserves the default workflow.')
    parser.add_argument('--payload-cap-seconds',type=int,help='Explicitly reduce the inherited payload deadline; stages174 only, requires owner cap.')
    parser.add_argument('--owner-cap-seconds',type=int,help='Explicitly reduce owner lifetime; stages174 only, reserve at least60 seconds after payload.')
    parser.add_argument('--maximum-faces', type=int, default=30000)
    parser.add_argument('--surface-policy',choices=('size-led',),help='Opt-in size196 surface or prisms198/prisms207/prisms208/prisms213/prisms216/prisms222 Prism-stop or complete210 complete-run engineering policy: requires explicit --surface-size h; defaults sampled position allowance to h/4 and normal allowance to 15 degrees before generation. Each advanced error flag independently overrides its default. Show effective policy before execution and achieved errors after. Omit this option to preserve profile behavior. No universal accuracy, edge-size, count or CFD guarantee.')
    parser.add_argument('--surface-normal-angle-deg', type=float, metavar='DEGREES', help='Advanced sampled surface normal-error allowance in degrees, strictly between 0 and 90, for size195/size196. Overrides only the angle in size-led mode; otherwise omission preserves the profile angle. Larger allowances can permit lower geometric accuracy. Separate from source-normal estimation, Prism and CFD requirements.')
    parser.add_argument('--surface-position-error', type=float, metavar='METRES', help='Advanced finite positive sampled position-error allowance in metres for size195/size196. Overrides only position in size-led mode; otherwise omission preserves the profile value. Projection tolerance/displacement remain separate disclosed controls; final numerical acceptance can refuse.')
    parser.add_argument('--surface-size', type=float, metavar='METRES', help='Nominal target triangle edge length in metres for the generic manual SI profile; must remain within its existing min/max. Geometry accuracy, features, Prism-host/shape constraints and MMGS may refine further. No exact edge-length or face-count guarantee. Omit to preserve profile sizing; separate from seed, Prism height and bulk size.')
    parser.add_argument('--prism-layers', type=int, help='Requested positive layer count; supply with both height and growth.')
    parser.add_argument('--prism-first-height', type=float, metavar='METRES', help='First full-normal height in metres; supply all three Prism controls.')
    parser.add_argument('--prism-growth', type=float, metavar='RATIO', help='Growing step ratio >=1: H*R**i. Later generic steps are paths along estimated original-wall normals, not verified later normal heights.')
    seed_mode = parser.add_mutually_exclusive_group()
    seed_mode.add_argument('--seed-spacing', type=float, metavar='METRES', help='Initial seed SI spacing; requires reviewed seed172 source. Separate from final surface/bulk sizing.')
    seed_mode.add_argument('--seed-cells-across', type=int, metavar='N', help='Initial cells across longest padded canonical support bound; realized grid may round upward. Requires seed172.')
    parser.add_argument('--seed-query-budget', type=int, metavar='SAMPLES', help='Explicit scalar grid query cap; no automatic escalation. Requires seed172.')
    parser.add_argument('--prepare-only', action='store_true', help='Prepare configuration without live grants or meshing. seed172 uses a bounded metadata owner. Does not require launch reviews.')
    parser.add_argument('--source-root', type=Path, default=SOURCE)
    parser.add_argument('--release-sha256', default=RELEASE)
    parser.add_argument('--stage-timing',choices=('bounded_operation_caller_v2',),help='Explicit181 bounded operational measurements; no acceptance changes.')
    parser.add_argument('--time-profile', choices=('released166','generous170','seed172','stages174','discovery175','measure181','measure182','metadata185','prism186','repair194','size195','size196','prisms198','prisms207','prisms208','complete210','prisms213','prisms216','prisms222'), default='released166')
    parser.add_argument('--review', type=Path, action='append', default=[], help='Two independent launcher reviews required for meshing.')
    args = parser.parse_args()
    try:return launch(args)
    finally:
        obs=sys.modules.get('stage_observations')
        if obs is not None:obs.finish('raised' if sys.exc_info()[1] is not None else 'returned')


if __name__ == '__main__':
    raise SystemExit(main())
