"""Bounded passive observation of the existing final construction batch.

Selection is frozen as equally spaced ascending canonical vertex IDs, at most
128, before the batch executes. No geometry call, replay, or function replacement.
"""
from contextlib import contextmanager
from collections import Counter
import hashlib
import inspect
import json
from pathlib import Path
import sys
import threading
import numpy as np


@contextmanager
def final_metric_capture(points, refs, output):
    import general_sdf_authority as ga
    import surface_metric_arithmetic as arithmetic
    import prism_host_metric as stock
    from sdf_cfd_mesher_v4.sdf import regional

    selected = np.linspace(0, len(points)-1, min(128, len(points)), dtype=int)
    selected_set = set(map(int, selected))
    rows = {int(i): dict(vertex_id=int(i), coordinates=points[i].tolist(),
                        coordinate_bits=points[i].tobytes().hex(), ref=int(refs[i]))
            for i in selected}
    counts = Counter()
    pending = {}
    failures = []
    previous = sys.getprofile()
    thread = threading.get_ident()
    functions = [ga._normal_variation_data, ga._preserve_geometry,
                 ga.intersect_tangent_metric, ga.apply_shape_metric,
                 arithmetic.preserve_geometry, arithmetic._accelerated,
                 stock._preserve_geometry, stock._exact_psd]
    normal = ga._normal_variation_data.__code__
    curvature = ga.curvature_size.__code__
    project = inspect.unwrap(ga.GeneralSdfAuthority.project_sheet_batch).__code__
    geometry = inspect.unwrap(ga.GeneralSdfAuthority.geometry_metrics_batch).__code__
    composed = inspect.unwrap(ga.GeneralSdfAuthority.composed_metrics_batch).__code__
    normals = inspect.unwrap(ga.GeneralSdfAuthority.construction_normals_batch).__code__
    finish = inspect.unwrap(ga.GeneralSdfAuthority.finish_shape).__code__
    prism = arithmetic._accelerated.__code__ if arithmetic._accelerated else None
    shape = ga.apply_shape_metric.__code__
    preserve = arithmetic.preserve_geometry.__code__
    numerical = regional.numerical_points.__code__
    held = ga._surface_queries.get()
    query = held[1].__code__ if held is not None else None
    codes = {normal, curvature, project, prism, shape, preserve, numerical, query}

    def clone(value):
        if isinstance(value, np.ndarray):
            return dict(values=value.tolist(), bits=value.tobytes().hex(),
                        dtype=value.dtype.str, shape=list(value.shape), strides=list(value.strides))
        if isinstance(value, dict):
            return {k: clone(v) for k,v in value.items()}
        if isinstance(value, (tuple,list)):
            return [clone(v) for v in value]
        if isinstance(value, np.generic):
            return value.item()
        return value

    def ancestor(frame, code):
        parent = frame.f_back
        while parent is not None:
            if parent.f_code is code:
                return parent
            parent = parent.f_back
        return None

    def point_id(frame):
        for code in (geometry, composed):
            parent = ancestor(frame, code)
            if parent is not None and 'i' in parent.f_locals:
                return int(parent.f_locals['i'])
        return None

    def observer(frame, event, arg):
        if frame.f_code not in codes or event not in ('call','return') or failures:
            return
        try:
            if threading.get_ident() != thread:
                raise RuntimeError('DiagnosticObserverThreadMismatch')
            code = frame.f_code
            v = frame.f_locals
            i = point_id(frame)
            chosen = i in selected_set
            if code is preserve:
                parent = frame.f_back.f_code
                phase = 'geometry' if parent is normal else 'prism' if parent is prism else 'shape' if parent is finish else 'unknown'
                if event == 'call':
                    if chosen:
                        pending[id(frame)] = dict(before=clone(v['result']), required=clone(v['required']))
                elif arg is not None:
                    stats = arg[1]
                    branch = ('already_PSD' if 'pre_repair_trials' not in stats else
                              'fallback' if stats['pre_repair_fallback'] else 'trial_'+str(stats['pre_repair_trials']))
                    counts[phase+'/'+branch] += 1
                    if chosen:
                        record = pending.pop(id(frame))
                        record.update(after=clone(arg[0]), stats=clone(stats), branch=branch)
                        rows[i][phase+'_preservation'] = record
                return
            if event != 'return' or arg is None:
                return
            if code is normal and chosen:
                rows[i]['normal_variation'] = {k:clone(v[k]) for k in
                    ('normal','u','v','probe_normals','du','dv','shape','values','vectors','directions','lengths')}
            elif code is curvature and chosen:
                rows[i].setdefault('curvature', []).append({k:clone(v.get(k)) for k in
                    ('kappa','radius','delta','h_chord','h_normal')} | dict(returned=float(arg)))
            elif code is project:
                parent = ancestor(frame, geometry)
                if parent is not None:
                    lo = int(parent.f_locals['lo'])
                    for vertex in selected:
                        offset = 4*(int(vertex)-lo)
                        if 0 <= offset < len(arg):
                            rows[int(vertex)]['raw_probe_sites'] = clone(v['points'][offset:offset+4])
                            rows[int(vertex)]['projected_probe_sites'] = clone(arg[offset:offset+4])
            elif code is numerical:
                parent = ancestor(frame, normals)
                gp = ancestor(frame, geometry)
                if parent is not None and gp is not None:
                    nv = parent.f_locals
                    # Distinguish center and four-probe batches by the existing
                    # geometry frame's progress, without copying the input batch.
                    central = 'normals' not in gp.f_locals
                    lo = int(nv['lo'])
                    base = 0 if central else int(gp.f_locals['lo'])
                    for vertex in selected:
                        offset = int(vertex)-lo if central else 4*(int(vertex)-base)-lo
                        width = 1 if central else 4
                        if 0 <= offset and offset+width <= len(v['points']):
                            gradient = np.array([[g[j] for g in arg.gradient] for j in range(offset,offset+width)])
                            rows[int(vertex)]['center_gradient' if central else 'probe_gradient'] = clone(gradient)
            elif code is prism and chosen:
                rows[i]['prism'] = {k:clone(v[k]) for k in ('matrix','n','geometry_sizes','final_sizes')}
                rows[i]['prism'].update(final=clone(arg[0]), stats=clone(arg[1]))
            elif code is shape and chosen:
                rows[i]['shape'] = dict(principal=clone(v['principal']), rank_one_result=clone(arg[0]), stats=clone(arg[1]))
            elif code is query and chosen:
                rows[i].setdefault('queries', []).append(dict(kind=v['kind'], phase=v.get('phase'),
                    branch='computed' if 'start' in v else 'cache_hit' if 'stored' in v else 'cold_lookup',
                    point_bits=np.asarray(v['value'], dtype='<f8').tobytes().hex(), ref=int(v['ref'])))
        except Exception as error:
            # Capture completeness is reported separately. Observation must not
            # change the numerical result or swallow its primary exception.
            failures.append(type(error).__name__+': '+str(error))
            pending.clear()

    # Setup and custody snapshot failures abort before the numerical batch.
    # Once yielded, diagnostic failures cannot replace its primary exception.
    before = held[2]() if held is not None else None
    ready = previous is None and threading.getprofile() is None and arithmetic._standard_preservation() and prism is not None
    if not ready:
        failures.append('PassiveCapturePreconditionFailed')
    if ready:
        sys.setprofile(observer)
    try:
        yield
    finally:
        if ready:
            sys.setprofile(previous)
        primary = sys.exc_info()[1]
        try:
            identities = functions == [ga._normal_variation_data, ga._preserve_geometry,
                ga.intersect_tangent_metric, ga.apply_shape_metric,
                arithmetic.preserve_geometry, arithmetic._accelerated,
                stock._preserve_geometry, stock._exact_psd]
            complete = all(all(k in row for k in ('normal_variation','curvature','raw_probe_sites',
                'projected_probe_sites','center_gradient','probe_gradient','geometry_preservation',
                'prism_preservation','shape_preservation','prism','shape','queries')) for row in rows.values())
            record = dict(schema='PassiveFinalMetricCapture194/1',
                rule='np.linspace(0,N-1,min(128,N),dtype=int); ascending canonical final IDs',
                population=len(points), selected_ids=selected.tolist(), max_detailed_points=128,
                batch=dict(shape=list(points.shape), strides=list(points.strides), dtype=points.dtype.str),
                counts=dict(counts), rows=list(rows.values()), failures=failures, pending_records=len(pending),
                complete=complete and not failures and not pending,
                observer_restored=sys.getprofile() is previous, function_identities_preserved=identities,
                accelerated_dispatch_preserved=arithmetic._standard_preservation(),
                query_snapshot_before=before, query_snapshot_after=held[2]() if held is not None else None,
                numpy_error_modes=np.geterr(),
                source_sha256={str(Path(m.__file__).resolve()):hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest()
                               for m in (ga,arithmetic,stock,regional)},
                scope='One existing final construction batch; no new query owner or evaluation. New points do not identify the historical47 cause.')
            Path(output,'FINAL_METRIC_CAPTURE194.json').write_text(json.dumps(record,indent=2,allow_nan=False),encoding='utf-8')
        except Exception as failure:
            message = 'PassiveFinalMetricCapture194 publication failed: '+type(failure).__name__+': '+str(failure)
            if primary is not None:
                primary.add_note(message)
            try:
                Path(output,'FINAL_METRIC_CAPTURE194_FAILURE.json').write_text(
                    json.dumps(dict(schema='PassiveFinalMetricCapture194Failure/1',
                        complete=False, error=message, primary_error=repr(primary),
                        observer_restored=sys.getprofile() is previous)), encoding='utf-8')
            except Exception as secondary:
                if primary is not None:
                    primary.add_note('Diagnostic failure receipt also failed: '+repr(secondary))
                else:
                    # Preserve a visible diagnostic error without changing a
                    # completed numerical computation into a different result.
                    print(message+'; receipt failed: '+repr(secondary), file=sys.stderr)
