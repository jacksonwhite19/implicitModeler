"""Private generic finite collar. No production acceptance or live credentials."""
from pathlib import Path
from fractions import Fraction as F
from collections import defaultdict
import sys, math, time, subprocess, json, hashlib
HERE=Path(__file__).resolve().parent;R=HERE;S=HERE;V4=HERE.parents[1]
sys.dont_write_bytecode=True
sys.path[:0]=[str(HERE),str(S),str(R),str(V4/'tools'),str(V4/'src')]
import numpy as np
import fixture_study as d
from sdf_cfd_mesher_v4.common.authority import array_hash
import local_termination_topology as topology
from shape_transition_domain import verify as verify_envelope,witness
from native_mixed_face_certificate import certify
from rolling_fast_state import verify_fresh_coverage
from background_independent import AABBIndex as IndependentIndex
from cavity_volume_audit import degree
TARGET=d.module('private_collar_existing_targets',d.TARGET_PATH)
PRODUCER='PrivateFiniteGmshCollar/1'


def inherited_digest(cells,owners,check):
    h=hashlib.sha256()
    for c in d.checked_items(cells,check):
        h.update(json.dumps(d.plain(c),sort_keys=True,separators=(',',':'),allow_nan=False).encode()+b'\n')
    for k,rows in d.checked_items(sorted(owners.items()),check):
        h.update(repr((tuple(map(int,k)),tuple((int(i),tuple(map(int,f))) for i,f in rows))).encode())
    return h.hexdigest()


def verify_inherited_join(state,cells,owners,expected,check):
    if inherited_digest(cells,owners,check)!=expected:raise ValueError('ChangedInheritedCellInputs')
    ids={c['id'] for c in cells};oldrows={}
    for k,rows in d.checked_items(state['owners'].items(),check):
        kept=[(i,f) for i,f in rows if i in ids]
        if kept:oldrows[k]=kept
    if inherited_digest(state['cells'][:len(cells)],oldrows,check)!=expected:
        raise ValueError('ChangedInheritedCellJoin')
    return dict(passed=True,ordered_cells_faces_owners_sha256=expected)


def surface(points,faces,check):
    from shape_transition_topology import _surface
    if (points.dtype!=np.dtype('<f8') or points.ndim!=2 or points.shape[1]!=3
            or not np.isfinite(points).all() or faces.dtype!=np.dtype('<i8')
            or faces.ndim!=2 or faces.shape[1]!=3 or len(points)==0
            or np.any(faces<0) or np.any(faces>=len(points))):raise ValueError('InvalidCollarSurface')
    _surface(faces,'Collar',check);edges=defaultdict(list);adj=defaultdict(set)
    for i,face in enumerate(faces):
        if i%1024==0:check()
        for a,b in zip(face,np.roll(face,-1)):
            edges[tuple(sorted((int(a),int(b))))].append((int(a),int(b)))
            adj[int(a)].add(int(b));adj[int(b)].add(int(a))
    if any(len(v)!=2 or v[0]!=v[1][::-1] for v in edges.values()):raise ValueError('OpenOrReversedCollarSurface')
    seen=set();todo=[0]
    while todo:
        i=todo.pop()
        if i not in seen:seen.add(i);todo.extend(adj[i]-seen)
    if seen!=set(range(len(points))):raise ValueError('DisconnectedCollarSurface')
    links=topology.vertex_links(faces)
    if not links['passed']:raise ValueError('InvalidCollarVertexLinks')
    return dict(vertices=len(points),faces=len(faces),edges=len(edges),links=links)


def propose(cap,faces,distance,check):
    surface(cap,faces,check)
    if not math.isfinite(distance) or distance<=0:raise ValueError('InvalidCollarDistance')
    centre=np.asarray(cap.mean(axis=0),dtype='<f8')
    if not np.isfinite(centre).all():raise ValueError('NonfiniteCollarMean')
    p=[tuple(F(float(z)) for z in row) for row in cap];c=tuple(F(float(z)) for z in centre)
    for i,f in enumerate(faces):
        if i%256==0:check()
        a,b,e=[p[j] for j in f]
        normal=topology.cross(topology.sub(b,a),topology.sub(e,a))
        if not any(normal) or topology.dot(normal,topology.sub(c,a))>=0:
            raise ValueError('StrictMeanKernelUnsupported',i)
    delta=[topology.sub(row,c) for row in p]
    radius2=max(topology.dot(v,v) for v in delta);limit2=F(distance)**2
    scale=float(distance/math.sqrt(float(radius2)))
    if not math.isfinite(scale) or scale<=0:raise ValueError('UnrepresentableCollarScale')
    if F(scale)**2*radius2>limit2:scale=math.nextafter(scale,0.)
    if scale<=0 or F(scale)**2*radius2>limit2:raise ValueError('UnresolvedCollarScale')
    out=np.empty_like(cap);a=F(scale)
    for i,row in enumerate(p):
        if i%256==0:check()
        for j in range(3):
            ideal=c[j]+(1+a)*(row[j]-c[j]);rounded=float(ideal)
            if ((ideal>row[j] and F(rounded)>ideal) or
                    (ideal<row[j] and F(rounded)<ideal)):
                rounded=math.nextafter(rounded,float(row[j]))
            if not min(row[j],ideal)<=F(rounded)<=max(row[j],ideal):
                raise ValueError('DirectedCollarRoundingFailed')
            out[i,j]=rounded
    if not np.isfinite(out).all():raise ValueError('NonfiniteCollarProposal')
    return out,dict(method='rounded_uniform_mean_kernel_dilation',centre=centre.tolist(),
        centre_definition='binary64 numpy arithmetic mean, thereafter exact dyadic',scale_hex=scale.hex(),
        maximum_radius_squared=str(radius2),maximum_distance_m=distance,
        exact_scale_bound=True,strict_kernel_faces=len(faces),cap_sha256=array_hash(cap),
        faces_sha256=array_hash(faces),outer_sha256=array_hash(out),quality_feedback=False)


def collision(points,faces,active,check):
    check();proof=certify(points,list(map(tuple,faces)),active,workers=4);check()
    result={k:proof[k] for k in ('passed','required_pairs','failures','proof_hash')}
    if not proof['passed']:
        result.update(independent_coverage_passed=False,coverage_status='not reached; failed collision certificate',methods=proof['methods'])
        return result
    methods=verify_fresh_coverage(points,list(map(tuple,faces)),active,proof,index_type=IndependentIndex);check()
    result.update(independent_coverage_passed=True,methods=dict(methods))
    return result


def boundary_envelope(cap,faces,outer,distance,check):
    # One-hot anchors on an actual incident source triangle, not nearest guesses.
    incident={}
    for i,f in enumerate(faces):
        for j,v in enumerate(f):incident.setdefault(int(v),(i,j))
    ids=np.arange(len(cap),2*len(cap),dtype='<i8');tids=[];weights=[]
    for v in range(len(cap)):
        i,j=incident[v];tids.append(i);weights.append(((0.,0.),(1.,0.),(0.,1.))[j])
    w=dict(vertex_ids=ids,triangle_ids=np.asarray(tids,dtype='<i8'),weights=np.asarray(weights,dtype='<f8'))
    return verify_envelope(np.vstack((cap,outer)),cap,faces,distance,w,check)


def boundary_shape(outer,faces,require_targets=True):
    if type(require_targets) is not bool:raise ValueError('ExplicitBoundaryShapeRequirementRequired')
    report=TARGET.population_report(TARGET.triangle_ratio(outer,faces),d.TARGETS['outer_triangles'])
    if require_targets and not report['passed']:raise ValueError('PrescribedOuterTriangleTargetsFailed')
    return report


def admit_boundary(cap,faces,outer,distance,check,out,*,require_shape_targets=True):
    report=dict(inner=surface(cap,faces,check),outer=surface(outer,faces,check))
    for i,f in enumerate(faces):
        if i%256==0:check()
        a,b,c=[tuple(F(float(z)) for z in outer[j]) for j in f]
        if not any(topology.cross(topology.sub(b,a),topology.sub(c,a))):raise ValueError('DegenerateOuterFace',i)
    report['envelope']=boundary_envelope(cap,faces,outer,distance,check)
    points=np.vstack((cap,outer));all_faces=np.vstack((faces,faces+len(cap)))
    report['collision']=collision(points,all_faces,range(len(all_faces)),check)
    d.publish(out/'BOUNDARY_COLLISION.json',report['collision'])
    if not report['collision']['passed']:raise ValueError('CollarBoundaryIntersection')
    check();enclosure=degree(outer,list(map(tuple,faces)),[tuple(F(float(z)) for z in row) for row in cap]);check()
    d.publish(out/'BOUNDARY_DEGREE.json',enclosure)
    if not enclosure['passed']:raise ValueError('CapNotStrictlyEnclosed')
    report['enclosure']=dict(passed=True,witnesses=len(cap))
    report['triangles']=boundary_shape(outer,faces,require_shape_targets)
    return report


def materialize(mesh,cap,oldcells,oldowners,p,t,inner,far,check):
    from background_materialization import build
    from shape_transition_topology import verify_attachment
    old=np.asarray(mesh['points']);ids=cap['cap_ids'];faces=cap['cap_triangles']
    if p[inner].tobytes()!=old[ids].tobytes():raise ValueError('ChangedCapMaterialization')
    if len(set(map(int,inner)))!=len(ids) or set(inner)&set(far):raise ValueError('InvalidCollarBoundaryMap')
    keep=np.ones(len(p),dtype=bool);keep[inner]=False;fresh=np.flatnonzero(keep)
    mapping=np.empty(len(p),dtype='<i8');mapping[inner]=ids;mapping[fresh]=np.arange(len(old),len(old)+len(fresh))
    points=np.vstack((old,p[fresh]));tets=mapping[t];outer=mapping[far][faces]
    additions=build(tets,max(c['id'] for c in oldcells)+1)
    owners={k:list(v) for k,v in oldowners.items()}
    for c in d.checked_items(additions,check):
        for f in c['faces']:owners.setdefault(tuple(sorted(f)),[]).append((c['id'],f))
    attachment=verify_attachment(additions,owners,ids[faces],outer,[c['id'] for c in oldcells],check)
    for face,owner in zip(ids[faces],cap['terminal_owners']):
        if oldowners[tuple(sorted(face))][0][0]!=int(owner):raise ValueError('ChangedActualTerminalOwner')
    compact=np.r_[ids,np.arange(len(old),len(points),dtype='<i8')]
    if len(set(map(int,compact)))!=len(compact) or points[:len(old)].tobytes()!=old.tobytes():raise ValueError('ChangedPrismPrefix')
    state=dict(points=points,original=old,cells=list(oldcells)+additions,additions=additions,
        owners=owners,outer=outer,tets=tets,cap_ids=ids,cap_faces=faces,compact_ids=compact,
        old_point_count=len(old),old_face_keys=set(oldowners),producer=PRODUCER,
        controls=dict(quality_targets=d.TARGETS),backend_to_global=mapping)
    return state,attachment


def rebind_reference(state,binding,check):
    # Existing comparison reads only its face coordinates. Give it an explicit
    # bijective view in frozen ancestor indices; no old interior is mesh output.
    oldfaces=np.asarray(binding['faces']);newfaces=np.asarray(state['outer'])
    if oldfaces.shape!=newfaces.shape:raise ValueError('ChangedReferenceFacePopulation')
    forward={};reverse={}
    for i,(a,b) in enumerate(zip(oldfaces,newfaces)):
        if i%1024==0:check()
        for x,y in zip(a,b):
            x,y=int(x),int(y)
            if (x in forward and forward[x]!=y) or (y in reverse and reverse[y]!=x):raise ValueError('InconsistentAncestryMap')
            forward[x]=y;reverse[y]=x
    expected=state['cap_faces']
    newids=np.unique(newfaces)
    # Explicit ancestral correspondence supplied by the backend cap-copy map.
    if not np.array_equal(newfaces,state['outer_vertex_ids'][expected]):raise ValueError('PermutedOuterAncestry')
    if len(forward)!=len(state['cap_ids']):raise ValueError('IncompleteAncestryMap')
    view=binding['points'].copy()
    for x,y in forward.items():view[x]=state['points'][y]
    return dict(points=view,outer=oldfaces),dict(old_ids=list(forward),new_ids=list(forward.values()),
        final_points_sha256=array_hash(state['points']),final_tets_sha256=array_hash(state['tets']),comparison_view_only=True)


def exact_geometry(state,cap,out,check,timings,hashes):
    from tet_layer_adapter import filling
    with d.stage('pure_vertex_envelope',check,timings):
        compact=state['points'][state['compact_ids']];original=state['original'][state['cap_ids']]
        changed,w,env=witness(compact,original,cap['cap_triangles'],.001,check)
        if changed.tobytes()!=compact.tobytes():raise ValueError('EnvelopeWouldChangeBackendCoordinates')
        env=verify_envelope(compact,original,cap['cap_triangles'],.001,w,check)
        d.publish(out/'ENVELOPE.json',dict(**env,hashes=hashes))
    with d.stage('complete_collision',check,timings):
        faces=[uses[0][1] for k,uses in sorted(state['owners'].items())]
        # No live old-old proof exists for this retained diagnostic; check all.
        collision_report=collision(state['points'],faces,range(len(faces)),check)
        d.publish(out/'COLLISION.json',dict(**collision_report,hashes=hashes))
        if not collision_report['passed']:raise ValueError('CompleteCollarCollisionFailure')
    with d.stage('exact_filling',check,timings):
        volume=filling(state);d.publish(out/'FILLING.json',dict(**volume,hashes=hashes))
        if not (volume['positive_cells'] and volume['exact_volume_balance'] and volume['interior_degree']['passed']):raise ValueError('ExactCollarFillingFailure')
    if hashes!=dict(points=array_hash(state['points']),tets=array_hash(state['tets']),outer=array_hash(state['outer'])):
        raise ValueError('ChangedFinalCollarDuringAcceptance')
    return dict(envelope=True,collision=True,coverage=True,filling=True,hashes=hashes)


def evaluate(mesh,cap,oldcells,oldowners,backend,out,check,timings,binding=None,references=None):
    from standalone_background import compare_boundary
    from tet_layer_adapter import quality,filling
    with np.load(backend/'NETGEN.npz',allow_pickle=False) as z:p,t=z['points'],z['tets']
    with np.load(backend/'INPUT.npz',allow_pickle=False) as z:outer=z['outer']
    with d.stage('final_mapping',check,timings):
        _,inner,far=compare_boundary(p,t,mesh['points'][cap['cap_ids']],cap['cap_triangles'],outer,cap['cap_triangles'])
        state,attachment=materialize(mesh,cap,oldcells,oldowners,p,t,inner,far,check)
        state['outer_vertex_ids']=state['backend_to_global'][far]
        hashes=dict(points=array_hash(state['points']),tets=array_hash(state['tets']),outer=array_hash(state['outer']))
        np.savez(out/'DIAGNOSTIC_PROPOSAL.npz',points=state['points'],tets=state['tets'],outer=state['outer'],
            compact_ids=state['compact_ids'],backend_to_global=state['backend_to_global'])
        d.publish(out/'ATTACHMENT.json',dict(**attachment,hashes=hashes,producer=PRODUCER))
    with d.stage('quality',check,timings):
        selected=[c['id'] for c in state['additions']]+list(dict.fromkeys(map(int,cap['terminal_owners'])))
        q=quality(state['points'],state['cells'],state['owners'],set(selected),state['outer'])
        boundary_vertices=set(state['backend_to_global'][np.r_[inner,far]])
        byid={c['id']:c for c in state['cells']}
        for failure in q['failures']:
            ids=failure.get('vertices',byid[failure['id']]['vertices'] if 'id' in failure else [])
            failure['location_m']=state['points'][list(ids)].mean(axis=0).tolist() if len(ids) else None
            margins={}
            for key,limit,sign in [('determinant',.0011,1),('aspect',1000,-1),('weight',.055,1),('skewness',3.63636,-1),('symmetric_volume_ratio',.01,1)]:
                if key in failure:margins[key]=sign*(failure[key]-limit)
            failure['gate_margins']=margins
        d.publish(out/'QUALITY.json',dict(**q,hashes=hashes,selected_cells=selected,
            all_fixed_tets=[c['id'] for c in state['additions'] if all(v in boundary_vertices for v in c['vertices'])]))
    with d.stage('shape_and_reference',check,timings):
        shape=TARGET.evaluate(state);d.publish(out/'SHAPE.json',shape)
        faceowners=[state['owners'][tuple(sorted(f))][0][0] for f in state['outer']]
        faceowner_set=set(faceowners)
        uniqueowners=[c['id'] for c in state['additions'] if c['id'] in faceowner_set]
        ownerregions=defaultdict(set);regions=cap.get('diagnostic_regions')
        if regions is not None:
            for owner,region in zip(faceowners,regions):ownerregions[owner].add(int(region))
        tets,front=TARGET.populations(state)
        d.publish(out/'FACE_OWNER_MAP.json',dict(cap_face_indices=list(range(len(faceowners))),outer_faces=state['outer'].tolist(),
            outer_owner_ids=faceowners,unique_owner_ids=uniqueowners,unique_owner_mean_ratios=TARGET.tetrahedron_ratio(state['points'],tets).tolist(),
            face_regions=None if regions is None else regions.tolist(),owner_regions={k:sorted(v) for k,v in ownerregions.items()},
            region_scope=cap.get('region_source','No descriptive regions supplied'),hashes=hashes))
        probe=d.module('private_collar_front_diagnostics',HERE/'front_probe.py')
        edges,pairs=probe.incidence(cap['cap_triangles']);local=state['points'][state['outer_vertex_ids']]
        lengths=np.linalg.norm(local[edges[:,0]]-local[edges[:,1]],axis=1)
        triangles=local[cap['cap_triangles']];areas=np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),axis=1)/2
        ratios=np.maximum(areas[pairs[:,0]]/areas[pairs[:,1]],areas[pairs[:,1]]/areas[pairs[:,0]])
        d.publish(out/'FRONT_DIAGNOSTICS.json',dict(outer_edges_m=d.stats(lengths),outer_triangle_areas_m2=d.stats(areas),
            neighbouring_face_area_ratios=d.stats(ratios),normal_turn_radians=d.stats(probe.angles(probe.normals(local,cap['cap_triangles']),pairs)),
            diagnostic_only=True,hashes=hashes))
        comparison=None
        if binding is not None:
            view,crosswalk=rebind_reference(state,binding,check);comparison=references.compare(view,binding)
            d.publish(out/'REFERENCE_CROSSWALK.json',crosswalk);d.publish(out/'COMPARISON.json',comparison)
    # Numerical gate failure is retained with full causal records; no repairs.
    if not q['passed'] or not shape['passed'] or (comparison is not None and not comparison['passed']):
        return state,dict(passed=False,classification='scientific_negative',quality=q,shape=shape,comparison=comparison,
            exact_geometry='not reached after quality/shape/reference rejection',hashes=hashes)
    exact_geometry(state,cap,out,check,timings,hashes)
    return state,dict(passed=True,classification='complete_retained_diagnostic',quality=q,shape=shape,comparison=comparison,
        hashes=hashes,live_source_fluid='unavailable; no live scientific admission',future_background='not tested')
