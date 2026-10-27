"""Explicit generic transition band. MMG proposes; existing V4 owners accept.

Shape and size are independent. V2 promises a verified vertex envelope, not
material row ancestry or a continuous whole-cell distance tube.
"""
from pathlib import Path
from contextlib import ExitStack
import os,subprocess,time
import numpy as np
from sdf_cfd_mesher_v4.common.authority import identity,array_hash,frozen,file_hash
from path_preserving_deduplication import durable_json
from immutable_state_access import guard
from owned_runtime import BIN
from background_materialization import build
from background_topology_repair import key,boundary
from prism_hotpath_v2 import reverse_cycle
from tet_layer_controls import validate,seed_controls
from shape_transition_metric import copied_seed_metric
from shape_transition_native import write_inputs,parse_output,restore_cap,restore_protected,verify_pins,NATIVE_DISTRIBUTION
from shape_transition_domain import witness,verify
from shape_transition_topology import verify_attachment

def _native_path(p):
    p=Path(p).resolve()
    if os.name!='nt':return str(p)
    return subprocess.check_output(['wsl.exe','-d',NATIVE_DISTRIBUTION,'-e','wslpath','-u',str(p)],text=True,encoding='utf-8').strip()

def envelope_binding(s):
    return identity(dict(cap_points=array_hash(s['original'][s['cap_ids']]),cap_faces=array_hash(s['cap_faces']),
        controls=identity(s['controls']),witnesses={k:array_hash(v) for k,v in s['vertex_envelope'].items()},
        compact_ids=array_hash(s['compact_ids']),outer=array_hash(s['outer'])))

def _anchor(s,v):
    j=v-s['old_point_count'];triangle=int(s['vertex_envelope']['triangle_ids'][j]);uv=s['vertex_envelope']['weights'][j]
    cap=s['original'][s['cap_ids'][s['cap_faces'][triangle]]];a,b,c=cap
    q=a+uv[0]*(b-a)+uv[1]*(c-a);normal=np.cross(b-a,c-a)
    return cap,uv,q,normal

def envelope_constraints(s,vertices,points):
    D=s['controls']['maximum_distance_m'];out=[]
    for v in vertices:
        cap,uv,q,normal=_anchor(s,v);delta=points[v]-q
        out.extend((1-float(delta@delta)/(D*D),float(normal@(points[v]-cap[0]))/(np.linalg.norm(normal)*D)))
        if s['bounds'] is not None:
            out.extend((points[v]-s['bounds'][:,0])/D);out.extend((s['bounds'][:,1]-points[v])/D)
    return out

def envelope_subset_passed(s,vertices,points):
    for v in vertices:
        cap,uv,_,_=_anchor(s,v)
        try:verify(np.vstack((cap,points[v])),cap,np.array([[0,1,2]],dtype='<i8'),s['controls']['maximum_distance_m'],
            dict(vertex_ids=np.array([3],dtype='<i8'),triangle_ids=np.array([0],dtype='<i8'),weights=uv.reshape(1,2)))
        except ValueError as e:
            if e.args==('VertexOutsideDeclaredEnvelope',):return False
            raise
    return True

def measure(s):
    ids={i for f in s['outer'] for i,_ in s['owners'][key(f)]};tets=np.asarray([c['vertices'] for c in s['additions'] if c['id'] in ids],dtype='<i8')
    pts=s['points'][tets];volume6=np.einsum('ni,ni->n',pts[:,1]-pts[:,0],np.cross(pts[:,2]-pts[:,0],pts[:,3]-pts[:,0]));lengths=sum(np.sum((pts[:,i]-pts[:,j])**2,axis=1) for i in range(4) for j in range(i+1,4))
    ratio=12*np.power(np.abs(volume6)/2,2/3)/lengths
    tri=s['points'][s['outer']];edges=np.stack((tri[:,1]-tri[:,0],tri[:,2]-tri[:,1],tri[:,0]-tri[:,2]),axis=1);squared=np.sum(edges*edges,axis=2)
    triangle_ratio=2*np.sqrt(3)*np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)/squared.sum(axis=1)
    return dict(outer_cells=len(tets),outer_faces=len(tri),outer_vertices=len(np.unique(s['outer'])),
        quantiles=[0,.01,.1,.5,.9,1],outer_cell_mean_ratio=np.quantile(ratio,[0,.01,.1,.5,.9,1]).tolist(),
        outer_triangle_mean_ratio=np.quantile(triangle_ratio,[0,.01,.1,.5,.9,1]).tolist(),
        diagnostic_only=True,ideal_reference='RegularTetAndEquilateralTriangle=1; not an acceptance threshold')

def initialize(points,oldcells,oldowners,ids,faces,terminal,controls,bounds,*,prism_volumes,prism_centres,check,output,resources):
    import tet_layer_adapter as adapter
    from standalone_tool import check_resources
    started=time.perf_counter();validate(controls)
    points=np.asarray(points,dtype='<f8');ids=np.asarray(ids,dtype='<i8');faces=np.asarray(faces,dtype='<i8')
    if output is None or type(resources) is not dict:raise ValueError('ExplicitShapeConstructionOutputAndResourcesRequired')
    folder=Path(output);folder.mkdir(parents=True,exist_ok=False)
    def tick():
        if check is not None:check()
        if time.perf_counter()-started>=controls['seconds']:raise ValueError('ShapeTransitionConstructionTimeLimit')
    tick();check_resources(folder.parent.parent,resources)
    seed=adapter.initialize(points,oldcells,oldowners,ids,faces,terminal,seed_controls(controls),bounds,
        prism_volumes=prism_volumes,prism_centres=prism_centres,check=tick)
    compact=seed['compact_ids'];lookup=np.empty(len(seed['points']),dtype='<i8');lookup[compact]=np.arange(len(compact))
    seed_points=seed['points'][compact];seed_tets=np.asarray([lookup[c['vertices']] for c in seed['additions']],dtype='<i8');cap=seed_points[:len(ids)]
    from background_mixed_geometry import batch_geometry
    n=len(ids);F=len(faces);R=controls['protected_seed_rows'];prefix_count=3*F*R
    seed_prefix=seed_tets[:prefix_count]
    native_cap=seed_points[R*n:(R+1)*n]
    if R:
        inner_faces=compact[R*n:(R+1)*n][faces]
        protected_ids={c['id'] for c in seed['additions'][:prefix_count]}
        terminal=[]
        for f in inner_faces:
            owned=[i for i,_ in seed['owners'][key(f)] if i in protected_ids]
            if len(owned)!=1:raise ValueError('MissingProtectedRowTerminalOwner')
            terminal.append(owned[0])
        byid={c['id']:c for c in seed['additions'][:prefix_count]}
        g=batch_geometry(seed['points'],[byid[i] for i in dict.fromkeys(terminal)])
        prism_volumes=np.asarray([g[i]['volume'] for i in terminal]);prism_centres=np.asarray([g[i]['centre'] for i in terminal])
    elif prism_volumes is None or prism_centres is None:
        byid={c['id']:c for c in oldcells};g=batch_geometry(points,[byid[i] for i in dict.fromkeys(terminal)])
        prism_volumes=np.asarray([g[i]['volume'] for i in terminal]);prism_centres=np.asarray([g[i]['centre'] for i in terminal])
    M,metric=copied_seed_metric(native_cap,faces,prism_centres,prism_volumes,rows=controls['seed_rows']-R,shape_progress=controls['shape_progress'],shape_power=controls['shape_power'],size_growth=controls['size_growth'])
    if R:M=np.concatenate([M[:n]]*R+[M])
    metric=dict(metric,protected_metric_extension='Constant from actual protected-row terminal owner',protected_volume_rows=R)
    tick()
    pins=write_inputs(folder,seed_points,seed_tets,faces,lookup[seed['outer']],M,
        protected_tetrahedra=np.arange(prefix_count,dtype='<i8'),protected_vertices=np.arange((R+1)*n,dtype='<i8') if R else ())
    binary=BIN/'shape_transition_mmg';pins[str(binary.resolve())]=file_hash(binary)
    outfile=folder/'OUTPUT.txt'
    cmd=['timeout','--kill-after=5s',str(min(controls['native_seconds'],controls['seconds']-(time.perf_counter()-started)))+'s',
        *map(_native_path,(binary,folder/'INPUT.mesh',folder/'METRIC.sol',outfile)),
        *map(str,(controls['metric_gradation'],controls['carrier_tolerance_m'],metric['hmin'],metric['hmax'],controls['memory_megabytes'],0))]
    if os.name=='nt':cmd=['wsl.exe','-d',NATIVE_DISTRIBUTION,'-e',*cmd]
    with ExitStack() as held:
        for name,digest in pins.items():held.enter_context(guard(name));assert file_hash(name)==digest
        with (folder/'NATIVE.stdout').open('x',encoding='utf-8') as log,(folder/'NATIVE.stderr').open('x',encoding='utf-8') as err:
            before=time.perf_counter();result=subprocess.run(cmd,stdout=log,stderr=err);native_seconds=time.perf_counter()-before
        verify_pins(pins);tick();check_resources(folder.parent.parent,resources)
        if result.returncode:raise ValueError('ShapeTransitionNativeStopped',result.returncode)
        held.enter_context(guard(outfile));digest=file_hash(outfile)
        native=parse_output(outfile);native,mapping,recovery=restore_cap(cap,faces,native)
        if R:
            native,protected,recovered=restore_protected(seed_points,np.arange((R+1)*n,dtype='<i8'),seed_tets[:prefix_count],native)
            if any(protected[i]!=mapping[i] for i in mapping):raise ValueError('InconsistentProtectedCapIdentity')
            mapping=protected;recovery.update(protected_volume=recovered)
        if file_hash(outfile)!=digest:raise ValueError('ChangedShapeNativeOutput')
    return materialize(seed,points,oldcells,oldowners,ids,faces,controls,bounds,native,mapping,recovery,metric,
        native_seconds=native_seconds,pins=pins,digest=digest,started=started,check=tick,folder=folder)


def materialize(seed,points,oldcells,oldowners,ids,faces,controls,bounds,native,mapping,recovery,metric,*,native_seconds,pins,digest,started,check,folder):
    """Private native proposal assembly. Caller protects input/output custody.

    This does not issue acceptance or a replay credential. The ordinary place
    and admit owners still evaluate every fresh transition tetrahedron.
    """
    tick=check;n=len(ids);R=controls['protected_seed_rows'];prefix_count=3*len(faces)*R
    compact=seed['compact_ids'];seed_points=seed['points'][compact];cap=seed_points[:n]
    tick();inner=set(mapping.values());other=np.asarray(sorted(set(range(len(native['points'])))-inner),dtype='<i8')
    inverse=np.empty(len(native['points']),dtype='<i8')
    for source,target in mapping.items():inverse[target]=source
    kept=(R+1)*n
    if set(mapping)!=set(range(kept)):raise ValueError('IncompleteProtectedVolumeMapping')
    inverse[other]=kept+np.arange(len(other));proposed=np.concatenate((seed_points[:kept],native['points'][other]))
    proposed,envelope,domain=witness(proposed,cap,faces,controls['maximum_distance_m'],tick)
    if proposed[:kept].tobytes()!=seed_points[:kept].tobytes():raise ValueError('ChangedProtectedSeedRowDuringEnvelopeProjection')
    global_ids=np.r_[ids,np.arange(len(points),len(points)+len(proposed)-n)]
    allpoints=np.concatenate((points,proposed[n:]));tetrows=global_ids[inverse[native['tets']]]
    additions=build(tetrows,max(c['id'] for c in oldcells)+1);outer=global_ids[inverse[native['faces'][native['refs']==1000000]]]
    oldfacekeys=set(oldowners);owners={k:list(v) for k,v in oldowners.items()}
    for c in additions:
        for f in c['faces']:owners.setdefault(key(f),[]).append((c['id'],f))
    attachment=verify_attachment(additions,owners,ids[faces],outer,{c['id'] for c in oldcells},tick)
    for f in ids[faces]:
        k=key(f)
        if [r for r in owners[k] if r[0] in {i for i,_ in oldowners[k]}]!=list(oldowners[k]):raise ValueError('ChangedOriginalCapOwnerBinding')
    if allpoints[:len(points)].tobytes()!=points.tobytes():raise ValueError('ChangedShapePrismPrefix')
    construction=dict(method='MMGJointBand/LogShapeRelaxation/2',native_seconds=native_seconds,metric=metric,recovery=recovery,
        native_input_pins=pins,native_output_sha256=digest,vertex_envelope=domain,seed=seed['construction'],
        protected_seed_rows=R,protected_seed_tetrahedra=prefix_count,attachment=attachment,
        input_tetrahedra=len(seed['additions']),output_tetrahedra=len(tetrows),outer_faces=len(outer),seconds=time.perf_counter()-started,proposal_only=True)
    durable_json(folder/'CONSTRUCTION.json',construction);tick()
    return dict(points=allpoints,cells=oldcells+additions,additions=additions,owners=owners,outer=outer,
        compact_ids=global_ids,cap_ids=np.asarray(ids,dtype='<i8'),cap_faces=np.asarray(faces,dtype='<i8'),
        old_point_count=len(points),original=points.copy(),controls=dict(controls),old_face_keys=frozenset(oldfacekeys),bounds=bounds,
        vertex_envelope={k:frozen(v,v.dtype.str) for k,v in envelope.items()},construction=construction)
