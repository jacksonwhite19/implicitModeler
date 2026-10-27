"""Diagnostic operands only; never a receipt, restart or admission capability."""
import hashlib,math
import numpy as np
from sdf_cfd_mesher_v4.common.authority import identity

def plain(value):
    if isinstance(value,np.ndarray):return plain(value.tolist())
    if isinstance(value,np.generic):return plain(value.item())
    if isinstance(value,float) and not math.isfinite(value):return {'float64_hex':value.hex()}
    if isinstance(value,dict):return {str(k):plain(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [plain(v) for v in value]
    return value

def emit(check,event,**payload):
    sink=getattr(check,'repair_diagnostics',None)
    if sink is None:return None
    body=plain(dict(schema='TransitionConstructionDiagnostic/1',proposal_only=True,
        scientific_authority=False,restart_supported=False,event=event,
        context=getattr(check,'transition_diagnostic_context',None),**payload))
    digest=identity(body);sink(event,dict(body,diagnostic_identity=digest))
    return digest

def array_record(a):
    a=np.ascontiguousarray(a)
    return dict(dtype=a.dtype.str,shape=list(a.shape),sha256=hashlib.sha256(a.tobytes()).hexdigest(),values=a)

def intervals(check,*,front,faces,owners,volume,area,normal,centre,dp,lower,upper,direction,dots,apex,projection,lo,hi,changed,row_index,controls):
    bad=np.flatnonzero(lo>hi);witnesses=[]
    for v in bad:
        incident=np.flatnonzero(apex==v);lf=int(incident[np.argmax((lower/projection)[incident])]);uf=int(incident[np.argmin((upper/projection)[incident])])
        witnesses.append(dict(vertex=int(v),incident_apex_faces=incident,lower_face=lf,upper_face=uf,
            lower_m=float(lo[v]),upper_m=float(hi[v]),margin_m=float(hi[v]-lo[v])))
    return emit(check,'actual_owner_shared_intervals',outcome='failure' if len(bad) else 'feasible_intervals',
        reason='SharedActualOwnerIntervalConflict' if len(bad) else None,row_index=row_index,controls=controls,
        units=dict(coordinates='m',area='m^2',volume='m^3',intervals='m',directions='dimensionless'),
        formula=dict(lower='max(4*.055/(1-.055)*dp, .03*volume/area)',upper='min(4*(1-.055)/.055*dp, 300*volume/area)',
            dp='dot(outward_cap_normal,cap_triangle_centroid-actual_owner_centre)',apex='max compact vertex index; staircase product role0',
            shared='lo=max(lower/projection), hi=min(upper/projection) over faces with same apex',
            directions='area-weighted cap normals; existing positive-cone LP where required',policy='Existing V4 row construction constants; no gate changes'),
        front=front,faces=faces,actual_owner_centres=owners,actual_owner_volumes=volume,cap_area=area,cap_normal=normal,cap_centre=centre,
        dp=dp,lower=lower,upper=upper,direction=direction,all_vertex_projections=dots,apex=apex,projection=projection,
        face_lower_depth=lower/projection,face_upper_depth=upper/projection,shared_lower=lo,shared_upper=hi,
        direction_cones_changed=changed,conflicting_vertices=bad,witnesses=witnesses)
