"""Bind the qualified local core relocation to arbitrary native shell owners."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import json,time,os
import numpy as np
import repair_bulk_core as r
from audit_mixed_bulk_probe import centres


def affected_scope(tets,bad,movable):
    """Keep immovable failures in the guard; they cannot disappear by omission."""
    return np.union1d(bad,np.flatnonzero(np.any(np.isin(tets,movable),axis=1)))


def run(folder,source_name='core',destination_name='repaired_core'):
    started=time.perf_counter();src=folder/source_name;dest=folder/destination_name;dest.mkdir(exist_ok=True)
    f=r.load('family_feasibility',r.V3/'fresh_supported_refinement_family_qualification_2026-09-09/fallback_scope_20260910/source/solve_coordinate_feasibility.py');f.SKEW_MAX=3.63636
    fg=r.load('family_foundation',r.V3/'foundation_aware_prism_shell_qualification/foundation_geometry.py')
    core={k:np.load(src/(k+'.npy')) for k in ['coordinates','tets','node_tags','interface_tags','farfield_tags','interface_owner']}
    x=core['coordinates'];t=core['tets'];order=np.argsort(core['node_tags']);tags=core['node_tags'][order]
    ci=order[np.searchsorted(tags,core['interface_tags'])];fixed=order[np.searchsorted(tags,np.r_[core['interface_tags'],core['farfield_tags']])]
    faces,owners=r.topology(t);v,d,a,c=r.cell_metrics(x,t,f);w,vr,sk,angle=r.face_metrics(x,faces,owners,v,c,fg)
    interface=folder/'interface';cap=np.load(interface/'connectivity.npy').reshape(-1,3)
    sc=np.empty((len(cap),3));sv=np.empty(len(cap))
    for path in interface.glob('owner_geometry_*.npy'):
        arity=path.stem.rsplit('_',1)[1];ids=np.load(interface/f'owner_faces_{arity}.npy');sc[ids],sv[ids]=centres(np.load(path),fg)
    area,fc=fg.face_area_centre_tri(x[ci[cap]]);normal=area/np.linalg.norm(area,axis=1)[:,None]
    own=core['interface_owner'];da=abs(np.einsum('ij,ij->i',area,fc-sc));db=abs(np.einsum('ij,ij->i',area,c[own]-fc));iw=np.minimum(da,db)/(da+db)
    bad=np.union1d(np.flatnonzero((d<.0011)|(v<=0)|(a>1000)),owners[(w<.055)|(vr<.01)|(sk>3.63636)].ravel())
    ir=np.minimum(sv,v[own])/np.maximum(sv,v[own]);isk=fg.internal_face_skewness(x[ci[cap]],fc,area,sc,c[own])
    bad_interface=np.flatnonzero((iw<.055)|(ir<.01)|(isk>3.63636))
    bad=np.union1d(bad,own[bad_interface]);movable=np.setdiff1d(np.unique(t[bad]),fixed)
    (dest/'REPAIR_SCOPE.json').write_text(json.dumps(dict(bad_cells=bad.tolist(),bad_interface_faces=bad_interface.tolist(),movable_vertices=movable.tolist(),fixed_vertices=fixed.tolist()),indent=2))
    if not len(movable):raise ValueError('NoInteriorRepairDegreesOfFreedom')
    affected=affected_scope(t,bad,movable);guard=np.any(np.isin(owners,affected),axis=1)
    scales=[];lap=[]
    for vertex in movable:
        neighbors=np.setdiff1d(np.unique(t[np.any(t==vertex,axis=1)]),[vertex]);scales.append(np.linalg.norm(x[neighbors]-x[vertex],axis=1).min());lap.append(x[neighbors].mean(0))
    inv=dict(movable=movable,affected=affected,guard_faces=faces[guard],guard_owners=owners[guard],guard_interface=np.flatnonzero(np.isin(own,affected)),bad_interface=bad_interface,local_scale=np.asarray(scales))
    auth=dest/'local_authority';auth.mkdir(exist_ok=True);f.AUTH=auth
    for key,value in dict(outer_faces=cap,prism_centres=sc,cap_normals=normal,d_prism=np.einsum('ij,ij->i',fc-sc,normal)).items():np.save(auth/(key+'.npy'),value)
    trials=[]
    for method in ['laplacian','constrained_movement','feasibility']:
        tick=time.perf_counter()
        if method=='laplacian':candidate=x.copy();candidate[movable]=lap;record={}
        else:candidate,record=f.solve_once(core,inv,1.,'baseline','movement' if method=='constrained_movement' else 'feasibility')
        checked=r.local_guard(core,inv,candidate,f,fg,[(ci[cap],own,sc,sv)])
        exact=np.array_equal(candidate[fixed].view('u8'),x[fixed].view('u8'))
        record.update(method=method,seconds=time.perf_counter()-tick,quality=checked,fixed_exact=exact)
        trials.append(record)
        (dest/'TRIALS.json').write_text(json.dumps(trials,indent=2,default=lambda z:z.tolist() if hasattr(z,'tolist') else str(z)))
        if checked['pass_gate'] and exact:
            for path in src.glob('*.npy'):
                if path.name!='coordinates.npy' and not (dest/path.name).exists():os.link(path,dest/path.name)
            np.save(dest/'coordinates.npy',candidate)
            from audit_native_core_quality import audit
            audit(dest)
            complete=json.loads((dest/'HARD_QUALITY.json').read_text())
            cv,cd,ca,cc=r.cell_metrics(candidate,t,f)
            db=abs(np.einsum('ij,ij->i',area,cc[own]-fc));iw=np.minimum(da,db)/(da+db)
            ir=np.minimum(sv,cv[own])/np.maximum(sv,cv[own]);isk=fg.internal_face_skewness(candidate[ci[cap]],fc,area,sc,cc[own])
            metrics=dict(weight_min=float(iw.min()),ratio_min=float(ir.min()),skew_max=float(isk.max()),weight_failures=int((iw<.055).sum()),ratio_failures=int((ir<.01).sum()),skew_failures=int((isk>3.63636).sum()))
            generation=json.loads((src/'GENERATION.json').read_text());generation.update(core=complete,interface=metrics,passed=complete['passed'] and not any(metrics[k] for k in ('weight_failures','ratio_failures','skew_failures')),repair_source=str(src))
            (dest/'GENERATION.json').write_text(json.dumps(generation,indent=2))
            if not generation['passed']:raise ValueError('FullCoreRepairValidationFailure')
            (dest/'REPAIR.json').write_text(json.dumps(dict(passed=True,seconds=time.perf_counter()-started,moved=int(np.any(candidate!=x,axis=1).sum()),affected=len(affected),fixed=len(fixed),initial_bad_cells=bad.tolist()),indent=2))
            return dest
    raise ValueError('LocalRelocationExhausted')

if __name__=='__main__':run(Path(sys.argv[1]))
