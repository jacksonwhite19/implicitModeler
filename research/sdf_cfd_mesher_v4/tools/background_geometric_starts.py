"""Geometry-only starts; existing raw QC and exact preservation admit edits."""
import numpy as np

def placement_seed(points,vertices,stars,owners,byid,fixed,scales,check=None,*,maxiter=None):
    from scipy.optimize import linprog
    import local_termination_topology as t
    fg=t.q.legacy_modules()['component_quality'].FG
    base=points[vertices].copy();goal=base.copy();records=[];useful=False
    def tick():
        if check is not None:check()
    def plane(face):
        S,C=fg.face_area_centre_tri(points[np.asarray(face,dtype='<i8')][None])
        length=float(np.linalg.norm(S[0]))
        if not np.isfinite(length) or not length>0:raise ValueError('InvalidGeometryStartPlane')
        return S[0]/length,C[0],length
    for j,v in enumerate(vertices):
        tick();rows=[];rhs=[];interfaces=[]
        faces={tuple(sorted(f)) for c in stars[v] for f in c['faces'] if v not in f}
        for k in sorted(faces):
            uses=owners.get(k,[])
            if len(uses)!=2:continue
            prism=[byid[i] for i,f in uses if byid[i]['kind']=='prism']
            core=[byid[i] for i,f in uses if byid[i]['kind']=='tetrahedron']
            if len(prism)!=1 or len(core)!=1 or v not in core[0]['vertices']:continue
            if set(core[0]['vertices'])-set(k)!={v}:continue
            if not set(k)<=fixed:raise ValueError('MovablePrismBoundaryUnsupported')
            face=next(f for i,f in uses if i==prism[0]['id']);normal,C,area=plane(face)
            geometry=t.geometry(points,prism[0]);dp=abs(float(normal@(C-geometry['centre'])))
            h0=float(normal@(points[v]-C));volume=geometry['volume']
            if not np.isfinite([dp,h0,volume]).all() or min(dp,h0,volume)<=0:raise ValueError('InvalidSignedInterfaceAltitude')
            lower=max(4*.055/(1-.055)*dp,.03*volume/area)
            upper=min(4*(1-.055)/.055*dp,300*volume/area)
            interfaces.append(dict(face=list(k),prism=prism[0]['id'],tet=core[0]['id'],dp=dp,altitude=h0,lower=lower,upper=upper))
            scale=4*dp;direction=normal*scales[j]/scale
            rows.extend([np.r_[-direction,1.],np.r_[direction,1.]])
            rhs.extend([(h0-lower)/scale,(upper-h0)/scale])
        if not interfaces:
            records.append(dict(vertex=v,reason='NoOwnedPrismInterface'));continue
        if any(r['lower']>r['upper'] for r in interfaces):
            records.append(dict(vertex=v,reason='EmptyFaceAltitudeVolumeInterval',interfaces=interfaces));continue
        # Every incident tetrahedron supplies the existing fixed-star kernel
        # plane. This is an initializer domain, not a substitute for exact cones.
        for cell in stars[v]:
            tick();face=next(f for f in cell['faces'] if v not in f);normal,C,_=plane(face)
            signed=float(normal@(points[v]-C));h0=abs(signed)
            if not np.isfinite(h0) or not h0>0:raise ValueError('InvalidGeometryStartKernel')
            normal*=1 if signed>0 else -1
            rows.append(np.r_[-normal*scales[j]/h0,1.]);rhs.append(1.)
        tick();fit=linprog([0.,0.,0.,-1.],A_ub=np.asarray(rows),b_ub=np.asarray(rhs),bounds=[(-1.,1.)]*3+[(None,None)],method='highs',options={} if maxiter is None else dict(maxiter=maxiter))
        tick();entry=dict(vertex=v,interfaces=interfaces,success=bool(fit.success),status=int(fit.status))
        if fit.success and np.isfinite(fit.x).all():
            goal[j]=base[j]+scales[j]*fit.x[:3];useful=True
            entry.update(normalized_slack=float(fit.x[3]),target=goal[j].tolist())
        records.append(entry)
    return (goal if useful else None),dict(method='TwoSidedAltitudeVolumeCompleteStarKernel/1',components=records,
        proposal_only=True,full_raw_quality_and_exact_preservation_required=True)
