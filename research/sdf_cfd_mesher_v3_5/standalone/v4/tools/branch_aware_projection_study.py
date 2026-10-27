"""Study-only continuation. No production imports this module.

For min-of-sphere distances retain the source sphere as an active constraint.
Enumerate its smooth surface, pair intersection circles, and triple vertices.
Choose the admitted point nearest the predictor, with canonical tie ordering.
This is a projection choice, not a capacity proof or mesh acceptance rule.
All admission uses the existing authoritative SDF evaluation error bound.
"""
from itertools import combinations
from types import SimpleNamespace
import numpy as np
from complete_capacity_fixtures import identity

VERSION='BranchAwareProjectionStudy/1'


def owner(points,spec):
    x=np.asarray(points);kind=spec['kind']
    if kind=='spheres':
        return np.argmin(np.linalg.norm(x[:,None]-np.array(spec['centres']),axis=2)-np.array(spec['radii']),axis=1)
    if kind=='wedge':
        a=np.array([[1,-spec['right_slope'],0],[-1,-spec['left_slope'],0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]],float)
        b=np.array([spec['half_gap'],spec['half_gap'],spec['closure_high'],-spec['closure_low'],spec['closure_z'],spec['closure_z']])
        return np.argmin((b-x@a.T)/np.linalg.norm(a,axis=1),axis=1)
    if kind=='frustum':
        rho=np.linalg.norm(x[:,:2],axis=1);p=np.c_[rho,x[:,2]];r=spec['radius'];s=spec['slope'];lo=spec['closure_low'];hi=spec['closure_high']
        seg=[([r+s*lo,lo],[r+s*hi,hi]),([0,lo],[r+s*lo,lo]),([0,hi],[r+s*hi,hi])];ds=[]
        for aa,bb in seg:
            a=np.array(aa);b=np.array(bb);t=np.clip((p-a)@(b-a)/np.dot(b-a,b-a),0,1)
            ds.append(np.sum((p-a-t[:,None]*(b-a))**2,axis=1))
        return np.argmin(ds,axis=0)
    raise ValueError('ExplicitBranchIdentityUnavailable')


def branch_identity(spec,source):
    return identity(dict(version=VERSION,sdf=spec,source=int(source)))


def sphere_project(points,target,sdf,spec,ancestry):
    """Bounded active-set projection of each point onto its ancestral branch.

    Degenerate algebraic constructions are omitted, never accepted by tolerance.
    The enumeration is bounded to the fixture's three spheres. No absence proof
    follows if no floating construction survives authoritative residual checks.
    """
    if spec['kind']!='spheres' or spec.get('cavity') or len(spec['radii'])>3:raise ValueError('UnsupportedSourceConstruction')
    points=np.asarray(points,float);ancestry=np.asarray(ancestry)
    if points.shape!=(len(ancestry),3) or not np.all(np.isfinite(points)) or not np.isfinite(target):raise ValueError('InvalidProjectionInputs')
    if np.any(ancestry<0) or np.any(ancestry>=len(spec['radii'])):raise ValueError('InvalidSourceIdentity')
    c=np.array(spec['centres']);r=np.array(spec['radii'])+float(target);out=[];records=[]
    for point,source in zip(np.asarray(points),ancestry):
        i=int(source);d=point-c[i];length=np.linalg.norm(d)
        if not length:raise ValueError('UndefinedSourceDirection')
        candidates=[((i,),c[i]+r[i]*d/length)]
        for j in range(len(c)):
            if j==i:continue
            axis=c[j]-c[i];distance=np.linalg.norm(axis)
            if not distance:continue
            n=axis/distance;a=(r[i]**2-r[j]**2+distance**2)/(2*distance);h2=r[i]**2-a*a
            if h2<0:continue
            centre=c[i]+a*n;t=point-centre-n*np.dot(point-centre,n);tn=np.linalg.norm(t)
            if tn==0:
                basis=np.eye(3)[int(np.argmin(abs(n)))];t=np.cross(n,basis);tn=np.linalg.norm(t)
            candidates.append((tuple(sorted((i,j))),centre+np.sqrt(h2)*t/tn))
        for j,k in combinations([j for j in range(len(c)) if j!=i],2):
            v=c[j]-c[i];dist=np.linalg.norm(v)
            if dist==0:continue
            ex=v/dist;w=c[k]-c[i];u=np.dot(ex,w);v=w-u*ex;length=np.linalg.norm(v)
            if length==0:continue
            ey=v/length;ez=np.cross(ex,ey);xx=(r[i]**2-r[j]**2+dist**2)/(2*dist)
            yy=(r[i]**2-r[k]**2+u*u+length*length-2*u*xx)/(2*length);zz=r[i]**2-xx*xx-yy*yy
            if zz<0:continue
            for sign in (-1,1):candidates.append((tuple(sorted((i,j,k))),c[i]+xx*ex+yy*ey+sign*np.sqrt(zz)*ez))
        valid=[]
        for ids,p in candidates:
            z=sdf.evaluate(p[None]);res=float(z.phi[0]-target)
            source_res=float(np.linalg.norm(p-c[i])-spec['radii'][i]-target)
            if np.all(np.isfinite(p)) and z.gradient_norm[0]>0 and abs(res)<=z.source_error_bound[0] and abs(source_res)<=z.source_error_bound[0]:
                valid.append((float(np.sum((p-point)**2)),ids,tuple(p),res,source_res))
        if not valid:raise ValueError(('NoAdmittedAncestralCandidate',i))
        best=min(valid,key=lambda v:(v[0],v[1],v[2]));out.append(best[2]);records.append(dict(source=i,branch=branch_identity(spec,i),active_sources=best[1],enumerated=len(candidates),admitted=len(valid),residual=best[3],source_residual=best[4],selection='NearestPredictorThenCanonicalIDs',unintended_branch_jump=False))
    return np.asarray(out),records


def continue_front(points,current,target,sdf,spec,ancestry,substeps=8,owner_provider=None):
    """Eight deterministic continuation steps; step count is a study parameter.

    Channels use ordinary authoritative Newton at each intermediate level.
    Feature changes are observed, not forbidden: forbidding a legitimate closest
    feature change would impose an unsupported scientific constraint.
    """
    from sdf_cfd_mesher_v4.advancing_front.predict import predict_next_front
    if not isinstance(substeps,int) or substeps<1:raise ValueError('InvalidStudyStepCount')
    p=np.asarray(points).copy();trace=[]
    ownership=(lambda p:owner(p,spec)) if owner_provider is None else owner_provider
    for step in range(1,substeps+1):
        distance=float(target) if step==substeps else float(current+(target-current)*(step/substeps))
        before=ownership(p)
        if spec['kind']=='spheres':p,rec=sphere_project(p,distance,sdf,spec,ancestry);iterations=np.ones(len(p),int)
        else:
            pred=predict_next_front(p,distance,sdf,iteration_cap=256);p=pred.coordinates;iterations=pred.iterations;rec=None
        trace.append(dict(step=step,target=distance,coordinates=p.tolist(),owners_before=before.tolist(),owners_after=ownership(p).tolist(),iterations=iterations.tolist(),source_records=rec))
    return SimpleNamespace(coordinates=p,trace=trace)


def check_source(point,target,spec,source,sdf):
    """Detect an inadmissible source-branch jump without a distance heuristic."""
    z=sdf.evaluate(np.asarray(point)[None]);res=abs(np.linalg.norm(point-np.array(spec['centres'][source]))-spec['radii'][source]-target)
    return bool(abs(z.phi[0]-target)<=z.source_error_bound[0] and res<=z.source_error_bound[0])
