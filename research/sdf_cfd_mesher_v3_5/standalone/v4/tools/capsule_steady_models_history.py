"""Strict steady history parser for explicitly admitted transported fields."""
import re
import numpy as np
from capsule_rans_pilot_evidence import NUMBER

TRANSPORT=dict(kOmegaSST=('k','omega'),kOmegaSSTLM=('k','omega','ReThetat','gammaInt'),SpalartAllmaras=('nuTilda',))

def parse(text,model):
    if model not in TRANSPORT:raise ValueError('UnsupportedSteadyModel')
    expected={'Ux','Uy','Uz','p',*TRANSPORT[model]};rows=[]
    for m in re.finditer(r'^Time = (\d+)s?\s*\n(.*?)(?=^Time = |\Z)',text,re.M|re.S):
        block=m[2];res={}
        for eq,a,b,n in re.findall(r'Solving for (\w+), Initial residual = ('+NUMBER+r'), Final residual = ('+NUMBER+r'), No Iterations (\d+)',block):
            if eq in res:raise ValueError('MultipleEquationSolvesRequireExplicitSemantics')
            res[eq]=dict(initial=float(a),final=float(b),iterations=int(n))
        if set(res)!=expected:raise ValueError('IncompleteOrUnexpectedTransportedEquation')
        vectors=re.findall(r'(pressure|viscous|porous)\s*:\s*\(([^)]+)\)',block)
        if [v[0] for v in vectors]!=['pressure','viscous','porous']*2:raise ValueError('IncompleteForcesMoments')
        arrays=[list(map(float,v.split())) for _,v in vectors]
        if any(len(v)!=3 for v in arrays):raise ValueError('WrongVectorDimension')
        flux={}
        for op,patch,value in re.findall(r'(sum|sumMag)\((wall|farfield)\) of phi = ('+NUMBER+r')',block):
            key=patch+'_'+op
            if key in flux:raise ValueError('DuplicateFluxMonitor')
            flux[key]=float(value)
        if len(flux)!=4:raise ValueError('IncompleteFlux')
        signed=flux['wall_sum']+flux['farfield_sum'];absolute=flux['wall_sumMag']+flux['farfield_sumMag']
        inflow=(absolute-signed)/2;outflow=(absolute+signed)/2
        if max(inflow,outflow)<=0:raise ValueError('NoThroughFlow')
        bounds=[dict(field=f,minimum=float(a),maximum=float(b),average=float(c)) for f,a,b,c in re.findall(r'bounding (\w+), min: ('+NUMBER+r') max: ('+NUMBER+r') average: ('+NUMBER+r')',block)]
        row=dict(iteration=int(m[1]),residuals=res,pressure_force=arrays[0],viscous_force=arrays[1],force=np.sum(arrays[:3],axis=0).tolist(),pressure_moment=arrays[3],viscous_moment=arrays[4],moment=np.sum(arrays[3:],axis=0).tolist(),flux=dict(**flux,signed=signed,inflow=inflow,outflow=outflow,normalized=abs(signed)/max(inflow,outflow)),bounds=bounds)
        if not all(np.isfinite(v) for r in res.values() for v in r.values()):raise ValueError('NonfiniteResidual')
        if not np.isfinite(arrays).all() or not all(np.isfinite(v) for v in flux.values()):raise ValueError('NonfiniteMonitor')
        rows.append(row)
    if [r['iteration'] for r in rows]!=list(range(1,len(rows)+1)):raise ValueError('NonContiguousFreshHistory')
    return rows

def windows(rows,model):
    import capsule_pressure_diagnostic as p
    assert [r['iteration'] for r in rows]==list(range(1,501))
    signals={eq:np.array([r['residuals'][eq]['initial'] for r in rows]) for eq in ('p','Ux','Uy','Uz',*TRANSPORT[model])}
    for key,prefix in [('force','F'),('moment','M')]:
        for j,axis in enumerate('xyz'):signals[prefix+axis]=np.array([r[key][j] for r in rows])
    result={}
    for lo in (201,301,401,451):
        a={k:v[lo-1:] for k,v in signals.items()}
        out=dict(signals={k:p.d.signal(v) for k,v in a.items()},cross={x+'/'+y:p.d.cross(a[x],a[y]) for x,y in [('Fx','My'),('Fy','Mx'),('Fx','Fy'),('Ux','Uy'),('Fx','Ux'),('Fy','Uy')]},lateral={})
        for prefix in ('F','M'):
            angle=np.unwrap(np.arctan2(a[prefix+'y'],a[prefix+'x']))*180/np.pi
            out['lateral'][prefix]=dict(magnitude=p.d.signal(np.hypot(a[prefix+'x'],a[prefix+'y'])),angle=p.d.signal(angle),first_angle=float(angle[0]),last_angle=float(angle[-1]))
            for axis in 'xyz':out['signals'][prefix+axis]['normalized_range']=out['signals'][prefix+axis]['range']/(3.460917477391561 if prefix=='F' else .5537467963826498)
        result[f'{lo}-500']=out
    return result
