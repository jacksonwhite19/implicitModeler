"""Admission and acceptance primitives. These do not execute or retry a solve."""
import hashlib
import math
import re
import numpy as np

def registry_audit(required,text):
    if 'FOAM FATAL' in text:raise ValueError('InvalidRegistryEvidence')
    names=set(re.findall(r'^\s+([A-Za-z][A-Za-z0-9_]*)\s+lib\S+\.so\s*$',text,re.M))
    missing=sorted(set(required)-names)
    return dict(passed=not missing,missing=missing)

def conservative_flux(values,wall_mask):
    values=np.asarray(values,dtype=np.float64);wall_mask=np.asarray(wall_mask,dtype=bool)
    if values.shape!=wall_mask.shape or not np.isfinite(values).all():raise ValueError('InvalidFlux')
    inlet=float(-values[values<0].sum());outlet=float(values[values>0].sum())
    if max(inlet,outlet)<=0:raise ValueError('NoThroughFlow')
    return dict(inflow=inlet,outflow=outlet,total=float(values.sum()),
        normalized=abs(float(values.sum()))/max(inlet,outlet),
        wall_abs=float(np.abs(values[wall_mask]).sum()),wall=float(values[wall_mask].sum()),
        farfield=float(values[~wall_mask].sum()))

def canonical_array_hash(array):
    a=np.asarray(array,dtype='<f8')
    if not np.isfinite(a).all():raise ValueError('NonfiniteArray')
    return hashlib.sha256(str(a.shape).encode()+a.tobytes(order='C')).hexdigest()

def stopping(rows,contract,q_area,length):
    n=contract['window'];count=contract['consecutive_disjoint_windows']
    if len(rows)<n*count:return False
    for i in range(count):
        window=rows[len(rows)-(i+1)*n:len(rows)-i*n]
        if any(not np.isfinite(r['residuals']).all() or not np.isfinite(
            [r['flux']['normalized'],r['flux']['wall_abs']]).all() for r in window):return False
        if any(r['fatal'] or max(r['residuals'])>contract['residual_max']
               or r['flux']['normalized']>contract['normalized_flux_max']
               or r['flux']['wall_abs']>contract['wall_absolute_flux_max'] for r in window):return False
        for key,scale,limit in [('force',q_area,contract['force_component_range_normalized_max']),
                                ('moment',q_area*length,contract['moment_component_range_normalized_max'])]:
            a=np.asarray([r[key] for r in window])
            if not np.isfinite(a).all() or np.any(np.ptp(a,axis=0)/scale>limit):return False
    return True

def startup_result(stdout,stderr):
    iterations=len(re.findall(r'^Time = ',stdout,re.M))
    fatal='FOAM FATAL' in stderr or 'FOAM FATAL' in stdout
    missing=re.findall(r'Unknown function type (\w+)',stderr)
    return dict(passed=False,iterations=iterations,fatal=fatal,unknown_functions=sorted(set(missing)),
        classification='V4CapsuleRANSRunAStartupFailedFunctionObjectRegistration' if missing else 'Unqualified')
