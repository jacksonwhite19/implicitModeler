"""Bounded arithmetic and evidence checks, not a skin-friction model.

No physical defaults, surface sizing, or geometry generation are provided.
"""
import math
from pathlib import Path
import numpy as np
from sdf_cfd_mesher_v4.common.authority import file_hash, identity


def first_height(speed, density, dynamic_viscosity, yplus, cf_design):
    values=(speed,density,dynamic_viscosity,yplus,cf_design)
    if not all(type(v) in (int,float) and math.isfinite(v) and v>0 for v in values):
        raise ValueError('MissingOrInvalidPhysicalInput')
    nu=dynamic_viscosity/density
    u_tau=speed*math.sqrt(cf_design/2)
    if not math.isfinite(u_tau) or u_tau<=0:raise ValueError('InvalidFrictionVelocity')
    h1=2*yplus*nu/u_tau
    if not math.isfinite(h1) or h1<=0:raise ValueError('InvalidFirstHeight')
    return dict(nu=nu,u_tau=u_tau,h1=h1,hex={k:v.hex() for k,v in [('nu',nu),('u_tau',u_tau),('h1',h1)]})


def verify_sources(rows, root):
    if not rows:raise ValueError('SourceEvidenceRequired')
    for row in rows:
        if file_hash(Path(root)/row['path'])!=row['sha256']:raise ValueError('SourceEvidenceMismatch')
    return identity(rows)


def require_converged_dump(data, admission):
    """Finite output and stored convergence admission are different evidence."""
    if not np.isfinite(data).all():raise ValueError('NonfiniteDump')
    if admission is None:raise ValueError('OriginalConvergenceAdmissionMissing')
    # These are the recovered V3 producer's checks, not new CFD tolerances.
    if admission.get('return_code')!=0 or not (0<=admission.get('final_global_rms',math.inf)<1e-4):
        raise ValueError('OriginalConvergenceAdmissionFailed')
    return True
