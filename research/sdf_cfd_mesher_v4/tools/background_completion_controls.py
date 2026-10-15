"""Platform-neutral completion controls shared by supervisor and backend.

Validation imports no live mesh, Windows custody owner or numerical solver.
Construction and final acceptance remain separate existing consumers.
"""
import math
CORE_SCHEMA='V4CoreMeshCompletion/1'
CORE_FIELDS={'schema','maxiter','ftol','seconds_per_sweep','preservation_max_depth','preservation_steps_per_vertex','support_policy'}

def validate_core(controls):
    if type(controls) is not dict or set(controls)!=CORE_FIELDS or controls['schema']!=CORE_SCHEMA or controls['support_policy']!='all_current_raw_defects':raise ValueError('ExplicitGenericCoreRepairControlsRequired')
    for name in ('maxiter','preservation_max_depth','preservation_steps_per_vertex'):
        if type(controls[name]) is not int or controls[name]<1:raise ValueError('InvalidGenericCoreRepairControl',name)
    for name in ('ftol','seconds_per_sweep'):
        if type(controls[name]) not in (int,float) or not math.isfinite(controls[name]) or controls[name]<=0:raise ValueError('InvalidGenericCoreRepairControl',name)
    return controls

def validate_completion(controls):
    if type(controls) is dict and controls.get('schema')==CORE_SCHEMA:return validate_core(controls)
    from background_interface_completion import validate
    return validate(controls)
