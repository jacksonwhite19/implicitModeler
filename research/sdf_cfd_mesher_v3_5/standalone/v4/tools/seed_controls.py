"""Explicit seed configuration and independent receipt arithmetic; no geometry parser."""
import copy
import math
import struct

METHOD='CanonicalMarchingCubesSeed/1'
PROTOCOL_BYTES=8*1024*1024
U32=(1<<32)-1
U64=(1<<64)-1

def validate_settings(s):
    common={'method','padding_m','timeout_seconds','maximum_queries'}
    modes=set(s)-common
    if s.get('method')!=METHOD or modes not in ({'spacing_m'},{'cells_across'}) or not common.issubset(s):
        raise ValueError('ExactlyOneExplicitSeedSpacingOrCellsAcrossRequired')
    for key in ['padding_m','timeout_seconds',*(['spacing_m'] if 'spacing_m' in s else [])]:
        if type(s[key]) not in (int,float) or not math.isfinite(s[key]) or s[key]<=0:
            raise ValueError('FinitePositiveSeedControlRequired',key)
    if type(s['maximum_queries']) is not int or not 1<=s['maximum_queries']<=U64:
        raise ValueError('PositiveU64SeedQueryBudgetRequired')
    if 'cells_across' in s and (type(s['cells_across']) is not int or not 1<=s['cells_across']<=U32):
        raise ValueError('PositiveU32SeedCellsAcrossRequired')
    return s

def configure(case, spacing=None, cells=None, queries=None):
    if spacing is not None and cells is not None:raise ValueError('MutuallyExclusiveSeedSpacingAndCellsAcross')
    effective=copy.deepcopy(case)
    if not any(v is not None for v in (spacing,cells,queries)):
        return effective,dict(source='case_profile',explicit_controls=None)
    try:s=effective['general_sdf']['seed']
    except KeyError as error:raise ValueError('ExplicitCanonicalSeedProfileRequired') from error
    if s.get('method')!=METHOD:raise ValueError('CanonicalSeedControlsRequireCanonicalSeedMethod')
    if spacing is not None:s.pop('cells_across',None);s['spacing_m']=spacing
    if cells is not None:s.pop('spacing_m',None);s['cells_across']=cells
    if queries is not None:s['maximum_queries']=queries
    validate_settings(s)
    return effective,dict(source='explicit_cli',explicit_controls=dict(spacing_m=spacing,cells_across=cells,maximum_queries=queries))

def query_controls(s):
    validate_settings(s)
    return {k:v for k,v in s.items() if k!='method'}

def f32(x):
    try:return struct.unpack('<f',struct.pack('<f',x))[0]
    except (OverflowError,struct.error):raise ValueError('CoordinateOutOfKernelRange')

def validate_plan(s, plan, description):
    """Check native arithmetic against its protected metadata descriptor, not a side parser."""
    validate_settings(s)
    scale=description['metres_per_model_unit']
    bounds=description['stored_metadata']['bounds_model']
    pad=f32(s['padding_m']/scale)
    low=[f32(v-pad) for v in bounds[0]];high=[f32(v+pad) for v in bounds[1]]
    extents=[f32(b-a) for a,b in zip(low,high)]
    if not all(math.isfinite(v) and v>0 for v in extents):raise ValueError('InvalidSeedBounds')
    lengths=[v*scale for v in extents];longest=max(extents)*scale
    spacing=s.get('spacing_m',longest/s.get('cells_across',1))
    if not math.isfinite(spacing) or spacing<=0:raise ValueError('InvalidDerivedSeedSpacing')
    ratio=longest/spacing
    requested=max(8,math.ceil(ratio)) if math.isfinite(ratio) else None
    r=requested if requested is not None and requested<U32 else None
    checked=lambda x:x if x<=U64 else None
    points=checked((r+1)**3) if r is not None else None
    cubes=checked(r**3) if r is not None else None
    edges=checked(3*r*(r+1)**2) if r is not None else None
    safe=all(x is not None and x<=U32 for x in (points,cubes,edges))
    reason='ExtractorIndexArithmeticLimit' if not safe else 'QueryBudgetExceeded' if points>s['maximum_queries'] else 'ADMITTED_CONSTRUCTION_ONLY'
    steps=[f32(v/f32(r))*scale for v in extents] if r is not None else None
    if safe and not all(math.isfinite(v) and v>0 for v in steps):reason='CoordinateOutOfKernelRange'
    expected=dict(schema='V4SeedAdmission/1',metres_per_model_unit=scale,
        bounds_m_realized=[[v*scale for v in low],[v*scale for v in high]],axis_lengths_m=lengths,longest_padded_extent_m=longest,
        padding_m_requested=s['padding_m'],spacing_m_requested=s.get('spacing_m'),cells_across_requested=s.get('cells_across'),
        spacing_m_effective=spacing,construction_floor_cells=8,resolution=r,grid_step_m=steps,predicted_grid_samples=points,
        maximum_queries=s['maximum_queries'],timeout_seconds=s['timeout_seconds'],protocol_limit_bytes=PROTOCOL_BYTES,
        admitted=reason=='ADMITTED_CONSTRUCTION_ONLY',reason=reason,scalar_grid_attempts=0,scalar_field_evaluations=0,geometry_acceptance=False)
    for key,value in expected.items():
        integer_fields={'cells_across_requested','construction_floor_cells','resolution','predicted_grid_samples',
            'maximum_queries','protocol_limit_bytes','scalar_grid_attempts','scalar_field_evaluations'}
        bad_type=(type(value) is bool and type(plan.get(key)) is not bool
            or key in integer_fields and value is not None and type(plan.get(key)) is not int
            or type(value) in (int,float) and type(plan.get(key)) is bool)
        if plan.get(key)!=value or bad_type:
            raise ValueError('SeedPlanReceiptMismatch',key,plan.get(key),value)
    return plan

def validate_extraction(s, data, plan):
    n=plan['predicted_grid_samples']
    counts=data.get('stage_counts',{})
    if (plan['admitted'] is not True or data.get('seed_plan')!=plan or data.get('resolution')!=plan['resolution']
        or data.get('bounds_m_realized')!=plan['bounds_m_realized'] or data.get('grid_step_m')!=plan['grid_step_m']
        or data.get('spacing_m_requested')!=s.get('spacing_m') or data.get('cells_across_requested')!=s.get('cells_across')
        or data.get('padding_m_requested')!=s['padding_m'] or n>s['maximum_queries']
        or any(type(data.get(k)) is not int or data[k]!=n for k in ('grid_points','grid_scalar_evaluations','scalar_evaluations'))
        or any(type(counts.get(k)) is not int or counts[k]!=n for k in ('requested_scalar_grid_attempts','scalar_grid_attempts','scalar_field_evaluations'))
        or any(type(counts.get(k)) is not int or counts[k]!=0 for k in ('conditioning_attempts','normal_stencil_attempts','projection_attempts'))
        or any(counts.get(k) is not False for k in ('budget_exceeded','timed_out','nonfinite_field','geometry_acceptance'))
        or data.get('route')!=METHOD or data.get('producer')!='V4StockScalarConstructionSeed/1'
        or data.get('continuous_acceptance') is not False or data.get('volume_refinement') is not False
        or any(data.get(k)!='NotRun' for k in ('seed_projection','seed_geometry_audit','full_grid_normals'))
        or counts.get('schema')!='V4ScalarSeedDiagnostics/1'):
        raise ValueError('ScalarSeedAccountingOrAdmissionMismatch')
