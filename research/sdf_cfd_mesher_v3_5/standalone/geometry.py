"""V3.5 source/requirements entry to the copied V4 surface workflow."""
from pathlib import Path
from contextlib import contextmanager
import json,sys
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'v4/tools'),str(ROOT/'v4/src')]
import owned_runtime
owned_runtime.activate()

def write_case(template,script,output,request):
    case=json.loads(Path(template).read_text())
    case['geometry']=dict(rhai=Path(script).read_text(),metres_per_model_unit=.001)
    case['purpose']='V3.5 generic standalone integration; explicit sampled numerical surface policy; no CFD qualification.'
    case['prisms']=dict(first_full_normal_thickness_m_hex=float(request.first_height).hex(),fluid_side='positive_sdf',later_path_lengths_m_hex=[float(h).hex() for h in request.nominal[1:]])
    case['surface']['prism_host']['first_height_m']=request.first_height
    case['surface']['prism_host']['minimum_owned_layer_thickness_m']=float(request.nominal[1])
    Path(output).write_text(json.dumps(case,indent=2)+'\n')
    return case

def carrier_selection(requested=None,*,origin='explicit_api'):
    if requested is not None and requested not in ('adaptive','nodal'):raise ValueError('UnsupportedSurfaceCarrierMode')
    return dict(schema='V35SurfaceConstructionSelection/1',requested=requested,
        effective='adaptive' if requested is None else requested,
        origin='v3_5_backward_compatible_default' if requested is None else origin,
        assurance='numerical',profile='fastprod',final_acceptance='unchanged')

@contextmanager
def surface(case,output,*,carrier_mode=None,carrier_origin='explicit_api'):
    import standalone_tool as cli
    from production_surface_authority import fastprod_surface_scope
    selection=carrier_selection(carrier_mode,origin=carrier_origin)
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    (output/'SURFACE_CONSTRUCTION.json').write_text(json.dumps(selection,indent=2)+'\n',encoding='utf-8')
    # Explicit V4 numerical engineering policy. No predicate is replaced.
    token=cli._surface_assurance.set('numerical')
    carrier_token=cli._carrier_assessment.set(selection['effective']=='adaptive')
    try:
        with fastprod_surface_scope(output/'surface/A/fresh'):
            path,record=cli.surface_stage(output,Path(case),profile='fastprod')
            yield path,record
    finally:
        cli._carrier_assessment.reset(carrier_token)
        cli._surface_assurance.reset(token)

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    with surface(args.case,args.output) as result:print(json.dumps(dict(surface=str(result[0]),authority=result[1]),default=str))
