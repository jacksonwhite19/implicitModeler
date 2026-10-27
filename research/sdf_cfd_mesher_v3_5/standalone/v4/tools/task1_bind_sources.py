"""Bind current native/query, schedule and scaling sources without changing V3."""
from prepare_source_fork import ROOT,REPO,H,sha,put
import json,shutil
a=json.loads((H/'NATIVE_SPLINE_PRODUCTION_AUTHORITY.json').read_text())
rows=[]
for key in ('accelerated_spline_source','accelerated_bridge_source'):
    from pathlib import Path
    p=Path(a[key]);assert sha(p)==a[key+'_sha256']
    dst=ROOT/'native/frozen'/p.name;dst.parent.mkdir(parents=True,exist_ok=True)
    assert not dst.exists();shutil.copyfile(p,dst)
    rows.append(dict(path=p.relative_to(REPO).as_posix(),sha256=sha(p),destination=dst.relative_to(ROOT).as_posix(),copied=True))
paths=[H/'NATIVE_SPLINE_PRODUCTION_AUTHORITY.json',H/'SMOOTHSPLINE_COMPLETE_OBJECT_EQUIVALENCE.json',H/'fresh_inputs/layers/FROZEN_CONTRACT.json',H/'SF2/A/wing/planner/input_state/increments.npy',H/'SF2/A/wing/planner/input_state/state.json',H/'scoped_operator_production_numerical_counterfactual_20260911/fixed_flux_velocity_reconstruction_20260912/prism_geometry_common_field_20260912/revised_conditioning_planner_full_shell_development_20260916/FULL_DOMAIN_PARTITION.json',H/'scoped_operator_production_numerical_counterfactual_20260911/fixed_flux_velocity_reconstruction_20260912/prism_geometry_common_field_20260912/revised_conditioning_planner_full_shell_development_20260916/STAGE_A.json',REPO/'src/sdf/aerospace/wing.rs']
for p in paths:rows.append(dict(path=p.relative_to(REPO).as_posix(),sha256=sha(p),copied=False))
put(ROOT/'authorities/TASK1_SOURCE_BINDING.json',dict(files=rows,source_math='Qualified cached NacaSpline, full closed section query; original reference scan retained',composition='Uniform extrusion distance formula from production wing.rs, evaluated in binary64; analytic chain rule; no new geometry representation'))
print('Bound',len(rows),'sources; copied only two small native source files')
