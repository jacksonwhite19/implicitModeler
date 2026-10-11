"""Retain exact focused-test commands, output, and exit status."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[3]
V4=ROOT/'research/sdf_cfd_mesher_v4'
OUT=V4/'results/generic_canonical_bridge'
env=os.environ.copy()
env['CARGO_TARGET_DIR']=str(ROOT/'target')
env['PYTHONPATH']=os.pathsep.join([str(V4/'src'),str(V4/'tools')])
env['PYTHONDONTWRITEBYTECODE']='1'
commands=[
 ['cargo','test','--offline','--manifest-path',str(V4/'canonical_kernel/Cargo.toml'),'--test','contract','--quiet'],
 [sys.executable,'-B',str(V4/'tools/test_canonical_transport.py')],
 [sys.executable,'-B','-m','pytest',str(V4/'tests/test_production_adaptive_surface.py'),'-q','-k',
  'verified_sealed_surface_loads or exact_projection_and_source_branch or sizing_spd_bounds_and_curvature_response or v3_trace_and_dependency_bindings or seed_scale_defect_reproducer','-p','no:cacheprovider'],
]
records=[]
for command in commands:
    start=time.perf_counter()
    result=subprocess.run(command,cwd=ROOT,env=env,capture_output=True,text=True,timeout=90)
    records.append(dict(command=command,seconds=time.perf_counter()-start,exit_code=result.returncode,stdout=result.stdout,stderr=result.stderr))
    print(result.stdout,result.stderr[-1800:],flush=True)
(OUT/'FOCUSED_TESTS.json').write_text(json.dumps(records,indent=2)+'\n')
if any(r['exit_code'] for r in records):raise SystemExit(1)
