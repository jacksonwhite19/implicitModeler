"""Focused checks and source binding. Does not repeat surface generation."""
import json, os, subprocess, sys
from pathlib import Path
sys.dont_write_bytecode=True
V4=Path(__file__).resolve().parents[1];ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'src'),str(V4/'tools')]
from historical_capsule_surface import sha,save


def main():
    out=V4/'results/production_surface_dispatch'
    commands=[
        [sys.executable,'-B',str(V4/'tools/test_surface_dispatch.py')],
        [sys.executable,'-B',str(V4/'tools/test_canonical_transport.py')],
        [sys.executable,'-B','-m','pytest',str(V4/'tests/test_production_adaptive_surface.py'),
         '--noconftest','-q','-k',
         'verified_sealed_surface_loads or exact_projection_and_source_branch or sizing_spd_bounds_and_curvature_response or v3_trace_and_dependency_bindings or seed_scale_defect_reproducer or manifold_topology_orientation_and_boundary or failed_audit_cannot_publish',
         '-p','no:cacheprovider']]
    env=dict(os.environ,PYTHONPATH=os.pathsep.join([str(V4/'src'),str(V4/'tools')]),PYTHONDONTWRITEBYTECODE='1')
    results=[]
    for command in commands:
        r=subprocess.run(command,env=env,capture_output=True,text=True,timeout=60)
        results.append(dict(command=command,exit_code=r.returncode,stdout=r.stdout,stderr=r.stderr))
        save(out/'FOCUSED_CHECKS.json',dict(results=results,early_failure=dict(
            command='python -B tools/test_surface_dispatch.py',
            error='ImportError: cannot import name SeedEdge from historical_capsule_surface',
            resolution='Harness now imports existing SeedEdge from procedural_authorities; no production or geometry change',
            surface_run_started=False)))
        if r.returncode:raise RuntimeError('FocusedCheckFailed')
    files=['production_adaptive_surface.py','historical_capsule_surface.py','surface_binding.py',
        'surface_execution.py','audit_adaptive_surface.py','replay_surface_dispatch.py','test_surface_dispatch.py',
        'check_surface_dispatch_evidence.py','production_surface_authority.py','production_capsule_cgal_seed.cpp']
    records=[dict(path='tools/'+p,sha256=sha(V4/'tools'/p)) for p in files]
    # Bind unchanged imported algorithms and native programs from the sealed trace.
    trace=json.loads((V4/'results/production_adaptive_capsule_surface_foundation/IMPLEMENTATION_TRACE.json').read_text())
    inherited=trace['v3_sources']+trace['v3_import_bindings']
    for r in inherited:assert sha(ROOT/r['path'])==r['sha256'],r['path']
    save(out/'SOURCE_BINDINGS.json',dict(current_v4=records,inherited_v3=inherited,
        inherited_algorithms_unchanged=True,qualified_native_seed_sha256=trace['native_seed_sha256'],
        qualified_intersection_sha256=trace['native_intersections_sha256'],
        qualified_mmgs_library_sha256=trace['library_sha256'],qualified_mmgs_runner_sha256=trace['adapter_sha256']))
    print(json.dumps(dict(check_groups=len(results),all_pass=True,v3_records_verified=len(inherited))))


if __name__=='__main__':main()
