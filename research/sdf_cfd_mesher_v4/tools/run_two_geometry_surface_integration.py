"""Two authorized cases through existing executors; fail closed on first failure.

No geometry, metric, projection, ownership, or certificate implementation here.
The supervisor limits retained output; existing native deadlines are unchanged.
"""
import argparse
import contextlib
import dataclasses
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import traceback

import numpy as np
import psutil

sys.dont_write_bytecode = True
V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
sys.path[:0] = [str(V4/'src'), str(V4/'tools')]
from historical_capsule_surface import (OUT as SEALED, V3, CapsuleGeometry,
    capsule_binding, SurfaceSizing, sha, wsl, identity)
from development_surface_policy import load_case, bind_development_policy
from surface_binding import bind_canonical
from production_adaptive_surface import ProductionAdaptiveSdfSurfaceMesher
from production_surface_authority import (load_production_wall, publish_surface,
    scientific_identity, BOUND_FIELDS)
from replay_surface_dispatch import verify_kernel_sources
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel, GeometryFailure

WING = V4/'cases/plain_wing_surface_integration_approved.json'
CAPSULE = V4/'cases/capsule_surface_integration_control.json'


def read(path):
    return json.loads(Path(path).read_text())


def save(path, value):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False,
        default=lambda x: x.tolist() if isinstance(x,np.ndarray) else str(x))+'\n', newline='\n')


def size(path):
    return sum(p.stat().st_size for p in Path(path).rglob('*') if p.is_file())


def inventory(path):
    return {str(p.relative_to(path)).replace('\\','/'):dict(bytes=p.stat().st_size,sha256=sha(p))
            for p in sorted(Path(path).rglob('*')) if p.is_file()}


def resource_plan():
    paths = [SEALED/'fresh', V4/'results/production_surface_dispatch/capsule']
    comparable = [dict(path=str(p.relative_to(ROOT)),bytes=size(p)) for p in paths]
    # Existing replay policy reserves twice a measured comparable footprint.
    # Four planned realizations = two cases, each fresh plus deterministic replay.
    allowance = 2*4*max(r['bytes'] for r in comparable)
    return dict(comparables=comparable,campaign_output_limit_bytes=allowance,
        minimum_free_bytes=allowance,native_stage_seconds=120,chart_total_seconds=300,
        chart_call_seconds=30,chart_output_limit_bytes=8*1024*1024,
        monitoring_seconds=1,
        rationale='Existing 2x comparable storage reserve times four realizations. Not a predicted wing mesh size or scientific gate. No reliable full-wing output or memory forecast exists.',
        time_scope='Existing run_native and labelled_seed deadlines unchanged. Python process memory measured, not predicted or capped.')


def compact_admission(binding):
    report = ProductionAdaptiveSdfSurfaceMesher().admission(binding)
    if not report['executable']:
        raise GeometryFailure('CampaignAdmissionFailure','preflight',report['issues'],binding.geometry_identity)
    return dict(executable=report['executable'],issues=report['issues'],
        geometry_identity=report['geometry_identity'],units=report['units'],precision=report['precision'],
        seed_route=report['seed_route'],references={k:len(v) for k,v in binding.references.items()},
        operation_provenance=report['operations'],qualification_identity=identity(dataclasses.asdict(binding.qualification)))


def compare(first, second, *, retained_capsule=False):
    old, new = read(first/'GENERATION.json'),read(second/'GENERATION.json')
    keys=('coordinate_hash','topology_hash','sizing_field_identity','residual_hash',
          'geometry_identity','configuration_identity','adaptation_identity','surface_identity',
          'carrier','mmgs_operation_rows','projections','initial_vertices','initial_triangles',
          'final_vertices','final_triangles')
    fields={k:old[k]==new[k] for k in keys}
    arrays={}
    for name in ('INITIAL.npz','SURFACE.npz','QUALITY.npz','GEOMETRY_CERTIFICATE.npz','REFERENCES.npz'):
        # Original capsule replay compares the 23 arrays in four sealed NPZs.
        # REFERENCES was introduced by the shared executor after that seal.
        if retained_capsule and name=='REFERENCES.npz':continue
        if not (first/name).exists() and not (second/name).exists():continue
        with np.load(first/name) as a, np.load(second/name) as b:
            arrays[name]={k:k in b.files and np.array_equal(a[k],b[k]) for k in a.files}
            arrays[name]['same_keys']=set(a.files)==set(b.files)
    a,b=read(first/'AUDIT.json'),read(second/'AUDIT.json')
    source={}
    if 'source_boundary' in a:
        for k in ('bounds','curve_evidence','unresolved','position_bound_max_m',
                  'normal_bound_max_rad','contract_identity'):
            source[k]=a['source_boundary'][k]==b['source_boundary'][k]
    return dict(fields=fields,arrays=arrays,source_boundary=source,
        independent_audits_passed=a['passed'] and b['passed'],
        passed=all(fields.values()) and all(all(v.values()) for v in arrays.values())
            and all(source.values()) and a['passed'] and b['passed'])


def worker(out):
    start=time.perf_counter();active='preflight';completed=[]
    try:
        wing=load_case(WING);cap=read(CAPSULE);plan=read(out/'RESOURCE_PLAN.json')
        if cap['approval']['status']!='approved' or not cap['execution']['authorized']:
            raise ValueError('CapsuleApprovalRequired')
        original=V4/wing['derived_from_proposal']['path']
        if sha(original)!=wing['derived_from_proposal']['sha256']:
            raise ValueError('OriginalProposalIdentityMismatch')
        proposal=read(original)
        for k in proposal:
            if k not in ('approval','execution') and wing[k]!=proposal[k]:
                raise ValueError('ApprovedPackageChanged:'+k)
        if any(os.environ.get(k) is not None for k in ('V3_MMGS_NOMOVE','SDF_LENGTH_SCALE')):
            raise ValueError('ConflictingMeshingEnvironment')
        source,direct=verify_kernel_sources()
        if source['source_sha256']!=wing['geometry']['kernel_source_digest']:
            raise ValueError('KernelSourceMismatch')
        definition=read(SEALED/'GEOMETRY.json')
        old=read(SEALED/'fresh/GENERATION.json')
        _,_,sealed=load_production_wall(SEALED/'fresh')
        if sealed['surface_identity']!=cap['sealed_scientific_identity'] or definition['definition']!=cap['definition']:
            raise ValueError('SealedCapsuleMismatch')
        capsule_policy=SurfaceSizing(**old['configuration'])
        trace=read(SEALED/'IMPLEMENTATION_TRACE.json')
        depbase=V3/'fresh_supported_refinement_family_qualification_2026-09-09'
        dep=read(depbase/'MMGS_DEPENDENCY_QUALIFICATION.json')
        chart=V3/'singular_sector_charts/sector_cdt'
        host=ROOT/'target/debug/canonical_kernel_host.exe'
        binaries={
            host:read(V4/'results/surface_policy_recovery/VERIFIED.json')['host_sha256'],
            chart:read(V4/'results/canonical_polygon_connection/SEED_CONNECTION.json')['cgal_sha256'],
            SEALED/'runtime/seed':trace['native_seed_sha256'],
            SEALED/'runtime/intersections':trace['native_intersections_sha256'],
            V3/'adaptive_surface_architecture/bin/mmgs_metric_contract_runner':dep['adapter_sha256'],
            depbase/'dependency/install/lib/libmmgs.so.5.8.0':dep['library_sha256']}
        if not dep['pass_gate']:raise ValueError('MMGSDependencyUnqualified')
        binary_rows=[]
        for p,h in binaries.items():
            if sha(p)!=h:raise ValueError('BinaryIdentityMismatch:'+str(p))
            if p!=host:
                subprocess.run(['wsl','-e','test','-x',wsl(p)],check=True,timeout=30)
                result=subprocess.run(['wsl','-e','env','LD_LIBRARY_PATH='+wsl(depbase/'dependency/install/lib'),
                    'ldd',wsl(p)],capture_output=True,text=True,timeout=30)
                if result.returncode or 'not found' in result.stdout:raise ValueError('NativeDependencyUnavailable:'+str(p)+result.stdout)
            binary_rows.append(dict(path=str(p.relative_to(ROOT)),sha256=h))
        case=next(c for c in direct['cases'] if c['name']==wing['geometry']['script_case'])
        if case['metres_per_model_unit']!=wing['geometry']['metres_per_model_unit']:
            raise ValueError('WingUnitMismatch')
        with CanonicalKernel([str(host)]) as owner:
            def wing_binding():
                g=owner.load_rhai(case['script'],case['metres_per_model_unit'])
                if g.identity!=wing['geometry']['canonical_identity'] or g.stored_topology()['topology_identity']!=wing['geometry']['topology_identity']:
                    raise ValueError('WingIdentityMismatch')
                b=bind_canonical(g,accuracy_limits=wing['accuracy_limits'])
                return bind_development_policy(b,SurfaceSizing(**wing['sizing_policy']))
            def control_binding():
                a=CapsuleGeometry(definition['definition'])
                if a.geometry_identity!=cap['geometry_identity']:raise ValueError('CapsuleGeometryIdentityMismatch')
                return capsule_binding(a,capsule_policy)
            bindings={'wing':wing_binding(),'capsule':control_binding()}
            admissions={name:compact_admission(b) for name,b in bindings.items()}
            for b in bindings.values():b.storage_budget_bytes=plan['campaign_output_limit_bytes']
            if shutil.disk_usage(out).free<plan['minimum_free_bytes']:raise ValueError('InsufficientCampaignStorage')
            preflight=dict(passed=True,source=source,binaries=binary_rows,admissions=admissions,
                output_locations={name:{mode:str(out/name/mode) for mode in ('fresh','replay')} for name in bindings},
                wing_approved_case_sha256=sha(WING),capsule_case_sha256=sha(CAPSULE),
                original_proposal_sha256=sha(original),wing_script_sha256=identity(case['script']),
                capsule_policy=dataclasses.asdict(capsule_policy),capsule_definition=definition,
                capsule_sealed_identity=sealed['surface_identity'],resources=plan,
                free_c=shutil.disk_usage(out).free,units='m',
                capsule_sign_checks={str(p):bindings['capsule'].authority.implicit_value(1,p) for p in ([0.,0.,0.],[.08,0.,0.])},
                dispatch='ProductionAdaptiveSdfSurfaceMesher.generate -> surface_execution.execute_surface',
                selection='Only polygon and capsule executable SurfaceBindings found in focused entry/factory inspection. Kernel sphere scalar transport is not a complete production binding.')
            save(out/'PREFLIGHT.json',preflight)
            print(json.dumps(dict(stage='both_preflights_passed',cases=list(bindings))),flush=True)
            entry=ProductionAdaptiveSdfSurfaceMesher();results={}
            for name in ('wing','capsule'):
                active=name+'/fresh';b=bindings[name]
                generation=entry.generate(b,out/active)
                active=name+'/replay'
                replay_binding=wing_binding() if name=='wing' else control_binding()
                replay_binding.storage_budget_bytes=plan['campaign_output_limit_bytes']
                entry.generate(replay_binding,out/active)
                comparison=compare(out/name/'fresh',out/name/'replay')
                save(out/(name.upper()+'_REPLAY.json'),comparison)
                if not comparison['passed']:raise ValueError('RequiredReplayMismatch:'+name)
                result=dict(generation=generation,replay_passed=True)
                if name=='capsule':
                    reference=compare(SEALED/'fresh',out/name/'fresh',retained_capsule=True)
                    record={k:generation[k] for k in BOUND_FIELDS if k!='implementation_identity'}
                    record['implementation_identity']=sealed['implementation_identity']
                    reference['reference_bound_scientific_identity']=scientific_identity(record)
                    reference['matches_sealed_identity']=reference['reference_bound_scientific_identity']==sealed['surface_identity']
                    save(out/'CAPSULE_SEALED_COMPARISON.json',reference)
                    if not reference['passed'] or not reference['matches_sealed_identity']:
                        raise ValueError('CapsuleSealedReplayMismatch')
                results[name]=result;completed.append(name)
            # Publication is delayed until BOTH cases and their replays pass.
            active='publication'
            impl=identity(dict(harness=sha(__file__),sources=wing['source_files']))
            for name in ('wing','capsule'):
                results[name]['published']=publish_surface(out/name/'fresh',results[name]['generation'],impl,
                    read(out/(name.upper()+'_REPLAY.json')))
            save(out/'RESULT.json',dict(classification='V4TwoGeometrySurfaceIntegrationQualifiedForTestedScope',
                passed=True,cases=results,seconds=time.perf_counter()-start,
                peak_working_set_bytes=psutil.Process().memory_info().peak_wset,
                host_peak_working_set_bytes=psutil.Process(owner.transport.process.pid).memory_info().peak_wset))
        return 0
    except Exception as error:
        record=dict(classification='V4TwoGeometrySurfaceIntegrationStopped',passed=False,
            active=active,completed=completed,second_case_started=(out/'capsule/fresh').exists(),
            error_type=type(error).__name__,error=str(error),
            structured_failure=getattr(error,'record',None),traceback=traceback.format_exc(),
            seconds=time.perf_counter()-start,peak_working_set_bytes=psutil.Process().memory_info().peak_wset,
            no_retry=True,no_publication=True)
        save(out/'STOP.json',record);print(json.dumps(record),flush=True)
        return 1


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--worker',action='store_true');args=parser.parse_args();out=args.output.resolve()
    if not out.is_relative_to((V4/'results').resolve()) or out.is_relative_to(SEALED.resolve()):
        raise ValueError('TaskOwnedNonsealedOutputRequired')
    if args.worker:return worker(out)
    if out.exists():raise ValueError('OutputMustBeNew; stopped campaigns are never restarted')
    out.mkdir(parents=True);plan=resource_plan();save(out/'RESOURCE_PLAN.json',plan)
    start=time.perf_counter();samples=[];breach=None
    with (out/'execution.log').open('w',newline='\n') as log:
        child=subprocess.Popen([sys.executable,'-B',__file__,'--worker','--output',str(out)],stdout=log,stderr=subprocess.STDOUT)
        while child.poll() is None:
            used=size(out);free=shutil.disk_usage(out).free
            try:memory=psutil.Process(child.pid).memory_info().peak_wset
            except psutil.NoSuchProcess:memory=None
            samples.append(dict(seconds=time.perf_counter()-start,bytes=used,free_bytes=free,peak_working_set_bytes=memory))
            if used>plan['campaign_output_limit_bytes'] or free<plan['minimum_free_bytes']:
                breach=samples[-1]
                # Only this newly created task's native command lines match.
                subprocess.run(['wsl','-e','pkill','-TERM','-f',re.escape(wsl(out))],timeout=15)
                proc=psutil.Process(child.pid)
                for descendant in proc.children(recursive=True):
                    try:descendant.terminate()
                    except psutil.NoSuchProcess:pass
                child.terminate();child.wait(timeout=15)
                save(out/'STOP.json',dict(classification='V4TwoGeometrySurfaceIntegrationStopped',
                    active='resource_guard',error='ResourceLimitBreach',measurement=breach,no_retry=True,
                    second_case_started=(out/'capsule/fresh').exists()))
                break
            time.sleep(plan['monitoring_seconds'])
    save(out/'MONITOR.json',dict(seconds=time.perf_counter()-start,exit_code=child.returncode,
        resource_breach=breach,samples=samples,retained_bytes=size(out),final_free_bytes=shutil.disk_usage(out).free))
    save(out/'MANIFEST.json',dict(files=inventory(out),scope='Before manifest itself; scientific outputs retained separately from compact Git evidence.'))
    print(json.dumps(dict(output=str(out),exit_code=child.returncode,stopped=(out/'STOP.json').exists(),
        final_logical_bytes=size(out),final_free_bytes=shutil.disk_usage(out).free)),flush=True)
    return child.returncode


if __name__=='__main__':raise SystemExit(main())
