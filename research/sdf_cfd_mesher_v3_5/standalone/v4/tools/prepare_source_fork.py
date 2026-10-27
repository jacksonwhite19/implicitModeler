"""One-time setup inventory and byte-preserving copy. No scientific execution."""
from pathlib import Path
import hashlib, json, subprocess, shutil, datetime, sys

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
V3 = REPO / 'research/sdf_cfd_mesher_v3'
H = V3 / 'fresh_supported_refinement_family_qualification_2026-09-09'
G = H / 'scoped_operator_production_numerical_counterfactual_20260911/fixed_flux_velocity_reconstruction_20260912/prism_geometry_common_field_20260912'
P = G / 'two_way_trust_region_development_20260915'
PKG = ROOT / 'src/sdf_cfd_mesher_v4'
SUP = PKG / 'legacy_full_stack_planner/_support'

def sha(p):
    with p.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def put(p, value):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(value, indent=2, sort_keys=True)+'\n', encoding='utf-8')

def candidates():
    rows=[]
    def add(src,dest,role,reason='Frozen scientific dependency; no behavior changes'):
        rows.append(dict(original_path=src.relative_to(REPO).as_posix(), destination=dest.relative_to(ROOT).as_posix(), original_sha256=sha(src), size_bytes=src.stat().st_size, role=role, reason=reason, decision='copy', source_owner='V3' if src.is_relative_to(V3) else 'production SDF dependency'))
    cert=V3/'continuous_prism_certificate'
    for p in sorted(cert.rglob('*')):
        if p.is_file() and p.suffix in ('.py','.json','.md'):
            add(p,PKG/'certificate/continuous_prism_certificate'/p.relative_to(cert),'continuous certificate')
    for name in ['common','accepted_inputs','component_graph','component_quality','component_compression','component_planner','diagnostic_convergence','linear_solve','trust_region','two_way_planner']:
        add(P/(name+'.py'),PKG/'legacy_full_stack_planner/_frozen'/(name+'.py'),'legacy '+name)
    for rel,role in [
        ('foundation_aware_prism_shell_qualification/foundation_geometry.py','quality'),
        ('prism_feasibility_contract/prospective_commit_adapter.py','quality predicates'),
        ('sdf_guided_advancing_front_qualification/accepted_front_state.py','front data and lineage'),
        ('complete_short_wing_continuation_2026-09-06/continuation_model.py','symbolic compression dependency'),
        ('finite_wing_terminal_prism_feasibility_2026-09-06/symbolic_prism.py','symbolic quality dependency')]:
        add(V3/rel,SUP/rel,role)
    for rel,role in [('prism_placement_counterfactual_20260913/geometry.py','weighted sigma and quality'),('prism_placement_sensitivity_audit_20260913/audit.py','sigma derivative dependency')]:
        add(G/rel,SUP/'context'/rel,role)
    for name in ['operator_common.py','operator_model.py']:
        add(G/'multi_field_reconstruction_operator_calibration_20260915'/name,PKG/'conditioning/_frozen'/name,'singular spectrum and constrained gain')
    for rel,role in [('smooth_naca_sdf_qualification_2026-09-06/smooth_authority.py','SDF geometry access'),('smooth_naca_sdf_qualification_2026-09-06/spline_bridge.rs','SDF native bridge'),('adaptive_surface_generic_qualification/procedural_authorities.py','surface identities and geometry interface')]:
        add(V3/rel,PKG/'sdf/_frozen'/Path(rel).name,role)
    add(REPO/'src/sdf/aerospace/naca_spline.rs',PKG/'sdf/_frozen/naca_spline.rs','production SDF evaluation','Required production SmoothSpline mathematics; source snapshot only, no Rust build')
    return rows

if __name__ == '__main__':
    mode=sys.argv[1]
    if mode=='plan':
        rows=candidates()
        assert all(r['size_bytes']<=100_000_000 for r in rows)
        put(ROOT/'authorities/SOURCE_COPY_PLAN.json',dict(authorization='User requested source-only setup copy; individually inspected dependencies',candidates=rows))
        status=subprocess.check_output(['git','status','--porcelain=v1'],cwd=REPO,text=True)
        put(ROOT/'authorities/START_STATE.json',dict(initial_branch='main',base_head='766c61c579f020f692db73c334152005ed09103d',branch=subprocess.check_output(['git','branch','--show-current'],cwd=REPO,text=True).strip(),status_porcelain=status,starting_worktree_dirty=True,setup_started_at='2026-09-16T11:42:21Z',snapshot_note='Recorded after branch creation and this V4 setup script; untracked research/ already existed at task entry. Original tracked and unrelated status unchanged.'))
        print(json.dumps(dict(candidates=len(rows),bytes=sum(r['size_bytes'] for r in rows),largest=max(r['size_bytes'] for r in rows))))
    elif mode=='copy':
        rows=json.loads((ROOT/'authorities/SOURCE_COPY_PLAN.json').read_text())['candidates']
        for r in rows:
            src=REPO/r['original_path'];dst=ROOT/r['destination']
            assert sha(src)==r['original_sha256'],src
            assert not dst.exists(),dst
            dst.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(src,dst)
            r['copied_sha256']=sha(dst);r['byte_identical']=r['copied_sha256']==r['original_sha256']
            assert r['byte_identical']
        put(ROOT/'authorities/V3_SOURCE_FORK_MANIFEST.json',dict(schema='V4.SourceFork/1',files=rows,unexplained_mismatches=0))
        print('Copied',len(rows),'files; zero mismatches')
    else:raise SystemExit('Use plan or copy')
