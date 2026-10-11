"""Read-only input/storage inventory for the fresh production surface study."""
import hashlib, json, os, shutil, subprocess, time
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
V4 = REPO / 'research/sdf_cfd_mesher_v4'
V3 = REPO / 'research/sdf_cfd_mesher_v3'
OUT = V4 / 'results/production_adaptive_capsule_surface_foundation'

def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''): h.update(b)
    return h.hexdigest()

def save(p, value):
    Path(p).parent.mkdir(parents=True, exist_ok=True)
    Path(p).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    if (OUT/'PREFLIGHT.json').exists(): raise RuntimeError('PreflightAlreadyRecorded')
    refs = [V3/'adaptive_surface_generic_qualification',
            V3/'adaptive_surface_architecture/src',
            V3/'adaptive_surface_architecture/mmg_install',
            V3/'held_out_geometry_cohort/src',
            V3/'f2_trailing_core_coordinate_qualification_2026-09-07/smooth_authority.py',
            V3/'f2_trailing_core_coordinate_qualification_2026-09-07/run_surface.py',
            V3/'f2_trailing_core_coordinate_qualification_2026-09-07/resolution.py',
            V3/'fresh_supported_refinement_family_qualification_2026-09-09/SF2/A/wing/FROZEN_SURFACE_CALL.json',
            V3/'fresh_supported_refinement_family_qualification_2026-09-09/SF2/A/wing/surface/A']
    inputs = []
    for root in refs:
        for p in ([root] if root.is_file() else root.rglob('*')):
            if p.is_file() and '__pycache__' not in str(p):
                inputs.append(dict(path=p.relative_to(REPO).as_posix(), bytes=p.stat().st_size, sha256=sha(p)))
    total = 0; largest = []; inaccessible = []
    for root, dirs, files in os.walk(V4/'results'):
        for name in files:
            p = Path(root)/name
            try: size=p.stat().st_size
            except OSError as exc:
                inaccessible.append(dict(path=str(p),error=str(exc))); continue
            total += size
            largest.append((size,p.relative_to(REPO).as_posix()))
    status = subprocess.check_output(['git','status','--short'], cwd=REPO, text=True).splitlines()
    result = dict(started_epoch=time.time(), starting_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
        branch=subprocess.check_output(['git','branch','--show-current'],cwd=REPO,text=True).strip(),
        starting_free_bytes=shutil.disk_usage(REPO).free, v4_result_tree_bytes=total,
        v3_reference_bytes=sum(v['bytes'] for v in inputs), reference_files=inputs,
        largest_inputs=sorted(inputs,key=lambda r:r['bytes'],reverse=True)[:20],
        largest_results=sorted(largest,reverse=True)[:20], inaccessible_entries=inaccessible, expected_scratch='Not measured.',
        dirty_state=status, unrelated_dirty_state=[s for s in status if 'research/sdf_cfd_mesher_v4' not in s])
    save(OUT/'PREFLIGHT.json',result)
    save(OUT/'CLEANUP.json',dict(pre_run_deleted=[],bytes_deleted=0,reason='No disposable task artifacts existed. Accepted authorities and old crash dumps are preserved.'))
    print(json.dumps({k:result[k] for k in ['starting_head','branch','starting_free_bytes','v4_result_tree_bytes','v3_reference_bytes','largest_inputs','expected_scratch']},indent=2))

if __name__ == '__main__': main()
