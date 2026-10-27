"""Add the current SF2 bridge after resolving its exact source ownership."""
from prepare_source_fork import ROOT, REPO, H, PKG, sha, put
import json, shutil
src=H/'smooth_authority.py'
dst=PKG/'sdf/_frozen/current_smooth_authority.py'
r=dict(original_path=src.relative_to(REPO).as_posix(),destination=dst.relative_to(ROOT).as_posix(),original_sha256=sha(src),size_bytes=src.stat().st_size,role='current SF2 SDF geometry access',reason='Current SF2 study owns this bridge; earlier bridge retained as dependency provenance',decision='copy',source_owner='V3')
plan=json.loads((ROOT/'authorities/SOURCE_COPY_PLAN.json').read_text())
plan['candidates'].append(r.copy())
put(ROOT/'authorities/SOURCE_COPY_PLAN.json',plan)
assert not dst.exists()
assert sha(src)==r['original_sha256']
shutil.copyfile(src,dst)
r.update(copied_sha256=sha(dst),byte_identical=sha(dst)==r['original_sha256'])
assert r['byte_identical']
manifest=json.loads((ROOT/'authorities/V3_SOURCE_FORK_MANIFEST.json').read_text())
manifest['files'].append(r)
put(ROOT/'authorities/V3_SOURCE_FORK_MANIFEST.json',manifest)
print('Added current SF2 geometry bridge; byte-identical')
