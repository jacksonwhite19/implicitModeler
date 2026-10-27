"""Remove only task-local byte duplicates after writing replacement evidence."""
from pathlib import Path
import hashlib,json
def digest(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def remove_duplicate(root,path,replacement,manifest,pinned=()):
    root=Path(root).resolve();path=Path(path).resolve();replacement=Path(replacement).resolve()
    if root not in path.parents or root not in replacement.parents or path==replacement:raise ValueError('OutsideTaskOrSameArtifact')
    if path in {Path(p).resolve() for p in pinned}:raise ValueError('ReferencedArtifact')
    if not replacement.is_file() or path.stat().st_size!=replacement.stat().st_size or digest(path)!=digest(replacement):raise ValueError('NoExactReplacement')
    row=dict(path=str(path.relative_to(root)),replacement=str(replacement.relative_to(root)),bytes=path.stat().st_size,sha256=digest(path),reason='Exact duplicate candidate array; quality/failure records and canonical replacement retained')
    manifest=Path(manifest);rows=json.loads(manifest.read_text()) if manifest.exists() else []
    rows.append(row);manifest.write_text(json.dumps(rows,indent=2)+'\n')
    path.unlink();return row
