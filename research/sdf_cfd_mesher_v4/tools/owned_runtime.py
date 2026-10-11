"""Locations and integrity checks for the single V4-owned runtime.

No geometry/case/result fallback is permitted here. Origins in MANIFEST are
provenance only. The installed copies are the executed dependencies.
"""
import hashlib
import json
import sys
from pathlib import Path

V4 = Path(__file__).resolve().parents[1]
RUNTIME = V4 / 'runtime'
SURFACE = RUNTIME / 'surface'
SOURCE = RUNTIME / 'source'
BIN = RUNTIME / 'bin'
LIB = RUNTIME / 'lib'


def activate():
    paths = (SURFACE, V4/'src/sdf_cfd_mesher_v4/sdf/_frozen',
             V4/'src/sdf_cfd_mesher_v4/legacy_full_stack_planner/_support')
    for path in reversed(paths):
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)


def verify():
    manifest = json.loads((RUNTIME/'MANIFEST.json').read_text(encoding='utf-8'))
    if manifest.get('schema') != 'V4OwnedRuntime/1':
        raise ValueError('UnsupportedOwnedRuntime')
    for row in manifest['files']:
        path = (V4/row['path']).resolve()
        if not path.is_relative_to(V4) or not path.is_file():
            raise ValueError('OwnedRuntimePathMissing:'+row['path'])
        with path.open('rb') as stream:
            actual = hashlib.file_digest(stream, 'sha256').hexdigest()
        if actual != row['sha256']:
            raise ValueError('OwnedRuntimeIdentityMismatch:'+row['path'])
    return manifest


def release_inventory():
    """Conservative owned code/binary closure, including shared legacy owners.

    An extra owned source is pinned even if not executed. Tests, results and
    provenance origins are not execution inputs. The release cannot omit files.
    """
    files={V4/'mesh.py',V4/'pyproject.toml',RUNTIME/'MANIFEST.json',RUNTIME/'MMGS_DEPENDENCY_QUALIFICATION.json'}
    for directory in (V4/'tools',V4/'src',RUNTIME):
        for path in directory.rglob('*'):
            if path.is_file() and path.name!='RELEASE.json' and path.suffix in ('.py','.rs','.cpp','.dll','.exe','.so','.pyd','.json'):
                files.add(path)
    for directory in (BIN,LIB):files.update(p for p in directory.rglob('*') if p.is_file())
    return [dict(path=p.relative_to(V4).as_posix(),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
            for p in sorted(files)]


def verify_release():
    path=RUNTIME/'RELEASE.json'
    if not path.exists():raise ValueError('StandaloneReleaseReviewRequired')
    release=json.loads(path.read_text(encoding='utf-8'))
    if (release.get('schema')!='V4StandaloneRelease/1' or release.get('review_passed') is not True
            or not release.get('independent_reviews')):raise ValueError('StandaloneReleaseReviewRequired')
    if release['sources']!=release_inventory():raise ValueError('ChangedStandaloneRelease')
    for row in release['independent_reviews']:
        p=(V4/row['path']).resolve()
        if not p.is_relative_to(V4) or hashlib.sha256(p.read_bytes()).hexdigest()!=row['sha256']:
            raise ValueError('ChangedStandaloneReview')
        review=json.loads(p.read_text(encoding='utf-8'))
        from sdf_cfd_mesher_v4.common.authority import identity
        if (review.get('schema')!='V4IndependentReview/1' or review.get('passed') is not True
                or review.get('reviewer') not in release['reviewers']
                or review.get('source_inventory_identity')!=identity(release['sources'])):
            raise ValueError('StandaloneReviewScopeMismatch')
        if 'retained_initial_revalidation' in release:
            scope=dict(changes=release['retained_initial_revalidation'],
                       prior_release_sha256=release['retained_initial_prior_release_sha256'])
            if review.get('retained_initial_revalidation_identity')!=identity(scope):
                raise ValueError('UnreviewedRetainedInitialRevalidation')
    verify()
    return release


activate()
