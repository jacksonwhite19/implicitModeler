"""Read-only domain-authority gate. A shell is not a bounded external domain.

This task stops before handoff construction if the production authority closure
does not declare the remaining domain. Historical study envelopes are evidence,
not an authorization to select a new production boundary.
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import sqlite3
import sys
from unittest.mock import patch

sys.dont_write_bytecode = True

from composable_state import CommittedState
from clean_sheet_l4_bootstrap import SURFACE, REQUEST
from qualify_recursive_layer6 import measured, metrics, denied
from path_preserving_deduplication import durable_json, file_hash
from production_surface_authority import load_production_wall
from production_prism_sizing import read_request
from sdf_cfd_mesher_v4.common.authority import identity
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from verify_recursive_layers9to11 import completion

V4 = Path(__file__).resolve().parents[1]
REPO = V4.parents[1]
SEALED = V4/'results/recursive_layers9to11'
OUT = V4/'results/sealed_l11_core_handoff'
EXPECTED = dict(
    authority_sha256='b7ea029f1f091f0bde2644949cbdf9a63d44259c01e712f5d1957a475343484d',
    identity='356e5a357bbbdf923f7dd3a2df0642632ba3497518b0305c44f372d6fec7f9b3',
    qualification='75d827a837748dfca5014b873a18f4076843752ef6789622565f6de6d2ba8e7b',
    scientific='48f39ddfae1f782f724321e0d8882f9454928c5c0c2898652220dcfa6db3f1a1',
    content='6b11b14c7b519467ad49a126261f0ddd608692d82ca387160fcc1b7ab10c5870',
    access='28ac22c32c78e5e890969dccc87c901c6e7e39b16f64140fe52e427185b20e64',
    checkpoint='7ec1858d40685831aab67c668c10a3b6cb5b07e292b6d2b36c6ac4b58a1bdbc7',
    surface='14770350129d217662b9e936b2b2bd39d9ea3e14b47ad7fedc9e47425da1aafd',
    geometry='a52676253aa7996b6ac4d453f40ae838aa9d569d883b8fa64fe6799cd22cbc65',
    request='ec28d584b04afb8bd4679d6719d0f600140b63dcff26a0cc50c2607537d33b91',
    schedule='ae2cfd505b0cb52218c4f000688df150e8f2c65f2d2d28de78785f1de39f0597')


def read(path):
    return json.loads(Path(path).read_text())


def require_equal(actual, expected, label):
    if actual != expected:
        raise ValueError('AuthorityMismatch:'+label)


def verify_record(record):
    require_equal(identity({k:v for k,v in record.items() if k != 'identity'}),
                  record['identity'], 'record')


def require_domain(authority):
    # This gate intentionally does not define a speculative domain schema.
    if authority is None:
        raise ValueError('MissingProductionCoreDomainAuthority')
    raise ValueError('DomainAuthorityNeedsSeparateQualification')


def state_domain_metadata():
    """Fresh-process view of the compact native authority and current outer set."""
    authority = read(SEALED/'AUTHORITY.json')
    require_equal(file_hash(SEALED/'AUTHORITY.json'),EXPECTED['authority_sha256'],'authority_file')
    tick = measured()
    with patch.object(sqlite3,'connect',denied), CommittedState(authority['reference']) as state:
        result = dict(authority=dict(state['authority']),expected=state.expected,
            outer_faces=len(state['outer']),
            outer_is_exact_prism_front=set(state['outer']) == set(state['prism_front'].values()),
            metrics=metrics(*tick))
        state.check()
    durable_json(OUT/'STATE_DOMAIN_METADATA.json',result)
    print(json.dumps(result,indent=2))


def v3_inventory():
    root = REPO/'research/sdf_cfd_mesher_v3'
    digest = hashlib.sha256()
    count = reparse = 0
    def visit(directory):
        nonlocal count, reparse
        for p in sorted(directory.iterdir(), key=lambda p:p.name):
            st = p.lstat()
            link = bool(st.st_file_attributes & 0x400)
            count += 1
            reparse += int(link)
            row = [str(p.relative_to(root)), st.st_size, st.st_mtime_ns, st.st_file_attributes]
            digest.update((json.dumps(row,separators=(',',':'))+'\n').encode())
            if not link and p.is_dir():
                visit(p)
    visit(root)
    return dict(entries=count,reparse=reparse,metadata_sha256=digest.hexdigest())


def run():
    tick = measured()
    before = v3_inventory()
    durable_json(OUT/'V3_BEFORE.json',before)
    authority = read(SEALED/'AUTHORITY.json')
    require_equal(file_hash(SEALED/'AUTHORITY.json'),EXPECTED['authority_sha256'],'authority_file')
    verify_record(authority)
    for key in ('identity','qualification','scientific','content'):
        require_equal(authority[key],EXPECTED[key],key)
    require_equal(authority['authoritative_layer'],11,'layer')
    require_equal(authority['reference']['root'],EXPECTED['access'],'access')
    checkpoint = Path(authority['reference']['directory'])/('ROOT_'+EXPECTED['access']+'.json')
    require_equal(file_hash(checkpoint),EXPECTED['checkpoint'],'checkpoint')
    protected = {str(SEALED/'AUTHORITY.json'):EXPECTED['authority_sha256'],
                 str(checkpoint):EXPECTED['checkpoint']}
    for name, digest in authority['files'].items():
        p = SEALED/name
        require_equal(file_hash(p),digest,'sealed_evidence:'+name)
        protected[str(p)] = digest
    wall, triangles, surface = load_production_wall(SURFACE)
    bound = read_request(REQUEST,surface)
    require_equal(surface['surface_identity'],EXPECTED['surface'],'surface')
    require_equal(surface['geometry_identity'],EXPECTED['geometry'],'geometry')
    require_equal(identity(bound['declaration']),EXPECTED['request'],'request')
    require_equal(bound['schedule_identity'],EXPECTED['schedule'],'schedule')
    source_paths = [REQUEST,SURFACE/'SURFACE_AUTHORITY.json',SURFACE.parent/'GEOMETRY.json']
    source_paths += [(REQUEST.parent/r['path']).resolve() for r in bound['declaration']['source_documents']]
    sources = {str(p):dict(sha256=file_hash(p),top_level_fields=sorted(read(p))) for p in source_paths}
    open_tick = measured()
    with patch.object(os,'link',denied), patch.object(sqlite3,'connect',denied), \
            patch.object(NativeCheckpointStore,'restart',denied), \
            CommittedState(authority['reference']) as state:
        require_equal(state.content_root,EXPECTED['content'],'trusted_content')
        require_equal(state.expected['geometry_identity'],EXPECTED['geometry'],'trusted_geometry')
        require_equal(state.expected['surface_identity'],EXPECTED['surface'],'trusted_surface')
        require_equal(state.expected['request'],EXPECTED['request'],'trusted_request')
        require_equal(state.expected['schedule'],EXPECTED['schedule'],'trusted_schedule')
        opened = metrics(*open_tick)
        extraction_tick = measured()
        front = completion(state)
        front['topology_identity'] = surface['topology_hash']
        extracted = metrics(*extraction_tick)
        semantic_components = sorted(state.components)
        stats = dict(state.stats)
        state.check()
    # These are inspected as text/JSON only. Never import a core runner: setup
    # in those runners leads directly to gmsh mesh.generate(3).
    trace = {
        V4/'tools/broader_geometry_core.py':
            'Explicit study-only envelope; exterior branch constructs min/max +/- span, not a production boundary authority.',
        V4/'tools/generate_native_interface_core.py':
            'Unconditionally selects V3 SF2 wing support and sizing constants; no binding to this capsule request.',
        V4/'src/sdf_cfd_mesher_v4/core/interface.py':
            'Extracts shell-owned outer faces only; supplies no outer core-domain boundary.',
        REPO/'research/sdf_cfd_mesher_v3/fresh_supported_refinement_family_qualification_2026-09-09/SF2/A/farfield/CANDIDATE_BINDING.json':
            'Historical wing SF2 reference, original_cap names wing; not adopted or bound by capsule production authority.'}
    evidence = []
    for p, finding in trace.items():
        text = p.read_text()
        evidence.append(dict(path=str(p.relative_to(REPO)),sha256=file_hash(p),finding=finding,
                             lines=[dict(number=i,line=line) for i,line in enumerate(text.splitlines(),1)
                                    if any(word in line for word in ('production CFD','lo=x.min','reference =','original_cap',
                                                                     'generate(3)','CoreSizeCallback','state[\'outer\']'))]))
    require_equal(bound['declaration']['fluid_side'],'positive_sdf','fluid_side')
    # Domain recovery result: no adopted bounded domain occurs in the sealed
    # authority/request closure. Do not promote unrelated historical artifacts.
    try:
        require_domain(None)
    except ValueError as exc:
        blocker = str(exc)
    for p,digest in protected.items():
        require_equal(file_hash(p),digest,'protected_after:'+p)
    after = v3_inventory()
    require_equal(after,before,'V3_unchanged')
    result = dict(schema='V4BoundedCoreDomainRecoveryAudit/1',
        classification='V4BoundedCoreHandoffBlockedMissingProductionCoreDomainAuthority',
        qualified=False,blocker=blocker,authoritative_prism_layer=11,
        verified_authorities=EXPECTED,protected_files=protected,source_documents=sources,
        current_front_binding=front,semantic_components=semantic_components,
        core_domain=dict(fluid_side='positive_sdf',inner_boundary='sealed L11 front',
            outer_boundary=None,auxiliary_boundaries=None,boundary_tags=None,
            expected_components=None,cavities_exclusions=None,bounded_volume=None,
            finding='Positive-SDF exterior is unbounded without an adopted outer boundary. No complete bounded core domain recovered.'),
        historical_trace=evidence,stopped_at='Required domain-authority gate before handoff construction',
        not_performed=['new manifold qualification','per-face core-side orientation qualification',
            'complete domain boundary assembly','cross-boundary collision/clearance',
            'terminal metric category audit','handoff construction','handoff reopen','handoff negative tests'],
        handoff=None,core_start_authorized=False,
        generated=dict(core_vertices=0,core_interior_points=0,tetrahedra=0,core_cells=0,extra_prism_layers=0),
        normal_path=dict(trusted_open=opened,front_binding=extracted,stats=stats,
            historical_proof_rows=0,historical_quality_rows=0,sqlite_rows=0,
            full_prefix_scientific_hashes=0,deep_copies=0,attested_hardlinks=0),
        V3=dict(before=before,after=after,changes=0),metrics=metrics(*tick),
        disk=dict(start=read(OUT/'PREFLIGHT.json')['free'],end=shutil.disk_usage(V4).free),
        recommended_next_task='RecoverAndQualifyV4CapsuleCoreDomainBoundaryAuthority')
    result['identity'] = identity(result)
    durable_json(OUT/'REPORT.json',result)
    print(json.dumps({k:result[k] for k in ('classification','blocker','current_front_binding','normal_path','metrics','identity')},indent=2))


if __name__ == '__main__':
    if len(sys.argv) == 2 and sys.argv[1] == 'metadata':
        state_domain_metadata()
    else:
        run()
