"""Compact complete-mesh authority. No generation or solver entry points."""
from contextlib import ExitStack
from pathlib import Path
import gzip
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import numpy as np
import capsule_user_domain_handoff as h
import capsule_c1_publication as core
import immutable_state_access as access
import recursive_epoch_recovery as renewal
from incremental_transaction import Store

V4 = Path(__file__).resolve().parents[1]
RESULTS = V4/'results'
CORE = RESULTS/'capsule_c1_core_qualification'
OUT = RESULTS/'capsule_complete_mesh_authority'
CLASS = 'V4ProductionAdaptiveCapsuleCompleteMeshQualified'
SCI = 'a1e9d5906a7a723ec9906c938a52a25921e23d867aa9f22a489162cba9770b4f'
CONTENT = 'bbc503c3a04dd5d4f5083d9237d20f51dafbc4578b437b1364c01ff4de5f5c81'
ACCESS = 'a2d8e3d3722fa3305cc5b23e87cd68120a70d79cdd732ef0d6c02a1dc7b566ba'
CHECKPOINT = 'cb2061add4c1e4150126d15c48135161f66fe651c62c3bc8bd42d8cf104c01ac'
CORE_ID = 'ed9075a03ccc08315c25a7a35f6d99f184703bd7283e52da00d562d53b3f9fed'
CERT = '9ce3df428c1784391429ebf18a3385429fde0b679c0fc85b32ec9499ac715c1a'
EXCLUSIONS = ['CFD solver configuration', 'turbulence-model suitability', 'physical boundary conditions',
    'iterative convergence', 'numerical error', 'mesh-resolution convergence', 'domain independence',
    'force/pressure accuracy', 'experimental CFD validation']


def contract():
    return h.seal(dict(schema='V4CompleteMeshAcceptance/1', classification=CLASS,
        hard_gates=dict(volume='>0', exact_jacobian='>0', determinant_min=0.0011,
        interpolation_weight_min=0.055, skewness_max=3.63636, symmetric_volume_ratio_min=0.01,
        aspect_ratio_max=1000, forbidden=0, unresolved=0, missing=0, duplicate=0, unused=0),
        obligations=['sealed geometry', 'valid cell and face topology', 'one connected fluid region',
        'valid owner/neighbour mapping', 'exact unsplit interface without Steiner points',
        'complete continuous coverage', 'qualified volume/boundary balance', 'exact domain and sizing bindings',
        'exact OpenFOAM conversion', 'Foundation 13 Mesh OK.', 'deterministic scientific identity'],
        diagnostics_only=['sigma', 'G', 'non-orthogonality'], unqualified=EXCLUSIONS))


def checked(path, expected=None):
    value = h.read(path)
    h.verify_record(value)
    if expected is not None:
        h.eq(value['identity'], expected, 'WrongAuthority:'+str(path))
    return value


def verify_files(files):
    for name, digest in files.items():
        h.eq(h.file_hash(V4/name), digest, 'ChangedEvidence:'+name)


def foam_count(path):
    with path.open() as stream:
        for line in stream:
            if re.fullmatch(r'\s*\d+\s*', line):
                return int(line)
    raise ValueError('MissingFoamCount')


def source_snapshot():
    """Reconstruct compact claims from authenticated sealed records, not prompt values."""
    files = {}; historical_sources = {}
    def bind(path, digest=None):
        path = Path(path).resolve()
        actual = h.file_hash(path)
        if digest is not None:
            h.eq(actual, digest, 'ChangedEvidence:'+str(path))
        files[os.path.relpath(path,V4)] = actual
    qualification = checked(CORE/'AUTHORITY.json', CORE_ID)
    bind(CORE/'AUTHORITY.json')
    for field in ('evidence_hashes', 'reports'):
        for name, digest in qualification[field].items():
            bind(CORE/name, digest)
    original = checked(CORE/'COMPLETE_AUTHORITY.json')
    h.eq([original[k] for k in ('scientific','content','access','checkpoint')],
         [SCI,CONTENT,ACCESS,CHECKPOINT], 'WrongCompleteMesh')
    bind(CORE/'COMPLETE_AUTHORITY.json')
    store = Store(CORE/'complete_mesh')
    body = store.root(ACCESS)
    bind(store.path/('ROOT_'+ACCESS+'.json'), CHECKPOINT)
    h.eq(h.identity(body['plan']['components']), CONTENT, 'WrongContent')
    for name, digest in body['plan']['components'].items():
        bind(CORE/name, digest)
    for name, record in body['produced'].items():
        bind(Path(name), record['sha256'])
    for name, digest in qualification['implementation'].items():
        bind(V4/name if '/' in name or '\\' in name else V4/'tools'/name, digest)
    l11, deep, _, _, _ = h.verify_sources()
    bind(h.SEALED/'AUTHORITY.json', h.EXPECTED['authority_sha256'])
    for name in ('FINAL_DEEP.json','FINAL_REPLAY.json','REPORT.json','AUTHORITIES_VERIFIED.json'):
        bind(h.SEALED/name, l11['files'][name])
    for path in (h.SURFACE/'SURFACE_AUTHORITY.json', h.REQUEST, h.SURFACE.parent/'GEOMETRY.json'):
        bind(path)
    handoff_authority = checked(h.OUT/'AUTHORITY.json', '55e31c4497e0629b7951f90347a555fbb3510755c00d531d0fe4d6bdf9fe44c5')
    bind(h.OUT/'AUTHORITY.json')
    for name, digest in handoff_authority['files'].items():
        if name.startswith(('tests', 'tools')) and h.file_hash(V4/name)!=digest:
            repo_path='research/sdf_cfd_mesher_v4/'+name.replace(chr(92),'/')
            commits=subprocess.check_output(['git','rev-list','HEAD','--',repo_path],text=True).splitlines()
            for commit in commits:
                blob=subprocess.check_output(['git','show',commit+':'+repo_path])
                if hashlib.sha256(blob).hexdigest()==digest:
                    historical_sources[name]=dict(commit=commit,sha256=digest); break
            else: raise ValueError('MissingHistoricalSource:'+name)
        else: bind(V4/name, digest)
    handoff = checked(h.OUT/'HANDOFF.json', handoff_authority['handoff'])
    domain = checked(h.OUT/'DOMAIN.json', handoff_authority['domain'])
    generic = checked(RESULTS/'generic_core_sizing/AUTHORITY.json', '77c1c794f90eb913c351f40c09ba68cd3aae79600080195dc64f02c9e64472c7')
    bind(RESULTS/'generic_core_sizing/AUTHORITY.json')
    qb = h.read(CORE/'QUALIFICATION_BINDINGS.json')
    bind(CORE/'QUALIFICATION_BINDINGS.json')
    generic_qualification=checked(RESULTS/'generic_core_sizing/QUALIFICATION.json',qb['generic_core_sizing']['identity'])
    bind(RESULTS/'generic_core_sizing/QUALIFICATION.json',qb['generic_core_sizing']['sha256'])
    for name,digest in generic['files'].items():
        path=(RESULTS/'generic_core_sizing'/name).resolve()
        if 'results' in path.parts: bind(path,digest)
    h.eq(qb['generic_core_sizing']['identity'], '228433a2031a6ce26867228ff84139f520379aec34a448641e74572f87f7dc40', 'WrongGenericQualification')
    sizing = core.gates(CORE)
    h.eq(sizing['identity'], '22b38117b2fcf6ab57ed51e2af26bcedad6bf161ff2116959844c4d3e58782f8', 'WrongSizing')
    arrays = {n:np.load(CORE/'assembly'/f'{n}.npy', mmap_mode='r') for n in ('coordinates','prisms','tets','farfield')}
    counts = dict(vertices=len(arrays['coordinates']), prisms=len(arrays['prisms']), tetrahedra=len(arrays['tets']),
                  cells=len(arrays['prisms'])+len(arrays['tets']), outer_triangles=len(arrays['farfield']))
    h.eq(counts, dict(vertices=227901,prisms=371646,tetrahedra=190387,cells=562033,outer_triangles=5182), 'WrongCounts')
    front = np.load(h.OUT/'FRONT.npy')
    h.need(np.array_equal(arrays['coordinates'][11*16895:12*16895], front), 'ChangedInterfaceCoordinates')
    patch = h.read(CORE/'CORE_INTERFACE_BINDINGS.json')
    rows = h.read(h.OUT/'INTERFACE.json')
    owners = np.load(CORE/'C1/FINAL_CORE_OWNERS.npy')
    expected_patch = {r['face']:dict(prism_owner=r['prism_owner'], core_neighbour='core:'+str(int(owner)),
                      vertices=r['vertices'], opposite_winding=True) for r,owner in zip(rows,owners)}
    h.eq(patch, expected_patch, 'ChangedInterfaceMapping')
    h.eq(len(patch), 33786, 'WrongInterfacePopulation')
    bind(CORE/'C1/FINAL_CORE_OWNERS.npy')
    assembly = h.read(CORE/'ASSEMBLY.json'); bind(CORE/'ASSEMBLY.json')
    for name,digest in assembly['hashes'].items():
        h.eq(h.array_hash(np.load(CORE/'assembly'/f'{name}.npy',mmap_mode='r')),digest,'ChangedCanonicalArray:'+name)
    scientific = h.identity(dict(schema='V4CapsuleCompleteCore/1',parent=h.EXPECTED['scientific'],
        sizing=sizing['identity'], assembly=assembly['hashes'], new_proofs=CERT,
        interface=h.identity(patch),core_cells=190387,prism_cells=371646))
    h.eq(scientific, SCI, 'WrongScientific')
    foundation = h.read(CORE/'FOUNDATION.json'); audit = h.read(CORE/'C1/NETGEN_AUDIT.json')
    summary = foundation['summary']
    h.need(foundation['passed'] and foundation['literal_mesh_ok'] and foundation['fidelity']['pass'], 'FoundationFailed')
    h.need(not any(audit['failures'].values()), 'QualityFailed')
    for k,v in summary.items():
        if 'failures' in k or k in ('nonpositive_cell_volumes','high_aspect_ratio_cells','highly_skew_faces'):
            h.eq(v,0,'FailedGate:'+k)
    h.need(summary['minimum_cell_volume']>0 and summary['minimum_determinant']>=.0011
        and summary['minimum_interpolation_weight']>=.055 and summary['maximum_face_skewness']<=3.63636
        and summary['minimum_face_volume_ratio']>=.01 and summary['maximum_cell_aspect_ratio']<=1000, 'HardGate')
    pm = CORE/'foundation/openfoam_case/constant/polyMesh'
    counts.update(faces=foam_count(pm/'faces'), internal_faces=foam_count(pm/'neighbour'), interface_triangles=len(patch), interface_vertices=len(front))
    h.eq((foam_count(pm/'points'),foam_count(pm/'owner'),counts['faces'],counts['internal_faces']),
         (227901,1329373,1329373,1290405),'WrongPolyMeshCounts')
    boundary = (pm/'boundary').read_text()
    patches = []
    for name, block in re.findall(r'(\w+)\s*\{([^{}]*)\}', boundary):
        if 'nFaces' not in block: continue
        values = dict(re.findall(r'(\w+)\s+([^;\s]+)\s*;',block))
        patches.append(dict(name=name, type=values['type'], physicalType=values.get('physicalType'),
            faces=int(values['nFaces']), startFace=int(values['startFace']),
            orientation='outward from fluid', role='capsule body' if name=='wall' else 'all six outer box planes'))
    h.eq([(p['name'],p['faces'],p['startFace']) for p in patches],
         [('wall',33786,1290405),('farfield',5182,1324191)],'WrongPatchTable')
    log = CORE/'foundation/checkMesh.stdout'
    bind(log, foundation['check']['stdout_sha256'])
    text = log.read_text()
    h.need('Mesh OK.' in text and 'Version:  13' in text, 'WrongFoundationValidation')
    nonortho = float(re.search(r'Mesh non-orthogonality Max:\s*([\d.]+)',text)[1])
    for key,stem in [('conversion','gmshToFoam'),('audit','foundationAudit')]:
        bind(CORE/'foundation'/f'{stem}.stdout', foundation[key]['stdout_sha256'])
    continuous = h.read(CORE/'CONTINUOUS.json')
    closure=h.read(CORE/'CERTIFICATE_REOPEN.json')
    h.need(closure['passed'] and all(closure[k]==0 for k in ('forbidden','unresolved','missing','duplicate','unused')), 'IncompleteClosure')
    h.eq(continuous['proof_hash'],CERT,'WrongCertificate')
    h.eq(sum(continuous['methods'].values()),19941573,'IncompleteMethods')
    volume = h.read(CORE/'WHOLE_VOLUME.json')
    h.need(volume['passed'] and abs(volume['difference'])<=volume['tolerance']==1e-9,'VolumeFailed')
    provenance = V4.parent/'sdf_cfd_mesher_v3/farfield_core_transition/run_farfield_complete.py'
    h.need('4096 * np.finfo(float).eps' in provenance.read_text() and '1e-9)' in provenance.read_text(),'VolumeProvenance')
    bind(provenance)
    upstream = dict(surface=h.EXPECTED['surface'],geometry=h.EXPECTED['geometry'],request=h.EXPECTED['request'],
        schedule=h.EXPECTED['schedule'],advancing_front=l11['qualification'],l11=l11['identity'],
        handoff=handoff_authority['identity'],handoff_scientific=handoff['identity'],domain=domain['identity'],
        generic_sizing=qb['generic_core_sizing']['identity'],sizing=sizing['identity'],core=qualification['identity'])
    return dict(historical_sources=historical_sources,upstream=upstream, mesh=dict(scientific=SCI,content=CONTENT,original_access=ACCESS,original_checkpoint=CHECKPOINT),
        counts=counts,quality=dict(summary=summary, maximum_nonorthogonality_degrees=nonortho,
        core_failure_counts=audit['failures'],exact_jacobian=audit['exact_jacobians']),
        interface=dict(mapping=h.identity(patch),preservation=audit['interface'],deferred=audit['deferred'],
        metrics=audit['extrema']['interface_faces'],determinant=audit['extrema']['recertified_prism_determinants']),
        continuous=dict(identity=CERT,inherited=33336032,new=19941573,combined=53277605,methods=continuous['methods'],
        closure={k:closure[k] for k in ('forbidden','unresolved','missing','duplicate','unused')},failures=continuous['failures']),
        volume=volume,domain=domain,sizing=sizing,foundation=foundation,
        openfoam=dict(directory=str(pm.relative_to(V4)),members={p.name:h.file_hash(p) for p in sorted(pm.iterdir()) if p.is_file()},
            patches=patches,environment='source /opt/openfoam13/etc/bashrc',
            command='checkMesh -constant -allTopology -allGeometry -case '+str(pm.parents[1]),literal='Mesh OK.',
            solver_boundary_conditions=None), evidence=files)


def certificate_verify():
    tick=h.measured(); digest=hashlib.sha256(b'['); count=0
    with gzip.open(CORE/'NEW_PROOFS.jsonl.gz','rb') as stream:
        for line in stream:
            if count: digest.update(b',')
            digest.update(line.rstrip(b'\r\n')); count+=1
    digest.update(b']'); h.eq((count,digest.hexdigest()),(19941573,CERT),'SavedCertificateCorrupt')
    return dict(passed=True,rows=count,canonical_identity=digest.hexdigest(),candidate_generation=0,
                historical_prism_proof_rows=0,metrics=h.metrics(*tick))


def renew():
    tick=h.measured(); old=Store(CORE/'complete_mesh').root(ACCESS)
    recovered=renewal.discover(RESULTS/'prism_assembly_optimization/epoch_access')
    parent=recovered['reference']
    with ExitStack() as held, h.guards(), h.CommittedState(parent) as state:
        h.eq(state.content_root,h.EXPECTED['content'],'WrongRenewedL11'); state.check()
        authenticated={}
        refs=renewal.refresh_records(old['plan']['references'],held,authenticated)
        produced=renewal.refresh_records(old['produced'],held,authenticated)
        body=renewal.rebind_body(old,'recursive',parent,refs,produced,access.epoch())
        h.eq(body['scientific'],SCI,'RenewalChangedScience')
        h.eq(h.identity(body['plan']['components']),CONTENT,'RenewalChangedContent')
        store=Store(OUT/'access'); root=store.initialize(body)
        with store.open(root): pass
    result=h.seal(dict(schema='V4CompleteMeshAccessRenewal/1',source=ACCESS,root=root,
        directory=str(store.path),checkpoint=h.file_hash(store.path/('ROOT_'+root+'.json')),
        parent=parent,old_epoch=old['epoch'],epoch=access.epoch(),scientific=SCI,content=CONTENT,
        qualified_procedure=h.file_hash(renewal.__file__),metrics=h.metrics(*tick)))
    h.durable_json(OUT/'ACCESS.json',result)
    return result


def handoff(authority):
    return h.seal(dict(schema='V4DownstreamCFDMeshHandoff/1',authority=authority['identity'],
        scientific=SCI,units='m',flow_axis=[0,0,1],fluid_side='positive_sdf',
        openfoam=authority['openfoam'],counts=authority['counts'],quality=authority['quality'],
        upstream=authority['upstream'],solver_settings=None,unqualified=EXCLUSIONS))


def validate(authority, downstream, expected_root):
    h.verify_record(authority); h.verify_record(downstream)
    h.eq(authority['identity'],expected_root,'StaleAuthority')
    h.eq(authority['classification'],CLASS,'WrongClassification')
    h.eq(authority['acceptance'],contract(),'ChangedAcceptance')
    h.eq(authority['access_reference'],checked(OUT/'ACCESS.json')['root'],'ChangedAccessBinding')
    h.eq(authority['implementation'],h.file_hash(__file__),'ChangedImplementation')
    snapshot=source_snapshot()
    for key,value in snapshot.items(): h.eq(authority[key],value,'ChangedClaim:'+key)
    h.eq(downstream,handoff(authority),'ChangedHandoff')
    return snapshot


def build():
    OUT.mkdir(exist_ok=True)
    tick=h.measured()
    status=subprocess.check_output(['git','status','--porcelain=v1'],text=True).splitlines()
    status=[s for s in status if 'formalize_capsule_complete_mesh.py' not in s and 'capsule_complete_mesh_authority' not in s]
    h.need(len(status)==155,'UnexpectedInitialDirtyCount')
    h.eq(subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'861c4078e611ec1ba432647aa87e9fc3f89a55c7','WrongHEAD')
    h.eq(subprocess.check_output(['git','branch','--show-current'],text=True).strip(),'feature/sdf-cfd-mesher-v4-advancing-front','WrongBranch')
    h.durable_json(OUT/'PREFLIGHT.json',dict(status=status,free_bytes=shutil.disk_usage(V4).free))
    snapshot=source_snapshot()
    h.durable_json(OUT/'CERTIFICATE_VERIFICATION.json',certificate_verify())
    renew()
    accepted=contract(); h.durable_json(OUT/'ACCEPTANCE.json',accepted)
    authority=h.seal(dict(schema='V4ProductionCompleteMeshAuthority/1',classification=CLASS,
        acceptance=accepted,implementation=h.file_hash(__file__),access_reference=h.read(OUT/'ACCESS.json')['root'],**snapshot))
    h.durable_json(OUT/'AUTHORITY.json',authority)
    h.durable_json(OUT/'CFD_HANDOFF.json',handoff(authority))
    h.durable_json(OUT/'BUILD.json',dict(passed=True,metrics=h.metrics(*tick),authority=authority['identity'],
        geometry_generated=0,proof_candidates_generated=0,CFD_solves=0,hardlinks_created=0))
    print(json.dumps({'authority':authority['identity'],'metrics':h.metrics(*tick)}))


def reopen():
    tick=h.measured(); authority=h.read(OUT/'AUTHORITY.json'); downstream=h.read(OUT/'CFD_HANDOFF.json')
    validate(authority,downstream,h.read(OUT/'BUILD.json')['authority'])
    renewed=checked(OUT/'ACCESS.json')
    h.eq(renewed['scientific'],SCI,'WrongRenewal')
    h.eq(h.file_hash(Path(renewed['directory'])/('ROOT_'+renewed['root']+'.json')),renewed['checkpoint'],'WrongCheckpoint')
    with h.guards(), Store(renewed['directory']).open(renewed['root']) as body, h.CommittedState(renewed['parent']) as parent:
        h.eq(body['scientific'],SCI,'WrongScience'); h.eq(h.identity(body['plan']['components']),CONTENT,'WrongContent')
        h.eq(parent.content_root,h.EXPECTED['content'],'WrongL11'); parent.check()
    result=dict(passed=True,authority=authority['identity'],metrics=h.metrics(*tick),
        historical_prism_proof_rows=0,historical_prism_quality_rows=0,complete_candidate_generation=0)
    h.durable_json(OUT/'REOPEN.json',result); print(json.dumps(result))


if __name__=='__main__':
    {'build':build,'reopen':reopen}[sys.argv[1]]()
