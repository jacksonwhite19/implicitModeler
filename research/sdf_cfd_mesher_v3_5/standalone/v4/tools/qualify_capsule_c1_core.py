"""User-bound C1 qualification. No parameter search or alternate candidate."""
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path
import capsule_sizing_control as control

OUT = control.V4/'results/capsule_c1_core_qualification'
G = float.fromhex('0x1.0ca937be1b9dcp-2')
HMAX = float.fromhex('0x1.83e602972fe65p-4')
START = 'fb656d46067af76e0540eeec9873f9cc913d1c88'
EVIDENCE = '62bd0776886282c449177d4fb902625f0783fabeb9e5a9b19893f5a3b85a523f'

def verify_qualifications():
    import capsule_user_domain_handoff as h
    records={}
    for path,expected,base in [
        (control.SIZING/'QUALIFICATION.json','228433a2031a6ce26867228ff84139f520379aec34a448641e74572f87f7dc40',control.SIZING),
        (control.HANDOFF/'AUTHORITY.json','55e31c4497e0629b7951f90347a555fbb3510755c00d531d0fe4d6bdf9fe44c5',control.V4)]:
        r=h.read(path);h.verify_record(r);h.eq(r['identity'],expected,'WrongQualification')
        for name,sha in r['files'].items():h.eq(h.file_hash(base/name),sha,'CorruptQualificationComponent')
        records[path.parent.name]=dict(identity=r['identity'],sha256=h.file_hash(path))
    control.write(OUT/'QUALIFICATION_BINDINGS.json',records)

def preflight():
    import capsule_user_domain_handoff as h
    from audit_sealed_l11_core_domain import v3_inventory
    root = control.V4.parents[1]
    head = subprocess.check_output(['git','rev-parse','HEAD'],cwd=root).decode().strip()
    branch = subprocess.check_output(['git','branch','--show-current'],cwd=root).decode().strip()
    assert head == START and branch == 'feature/sdf-cfd-mesher-v4-advancing-front'
    status = subprocess.check_output(['git','status','--porcelain=v1','-z'],cwd=root)
    OUT.mkdir(exist_ok=False)
    control.write(OUT/'PREFLIGHT.json',dict(head=head,branch=branch,status_hex=status.hex(),
        free_bytes=shutil.disk_usage(root).free,v3=v3_inventory()))
    tick=time.perf_counter()
    control.reopen_control_manifest(control.OUT/'CONTROL_MANIFEST.json',EVIDENCE)
    verify_qualifications()
    h.reopen('36f3bc9ecd8f4728fda53a3c43d686b62a0553931c0f5956604dcb35cd7afa52')
    architecture=h.read(control.SIZING/'AUTHORITY.json');h.verify_record(architecture)
    assert architecture['identity']=='77c1c794f90eb913c351f40c09ba68cd3aae79600080195dc64f02c9e64472c7'
    for name,sha in architecture['files'].items():
        assert control.digest(control.SIZING/Path(name))==sha, name
    binding=h.seal(dict(status='CapsuleCaseProductionSizingControl',g=G,g_hex=G.hex(),
        h_max=HMAX,h_max_hex=HMAX.hex(),regional_caps=[],selection='Explicit user selection of retained C1',
        target_cap_not_realized_edge_guarantee=True,universal_default=False,
        architecture=architecture['scientific'],implementation=architecture['implementation'],
        l11=h.EXPECTED['identity'],handoff='55e31c4497e0629b7951f90347a555fbb3510755c00d531d0fe4d6bdf9fe44c5',
        domain='63c749602dfd1f0ae481a640127249b7fc031f894cabf31c565cd0163c3eec43',selection_evidence=EVIDENCE))
    h.durable_json(OUT/'SIZING_AUTHORITY.json',binding)
    control.write(OUT/'CANDIDATES.json',dict(candidates=[dict(id='C1',g=G,h_max=HMAX,
        status='CapsuleCaseProductionSizingControl',sizing_authority=binding['identity'])]))
    control.write(OUT/'INPUT_VERIFICATION.json',dict(passed=True,seconds=time.perf_counter()-tick,
        sizing_authority=binding['identity']))

def generate():
    from capsule_c1_publication import validate_sizing
    binding=json.loads((OUT/'SIZING_AUTHORITY.json').read_text());validate_sizing(binding)
    candidates=json.loads((OUT/'CANDIDATES.json').read_text())['candidates']
    assert len(candidates)==1 and candidates[0]['id']=='C1'
    assert candidates[0]['g']==G and candidates[0]['h_max']==HMAX
    assert candidates[0]['sizing_authority']==binding['identity']
    control.OUT=OUT
    result=control.run('C1')
    if result['status']!='generated_interface_passed':raise ValueError(result)
    result.update(production_selected=True,selection_authority=json.loads((OUT/'SIZING_AUTHORITY.json').read_text())['identity'])
    control.write(OUT/'C1/RESULT.json',result)
    import capsule_control_optimize as opt
    opt.OUT=OUT
    opt.run('C1')

def audit():
    import numpy as np
    import capsule_core_control_audit as a
    a.OUT=OUT
    equality={}
    for filename in ('OUTER.npz','NETGEN.npz','FINAL_CORE_OWNERS.npy'):
        fresh=np.load(OUT/'C1'/filename);old=np.load(control.OUT/'C1'/filename)
        same=all(np.array_equal(fresh[k],old[k]) for k in fresh.files) if hasattr(fresh,'files') else np.array_equal(fresh,old)
        equality[filename]=bool(same)
    control.write(OUT/'BACKEND_COMPARISON.json',equality)
    if not all(equality.values()):raise ValueError('BackendDivergedFromRetainedC1')
    a.prepare_prisms()
    result=a.audit('C1','NETGEN.npz')
    a.outer_quality('C1')
    print(json.dumps(result,indent=2))

def assemble():
    import numpy as np
    import capsule_user_domain_handoff as h
    tick=h.measured()
    authority,_,wall,tri,_=h.verify_sources()
    dest=OUT/'assembly';dest.mkdir(exist_ok=False)
    with h.guards(),h.CommittedState(authority['reference']) as state:
        vertices=state['vertices'];n=len(wall)
        x=np.array([vertices[f'layer:{layer}:source:{i}'] for layer in range(12) for i in range(n)])
        state.check()
    front,oriented,_=control.field_inputs()
    assert np.array_equal(x[-n:],front)
    core=np.load(OUT/'C1/NETGEN.npz');p,t=core['points'],core['tets']
    lookup={tuple(v):i for i,v in enumerate(p)}
    ci=np.array([lookup[tuple(v)] for v in front]);other=np.setdiff1d(np.arange(len(p)),ci)
    mapping=np.empty(len(p),dtype=np.int64);mapping[ci]=11*n+np.arange(n);mapping[other]=len(x)+np.arange(len(other))
    prisms=np.concatenate([np.concatenate([tri+(layer-1)*n,tri+layer*n],axis=1) for layer in range(1,12)])
    outer=np.load(OUT/'C1/OUTER.npz');oi=np.array([lookup[tuple(v)] for v in outer['points']])
    arrays=dict(coordinates=np.concatenate([x,p[other]]),prisms=prisms,tets=mapping[t],wall=tri,
        farfield=mapping[oi[outer['triangles']]],core_to_complete=mapping)
    for name,value in arrays.items():np.save(dest/(name+'.npy'),value)
    control.write(OUT/'ASSEMBLY.json',dict(shell_prisms=len(prisms),core_tets=len(t),vertices=len(arrays['coordinates']),
        source='Authenticated recursive vertex access; sealed source-column connectivity',
        interface_exact=True,old_proof_rows=0,old_quality_rows=0,metrics=h.metrics(*tick),
        hashes={name:h.array_hash(value) for name,value in arrays.items()}))

def certify():
    import numpy as np
    import capsule_user_domain_handoff as h
    from certify_repaired_complete import unique_faces
    from native_mixed_face_certificate import certify as complete
    tick=h.measured();folder=OUT/'assembly'
    x=np.load(folder/'coordinates.npy');p=np.load(folder/'prisms.npy');t=np.load(folder/'tets.npy')
    quads=unique_faces(p,[[0,1,4,3],[1,2,5,4],[2,0,3,5]])
    caps=unique_faces(p,[[0,2,1],[3,4,5]])
    core=unique_faces(t,[[0,2,1],[0,1,3],[1,2,3],[2,0,3]])
    oldcaps={tuple(sorted(f)) for f in caps}
    new=[tuple(map(int,f)) for f in core if tuple(sorted(f)) not in oldcaps]
    faces=[tuple(map(int,f)) for f in quads]+[tuple(map(int,f)) for f in caps]+new
    old=len(quads)+len(caps)
    result=complete(x,faces,range(old,len(faces)))
    proofs=result.pop('proofs')
    # Persist all demanded new evidence, not just a sampled summary.
    import gzip
    with gzip.open(OUT/'NEW_PROOFS.jsonl.gz','wt',encoding='utf8') as stream:
        for row in proofs:stream.write(json.dumps(row,sort_keys=True,separators=(',',':'))+'\n')
    result.update(immutable_prism_proofs_reused=33336032,old_faces=old,new_faces=len(new),
        old_proof_payloads_read=0,old_proof_payloads_recomputed=0,metrics=h.metrics(*tick))
    control.write(OUT/'CONTINUOUS.json',result)
    if not result['passed']:raise ValueError('CompleteContinuousCertificationFailed')

def foundation():
    import numpy as np
    import shlex
    from qualify_repaired_foundation import load,stream_hash
    v3=control.V3;stage=OUT/'foundation';stage.mkdir(exist_ok=False)
    complete=load('readonly_c1_complete',v3/'farfield_core_transition/run_farfield_complete.py')
    bounded=load('readonly_c1_conversion',v3/'pmp_failure_forensics/surface_acceptance_contract/bounded_downstream_integration.py')
    x,p,t,w,f=[np.load(OUT/'assembly'/(k+'.npy')) for k in ('coordinates','prisms','tets','wall','farfield')]
    transfer=complete.write_msh(stage/'complete_transfer.msh',x,w,f,p,t)
    case=stage/'openfoam_case';complete.old.create_case(case)
    quote=lambda path:shlex.quote(str(path.resolve()))
    prefix='source /opt/openfoam13/etc/bashrc >/dev/null 2>&1; '
    conversion=complete.timed(prefix+'gmshToFoam '+quote(stage/'complete_transfer.msh')+' -case '+quote(case),stage,'gmshToFoam',timeout=1200)
    if conversion['exit_code']:raise ValueError('ConversionFailed')
    dest=stage/'foundation_audit';dest.mkdir()
    audit=complete.timed(prefix+quote(v3/'hxt_complete_mesh_preserved_core/bin/foundationCompleteAudit')+' -case '+quote(case)+' -outputDir '+quote(dest),stage,'foundationAudit',timeout=1200)
    check=complete.timed(prefix+'checkMesh -constant -allTopology -allGeometry -case '+quote(case),stage,'checkMesh',timeout=1200)
    summary=json.loads((dest/'foundation_audit_summary.json').read_text())
    fidelity=bounded.conversion_fidelity_bounded(dest,case,x,p,t,w,f,p[-len(w):,3:])
    text=(stage/'checkMesh.stdout').read_text()
    passed=('Mesh OK.' in text and check['exit_code']==0 and audit['exit_code']==0 and fidelity['pass']
        and summary['minimum_cell_volume']>0 and summary['minimum_determinant']>=.0011
        and summary['minimum_interpolation_weight']>=.055 and summary['minimum_face_volume_ratio']>=.01
        and summary['maximum_cell_aspect_ratio']<=1000 and summary['maximum_face_skewness']<=3.63636)
    control.write(OUT/'FOUNDATION.json',dict(passed=passed,summary=summary,fidelity=fidelity,
        conversion=conversion,audit=audit,check=check,transfer=transfer,literal_mesh_ok='Mesh OK.' in text,
        failed_lines=[line for line in text.splitlines() if 'Failed' in line or line.strip().startswith('***')]))
    if not passed:raise ValueError('CompleteFoundationGateFailed')

def account_bindings():
    import capsule_user_domain_handoff as h
    from incremental_pair_storage import PairIncidence
    tick=h.measured();authority,_,_,_,_=h.verify_sources()
    folder=h.SEALED/'L11';layer=h.read(folder/'AUTHORITY.json');h.verify_record(layer)
    with h.CommittedState(authority['reference']) as base:
        changed={'face:'+r['face']:None for r in h.read(h.OUT/'INTERFACE.json')}
        with PairIncidence(base,folder/'INCIDENCE_MANIFEST.json',layer['incidence_root']) as index:
            result=index.affected(changed)
    result.update(changed_face_factors=len(changed),old_proof_payloads_read=0,old_proof_payloads_rewritten=0,
        binding_semantics='Factorized complete-mesh owner/neighbour overlay; immutable prism geometry conclusions retained',metrics=h.metrics(*tick))
    control.write(OUT/'BINDING_ACCOUNTING.json',result)

def replay():
    import numpy as np
    dest=OUT/'replay';dest.mkdir(exist_ok=False)
    binding=json.loads((OUT/'SIZING_AUTHORITY.json').read_text())
    from capsule_c1_publication import validate_sizing
    validate_sizing(binding)
    assert binding['g_hex']==G.hex() and binding['h_max_hex']==HMAX.hex()
    shutil.copyfile(OUT/'CANDIDATES.json',dest/'CANDIDATES.json')
    tick=time.perf_counter();control.OUT=dest
    r=control.run('C1')
    if r['status']!='generated_interface_passed':raise ValueError('ReplayGenerationFailed')
    r.update(production_selected=True,selection_authority=binding['identity'],qualification_replay=True)
    control.write(dest/'C1/RESULT.json',r)
    import capsule_control_optimize as opt
    opt.OUT=dest;opt.run('C1')
    equality={}
    for filename in ('OUTER.npz','NETGEN.npz','FINAL_CORE_OWNERS.npy'):
        a=np.load(OUT/'C1'/filename);b=np.load(dest/'C1'/filename)
        equality[filename]=bool(all(np.array_equal(a[k],b[k]) for k in a.files) if hasattr(a,'files') else np.array_equal(a,b))
    control.write(OUT/'REPLAY_GEOMETRY.json',dict(passed=all(equality.values()),equality=equality,
        seconds=time.perf_counter()-tick,independent_gmsh_generation=True,parameters=binding['identity']))
    if not all(equality.values()):raise ValueError('DeterministicReplayFailed')

if __name__=='__main__':
    {'preflight':preflight,'generate':generate,'audit':audit,'assemble':assemble,'certify':certify,
     'foundation':foundation,'replay':replay,'verify':verify_qualifications,'bindings':account_bindings}[sys.argv[1]]()
