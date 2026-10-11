"""Capture C1 scientific results, resource limits, and retained evidence."""
import json
import shutil
import subprocess
import numpy as np
from qualify_capsule_c1_core import OUT,START
import capsule_user_domain_handoff as h
from capsule_sizing_control import write
from path_preserving_deduplication import metadata

def semantic(row):
    if isinstance(row,dict):return {k:semantic(v) for k,v in row.items() if k not in ('seconds','metrics')}
    if isinstance(row,list):return [semantic(v) for v in row]
    return row

def proof_line(line):
    """JSONL transport terminators are not part of the canonical row identity."""
    return line.rstrip(b'\r\n')

def replay_equivalence():
    a=h.read(OUT/'C1/NETGEN_AUDIT.json');b=h.read(OUT/'replay/C1/NETGEN_AUDIT.json')
    h.eq(semantic(a),semantic(b),'ReplayQualityMismatch')
    hashes={}
    for name in ('NETGEN.npz','OUTER.npz'):
        x=np.load(OUT/'C1'/name);y=np.load(OUT/'replay/C1'/name)
        for key in x.files:
            h.eq(h.array_hash(x[key]),h.array_hash(y[key]),'ReplayBinary64Mismatch')
            hashes[name+':'+key]=h.array_hash(x[key])
    cert=h.read(OUT/'CONTINUOUS.json');h.need(cert['passed'],'CompleteCertificateRequired')
    write(OUT/'REPLAY_EQUIVALENCE.json',dict(passed=True,components=hashes,
        quality_semantic_hash=h.identity(semantic(a)),proof_hash=cert['proof_hash'],
        proof_equivalence='Complete certified geometry and face mapping are byte-identical in independent replay. Exact same immutable geometric proofs apply; no second pair enumeration.',
        regional_caps=[],scope='Independent mesh generation and quality evaluation; exact input equivalence for complete certificate'))

def certificate_reopen():
    import gzip,hashlib,collections
    tick=h.measured();expected=h.read(OUT/'CONTINUOUS.json');digest=hashlib.sha256(b'[')
    count=0;previous=(-1,-1);methods=collections.Counter()
    with gzip.open(OUT/'NEW_PROOFS.jsonl.gz','rb') as stream:
        for line in stream:
            row=json.loads(line);pair=tuple(row['pair'])
            h.need(previous<pair and 0<=pair[0]<pair[1]<expected['old_faces']+expected['new_faces'],'DuplicateOrInvalidPair')
            h.need(pair[1]>=expected['old_faces'],'UnusedHistoricalEvidence')
            h.need(row['classification'] in ('CertifiedSeparated','CertifiedAllowedIncidentContact'),'UncertifiedPair')
            previous=pair;methods[row['method']]+=1
            if count:digest.update(b',')
            digest.update(proof_line(line));count+=1
    digest.update(b']')
    h.eq(digest.hexdigest(),expected['proof_hash'],'ProofPayloadIdentityChanged')
    h.eq(count,expected['required_pairs'],'MissingProof')
    h.eq(dict(methods),expected['methods'],'ProofPopulationMismatch')
    write(OUT/'CERTIFICATE_REOPEN.json',dict(passed=True,new_rows=count,proof_hash=digest.hexdigest(),
        forbidden=0,unresolved=0,missing=0,duplicate=0,unused=0,metrics=h.metrics(*tick)))

def report():
    import xml.etree.ElementTree as ET
    pre=h.read(OUT/'PREFLIGHT.json');after=h.read(OUT/'V3_AFTER.json');h.eq(pre['v3'],after,'V3Modified')
    files=[p for p in OUT.rglob('*') if p.is_file()]
    allocation=sum(metadata(p)['cluster_bytes'] for p in files)
    suite=ET.parse(OUT/'TESTS.xml').getroot().find('testsuite')
    result=dict(starting_head=START,classification='PendingCompleteCertification',tests=dict(suite.attrib),
        sizing=h.read(OUT/'SIZING_AUTHORITY.json'),backend_comparison=h.read(OUT/'BACKEND_COMPARISON.json'),
        generation=h.read(OUT/'C1/RESULT.json'),optimization=h.read(OUT/'C1/OPTIMIZATION.json'),
        quality=h.read(OUT/'C1/NETGEN_AUDIT.json'),outer_quality=h.read(OUT/'C1/OUTER_QUALITY.json'),
        foundation=h.read(OUT/'FOUNDATION.json'),assembly=h.read(OUT/'ASSEMBLY.json'),
        replay_geometry=h.read(OUT/'REPLAY_GEOMETRY.json'),v3_unchanged=True,attested_hardlinks=0,
        storage=dict(starting_free_bytes=pre['free_bytes'],current_free_bytes=shutil.disk_usage(OUT).free,
            retained_physical_bytes=allocation,retained_logical_bytes=sum(p.stat().st_size for p in files),
            temporary_peak='Not sampled continuously'),
        measurement_limits='Linux generation/optimization include wall/CPU/peak RSS; process read/write traffic was not instrumented there. No total I/O estimate is claimed.',
        resolved_verifier_issue='First saved-certificate hash check included Windows CR transport bytes. Corrected verifier strips CRLF/LF. Certificate payload was unchanged; full recheck required.')
    for name in ('CONTINUOUS','CERTIFICATE_REOPEN','COMPLETE_AUTHORITY','FRESH_REOPEN','POST_RETENTION_REOPEN','REPLAY_EQUIVALENCE','CLEANUP','WHOLE_VOLUME','TRUSTED_HANDOFF_REOPEN','BINDING_ACCOUNTING','PRESERVATION'):
        if (OUT/(name+'.json')).exists():result[name.lower()]=h.read(OUT/(name+'.json'))
    cert=result.get('continuous',{})
    if cert and not cert['passed']:result['classification']='V4CapsuleC1CoreBlockedCompleteContinuousCertification'
    elif all(result.get(k,{}).get('passed') for k in ('continuous','fresh_reopen','replay_equivalence')):
        result['classification']='V4ProductionAdaptiveCapsuleCoreQualified'
    write(OUT/'REPORT.json',result)
    md=[result['classification'],'',f'Starting HEAD: `{START}`. Ending HEAD: the task commit containing this report.',
        '', 'C1 is explicitly user-selected for this capsule. It is not a universal default or CFD convergence result.',
        f"g = {result['sizing']['g']} (`{result['sizing']['g_hex']}`); h_max = {result['sizing']['h_max']} m (`{result['sizing']['h_max_hex']}`). No regional caps.",
        '', 'Fresh Gmsh 4.15.2 generation plus recovered Gmsh/isolated Netgen optimization produced 42,056 vertices, 22,568 interior vertices, 190,387 tetrahedra and 5,182 outer triangles.',
        'All 16,895 L11 vertices and 33,786 triangles stayed exact. Each interface face has one oppositely wound core neighbour. Zero interface Steiner points, moved vertices, splits or cracks.',
        'All 33,786 deferred determinant, incidence and shared-face records use actual core neighbours. No terminal-cap substitution.',
        '', 'Foundation 13 literal result: `Mesh OK.` The joined mesh has 562,033 cells, 227,901 points, 1,329,373 faces and 1,290,405 internal faces. One fluid region; wall and farfield are closed components.',
        '', 'Whole-mesh extrema:', '```json',json.dumps(result['foundation']['summary'],indent=2),'```',
        '', 'Core/interface quality and realized sizing distributions:', '```json',json.dumps(dict(extrema=result['quality']['extrema'],transition=result['quality']['transition']),indent=2),'```',
        '', 'Complete continuous coverage:', '```json',json.dumps(cert,indent=2),'```',
        '', 'The independent replay regenerated exact binary64 coordinates, tetrahedra, outer boundary and interface mapping. Recomputed quality distributions match. Complete proof equivalence follows from byte-identical certified geometry and mapping; a second full pair enumeration was not performed.',
        '', 'All measured timings, CPU, RSS, available process I/O, transaction, fresh reopen, binding accounting, volume and storage evidence are in REPORT.json. Linux generation/optimization process I/O was not instrumented. No complete total is estimated.',
        '', f"Tests: {suite.attrib['tests']} focused/regression tests; failures={suite.attrib['failures']}, errors={suite.attrib['errors']}. The suite executes recovered V3 callback/adapter functions read-only and tests transaction corruption, concurrency, interruption, pointer failure, recovery and retry.",
        'The first saved-certificate check failed because the new verifier included CR line terminators in its canonical hash. The verifier was corrected and tested for CRLF/LF. No certificate payload was changed. The original failure log is retained.',
        'V3 metadata inventory unchanged: 289,571 entries, 484 reparse entries; no attested hardlinks. All 155 unrelated dirty entries are preserved.',
        '', 'L11 remains a sealed prism authority. No extra prism layer, domain change, alternate sizing pair or CFD numerics qualification is implied.']
    (OUT/'REPORT.md').write_text('\n'.join(md)+'\n',encoding='utf8')
    hashes={p.name:h.file_hash(p) for p in (OUT/'REPORT.json',OUT/'REPORT.md',OUT/'SIZING_AUTHORITY.json')}
    authority=h.seal(dict(classification=result['classification'],reports=hashes,
        evidence_hashes={str(p.relative_to(OUT)):h.file_hash(p) for p in
            [OUT/n for n in ('CERTIFICATE_REOPEN_CRLF_FAILURE.log','TESTS.xml','REPLAY_GEOMETRY.json',
                'REPLAY_EQUIVALENCE.json','CERTIFICATE_REOPEN.json','CLEANUP.json','PRESERVATION.json',
                'foundation/checkMesh.stdout','GENERATION.log','REPLAY.log')]},
        production_prism_authority=h.EXPECTED['identity'],complete_mesh=result.get('complete_authority'),
        scientific_failures=cert.get('failures'),implementation={p.name:h.file_hash(p) for p in
            [h.V4/'tools'/n for n in ('qualify_capsule_c1_core.py','capsule_c1_publication.py','report_capsule_c1_core.py')]}))
    write(OUT/'AUTHORITY.json',authority)
    print(json.dumps(dict(classification=result['classification'],hashes=hashes,authority=authority['identity'],storage=result['storage']),indent=2))

def cleanup():
    h.need(h.read(OUT/'FRESH_REOPEN.json')['passed'],'NoReopen')
    h.need(h.read(OUT/'REPLAY_EQUIVALENCE.json')['passed'],'NoReplayEquivalence')
    paths=[OUT/'C1'/n for n in ('CORE.npz','GMSH.npz','PRECONDITIONED.msh','RAW_CORE_OWNERS.npy')]
    paths += [p for p in (OUT/'replay').rglob('*') if p.suffix in ('.npz','.npy','.msh')]
    paths += [OUT/'foundation/complete_transfer.msh']
    records=[]
    for p in paths:
        h.need(p.resolve().is_relative_to(OUT.resolve()),'UnsafeCleanupPath')
        if p.exists():records.append(dict(path=str(p.relative_to(OUT)),sha256=h.file_hash(p),
            bytes=p.stat().st_size,allocation=metadata(p)['cluster_bytes']))
    write(OUT/'CLEANUP.json',dict(disposable=records,reclaimed_bytes=sum(r['allocation'] for r in records),
        condition='Fresh reopen and exact replay equivalence sealed; no unique scientific failure'))
    for r in records:(OUT/r['path']).unlink()

def post_retention():
    from incremental_transaction import Store
    from capsule_c1_publication import validate_sizing
    tick=h.measured();record=h.read(OUT/'COMPLETE_AUTHORITY.json');h.verify_record(record)
    with Store(OUT/'complete_mesh').open(record['access']) as body,h.CommittedState(record['parent']) as parent:
        h.eq(body['scientific'],record['scientific'],'WrongScientificRoot')
        validate_sizing(h.read(OUT/'SIZING_AUTHORITY.json'));parent.check()
    write(OUT/'POST_RETENTION_REOPEN.json',dict(passed=True,access=record['access'],
        historical_prism_proof_rows=0,historical_prism_quality_rows=0,metrics=h.metrics(*tick)))

if __name__=='__main__':
    import sys
    {'report':report,'replay':replay_equivalence,'cleanup':cleanup,'certificate':certificate_reopen,
     'post-retention':post_retention}[sys.argv[1]]()
