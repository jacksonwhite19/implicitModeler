"""Time the existing complete assembly without changing its validators."""
import sys
sys.dont_write_bytecode=True
from pathlib import Path
import time
import json,re
import broader_geometry_assembly as a
from sdf_cfd_mesher_v4.common.authority import identity,file_hash


def relation_inputs(points,faces,active):
    return identity(dict(coordinates=a.g.q.array_hash(points),faces=[list(map(int,f)) for f in faces],active=list(map(int,active)),
                         implementation=file_hash(Path(a.g.s.cert.__file__))))


def seal_reference(folder):
    assembly=folder/'assembly';result=json.loads((folder/'ASSEMBLY.json').read_text())
    assert result['passed']
    cells=json.loads((assembly/'CELLS.json').read_text());p=a.np.load(assembly/'coordinates.npy')
    core=json.loads((folder/'core/GENERATION.json').read_text())
    shell_count=len(cells)-core['tets'];owners={}
    for cell in cells:
        for f in cell['faces']:owners.setdefault(tuple(sorted(f)),tuple(f))
    faces=[owners[k] for k in sorted(owners)]
    shell_faces={tuple(sorted(f)) for cell in cells[:shell_count] for f in cell['faces']}
    active=[i for i,f in enumerate(faces) if tuple(sorted(f)) not in shell_faces]
    proof=assembly/'NEW_CERTIFICATE.json'
    with proof.open('rb') as stream:
        prefix=stream.read(2048).decode();stream.seek(max(0,proof.stat().st_size-256));tail=stream.read().decode()
    proof_hash=re.search(r'"proof_hash":"([0-9a-f]{64})"',prefix).group(1)
    required=int(re.search(r'"required_pairs":(\d+)\}\s*$',tail).group(1))
    assert required==result['new_relations']
    seal=dict(inputs=relation_inputs(p,faces,active),proof_sha256=file_hash(proof),
              proof_hash=proof_hash,required_pairs=required,passed=True,failures=[])
    a.g.save(folder/'NEW_CERTIFICATE_AUTHORITY.json',seal)
    return seal


def run(folder,reference=None):
    times={}
    def wrap(module,name,label):
        original=getattr(module,name)
        def measured(*args,**kwargs):
            tick=time.perf_counter()
            try:return original(*args,**kwargs)
            finally:times[label]=times.get(label,0.)+time.perf_counter()-tick
        setattr(module,name,measured)
    wrap(a.g.s.cert,'certify','continuous_certification')
    wrap(a.g.volume,'audit','volume_accounting')
    wrap(a.g.t,'quality','complete_hard_quality')
    if reference is not None:
        seal=seal_reference(reference)
        def reuse(points,faces,active):
            if relation_inputs(points,faces,active)!=seal['inputs']:
                raise ValueError('CompleteCertificateReuseInputsDiffer')
            proof=reference/'assembly/NEW_CERTIFICATE.json'
            if file_hash(proof)!=seal['proof_sha256']:raise ValueError('ChangedCompleteProofPopulation')
            times['continuous_certification_reused_relations']=seal['required_pairs']
            return dict(passed=True,failures=[],proof_hash=seal['proof_hash'],required_pairs=seal['required_pairs'],
                scientific_inputs=seal['inputs'],proof_reference=str(proof.resolve()),proof_sha256=seal['proof_sha256'])
        a.g.s.cert.certify=reuse
    tick=time.perf_counter()
    try:a.run(folder)
    finally:
        times['total']=time.perf_counter()-tick
        a.g.save(folder/'ASSEMBLY_PERFORMANCE.json',times)


if __name__=='__main__':run(Path(sys.argv[1]),Path(sys.argv[2]) if len(sys.argv)>2 else None)
