"""Bounded mixed shell/core assembly with unchanged exact validators."""
import sys
sys.dont_write_bytecode=True
from pathlib import Path
import argparse,json,time
from collections import Counter
import numpy as np
import psutil
import generalize_local_remesh as g
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.mixed_front import geometry_inputs, scientific_hash


def run(folder, core_name='core', certificate_provider=None, *, sealed_state=None):
    tick=time.perf_counter(); state=NativeCheckpointStore(folder/'checkpoints').restart() if sealed_state is None else sealed_state
    if sealed_state is not None:
        seal=json.loads((folder/'SEAL.json').read_text())
        if not seal['passed']:raise ValueError('CoreRequiresSealedShell')
        from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import scientific_digest
        if any(scientific_digest(state[k])!=v for k,v in seal['geometry_components'].items()):raise ValueError('SealedCoreInputMismatch')
    interface=json.loads((folder/'interface/INTERFACE.json').read_text())
    if Path(core_name).name!=core_name: raise ValueError('CoreMustBeLocalCaseOutput')
    core=folder/core_name; quality=json.loads((core/'GENERATION.json').read_text())
    if not quality['passed']: raise ValueError('CoreHardGatesNotPassed')
    keys=sorted(state['vertices']); index={k:i for i,k in enumerate(keys)}
    p=np.array([state['vertices'][k] for k in keys])
    cx=np.load(core/'coordinates.npy'); ct=np.load(core/'tets.npy')
    tags=np.load(core/'node_tags.npy'); it=np.load(core/'interface_tags.npy')
    order=np.argsort(tags); ci=order[np.searchsorted(tags[order],it)]
    mapping=np.empty(len(cx),dtype=int)
    mapping[ci]=[index[k] for k in interface['vertex_ids']]
    assert np.array_equal(p[mapping[ci]].view('u8'),cx[ci].view('u8'))
    other=np.setdiff1d(np.arange(len(cx)),ci); mapping[other]=len(p)+np.arange(len(other))
    p=np.concatenate([p,cx[other]])
    cells=[]
    for _,c in sorted(state['cells'].items()):
        cells.append(dict(id=len(cells),column=int(c['source']),layer=c['layer'],kind=c['kind'],
            vertices=[index[v] for v in c['vertices']],faces=[tuple(index[v] for v in f) for f in c['faces']]))
    shell_faces={tuple(sorted(f)) for c in cells for f in c['faces']}
    for tet in mapping[ct]:
        cells.append(dict(id=len(cells),column=-1,layer=0,kind='tetrahedron',vertices=tet.tolist(),
            faces=[tuple(tet[list(f)]) for f in [(0,2,1),(0,1,3),(1,2,3),(2,0,3)]]))
    ft=np.load(core/'farfield_tags.npy'); fi=order[np.searchsorted(tags[order],ft)]
    far=[tuple(f) for f in mapping[fi][np.load(core/'far_faces.npy')]]
    wall=[tuple(index['layer:0:source:'+str(v)] for v in f) for f in state['wall']['triangles'].values()]
    owners,ownership=g.t.ownership(cells,far,[tuple(reversed(f)) for f in wall])
    assert ownership['passed'],ownership
    gate=g.t.quality(p,cells,owners,0)
    if not gate['passed']:
        g.save(folder/'COMPLETE_QUALITY_FAILURE.json',gate)
        raise ValueError('CompleteMixedHardQualityFailure')
    # Receipt reuse is conditional on every original scientific dependency.
    inputs=geometry_inputs(state); reused=0
    for _,receipt in state['certificate'].items():
        assert all(inputs[k]==v for k,v in receipt['inputs'].items())
        assert receipt['value']['classification'] in ('CertifiedSeparated','CertifiedAllowedIncidentContact')
        reused+=1
    faces=[rows[0][1] for _,rows in sorted(owners.items())]
    active=[i for i,f in enumerate(faces) if tuple(sorted(f)) not in shell_faces]
    cert=(certificate_provider or g.s.cert.certify)(p,faces,active)
    volume=g.volume.audit(p,cells,far,wall)
    passed=cert['passed'] and volume['interior_degree']['passed'] and volume['exact_volume_balance'] and volume['positive_cells']
    out=folder/'assembly';out.mkdir(exist_ok=True)
    np.save(out/'coordinates.npy',p); g.save(out/'CELLS.json',cells)
    g.save(out/'QUALITY.json',gate);g.save(out/'NEW_CERTIFICATE.json',cert);g.save(out/'VOLUME.json',volume)
    result=dict(passed=passed,scope='bounded complete geometry; no invented CFD controls',
        shell_scientific=scientific_hash(state),cells=dict(Counter(c['kind'] for c in cells)),
        total_cells=len(cells),ownership=ownership,interface_coordinates_exact=True,
        shell_proofs_reused=reused,new_relations=cert['required_pairs'],
        total_relations=reused+cert['required_pairs'],forbidden_or_unresolved=cert['failures'],
        missing=0,duplicates=0,volume_passed=bool(passed),seconds=time.perf_counter()-tick,
        peak_rss_bytes=psutil.Process().memory_info().peak_wset)
    g.save(folder/'ASSEMBLY.json',result)
    assert passed


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('folder',type=Path)
    parser.add_argument('--core',default='core')
    args=parser.parse_args();run(args.folder,args.core)
