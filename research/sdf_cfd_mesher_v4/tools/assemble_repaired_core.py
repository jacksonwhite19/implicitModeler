"""Storage-only assembly of the native authoritative shell and repaired core."""
import sys
sys.dont_write_bytecode=True
import gc,time,json
import numpy as np
from repair_bulk_core import OUT,IFACE,V3,load,write,cell_metrics,local_guard
from native_memory_artifacts import materialize
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.common.authority import array_hash


def main():
    started=time.perf_counter();dest=OUT/'assembly';dest.mkdir(exist_ok=True)
    core={k:np.load(OUT/'repaired_A'/(k+'.npy'),mmap_mode='r') for k in ['coordinates','tets','node_tags','interface_tags','farfield_tags','interface_owner','farfield_owner']}
    order=np.argsort(core['node_tags']);tags=core['node_tags'][order]
    ci=order[np.searchsorted(tags,core['interface_tags'])];fi=order[np.searchsorted(tags,core['farfield_tags'])]
    meta=json.loads((IFACE/'INTERFACE.json').read_text())
    support=np.load(V3/'fresh_supported_refinement_family_qualification_2026-09-09/SF2/A/support/SUPPORT.npz')
    fg=load('assembly_foundation',V3/'foundation_aware_prism_shell_qualification/foundation_geometry.py')
    f=load('assembly_feasibility',V3/'fresh_supported_refinement_family_qualification_2026-09-09/fallback_scope_20260910/source/solve_coordinate_feasibility.py')
    bc,bv=fg.prism_cell_centres(np.concatenate([support['positions'][0,support['faces']],support['positions'][1,support['faces']]],axis=1))
    ids=np.unique(core['farfield_owner']);v,d,a,c=cell_metrics(core['coordinates'],core['tets'][ids],f)
    j=np.searchsorted(ids,core['farfield_owner']);q=core['coordinates'][fi[support['faces']]];av,fc=fg.face_area_centre_tri(q)
    da=np.abs(np.einsum('ij,ij->i',av,fc-c[j]));db=np.abs(np.einsum('ij,ij->i',av,bc-fc))
    w=np.minimum(da,db)/(da+db);r=np.minimum(v[j],bv)/np.maximum(v[j],bv);s=fg.internal_face_skewness(q,fc,av,c[j],bc)
    result=dict(weight_min=float(w.min()),ratio_min=float(r.min()),skew_max=float(s.max()),failed_weight=int((w<.055).sum()),failed_ratio=int((r<.01).sum()),failed_skew=int((s>3.63636).sum()))
    write('SUPPORT_INTERFACE.json',result)
    assert not any(result[k] for k in ['failed_weight','failed_ratio','failed_skew'])
    assert np.array_equal(core['coordinates'][fi],support['positions'][0])
    with materialize('fresh') as source:
        store=NativeCheckpointStore(source/'checkpoints');state=store.restart();b=state['cells'].backing
        nv=len(b.coordinates(0));layers=b.layer;nc=len(b.triangles)
        shelln=nv*(layers+1);other=np.setdiff1d(np.arange(len(core['coordinates'])),ci)
        mapping=np.empty(len(core['coordinates']),dtype=np.int64)
        mapping[ci]=np.array([int(k.split(':')[1])*nv+int(k.split(':')[3]) for k in meta['vertex_ids']])
        mapping[other]=shelln+np.arange(len(other));support_start=shelln+len(other)
        xyz=np.lib.format.open_memmap(dest/'coordinates.npy',mode='w+',dtype='<f8',shape=(support_start+len(fi),3))
        for layer in range(layers+1):xyz[layer*nv:(layer+1)*nv]=b.coordinates(layer)
        assert np.array_equal(xyz[mapping[ci]].view('u8'),core['coordinates'][ci].view('u8'))
        xyz[shelln:support_start]=core['coordinates'][other];xyz[support_start:]=support['positions'][1]
        prisms=np.lib.format.open_memmap(dest/'prisms.npy',mode='w+',dtype='<i8',shape=(layers*nc+len(support['faces']),6))
        for layer in range(1,layers+1):
            prisms[(layer-1)*nc:layer*nc,:3]=b.triangles+(layer-1)*nv
            prisms[(layer-1)*nc:layer*nc,3:]=b.triangles+layer*nv
        prisms[layers*nc:,:3]=mapping[fi[support['faces']]]
        prisms[layers*nc:,3:]=support_start+support['faces']
        np.save(dest/'tets.npy',mapping[core['tets']]);np.save(dest/'wall.npy',b.triangles)
        np.save(dest/'farfield.npy',support_start+support['faces']);np.save(dest/'core_to_complete.npy',mapping)
        # Preserve all native cell coordinates, not independently regenerated shell geometry.
        write('ASSEMBLY.json',dict(shell_prisms=layers*nc,shell_transitions=0,support_prisms=len(support['faces']),core_tets=len(core['tets']),total_cells=len(prisms)+len(core['tets']),wall_exact=True,first_layer_exact=True,interface_coordinates_exact=True,interface_connectivity_exact=True,wall_hash=array_hash(b.coordinates(0)),first_hash=array_hash(b.coordinates(1)),seconds=time.perf_counter()-started,publication='pending complete continuous certificate'))
        del b,state,store;gc.collect()
    xyz.flush();prisms.flush()


if __name__=='__main__':main()
