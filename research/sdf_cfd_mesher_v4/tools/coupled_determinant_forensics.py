"""Reproduce all controlling inputs of the preserved failing cell, read-only."""
import sys
sys.dont_write_bytecode=True
from pathlib import Path
import json,time
import numpy as np
import generalize_local_remesh as g
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.mixed_front import face_key
from sdf_cfd_mesher_v4.common.authority import file_hash


def run():
    root=Path(__file__).resolve().parents[1];out=root/'results/coupled_shell_remesh_determinant'
    prior=root/'results/broader_mixed_geometry_end_to_end/general_asymmetric_passage/A'
    source=prior/'FIXED_SHELL_COUPLING_OBSTRUCTION.json';r=json.loads(source.read_text())['failures'][0]
    state=NativeCheckpointStore(prior/'checkpoints').restart()
    key=r['scientific_cell'];cell=state['cells'][key]
    def geometry(cell):
        keys=cell['vertices'];p=np.array([state['vertices'][k] for k in keys])
        return g.t.geometry(p,dict(vertices=list(range(len(keys))),faces=[tuple(keys.index(k) for k in f) for f in cell['faces']]))
    geo=geometry(cell);cq=g.q.legacy_modules()['component_quality']
    core=prior/'core';xyz=np.load(core/'coordinates.npy');tets=np.load(core/'tets.npy')
    tags=np.load(core/'node_tags.npy');it=np.load(core/'interface_tags.npy');order=np.argsort(tags)
    indices=order[np.searchsorted(tags[order],it)]
    meta=json.loads((prior/'interface/INTERFACE.json').read_text());core_index=dict(zip(meta['vertex_ids'],indices))
    rows=[];mask=[]
    for j,face in enumerate(cell['faces']):
        owners=state['faces'][face_key(face)]['owners'];local=len(owners)==2;mask.append(local)
        if local:
            neighbor=next(k for k in owners if k!=key);other=geometry(state['cells'][neighbor])
        else:
            ids=[core_index[k] for k in face]
            selected=np.flatnonzero(np.isin(tets,ids).sum(axis=1)==3);assert len(selected)==1
            neighbor='core:'+str(int(selected[0]));tet=tets[selected[0]]
            other=g.t.geometry(xyz,dict(vertices=tet,faces=[tuple(tet[list(f)]) for f in [(0,2,1),(0,1,3),(1,2,3),(2,0,3)]]))
        vs=np.array([state['vertices'][k] for k in face])[None]
        sf,cf=cq.FG.face_area_centre_tri(vs)
        wt,sk,vr,no=[float(z[0]) for z in cq.face_metrics(vs,sf,cf,geo['centre'][None],other['centre'][None],np.array([geo['volume']]),np.array([other['volume']]))]
        rows.append(dict(vertices=face,area_vector=geo['areas'][j],face_centre=cf[0],
            shell_local_coupled=local,complete_coupled=True,owner_centre=geo['centre'],owner_volume=geo['volume'],
            neighbor=neighbor,neighbor_centre=other['centre'],neighbor_volume=other['volume'],
            weight=wt,skewness=sk,symmetric_volume_ratio=vr,nonorthogonality=no))
    def tensor(mask):
        s=geo['areas'][mask];avg=np.linalg.norm(s,axis=1).mean();a=s/avg;t=a.T@a
        return dict(mean_area=avg,tensor=t,determinant=float(np.linalg.det(t)))
    local=tensor(mask);complete=tensor([True]*len(mask))
    # Cyclic face storage can alter reduction order by roundoff; the original
    # ordered reproducer remains the exact binary64 authority for quoted values.
    assert local['determinant']>=.0011 and complete['determinant']<.0011
    g.save(out/'FORENSICS.json',dict(source=str(source.relative_to(root)),source_sha256=file_hash(source),
        reproduced_original=r,scientific_cell=key,faces=rows,local=local,complete=complete,
        root_cause='Exposed face becomes coupled. The added area vector changes both mean-area normalization and tensor. Core vertex relocation cannot alter these shell-only inputs.',
        determinant_dependencies=['shell face area vectors','coupled face incidence'],
        other_metric_dependencies=dict(weight=['face centre','owner/neighbour centres','face area'],
            skewness=['face centre','owner/neighbour centres','face vertices'],
            volume_ratio=['owner/neighbour volumes'],nonorthogonality=['face area','owner/neighbour centres']),
        scope='Historical cell ID used only to reproduce the sealed failure; production seeding has no ID exception'))


if __name__=='__main__':run()
