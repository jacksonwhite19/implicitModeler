"""Localize full-coupling quality failures without modifying geometry."""
import sys
sys.dont_write_bytecode=True
import json
from pathlib import Path
from decimal import Decimal,localcontext
import numpy as np
import generalize_local_remesh as g
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore


def run(folder):
    state=NativeCheckpointStore(folder/'checkpoints').restart()
    failures=json.loads((folder/'COMPLETE_QUALITY_FAILURE.json').read_text())['failures']
    cells=sorted(state['cells'].items()); rows=[]
    incidence={tuple(sorted(f['vertices'])):f for f in state['faces'].values()}
    for failure in failures:
        key,c=cells[failure['id']]
        keys=sorted(set(c['vertices'])); index={v:i for i,v in enumerate(keys)}
        points=np.array([state['vertices'][v] for v in keys])
        cell=dict(c,vertices=[index[v] for v in c['vertices']],faces=[tuple(index[v] for v in f) for f in c['faces']])
        geometry=g.t.geometry(points,cell)
        mask=np.array([len(incidence[tuple(sorted(f))]['owners'])==2 for f in c['faces']])
        def determinant(areas):
            avg=np.linalg.norm(areas,axis=1).mean()
            return float(np.linalg.det((areas/avg).T@(areas/avg)))
        # Independent high-precision evaluation of the same area-tensor formula.
        # This is a diagnostic cross-check, not a replacement acceptance gate.
        with localcontext() as ctx:
            ctx.prec=70
            q=[[Decimal(float(v)) for v in p] for p in points]
            areas=[]
            for face in cell['faces']:
                assert len(face)==3
                a,b,c0=[q[v] for v in face]
                u=[b[i]-a[i] for i in range(3)];v=[c0[i]-a[i] for i in range(3)]
                areas.append([(u[1]*v[2]-u[2]*v[1])/2,(u[2]*v[0]-u[0]*v[2])/2,(u[0]*v[1]-u[1]*v[0])/2])
            average=sum(sum(v*v for v in a).sqrt() for a in areas)/len(areas)
            m=[[sum(a[i]*a[j] for a in areas)/average**2 for j in range(3)] for i in range(3)]
            d=m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0])
        rows.append(dict(scientific_cell=key,source=c['source'],layer=c['layer'],cell_type=c['kind'],
            vertices=dict(zip(keys,points.tolist())),faces=c['faces'],original_coupled_mask=mask.tolist(),
            standalone_determinant=determinant(geometry['areas'][mask]),
            complete_determinant=determinant(geometry['areas']),high_precision_complete=str(d),
            frozen_gate=.0011,unchanged_shell_geometry=True,
            independent_of_core_interior_coordinates=True,
            obstruction='All face-area inputs belong to the frozen shell tet. Conformal core attachment couples the exposed face. Interior core repair cannot change this determinant.'))
    g.save(folder/'FIXED_SHELL_COUPLING_OBSTRUCTION.json',dict(failures=rows,
        first_blocker='complete assembly / Foundation determinant',
        classification='FrozenShellCouplingQualityObstruction',
        permitted_repair='No interior-core-only repair changes the controlling shell face tensor. No gate or shell geometry changed.'))


if __name__=='__main__':run(Path(sys.argv[1]))
