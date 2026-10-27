"""Characterize rejected fixed-connectivity relocation scopes, not infeasibility."""
import json
import numpy as np
from run_geometry_family import OUT,g


def run():
    rows=[]
    for case in sorted(OUT.iterdir()):
        if not case.is_dir() or case.name=='sealed':continue
        for p in sorted(case.glob('*/repaired_*/REPAIR_SCOPE.json')):
            scope=json.loads(p.read_text());source=p.parent.parent/p.parent.name.removeprefix('repaired_')
            if not (source/'tets.npy').exists():continue
            t=np.load(source/'tets.npy',mmap_mode='r');bad=np.asarray(scope['bad_cells'],int)
            fixed=np.asarray(scope['fixed_vertices'],int)
            immovable=bad[np.all(np.isin(t[bad],fixed),axis=1)]
            quality=json.loads((source/'GENERATION.json').read_text())
            rows.append(dict(case=case.name,run=p.parent.parent.name,core=source.name,bad_cells=len(bad),
                bad_tets_with_all_vertices_fixed=immovable.tolist(),movable_vertices=len(scope['movable_vertices']),
                measured_quality=quality['core'],interface_quality=quality['interface'],
                interpretation='All-fixed bad tetrahedra cannot be changed by interior vertex relocation with this connectivity. This is not proof that another conformal tetrahedralization is infeasible.'))
    g.save(OUT/'CORE_LIMITS.json',dict(scope='Measured limitation of the preserved core generation and relocation routes',cases=rows))


if __name__=='__main__':run()
