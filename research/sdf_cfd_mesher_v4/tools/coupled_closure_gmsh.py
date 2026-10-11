"""Generate each independent diagnostic closure through the existing Gmsh route."""
import sys
sys.dont_write_bytecode=True
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import json,time,resource
import numpy as np
import gmsh
from broader_geometry_core import extract_all_volumes
from repair_bulk_core import load,V3


def run(path):
    data=json.loads(path.read_text());rows=[];tick=time.perf_counter()
    old=load('coupled_core_reference',V3/'airfoil_extrusion_qualification/downstream_runner/run_airfoil_downstream.py')
    gmsh.initialize([],False)
    try:
        old.setup_gmsh(gmsh,1)
        gmsh.option.setNumber('General.Terminal',0)
        for stencil in data['stencils']:
            gmsh.clear();gmsh.model.add('independent_core_side_closure')
            x=np.asarray(stencil['coordinates']);faces=np.asarray(stencil['topology'])
            gmsh.model.addDiscreteEntity(2,1)
            gmsh.model.mesh.addNodes(2,1,[1,2,3,4],x.ravel().tolist())
            gmsh.model.mesh.addElementsByType(1,2,[1,2,3,4],(faces+1).ravel().tolist())
            gmsh.model.mesh.reclassifyNodes();gmsh.model.mesh.createTopology(False,True)
            gmsh.model.addDiscreteEntity(3,1,[1]);gmsh.model.mesh.createGeometry(gmsh.model.getEntities(3))
            gmsh.model.mesh.generate(3)
            core=extract_all_volumes(gmsh,old)
            assert len(core['tets'])==1 and len(core['coordinates'])==4
            assert {p.tobytes() for p in x}=={p.tobytes() for p in core['coordinates']}
            rows.append(dict(shell_cell=stencil['shell_cell'],tetrahedra=1,coordinates_exact=True))
    finally:gmsh.finalize()
    result=dict(passed=True,closure_identity=data['closure_identity'],closures=len(rows),
        generated_tets=len(rows),rows=rows,seconds=time.perf_counter()-tick,
        peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
        scope='Independent diagnostic closed volumes. Not a stitched or final core authority.')
    path.with_name(path.stem+'_GMSH.json').write_text(json.dumps(result,sort_keys=True,indent=2)+'\n')


if __name__=='__main__':run(Path(sys.argv[1]))
