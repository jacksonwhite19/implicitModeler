"""Run with the existing WSL Python/Gmsh. Ingest only; no core generation."""
import json,sys,hashlib
from pathlib import Path
import gmsh
p=Path(sys.argv[1]);data=json.loads(p.read_text());xyz=data['coordinates'];tri=data['triangles']
gmsh.initialize(['cavity-ingestion','-nopopup']);gmsh.option.setNumber('General.Terminal',0);gmsh.option.setNumber('General.NumThreads',1)
try:
    gmsh.model.add('bounded_repaired_outer_interface');gmsh.model.addDiscreteEntity(2,1)
    gmsh.model.mesh.addNodes(2,1,list(range(1,len(xyz)+1)),[x for row in xyz for x in row])
    gmsh.model.mesh.addElementsByType(1,2,list(range(1,len(tri)+1)),[x for row in tri for x in row])
    tags,coords,_=gmsh.model.mesh.getNodes();mapping={int(tag):coords[3*i:3*i+3].tolist() for i,tag in enumerate(tags)}
    types,etags,nodes=gmsh.model.mesh.getElements(2,1)
    assert list(types)==[2] and nodes[0].tolist()==[x for row in tri for x in row]
    assert all(mapping[i+1]==row for i,row in enumerate(xyz))
    gmsh.model.mesh.createEdges();edges,edge_nodes=gmsh.model.mesh.getAllEdges()
    result=dict(passed=True,gmsh_version=gmsh.__version__,nodes=len(tags),triangles=len(tri),edges=len(edges),
        exact_coordinate_roundtrip=True,exact_connectivity_roundtrip=True,input_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
        scope='Discrete surface ingestion only; no tetrahedral core or CFD adequacy claim')
finally:gmsh.finalize()
(p.parent/'CORE_INGESTION.json').write_text(json.dumps(result,sort_keys=True)+'\n',encoding='utf8')
print(result)
