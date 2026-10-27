"""Existing WSL Gmsh: one prescribed tetrahedral cavity outside an interface."""
import json,sys,math
from pathlib import Path
import gmsh
p=Path(sys.argv[1]);z=json.loads(p.read_text());x=z['coordinates'];tri=z['triangles']
index=min(range(len(tri)),key=lambda i:sum(x[j-1][0] for j in tri[i]))
ids=tri[index];a,b,c=[x[i-1] for i in ids]
u=[b[j]-a[j] for j in range(3)];v=[c[j]-a[j] for j in range(3)]
n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];nn=math.sqrt(sum(w*w for w in n))
h=min(math.dist(a,b),math.dist(b,c),math.dist(c,a))/2
d=[(a[j]+b[j]+c[j])/3+h*n[j]/nn for j in range(3)];points=[a,b,c,d]
faces=[(1,3,2),(1,2,4),(2,3,4),(3,1,4)]
gmsh.initialize(['bounded-tet','-nopopup']);gmsh.option.setNumber('General.Terminal',0);gmsh.option.setNumber('General.NumThreads',1)
try:
    gmsh.model.add('single_local_core_cavity');geo=gmsh.model.geo
    for i,q in enumerate(points,1):geo.addPoint(*q,2*h,i)
    edges={};surfaces=[]
    for f in faces:
        lines=[]
        for i,j in zip(f,f[1:]+f[:1]):
            key=tuple(sorted((i,j)))
            if key not in edges:edges[key]=geo.addLine(*key)
            lines.append(edges[key] if i<j else -edges[key])
        surfaces.append(geo.addPlaneSurface([geo.addCurveLoop(lines)]))
    geo.addVolume([geo.addSurfaceLoop(surfaces)]);geo.synchronize()
    gmsh.option.setNumber('Mesh.MeshSizeMin',10*h);gmsh.option.setNumber('Mesh.MeshSizeMax',10*h)
    gmsh.option.setNumber('Mesh.Optimize',0);gmsh.model.mesh.generate(3)
    tags,coords,_=gmsh.model.mesh.getNodes();types,element_tags,nodes=gmsh.model.mesh.getElements(3)
    result=dict(base_face=index,base_vertex_ids=ids,apex=d,gmsh_version=gmsh.__version__,nodes={str(int(tag)):coords[3*i:3*i+3].tolist() for i,tag in enumerate(tags)},
        element_types=list(map(int,types)),elements=[v.tolist() for v in nodes],prescribed_points=points,
        scope='One small prescribed cavity outside the leftmost curved-passage face; not a global core')
finally:gmsh.finalize()
(p.parent/'TET_SMOKE_RAW.json').write_text(json.dumps(result,sort_keys=True)+'\n',encoding='utf8')
