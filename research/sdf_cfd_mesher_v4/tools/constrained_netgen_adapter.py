"""Import a validated compact cavity with complete Netgen vertex ownership.

Gmsh 4.15.2 buildNetgenStructure imports region mesh_vertices and boundary
vertices, not vertices on embedded point entities. All strictly interior mesh
vertices must therefore be volume-classified before Netgen optimization.
"""
import numpy as np

def load_volume_classified(gmsh,x,t,faces):
    boundary=np.unique(faces);used=np.unique(t);inside=np.setdiff1d(used,boundary)
    if not set(boundary)<=set(used):raise ValueError('UnusedCavityBoundaryVertex')
    gmsh.clear();gmsh.model.add('validated_volume_classification')
    gmsh.model.addDiscreteEntity(2,1)
    gmsh.model.mesh.addNodes(2,1,(boundary+1).tolist(),x[boundary].ravel().tolist())
    gmsh.model.mesh.addElementsByType(1,2,np.arange(1,len(faces)+1).tolist(),(faces+1).ravel().tolist())
    gmsh.model.addDiscreteEntity(3,1,[1])
    gmsh.model.mesh.addNodes(3,1,(inside+1).tolist(),x[inside].ravel().tolist())
    gmsh.model.mesh.addElementsByType(1,4,np.arange(len(faces)+1,len(faces)+1+len(t)).tolist(),(t+1).ravel().tolist())
    gmsh.model.mesh.createGeometry(gmsh.model.getEntities(3))
    return dict(boundary_vertices=len(boundary),volume_vertices=len(inside),embedded_point_vertices=0)
