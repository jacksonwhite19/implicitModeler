"""Direct numerical descriptors. No risk thresholds or health classes."""
import numpy as np

def angle(a,b):
    return np.arccos(np.clip(np.einsum('...i,...i->...',a,b),-1.,1.))
def unit(v):
    n=np.linalg.norm(v,axis=-1)
    return np.divide(v,n[...,None],out=np.zeros_like(v),where=n[...,None]!=0),n
def edge_descriptors(current,previous,base,normals,edges):
    a,b=edges.T;v=current[b]-current[a];pv=previous[b]-previous[a];bv=base[b]-base[a]
    uv,length=unit(v);_,pl=unit(pv);_,bl=unit(bv)
    dn=normals[b]-normals[a]
    dot=np.einsum('ni,ni->n',normals[a],normals[b])
    relative=(current[b]-previous[b])-(current[a]-previous[a])
    return dict(length=length,ratio=np.divide(length,bl,out=np.full_like(length,np.nan),where=bl!=0),normal_dot=dot,normal_angle=np.arccos(np.clip(dot,-1,1)),separation_change=length-pl,normal_convergence=np.einsum('ni,ni->n',dn,uv),relative_motion_along_edge=np.einsum('ni,ni->n',relative,uv))
def triangle_geometry(points,faces):
    p=points[faces];cross=np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0]);normal,twice=unit(cross)
    lengths=np.linalg.norm(p[:,[1,2,0]]-p[:,[0,1,2]],axis=2)
    return normal,twice*.5,lengths
def triangle_descriptors(current,previous,base,faces):
    n,a,l=triangle_geometry(current,faces);pn,pa,_=triangle_geometry(previous,faces);_,ba,_=triangle_geometry(base,faces)
    dot=np.einsum('ni,ni->n',n,pn)
    finite=np.isfinite(current[faces]).all(axis=(1,2))
    return dict(area=a,ratio=np.divide(a,ba,out=np.full_like(a,np.nan),where=ba!=0),normal=n,previous_orientation=dot,min_edge=l.min(1),max_edge=l.max(1),finite=finite,nondegenerate=a!=0,orientation_reversal=dot<0)
