"""Analytic SDF fixture authorities, isolated from the accepted SF2 geometry."""
from types import SimpleNamespace
import numpy as np

class AnalyticSDF:
    def __init__(self, source): self.source=dict(source)
    def evaluate(self, points):
        x=np.asarray(points,float);s=self.source;k=s['kind'];n=len(x)
        if k=='parallel_gap':
            phi=s['spacing']/2-np.abs(x[:,0]);g=np.zeros_like(x);g[:,0]=-np.sign(x[:,0])
        elif k in ('spherical_cavity','cylindrical_cavity'):
            v=x.copy()
            if k=='cylindrical_cavity':v[:,2]=0
            length=np.linalg.norm(v,axis=1);phi=s['radius']-length
            g=np.divide(-v,length[:,None],out=np.zeros_like(v),where=length[:,None]!=0)
        elif k=='open_wedge':
            phi=(3*x[:,0]-4*np.abs(x[:,2]))/5;g=np.zeros_like(x);g[:,0]=.6;g[:,2]=-.8*np.sign(x[:,2])
        else:raise ValueError(k)
        norm=np.linalg.norm(g,axis=1);normal=np.divide(g,norm[:,None],out=np.zeros_like(g),where=norm[:,None]!=0)
        # Evaluation-roundoff envelope, not a geometric clearance/gate.
        error=8*np.finfo(float).eps*(1+np.linalg.norm(x,axis=1)+abs(s.get('radius',s.get('spacing',0))))
        return SimpleNamespace(phi=phi,gradient=g,gradient_norm=norm,normal=normal,source_error_bound=error,
                               section_curvature=np.zeros(n),boundary_normal_selection=np.zeros(n,dtype=np.int64))


def cohort(targets):
    midpoint=lambda k:float((targets[k-2]+targets[k-1])/2)
    return [
        dict(id='parallel_limited',kind='parallel_gap',spacing=2*midpoint(10)),
        dict(id='parallel_feasible',kind='parallel_gap',spacing=4*float(targets[-1])),
        dict(id='near_contact',kind='parallel_gap',spacing=float(targets[0])),
        dict(id='symmetric_contact',kind='parallel_gap',spacing=2*float(targets[7])),
        dict(id='concave_sphere',kind='spherical_cavity',radius=midpoint(13)),
        dict(id='concave_cylinder',kind='cylindrical_cavity',radius=midpoint(16)),
        dict(id='curved_feasible',kind='spherical_cavity',radius=2*float(targets[-1])),
        *[dict(id='narrowing_station_'+str(i),kind='open_wedge',station=float(d)*5/3) for i,d in enumerate((midpoint(5),midpoint(11),midpoint(18)))]]


def triangle(source, depth=0.):
    k=source['kind']
    if k=='parallel_gap':
        x=-source['spacing']/2+depth
        return np.array([[x,0.,0.],[x,1.,0.],[x,0.,1.]])
    if k=='spherical_cavity':
        r=source['radius']-depth
        return np.array([[r,0.,0.],[0.,0.,r],[0.,r,0.]])
    if k=='cylindrical_cavity':
        r=source['radius']-depth
        return np.array([[r,0.,0.],[r,0.,1.],[0.,r,0.]])
    x=source['station']; z=.75*x-1.25*depth
    return np.array([[x,0,z],[x+1,0,z+.75],[x,1,z]])
