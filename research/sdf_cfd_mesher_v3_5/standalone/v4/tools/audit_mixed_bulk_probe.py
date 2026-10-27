"""Mixed-owner interface gates using established Foundation pyramid sums."""
import sys
sys.dont_write_bytecode=True
import numpy as np,json
from repair_bulk_core import OUT,V3,load,write

def centres(p,fg):
    if p.shape[1]==6:return fg.prism_cell_centres(p)
    patterns=([[0,2,1],[0,1,3],[1,2,3],[2,0,3]] if p.shape[1]==4 else [[0,3,2,1],[0,1,4],[1,2,4],[2,3,4],[3,0,4]])
    values=[(fg.face_area_centre_tri if len(f)==3 else fg.face_area_centre_quad)(p[:,f]) for f in patterns]
    fa=np.stack([v[0] for v in values],axis=1);fc=np.stack([v[1] for v in values],axis=1)
    fa=np.where((np.einsum('nij,nij->ni',fa,fc-p.mean(1)[:,None])<0)[:,:,None],-fa,fa)
    ce=fc.mean(1);v3=np.einsum('nij,nij->ni',fa,fc-ce[:,None]);total=v3.sum(1)
    return (v3[:,:,None]*(.75*fc+.25*ce[:,None])).sum(1)/total[:,None],total/3

def main():
    folder=OUT/'mixed_core';interface=OUT/'mixed_owner_corrected';fg=load('mixed_foundation',V3/'foundation_aware_prism_shell_qualification/foundation_geometry.py')
    x=np.load(folder/'coordinates.npy');tets=np.load(folder/'tets.npy');own=np.load(folder/'interface_owner.npy');tags=np.load(folder/'node_tags.npy');it=np.load(folder/'interface_tags.npy')
    order=np.argsort(tags);idx=order[np.searchsorted(tags[order],it)]
    sx=np.load(interface/'points.npy');assert np.array_equal(sx.view('u8'),x[idx].view('u8'))
    sc=np.empty((len(own),3));sv=np.empty(len(own));counts={}
    for arity in [4,5,6]:
        p=np.load(interface/f'owner_geometry_{arity}.npy');ids=np.load(interface/f'owner_faces_{arity}.npy');sc[ids],sv[ids]=centres(p,fg);counts[str(arity)]=len(ids)
    tc,tv=centres(x[tets],fg);faces=np.load(interface/'connectivity.npy').reshape(-1,3);q=sx[faces];a,fc=fg.face_area_centre_tri(q)
    da=np.abs(np.einsum('ij,ij->i',a,fc-sc));db=np.abs(np.einsum('ij,ij->i',a,tc[own]-fc));w=np.minimum(da,db)/(da+db);r=np.minimum(sv,tv[own])/np.maximum(sv,tv[own]);sk=fg.internal_face_skewness(q,fc,a,sc,tc[own])
    result=dict(interface_exact=True,owner_arities=counts,weight_min=float(w.min()),ratio_min=float(r.min()),skew_max=float(sk.max()),weight_failures=int((w<.055).sum()),ratio_failures=int((r<.01).sum()),skew_failures=int((sk>3.63636).sum()),core=json.loads((folder/'HARD_QUALITY.json').read_text()))
    result['passed']=result['core']['passed'] and not any(result[k] for k in ['weight_failures','ratio_failures','skew_failures'])
    write('MIXED_REGRESSION.json',result)

if __name__=='__main__':main()
