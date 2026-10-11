"""Study-only subdomain covering; NOT used by the optimized production path.

The measured five residual CAD pairs did not admit this bounded early proof.

No unsuccessful covering proves anything. Ownership on a subdomain boundary
can only come from an original common vertex or original common perimeter edge.
"""
from fractions import Fraction as Q
from .incremental import kernel

def subdivided_separation(a,b,depth=1):
    if depth not in (1,2):raise ValueError('ComputationalSubdivisionBound')
    r=kernel.module('rank3_certificate');contact=r.contact(a,b)
    if len(contact['vertices'])>2 or (len(contact['vertices'])==2 and len(contact['edges'])!=1):return None
    if not a.valid()[0] or not b.valid()[0]:return None
    def label(p,owner,u,v):
        for i,uv in enumerate(r.PARAM_CORNERS):
            if (u,v)==uv:return ('vertex',p.ids[i])
        for i,j in r.EDGE_CORNERS:
            x,y=r.PARAM_CORNERS[i],r.PARAM_CORNERS[j]
            if x[0]==y[0] and u==x[0]:t=(v-x[1])/Q(y[1]-x[1])
            elif x[1]==y[1] and v==x[1]:t=(u-x[0])/Q(y[0]-x[0])
            else:continue
            lo,hi=p.ids[i],p.ids[j]
            if str(lo)>str(hi):lo,hi,t=hi,lo,1-t
            return ('edge',lo,hi,t)
        return ('interior',owner,u,v)
    def pieces(p,owner):
        n=2**depth;out=[]
        for i in range(n):
            for j in range(n):
                box=(Q(i,n),Q(i+1,n),Q(j,n),Q(j+1,n))
                uv=((box[0],box[2]),(box[1],box[2]),(box[0],box[3]),(box[1],box[3]))
                out.append(kernel.Patch(p.subcontrol(box),tuple(label(p,owner,*q) for q in uv)))
        return out
    proof=[]
    for i,pa in enumerate(pieces(a,'a')):
        for j,pb in enumerate(pieces(b,'b')):
            c=r.contact(pa,pb);lo,hi=pa.bounds;lb,hb=pb.bounds
            if any(hi[k]<lb[k] or hb[k]<lo[k] for k in range(3)):method='ExactSubdomainAABB'
            else:
                result=r.supporting_planes(pa,pb,c)
                if result is None:return None
                method=result['method']
            proof.append([i,j,method])
    return dict(classification='CertifiedAllowedIncidentContact' if contact['vertices'] or contact['edges'] else 'CertifiedSeparated',
        method='ExactDyadicSubdomainCover',depth=depth,subdomain_proofs=proof,
        ownership='Restriction of original ordered topology; interior labels are private to each original surface')
