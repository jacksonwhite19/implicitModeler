"""Exact convex-hull screening and unchanged bilinear-kernel face tilings."""
from itertools import combinations
import local_termination_topology as t
np=t.np;F=t.F;q=t.q
kernel=q.kernel

def rational(points):return [tuple(F(float(x)) for x in p) for p in points]
def legal_ids(a,b):
    shared=set(a)&set(b)
    ea={frozenset((x,y)) for x,y in zip(a,a[1:]+a[:1])}
    eb={frozenset((x,y)) for x,y in zip(b,b[1:]+b[:1])}
    return shared,ea&eb

def hull_proof(points,a,b):
    """Float ordering only chooses exact predicates; never accepts float signs."""
    A=points[list(a)];B=points[list(b)];V=np.r_[A,B];recipes=[];axes=[]
    edgesA=list(combinations(range(len(a)),2));edgesB=list(combinations(range(len(a),len(V)),2))
    for ids in list(combinations(range(len(a)),3))+list(combinations(range(len(a),len(V)),3)):
        i,j,k=ids;recipes.append(('face',i,j,k));axes.append(np.cross(V[j]-V[i],V[k]-V[i]))
    for i,j in edgesA:
        for k,l in edgesB:recipes.append(('edges',i,j,k,l));axes.append(np.cross(V[j]-V[i],V[l]-V[k]))
    # In-plane axes handle coplanar polygons.
    for offset,n in ((0,len(a)),(len(a),len(b))):
        for i,j in combinations(range(offset,offset+n),2):
            recipes.append(('inplane',offset,offset+1,offset+2,i,j))
            axes.append(np.cross(np.cross(V[offset+1]-V[offset],V[offset+2]-V[offset]),V[j]-V[i]))
    axes=np.array(axes);scale=np.linalg.norm(axes,axis=1);norm=np.divide(axes,scale[:,None],out=np.zeros_like(axes),where=scale[:,None]!=0)
    aa=A@norm.T;bb=B@norm.T;gaps=np.maximum(bb.min(0)-aa.max(0),aa.min(0)-bb.max(0));order=np.argsort(-gaps,kind='stable')
    exact=rational(V);shared,edges=legal_ids(a,b);labels=list(a)+list(b)
    def zero_legal(indices):
        ids={labels[i] for i in indices}
        return bool(ids) and (len(ids)==1 and ids<=shared or any(ids<=e for e in edges))
    for index in order:
        recipe=recipes[index];kind,*ids=recipe
        if kind=='face':i,j,k=ids;n=t.cross(t.sub(exact[j],exact[i]),t.sub(exact[k],exact[i]))
        elif kind=='edges':i,j,k,l=ids;n=t.cross(t.sub(exact[j],exact[i]),t.sub(exact[l],exact[k]))
        else:i,j,k,l,m=ids;n=t.cross(t.cross(t.sub(exact[j],exact[i]),t.sub(exact[k],exact[i])),t.sub(exact[m],exact[l]))
        if not any(n):continue
        values=[t.dot(n,x) for x in exact];x=values[:len(a)];y=values[len(a):]
        for lo,hi,left,right in ((max(x),min(y),range(len(a)),range(len(a),len(V))),(max(y),min(x),range(len(a),len(V)),range(len(a)))):
            if lo<hi:return 'ExactHullSeparated'
            if lo==hi:
                za=[i for i in left if values[i]==lo];zb=[i for i in right if values[i]==hi]
                if zero_legal(za) or zero_legal(zb):return 'ExactHullLegalContact'
    return None

def tiles(points,face):
    """Partition a triangle into 3 and a bilinear quad into 4 exact quads.

    All shared edges use the same exact midpoint and ID. Rational arithmetic
    avoids approximating triangle centroids. No original face is planarized.
    """
    p=rational(points[list(face)]);n=len(face);mid=[tuple((p[i][j]+p[(i+1)%n][j])/2 for j in range(3)) for i in range(n)]
    centre=tuple(sum(v[j] for v in p)/n for j in range(3));cid='face:'+','.join(map(str,sorted(face)))
    edgeids=['edge:'+','.join(map(str,sorted((face[i],face[(i+1)%n])))) for i in range(n)]
    out=[]
    for i in range(n):
        poly=[p[i],mid[i],centre,mid[(i-1)%n]];ids=[str(face[i]),edgeids[i],cid,edgeids[(i-1)%n]]
        patch=kernel.Patch(tuple(poly[j] for j in (0,1,3,2)),tuple(ids[j] for j in (0,1,3,2)))
        valid,why=patch.valid()
        if not valid:raise ValueError('InvalidFaceTile:'+why)
        out.append(patch)
    return out

def pair(points,a,b):
    proof=hull_proof(points,a,b)
    if proof:return dict(classification='CertifiedSeparated' if proof=='ExactHullSeparated' else 'CertifiedAllowedIncidentContact',method=proof)
    results=[];session=q.CertificateSession()
    for aa in tiles(points,a):
        for bb in tiles(points,b):
            z=session.pair(aa,bb);results.append(z)
            if z['classification'] not in ('CertifiedSeparated','CertifiedAllowedIncidentContact'):return dict(classification=z['classification'],method='ExactFaceTiling',detail=z)
    return dict(classification='CertifiedAllowedIncidentContact' if any(z['classification']=='CertifiedAllowedIncidentContact' for z in results) else 'CertifiedSeparated',method='ExactFaceTiling',subproofs=results)

def certify(points,faces,active):
    lo=np.array([points[list(f)].min(0) for f in faces]);hi=np.array([points[list(f)].max(0) for f in faces]);active=set(active);rows=[];counts=t.Counter();bad=[]
    for i in range(len(faces)):
        candidates=np.flatnonzero(np.all(hi[i]>=lo[i+1:],axis=1)&np.all(hi[i+1:]>=lo[i],axis=1))+i+1
        for j in candidates:
            if i not in active and int(j) not in active:continue
            z=pair(points,faces[i],faces[int(j)]);counts[z['method']]+=1
            row=dict(pair=[i,int(j)],**z);rows.append(row)
            if z['classification'] not in ('CertifiedSeparated','CertifiedAllowedIncidentContact'):bad.append(row)
            if len(rows)%2000==0:print('exact faces',len(rows),'failures',len(bad),flush=True)
    return dict(passed=not bad,required_pairs=len(rows),methods=counts,failures=bad,proof_hash=q.identity(rplain(rows)),proofs=rows,
        coverage='All AABB-overlapping unique-face pairs touching changed geometry; control hulls bound exact bilinear and planar images')

def rplain(x):return t.r.w.clean(x)

def triangle_intersection_points(points,a,b):
    """Independent rational segment/plane clipping for the frozen-cap audit."""
    A=rational(points[list(a)]);B=rational(points[list(b)]);out=set()
    for X,Y in ((A,B),(B,A)):
        n=t.cross(t.sub(Y[1],Y[0]),t.sub(Y[2],Y[0]))
        half=[t.cross(n,t.sub(Y[(i+1)%3],Y[i])) for i in range(3)]
        def inside(p):return all(t.dot(h,t.sub(p,y))>=0 for h,y in zip(half,Y))
        for i in range(3):
            p=X[i];end=X[(i+1)%3];d=t.sub(end,p);s=t.dot(n,t.sub(p,Y[0]));e=t.dot(n,t.sub(end,Y[0]))
            if s==e==0:
                lo=F(0);hi=F(1);empty=False
                for h,y in zip(half,Y):
                    v=t.dot(h,t.sub(p,y));slope=t.dot(h,d)
                    if slope>0:lo=max(lo,-v/slope)
                    elif slope<0:hi=min(hi,-v/slope)
                    elif v<0:empty=True
                if not empty and lo<=hi:
                    out.update(tuple(p[j]+alpha*d[j] for j in range(3)) for alpha in (lo,hi))
            elif s!=e:
                alpha=s/(s-e)
                if 0<=alpha<=1:
                    x=tuple(p[j]+alpha*d[j] for j in range(3))
                    if inside(x):out.add(x)
    return sorted(out)
