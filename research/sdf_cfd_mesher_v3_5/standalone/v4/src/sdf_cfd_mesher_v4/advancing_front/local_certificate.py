"""Bounded production-certificate evidence for proposed and forecast geometry."""
from collections import Counter
import hashlib,time
import numpy as np
from ..certificate.continuous_prism_certificate import kernel
from ..certificate.continuous_prism_certificate.runtime import CertificateSession,canonical_bytes
from ..forecast.detailed.continuous import SideIndex


class CertificateStop(RuntimeError):
    """Fail closed without altering the qualified mathematical kernel."""
    def __init__(self,error):
        self.record=dict(classification='ExactCertificateEvaluationUnavailable',exception_type=type(error).__name__,message=str(error))
        super().__init__('CertificateCannotClose:'+str(error))


def production_pair(session,a,b):
    try:return session.pair(a,b)
    except ArithmeticError as error:raise CertificateStop(error) from error


def updated_index(original,forecast,changed_edges):
    index=object.__new__(SideIndex);index.front=original.front;index.topology=original.topology;index.forecast=forecast
    index.lo=original.lo.copy();index.hi=original.hi.copy();e=np.asarray(changed_edges,dtype=int);pairs=index.topology.edges[e]
    lo=np.minimum(index.front.coordinates[pairs[:,0]],index.front.coordinates[pairs[:,1]])
    hi=np.maximum(index.front.coordinates[pairs[:,0]],index.front.coordinates[pairs[:,1]])
    for x in forecast.vertex['coordinates']:
        lo=np.minimum(lo,np.minimum(x[pairs[:,0]],x[pairs[:,1]]));hi=np.maximum(hi,np.maximum(x[pairs[:,0]],x[pairs[:,1]]))
    index.lo[e]=lo;index.hi[e]=hi
    return index


def certify_all(index,patch,*,session=None,seconds_budget=180,max_pairs=250000):
    """Collect exact forbidden pairs; do not mistake one witness for full scope."""
    started=time.perf_counter();edges=index.candidates(patch.side_edges);ep=index.topology.edges[edges]
    q=[];labels=[];sideids=[];previous=index.front.coordinates;core=set(map(int,patch.side_edges))
    for k,x in enumerate(index.forecast.vertex['coordinates'],1):
        q.extend(np.stack((previous[ep[:,0]],previous[ep[:,1]],x[ep[:,0]],x[ep[:,1]]),axis=1))
        labels.extend(((k-1,int(a)),(k-1,int(b)),(k,int(a)),(k,int(b))) for a,b in ep)
        sideids.extend((k,int(e)) for e in edges);previous=x
    q=np.asarray(q);lo,hi=kernel.enumerator.boxes(q);pairs,_=kernel.enumerator.bvh_pairs(lo,hi)
    pairs=sorted(((a,b) for a,b in pairs if sideids[a][1] in core or sideids[b][1] in core),key=lambda ab:(max(sideids[ab[0]][0],sideids[ab[1]][0]),sideids[ab[0]],sideids[ab[1]]))
    if len(pairs)>max_pairs:return dict(complete=False,reason='ComputationalPairBound',candidate_pairs=len(pairs))
    exact={};counts=Counter();bad=[];invalid=[];unresolved=[];h=hashlib.sha256();session=session or CertificateSession()
    def get(i):
        if i not in exact:
            p=kernel.Patch.binary(q[i],labels[i]);valid,reason=p.valid();exact[i]=p if valid else None
            if not valid:invalid.append(dict(side=sideids[i],reason=reason))
        return exact[i]
    for i,(a,b) in enumerate(pairs):
        if time.perf_counter()-started>seconds_budget:
            return dict(complete=False,reason='ComputationalCertificateTimeBound',completed_pairs=i,candidate_pairs=len(pairs),forbidden=bad,invalid_surfaces=invalid,unresolved=unresolved)
        pa,pb=get(a),get(b)
        if pa is None or pb is None:continue
        r=production_pair(session,pa,pb);record=dict(side_a=sideids[a],side_b=sideids[b],**r);h.update(canonical_bytes(record));counts[r['classification']]+=1
        if r['classification']=='CertifiedForbiddenIntersection':bad.append(record)
        elif r['classification'] not in ('CertifiedSeparated','CertifiedAllowedIncidentContact'):unresolved.append(record)
    for i,(_,e) in enumerate(sideids):
        if e in core:get(i)
    return dict(complete=True,candidate_pairs=len(pairs),completed_pairs=len(pairs),candidate_source_edges=edges.tolist(),counts=dict(counts),forbidden=bad,
                invalid_surfaces=invalid,unresolved=unresolved,records_sha256=h.hexdigest(),layers=len(index.forecast.vertex['coordinates']),
                complete_input_edges=len(index.lo),scope='Core side images versus complete conservative side candidates; no side-cap/full-volume certificate')


def certificate_passed(record):
    return record.get('complete',False) and not record.get('forbidden') and not record.get('invalid_surfaces') and not record.get('unresolved')


def intersection_identity(record):
    return tuple(sorted((tuple(record['side_a']),tuple(record['side_b']))))


def pair_removed(before,after):
    return before['classification']=='CertifiedForbiddenIntersection' and after['classification'] in ('CertifiedSeparated','CertifiedAllowedIncidentContact')


def new_intersections(before,after):
    old={intersection_identity(r) for r in before.get('forbidden',[])}
    return [r for r in after.get('forbidden',[]) if intersection_identity(r) not in old]
