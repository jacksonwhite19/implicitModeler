"""Core-versus-complete-input side candidates; frozen production pair kernel.

Bilinear patches lie in the convex hull of their four binary64 vertices.
Swept edge AABBs conservatively include all 20 layers. The broad scan reads all
edges; exact work uses only bounded candidate sides. It never grows a recursive
connected ownership domain. Both same-layer and cross-layer pairs are covered.
"""
import time,hashlib
from collections import Counter
import numpy as np
from ...certificate.continuous_prism_certificate import kernel
from ...certificate.continuous_prism_certificate.runtime import CertificateSession,canonical_bytes
from .patch import LocalityFailure

class SideIndex:
    def __init__(self,front,forecast,topology):
        self.front=front;self.forecast=forecast;self.topology=topology
        # AABB storage scales with source edges, never columns times DOFs.
        self.lo=np.minimum(front.coordinates[topology.edges[:,0]],front.coordinates[topology.edges[:,1]])
        self.hi=np.maximum(front.coordinates[topology.edges[:,0]],front.coordinates[topology.edges[:,1]])
        for x in forecast.vertex['coordinates']:
            self.lo=np.minimum(self.lo,np.minimum(x[topology.edges[:,0]],x[topology.edges[:,1]]))
            self.hi=np.maximum(self.hi,np.maximum(x[topology.edges[:,0]],x[topology.edges[:,1]]))

    def candidates(self,core_edges,*,max_edges=2048):
        selected=np.zeros(len(self.lo),bool)
        padding=128*np.finfo(float).eps # unchanged qualified broad-phase padding
        for e in core_edges:
            selected|=np.all(self.hi>=self.lo[e]-padding,axis=1)&np.all(self.lo<=self.hi[e]+padding,axis=1)
        ids=np.flatnonzero(selected)
        if len(ids)>max_edges or len(ids)==len(self.lo):raise LocalityFailure('ContinuousCandidateScopeExceedsCallerBound')
        return ids

    def certify(self,patch,*,session=None,max_edges=2048,seconds_budget=180,progress=None):
        started=time.perf_counter();edges=self.candidates(patch.side_edges,max_edges=max_edges)
        ep=self.topology.edges[edges];q=[];labels=[];sideids=[];core=set(map(int,patch.side_edges))
        previous=self.front.coordinates
        for k,x in enumerate(self.forecast.vertex['coordinates'],1):
            q.extend(np.stack((previous[ep[:,0]],previous[ep[:,1]],x[ep[:,0]],x[ep[:,1]]),axis=1))
            labels.extend(((k-1,int(a)),(k-1,int(b)),(k,int(a)),(k,int(b))) for a,b in ep)
            sideids.extend((k,int(e)) for e in edges);previous=x
        q=np.asarray(q);lo,hi=kernel.enumerator.boxes(q)
        pairs,_=kernel.enumerator.bvh_pairs(lo,hi)
        pairs=sorted((a,b) for a,b in pairs if sideids[a][1] in core or sideids[b][1] in core)
        session=CertificateSession() if session is None else session
        exact={};counts=Counter();methods=Counter();h=hashlib.sha256();evidence=[]
        def get(i):
            if i not in exact:
                p=kernel.Patch.binary(q[i],labels[i]);valid,reason=p.valid()
                if not valid:raise ValueError('InvalidSidePatch:'+str(reason))
                exact[i]=p
            return exact[i]
        for i,(a,b) in enumerate(pairs):
            if time.perf_counter()-started>seconds_budget:
                return dict(classification='LocalContinuousEvidenceIncomplete',reason='OperationalPairTimeBudget',completed_pairs=i,candidate_pairs=len(pairs),counts=dict(counts),methods=dict(methods),records_sha256=h.hexdigest(),seconds=time.perf_counter()-started,candidate_edges=edges.tolist())
            r=session.pair(get(a),get(b));record=dict(side_a=sideids[a],side_b=sideids[b],**r)
            h.update(canonical_bytes(record));counts[r['classification']]+=1;methods[r['method']]+=1
            if progress and (i%50==0):progress(dict(completed=i+1,total=len(pairs),seconds=time.perf_counter()-started,last_pair=[sideids[a],sideids[b]]))
            if r['classification'] not in ('CertifiedSeparated','CertifiedAllowedIncidentContact'):
                return dict(classification='LocalContinuousEvidenceIncomplete',reason='NonpassingProductionPair',failure=record,completed_pairs=i+1,candidate_pairs=len(pairs),counts=dict(counts),methods=dict(methods),records_sha256=h.hexdigest(),seconds=time.perf_counter()-started,candidate_edges=edges.tolist())
        # Core patch validity must also be checked if a side had no partner.
        for i,(_,e) in enumerate(sideids):
            if e in core:get(i)
        return dict(classification='LocalCoreSidePairsCertified',completed_pairs=len(pairs),candidate_pairs=len(pairs),counts=dict(counts),methods=dict(methods),records_sha256=h.hexdigest(),seconds=time.perf_counter()-started,candidate_edges=edges.tolist(),complete_source_edges=len(self.lo),layers=20,cross_layer_pairs_included=True,side_cap_and_cell_overlap_certificate=False)
