"""Deterministic scalar front construction; proposals never certify filling."""
from fractions import Fraction
import math
import time
import numpy as np
from background_local_repair import star_index
from background_topology_repair import key
from finite_support import transition_front_targets as targets
from finite_research.flexible_controls import validate
from sdf_cfd_mesher_v4.common.authority import array_hash,identity


class SearchExpired(TimeoutError):
    pass


def rules(values,target):
    scores=np.asarray(values,dtype=float)
    if scores.ndim!=1 or not len(scores) or not np.isfinite(scores).all():
        raise ValueError('InvalidPollScorePopulation')
    result=[]
    threshold=target['minimum']
    result.append((int(np.count_nonzero(scores<threshold)),
        float(np.maximum(threshold-scores,0).sum())))
    sorted_scores=None
    for row in target['population_targets']:
        need=math.ceil(Fraction(str(row['fraction']))*len(scores))
        count=int(np.count_nonzero(scores>=row['minimum']))
        if sorted_scores is None:sorted_scores=np.sort(scores,kind='stable')
        best=sorted_scores[-need:]
        result.append((max(0,need-count),float(np.maximum(row['minimum']-best,0).sum())))
    return tuple(result)


def merit(populations,policy):
    return tuple(rule for name in targets.POPULATIONS
        for rule in rules(populations[name],policy[name]))


def improves(before,after):
    if len(before)!=6 or len(after)!=6:raise ValueError('SixPollRulesRequired')
    if any((old[0]==0 and new[0]!=0) or (old[0]>0 and new[0]>old[0])
            for old,new in zip(before,after)):
        return False
    return tuple(map(sum,zip(*after)))<tuple(map(sum,zip(*before)))


def complete_scope(star,owners,byid):
    changed={cell['id'] for cell in star}
    faces={key(face):owners[key(face)] for cell in star for face in cell['faces']}
    needed={owner for uses in faces.values() for owner,_ in uses}
    if (not star or not changed<=needed or not needed<=set(byid)
            or any(len(uses) not in (1,2) for uses in faces.values())):
        raise ValueError('IncompletePollNeighbourClosure')
    for face,uses in faces.items():
        for owner,oriented in uses:
            if key(oriented)!=face or not any(key(candidate)==face for candidate in byid[owner]['faces']):
                raise ValueError('ChangedPollOwnerIncidence')
    return changed,faces,needed


def population_arrays(state,points):
    tets,faces=targets.populations(state)
    return {targets.POPULATIONS[0]:targets.tetrahedron_ratio(points,tets),
        targets.POPULATIONS[1]:targets.triangle_ratio(points,faces)}


class Poll:
    def __init__(self,state,check,quality):
        self.started=time.perf_counter();self.search_deadline=None
        self.state=state;self.check=check;self.quality=quality
        self.controls=validate(state['controls']);self.extent=self.controls['envelope']
        self.points=state['points'].copy();self.initial=self.points.copy()
        self.front=np.asarray(state['outer_vertex_ids'],dtype='<i8')
        self.cap=np.asarray(state['cap_ids'],dtype='<i8');self.cap_faces=np.asarray(state['cap_faces'])
        if (len(self.front)!=len(self.cap) or len(set(map(int,self.front)))!=len(self.front)
                or np.any(self.front<state['old_point_count'])
                or len(set(map(int,self.cap)))!=len(self.cap)):
            raise ValueError('CompleteUniqueSharedFrontMapRequired')
        self.anchor=self.points[self.cap].copy()
        delta=self.points[self.front]-self.anchor
        self.depth=np.linalg.norm(delta,axis=1)
        if not np.isfinite(self.depth).all() or np.any(self.depth<=0):
            raise ValueError('NonfiniteOrZeroPollDirection')
        self.direction=delta/self.depth[:,None]
        self.upper_depth=np.full(len(self.front),np.inf)
        for index,anchor in enumerate(self.anchor):
            for axis,component in enumerate(self.direction[index]):
                if component:
                    wall=state['bounds'][axis,1 if component>0 else 0]
                    self.upper_depth[index]=min(self.upper_depth[index],float((wall-anchor[axis])/component))
            if self.extent['mode']=='strict':
                self.upper_depth[index]=min(self.upper_depth[index],self.extent['maximum_distance_m'])
            self.upper_depth[index]=float(np.nextafter(self.upper_depth[index],0.))
            if not math.isfinite(self.upper_depth[index]) or self.upper_depth[index]<=0:
                raise ValueError('InvalidPollRayWorldIntersection')
        self.lookup={int(vertex):index for index,vertex in enumerate(self.front)}
        self.byid={cell['id']:cell for cell in state['cells']}
        if len(self.byid)!=len(state['cells']):raise ValueError('DuplicatePollCellIDs')
        self.stars=star_index(state['cells'],set(self.lookup))
        self.outer_tets,self.faces=targets.populations(state)
        self.populations=population_arrays(state,self.points)
        self.current=merit(self.populations,self.controls['quality_targets'])
        self.tet_indices={int(vertex):[] for vertex in self.front}
        self.face_indices={int(vertex):[] for vertex in self.front}
        for rows,incidence in ((self.outer_tets,self.tet_indices),(self.faces,self.face_indices)):
            for row_index,row in enumerate(rows):
                if row_index%1024==0:self.check()
                for vertex in row:
                    if int(vertex) in incidence:incidence[int(vertex)].append(row_index)
            for vertex in incidence:incidence[vertex]=np.asarray(incidence[vertex],dtype='<i8')
        local=[]
        cap_triangles=self.anchor[self.cap_faces]
        areas=np.linalg.norm(np.cross(cap_triangles[:,1]-cap_triangles[:,0],
            cap_triangles[:,2]-cap_triangles[:,0]),axis=1)/2
        if not np.isfinite(areas).all() or np.any(areas<=0):raise ValueError('DegeneratePollCap')
        cap_incidence=[[] for unused in self.cap]
        for face_index,face in enumerate(self.cap_faces):
            for vertex in face:cap_incidence[vertex].append(face_index)
        self.cap_incidence=[np.asarray(rows,dtype='<i8') for rows in cap_incidence]
        for index in range(len(self.cap)):
            self.check()
            incident=areas[self.cap_incidence[index]]
            if not len(incident):raise ValueError('MissingPollCapIncidence')
            local.append(float(np.median(np.sqrt(4*incident/np.sqrt(3)))))
        self.steps=np.asarray(local);self.last_representable=self.steps.copy()
        self.exhausted=set();self.dirty=set()
        self.stats=dict(attempted=0,rejected=0,committed=0,sweeps=0,
            closure_cell_visits=0,closure_face_visits=0,cache_invalidations=0)

    def eligible(self):
        active=set(self.dirty)
        for name,geometry in zip(targets.POPULATIONS,(self.outer_tets,self.faces)):
            scores=self.populations[name];policy=self.controls['quality_targets'][name]
            thresholds=[]
            if np.any(scores<policy['minimum']):thresholds.append(policy['minimum'])
            for row in policy['population_targets']:
                need=math.ceil(Fraction(str(row['fraction']))*len(scores))
                if np.count_nonzero(scores>=row['minimum'])<need:thresholds.append(row['minimum'])
            for threshold in thresholds:active.update(map(int,geometry[scores<threshold].ravel()))
        return sorted((active & set(self.lookup))-self.exhausted)

    def score_trial(self,vertex):
        values={name:array.copy() for name,array in self.populations.items()}
        indices=self.tet_indices[vertex]
        values[targets.POPULATIONS[0]][indices]=targets.tetrahedron_ratio(self.points,self.outer_tets[indices])
        indices=self.face_indices[vertex]
        values[targets.POPULATIONS[1]][indices]=targets.triangle_ratio(self.points,self.faces[indices])
        return values,merit(values,self.controls['quality_targets'])

    def admissible(self,vertex,depth,scope):
        if not math.isfinite(depth) or depth<=0:return False
        index=self.lookup[vertex];candidate=self.anchor[index]+depth*self.direction[index]
        bounds=self.state['bounds']
        if (not np.isfinite(candidate).all() or np.any(candidate<=bounds[:,0])
                or np.any(candidate>=bounds[:,1])):return False
        if self.extent['mode']=='strict':
            from finite_support.cap_relative_placement import exact_distance_within
            if not exact_distance_within(candidate,self.anchor[index],self.extent['maximum_distance_m']):return False
        from shape_transition_domain import _cross
        incident=self.cap_faces[self.cap_incidence[index]]
        exact_candidate=tuple(Fraction(float(value)) for value in candidate)
        outward=False
        for face in incident:
            origin,second,third=[tuple(Fraction(float(value)) for value in row) for row in self.anchor[face]]
            normal=_cross(tuple(value-start for value,start in zip(second,origin)),
                tuple(value-start for value,start in zip(third,origin)))
            if sum(value*(point-start) for value,point,start in zip(normal,exact_candidate,origin))>0:
                outward=True;break
        if not outward:return False
        self.points[vertex]=candidate
        changed,faces,needed=scope
        self.stats['closure_cell_visits']+=len(needed)
        self.stats['closure_face_visits']+=len(faces)
        report=self.quality(self.points,self.state['cells'],self.state['owners'],
            changed,self.state['outer'])
        if type(report) is not dict or type(report.get('passed')) is not bool:
            raise ValueError('InvalidPollQualityOwnerReturn')
        return report['passed']

    def nominal_cost(self,vertex,candidate):
        index=self.lookup[vertex];nominal=self.extent['nominal_distance_m']
        return float(abs(np.linalg.norm(candidate-self.anchor[index])-nominal)/nominal)

    def trial_depth(self,index,sign):
        current=self.depth[index]
        depth=min(self.upper_depth[index],max(float(np.nextafter(0.,1.)),current+sign*self.steps[index]))
        if depth==current:return None
        while True:
            self.tick()
            candidate=self.anchor[index]+depth*self.direction[index]
            bounds=self.state['bounds']
            inside=np.isfinite(candidate).all() and np.all(candidate>bounds[:,0]) and np.all(candidate<bounds[:,1])
            if inside and self.extent['mode']=='strict':
                from finite_support.cap_relative_placement import exact_distance_within
                inside=exact_distance_within(candidate,self.anchor[index],self.extent['maximum_distance_m'])
            if inside:return depth
            smaller=current+(depth-current)/2
            if smaller==depth or smaller==current:return None
            depth=smaller

    def visit(self,vertex):
        index=self.lookup[vertex];old=self.points[vertex].copy()
        scope=complete_scope(self.stars[vertex],self.state['owners'],self.byid)
        candidates=[];representable=False
        for sign in (-1,1):
            self.tick();self.stats['attempted']+=1
            depth=self.trial_depth(index,sign)
            try:
                if depth is None:
                    self.stats['rejected']+=1;continue
                if not self.admissible(vertex,depth,scope):
                    self.stats['rejected']+=1;continue
                self.tick()
                candidate=self.points[vertex].copy()
                if candidate.tobytes()==old.tobytes():continue
                representable=True
                self.last_representable[index]=self.steps[index]
                if not all(np.isfinite(value).all() for value in self.populations.values()):
                    raise ValueError('CorruptCommittedPollPopulation')
                values,score=self.score_trial(vertex)
                self.tick()
                if not improves(self.current,score):
                    self.stats['rejected']+=1;continue
                candidates.append((tuple(map(sum,zip(*score))),self.nominal_cost(vertex,candidate),
                    sign,candidate,depth,values,score))
            finally:self.points[vertex]=old
        if candidates:
            self.tick()
            best=min(candidates,key=lambda row:row[:3])
            self.points[vertex]=best[3];self.depth[index]=best[4]
            self.populations=best[5];self.current=best[6];self.stats['committed']+=1
            self.invalidate(scope)
            return True
        self.steps[index]/=2
        if not representable and (self.steps[index]==0 or all(
                self.depth[index]+sign*self.steps[index]==self.depth[index] for sign in (-1,1))):
            self.exhausted.add(vertex)
        return False

    def invalidate(self,scope):
        affected={int(value) for owner in scope[2] for value in self.byid[owner]['vertices']} & set(self.lookup)
        self.dirty.update(affected);self.exhausted.difference_update(affected)
        for vertex in affected:
            index=self.lookup[vertex]
            if self.steps[index]<self.last_representable[index]:self.steps[index]=self.last_representable[index]
        self.stats['cache_invalidations']+=len(scope[0])

    def tick(self):
        self.check()
        if self.search_deadline is not None and time.perf_counter()>=self.search_deadline:
            raise SearchExpired('SharedFrontSearchDeadline')

    def run(self,search_seconds):
        if type(search_seconds) not in (int,float) or not math.isfinite(search_seconds) or search_seconds<=0:
            raise ValueError('ExplicitPollSearchBudgetRequired')
        started=self.started;deadline=started+search_seconds;self.search_deadline=deadline;reason='shape_passed'
        if time.perf_counter()>=deadline:reason='search_budget_exhausted'
        while reason=='shape_passed' and any(row[0] for row in self.current):
            self.check()
            if time.perf_counter()>=deadline:reason='search_budget_exhausted';break
            queue=self.eligible();self.dirty.clear()
            if not queue:reason='representability_stagnation';break
            self.stats['sweeps']+=1
            for vertex in queue:
                self.check()
                if time.perf_counter()>=deadline:reason='search_budget_exhausted';break
                try:self.visit(vertex)
                except SearchExpired:reason='search_budget_exhausted';break
                if not any(row[0] for row in self.current):break
            if reason=='search_budget_exhausted':break
        full=population_arrays(self.state,self.points)
        if any(not np.array_equal(full[name],self.populations[name]) for name in targets.POPULATIONS):
            raise ValueError('PollIncrementalPopulationMismatch')
        if merit(full,self.controls['quality_targets'])!=self.current:
            raise ValueError('PollFullMeritMismatch')
        protected=np.ones(len(self.points),dtype=bool);protected[self.front]=False
        if self.points[protected].tobytes()!=self.initial[protected].tobytes():
            raise ValueError('ChangedProtectedPollCoordinates')
        return self.points,dict(schema='SharedFrontDepthPollResult/1',
            passed=reason=='shape_passed' and not any(row[0] for row in self.current),stop_reason=reason,
            seconds=time.perf_counter()-started,counts=self.stats,final_rules=self.current,
            q0_points_sha256=array_hash(self.initial),q1_points_sha256=array_hash(self.points),
            controls_identity=identity(self.controls),full_incremental_reconciled=True,
            moved_vertices=int(np.count_nonzero(np.any(self.initial[self.front]!=self.points[self.front],axis=1))),
            geometry_certified=False)
