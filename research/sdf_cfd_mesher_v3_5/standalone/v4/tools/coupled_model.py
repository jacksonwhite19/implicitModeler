"""Private coupled patch proposals; existing V4 owners decide quality/geometry."""
import time
import numpy as np
from background_materialization import build
from background_topology_repair import key, boundary, cavity_geometry
from background_local_repair import LocalRepair
from background_mixed_geometry import batch_geometry
from background_core_repair import atoms
from tet_layer_adapter import quality, local_outer_faces
from shape_transition_builder import envelope_constraints, envelope_subset_passed
from sdf_cfd_mesher_v4.common.authority import array_hash,identity


class CoupledPatch:
    def __init__(self, source, selected_ids, retired_ids, replacement_cells,
                 candidate_points, envelope_state, new_site_ids, *, check):
        check();self.source=source;self.check=check
        self.ids=set(map(int,selected_ids));self.retired=set(map(int,retired_ids))
        if not self.retired<=self.ids:raise ValueError('RetirementOutsideCollar')
        byid=source['byid'];editable=source['editable_ids']
        if not self.ids<=editable:raise ValueError('InheritedCellInCollar')
        self.old=[byid[i] for i in sorted(self.ids)]
        self.cells=[byid[i] for i in sorted(self.ids-self.retired)]+list(replacement_cells)
        self.active={c['id'] for c in self.cells}
        if len(self.active)!=len(self.cells) or self.active&(set(byid)-self.ids):
            raise ValueError('AliasedCoupledCellIdentities')
        oldchain,faces,_=boundary(self.old)
        if boundary(self.cells)[0]!=oldchain:raise ValueError('ChangedCollarBoundaryIncidence')
        self.boundary=np.asarray(faces,dtype='<i8');boundary_vertices=set(map(int,self.boundary.ravel()))
        self.boundary_vertices=np.asarray(sorted(boundary_vertices),dtype='<i8')
        self.new_site_ids=set(map(int,new_site_ids));self.original_n=len(source['points'])
        if (np.asarray(candidate_points).shape!=(self.original_n+len(self.new_site_ids),3) or
            self.new_site_ids!=set(range(self.original_n,len(candidate_points))) or
            not np.isfinite(candidate_points).all()):raise ValueError('ExplicitAppendedSiteNamespaceRequired')
        self.points=np.asarray(candidate_points,dtype='<f8').copy()
        prefix=source['old_point_count']
        used={v for c in self.cells for v in c['vertices']}
        self.moving=sorted((used-boundary_vertices)-set(range(prefix)))
        if not self.new_site_ids<=set(self.moving):raise ValueError('InsertedSiteNotCollarInterior')
        source_moving={v for v in self.moving if v<self.original_n}
        for v in source_moving:
            if not set(source['stars'][v])<=self.ids:raise ValueError('IncompleteMovedVertexStar')
        if source_moving&source['physical_outer_vertices']:raise ValueError('MovedPhysicalOuterInFixedBoundaryTest')
        unchanged=np.ones(self.original_n,dtype=bool);unchanged[list(source_moving)]=False
        if self.points[:self.original_n][unchanged].tobytes()!=source['points'][unchanged].tobytes():
            raise ValueError('ChangedOutsideCollarCoordinates')
        if self.points[:prefix].tobytes()!=source['points'][:prefix].tobytes():raise ValueError('ChangedInheritedPrismPrefix')
        self.external={};owners={}
        for face in faces:
            k=key(face);uses=source['owners'][k]
            outside=[(i,f) for i,f in uses if i not in self.ids]
            if len(outside)>1:raise ValueError('InvalidActualOutsideOwner')
            if not outside and k not in source['physical_outer_keys']:raise ValueError('MissingActualOutsideOwner')
            owners[k]=list(outside)
            for i,_ in outside:self.external[i]=byid[i]
        for c in self.cells:
            for f in c['faces']:owners.setdefault(key(f),[]).append((c['id'],f))
        from prism_hotpath_v2 import reverse_cycle
        for k,uses in owners.items():
            if len(uses)==1:
                if k not in source['physical_outer_keys']:raise ValueError('UnexpectedCoupledBoundaryFace')
            elif len(uses)!=2 or not reverse_cycle(uses[0][1],uses[1][1]):raise ValueError('InvalidCoupledNeighbourIncidence')
        self.owners=owners;self.all_cells=self.cells+list(self.external.values())
        self.outer=local_outer_faces(self.cells,source['physical_outer_keys'])
        oldowners={k:list(source['owners'][k]) for c in self.old for f in c['faces'] for k in [key(f)]}
        self.old_quality=quality(source['points'],self.old+list(self.external.values()),oldowners,self.ids,
                                 local_outer_faces(self.old,source['physical_outer_keys']))
        self.initial_quality=quality(self.points,self.all_cells,owners,self.active,self.outer)
        self.envelope_state=dict(envelope_state,points=self.points)
        self.model=LocalRepair(self.points,self.all_cells,owners,self.active,
                              set(range(len(self.points)))-set(self.moving),self.initial_quality['failures'])
        self.model.scope(self.cells);self.needed={i for uses in owners.values() for i,_ in uses}
        geom=batch_geometry(self.points,self.cells)
        self.volumes=np.asarray([abs(geom[c['id']]['volume']) for c in self.cells])
        if np.any(self.volumes<=0):raise ValueError('NonpositiveCoupledInitializer')
        self.scale=source['controls']['maximum_distance_m']/source['controls']['seed_rows']
        self.base=self.points[self.moving].copy();self.cache_key=None;self.cache_value=None;self.evaluations=0
        self.labels=[]
        for c in self.cells:
            self.labels.extend(dict(kind='cell_'+m,cells=[c['id']],metric=m) for m in ('volume','determinant','aspect'))
        internal=[(k,u) for k,u in sorted(owners.items()) if len(u)==2]
        for metric in ('weight','volume_ratio','skewness'):
            self.labels.extend(dict(kind='face_'+metric,cells=[i for i,_ in uses],face=list(k),metric=metric) for k,uses in internal)
        # The freshly evaluated initializer may already be admissible. This
        # binding is call-local and never imports a prior report as evidence.
        self.source_stamp=self._source_stamp();self.candidate_stamp=self._candidate_stamp()
        self.solved_hash=array_hash(self.points);self.solved_quality=identity(self.initial_quality)
        self.initial_points_hash=self.solved_hash;self.initial_quality_identity=self.solved_quality
        check()

    def _candidate_stamp(self):
        s=self.envelope_state
        return identity(dict(cells=self.all_cells,owners={str(k):v for k,v in sorted(self.owners.items())},outer=array_hash(self.outer),
            selected_ids=sorted(self.ids),active_ids=sorted(self.active),moving=self.moving,
            new_site_ids=sorted(self.new_site_ids),
            old_point_count=s['old_point_count'],cap_ids=array_hash(s['cap_ids']),cap_points=array_hash(s['original'][s['cap_ids']]),
            cap_faces=array_hash(s['cap_faces']),controls=s['controls'],
            bounds=None if s['bounds'] is None else array_hash(s['bounds']),
            envelope={k:array_hash(v) for k,v in s['vertex_envelope'].items()}))

    def _source_stamp(self):
        return identity(dict(points=array_hash(self.source['points']),
            cells=[self.source['byid'][i] for i in sorted(self.ids|set(self.external))],
            owners={str(key(f)):self.source['owners'][key(f)] for c in self.old for f in c['faces']},
            controls=self.source['controls']))

    def values(self,z):
        self.check();stamp=tuple(z)
        if stamp!=self.cache_key:
            self.model.points[self.moving]=self.base+self.scale*np.asarray(z).reshape(-1,3)
            ordinary=self.model.constraints(self.cells,self.owners,self.needed,self.volumes,prospective_outer=self.outer)
            if len(ordinary)!=len(self.labels):raise ValueError('ChangedConstraintLabelPopulation')
            extra=envelope_constraints(self.envelope_state,self.moving,self.model.points)
            self.cache_value=np.r_[ordinary,extra];self.cache_key=stamp;self.evaluations+=1
        return self.cache_value

    def solve(self,*,maxiter,ftol):
        from scipy.optimize import minimize
        if self._source_stamp()!=self.source_stamp or self._candidate_stamp()!=self.candidate_stamp:
            raise ValueError('ChangedCoupledModelInputs')
        if array_hash(self.points)!=self.initial_points_hash or identity(self.initial_quality)!=self.initial_quality_identity:
            raise ValueError('ChangedCoupledInitializer')
        started=time.perf_counter();zero=np.zeros(3*len(self.moving));initial=self.values(zero).copy();passing=initial>=0
        try:
            fit=minimize(lambda y:-float(y[-1]),np.r_[zero,initial.min()],method='SLSQP',
                bounds=[(-1.,1.)]*len(zero)+[(None,None)],
                constraints=[dict(type='ineq',fun=lambda y:np.r_[self.values(y[:-1])-y[-1],self.values(y[:-1])[passing]])],
                options=dict(maxiter=maxiter,ftol=ftol))
            after=self.values(fit.x[:-1]);candidate=self.points.copy()
            candidate[self.moving]=self.base+self.scale*fit.x[:-1].reshape(-1,3)
        finally:self.model.points[self.moving]=self.base
        self.check()
        if self._source_stamp()!=self.source_stamp or self._candidate_stamp()!=self.candidate_stamp:
            raise ValueError('ChangedCoupledModelInputs')
        q=quality(candidate,self.all_cells,self.owners,self.active,self.outer)
        if array_hash(self.points)!=self.initial_points_hash or identity(self.initial_quality)!=self.initial_quality_identity:
            raise ValueError('ChangedCoupledInitializer')
        initial_atoms=atoms(self.initial_quality);terminal_atoms=atoms(q)
        no_new_atoms=terminal_atoms<=initial_atoms
        passing_preserved=bool(np.all(after[passing]>=0))
        margin_improved=bool(after.min()>initial.min())
        endpoint_selected=bool(no_new_atoms and passing_preserved and (margin_improved or q['passed']))
        reason=('EndpointAdopted' if endpoint_selected else 'NewFailureAtoms' if not no_new_atoms else
                'InitiallyPassingConstraintRegression' if not passing_preserved else 'NoMarginImprovement')
        terminal=dict(quality=q,minimum_slack=float(after.min()),
            active_constraints=[dict(label,slack=float(v)) for label,v in zip(self.labels,after)],
            no_new_failure_atoms=no_new_atoms,new_failure_atom_count=len(terminal_atoms-initial_atoms),
            initially_passing_constraints_preserved=passing_preserved,minimum_slack_improved=margin_improved,
            ordinary_quality_passed=bool(q['passed']),selected=endpoint_selected,selection_reason=reason)
        if not endpoint_selected:
            candidate=self.points.copy();q=self.initial_quality;after=initial
        self.solved_hash=array_hash(candidate);self.solved_quality=identity(q)
        rows=[dict(label,slack=float(v)) for label,v in zip(self.labels,after)]
        local=dict(passed=q['passed'],quality=q,solver_success=bool(fit.success),solver_message=str(fit.message),
            iterations=int(fit.nit),evaluations=self.evaluations,minimum_slack_before=float(initial.min()),
            minimum_slack_after=float(after.min()),seconds=time.perf_counter()-started,
            selected_cells=len(self.old),candidate_cells=len(self.cells),moving_vertices=len(self.moving),
            coordinate_plus_margin_variables=len(zero)+1,active_constraints=rows,
            scientific_mesh_acceptance=False,original_quality=self.old_quality,terminal=terminal,
            selected_proposal='terminal' if endpoint_selected else 'initializer')
        return candidate,local

    def validate(self,candidate,quality_result,*,membership):
        self.check()
        if self._source_stamp()!=self.source_stamp:raise ValueError('ChangedCoupledSourceInputs')
        if self._candidate_stamp()!=self.candidate_stamp:raise ValueError('ChangedCoupledCandidateInputs')
        if array_hash(candidate)!=self.solved_hash or identity(quality_result)!=self.solved_quality:
            raise ValueError('ChangedCoupledQualityInputs')
        if not quality_result['passed']:raise ValueError('CoupledPatchQualityFailure')
        before_atoms=atoms(self.old_quality);after_atoms=atoms(quality_result)
        if not after_atoms<before_atoms:raise ValueError('NoStrictCoupledFailureReduction')
        if candidate[self.boundary_vertices].tobytes()!=self.source['points'][self.boundary_vertices].tobytes():
            raise ValueError('ChangedTrueCollarBoundaryCoordinates')
        prefix=self.source['old_point_count']
        if candidate[:prefix].tobytes()!=self.source['points'][:prefix].tobytes():raise ValueError('ChangedInheritedPrismPrefix')
        same=np.ones(self.original_n,dtype=bool);same[[v for v in self.moving if v<self.original_n]]=False
        if candidate[:self.original_n][same].tobytes()!=self.source['points'][same].tobytes():raise ValueError('ChangedOutsideCollarCoordinates')
        if not envelope_subset_passed(self.envelope_state,self.moving,candidate):raise ValueError('MovedPointOutsideEnvelope')
        oldrows=np.asarray([c['vertices'] for c in self.old],dtype='<i8')
        for v in self.moving:
            self.check()
            if not membership(candidate[v],self.source['points'],oldrows):raise ValueError('MovedPointOutsideOldCollar')
        before=cavity_geometry(self.source['points'],self.old);self.check()
        after=cavity_geometry(candidate,self.cells);self.check()
        if before['boundary_chain']!=after['boundary_chain']:raise ValueError('ChangedExactCoupledBoundary')
        return dict(passed=True,scope='Relative local edit only; fresh global scientific admission remains mandatory',
                    before=before,after=after,original_prism_prefix_unchanged=True,true_boundary_unchanged=True,
                    exact_envelope_passed=True,all_moving_sites_inside_old_union=True,
                    points_sha256=array_hash(candidate),scientific_mesh_acceptance=False)
