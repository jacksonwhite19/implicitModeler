"""Call-scoped live transition handoff; no saved receipt can recreate it."""
from contextlib import contextmanager,ExitStack
from weakref import WeakKeyDictionary
from fractions import Fraction as F
import json,time
import numpy as np
from sdf_cfd_mesher_v4.common.authority import frozen,array_hash,identity,file_hash
from path_preserving_deduplication import durable_json
from immutable_state_access import guard
from tet_layer_adapter import initialize,place,admit,validate
from tet_layer_controls import variable_shape
from sdf_cfd_mesher_v4.sdf.operations import real_value_region, positive_real_value_region
from sdf_cfd_mesher_v4.sdf.regional import Interval

_issued=WeakKeyDictionary()
_ARRAY_FIELDS=frozenset(('points','tets','front','triangles','outer_ids','volumes','centres'))

def _capture(**data):
    for name in _ARRAY_FIELDS & data.keys():
        value=data[name]
        if type(value) is not np.ndarray or value.dtype.hasobject:raise ValueError('InvalidTransitionCapture')
        data[name]=(value.tobytes(),value.dtype.str,tuple(value.shape))
    return data

def _array(data,name):
    raw,dtype,shape=data[name]
    return np.frombuffer(raw,dtype=dtype).reshape(shape)

class TransitionInputs:
    __slots__=('__weakref__',)
    def __init__(self,*args,**kwargs):raise TypeError('LiveTetTransitionFactoryRequired')
    def _open(self):
        if self not in _issued:raise ValueError('ClosedTetTransitionInputs')
        return _issued[self]
    def require(self,parent):
        d=self._open()
        if parent is not d['parent']:raise ValueError('ChangedTetTransitionParent')
        parent.require(parent.view)
        return True
    @property
    def front(self):return _array(self._open(),'front')
    @property
    def triangles(self):return _array(self._open(),'triangles')
    @property
    def volumes(self):return _array(self._open(),'volumes')
    @property
    def centres(self):return _array(self._open(),'centres')
    @property
    def report(self):return json.loads(self._open()['report_json'])
    @property
    def metadata(self):return json.loads(self._open()['metadata_json'])
    @property
    def cells_count(self):return self._open()['tets'][2][0]
    def require_merged(self,parent,points,tets):
        self.require(parent);d=self._open();n=d['points'][2][0];m=d['tets'][2][0]
        if (type(points) is not np.ndarray or type(tets) is not np.ndarray or
                points.dtype.str!=d['points'][1] or tets.dtype.str!=d['tets'][1] or
                points.ndim!=2 or points.shape[1:]!=d['points'][2][1:] or len(points)<n or
                tets.ndim!=2 or tets.shape[1:]!=d['tets'][2][1:] or len(tets)<m or
                points[:n].tobytes()!=d['points'][0] or tets[:m].tobytes()!=d['tets'][0]):raise ValueError('ChangedProtectedTetTransition')
    def merge(self,parent,points,tets,farpoints,fartriangles):
        self.require(parent);d=self._open();arrays={name:_array(d,name) for name in ('points','tets','front','triangles','outer_ids')}
        from standalone_background import compare_boundary
        _,inner,farids,incidence=compare_boundary(points,tets,arrays['front'],arrays['triangles'],farpoints,fartriangles,_return_call_data=True)
        other=np.setdiff1d(np.arange(len(points)),inner);mapping=np.empty(len(points),dtype='<i8');mapping[inner]=arrays['outer_ids'];mapping[other]=len(arrays['points'])+np.arange(len(other))
        combined=np.concatenate((arrays['points'],points[other]));rows=np.concatenate((arrays['tets'],mapping[tets]))
        self.require_merged(parent,combined,rows)
        return combined,rows

def bisect_tet(pts):
    from itertools import combinations
    i,j=max(combinations(range(4),2),key=lambda ij:(sum((pts[ij[0]][k]-pts[ij[1]][k])**2 for k in range(3)),ij))
    middle=tuple((pts[i][k]+pts[j][k])/2 for k in range(3));left=list(pts);right=list(pts);left[i]=middle;right[j]=middle
    return tuple(left),tuple(right)

def positive_tet(vertices,bound,depth,boxes,check):
    """Exact two-child longest-edge cover; source region/sign predicate unchanged."""
    vertices=tuple(tuple(F(float(v)) for v in row) for row in vertices);pending=[(vertices,0)];queries=0;minimum=None
    while pending:
        check();pts,level=pending.pop();queries+=1
        if queries>boxes:raise ValueError('TetTransitionSourceFluidBoxBudget')
        region=tuple(Interval(min(p[j] for p in pts),max(p[j] for p in pts)) for j in range(3));value=bound(region);lo=getattr(value,'lo',value)
        if getattr(value,'hi',1)<=0:raise ValueError('TetTransitionSourceSolidRegion',dict(level=level,queries=queries))
        if lo>0:minimum=lo if minimum is None else min(minimum,lo);continue
        if level>=depth:raise ValueError('TetTransitionSourceFluidUnresolved')
        pending.extend((child,level+1) for child in bisect_tet(pts))
    return queries,minimum

@contextmanager
def fluid_owner(parent,check,controls,progress=None):
    a=parent._open()['origin'].provider.binding.authority
    if not all(callable(getattr(a,n,None)) for n in ('verify','prism_query_scope','verified_operation')) or not hasattr(getattr(a,'queries',None),'_scope'):raise ValueError('TetTransitionContinuousFluidCapabilityRequired')
    with a.prism_query_scope(),a.verified_operation('tet_transition_fluid'):
        expression,scale,budgets,_=a.queries._scope._batch_inputs()
        def fluid(points,cells):
            a.verify();queries=0;lower=None;completed=0;last_progress=time.perf_counter()
            for c in cells:
                def bound(box):
                    a.verify();return positive_real_value_region(expression,box,scale,precision_bits=budgets[2])
                count,lo=positive_tet(points[c['vertices']],bound,controls['fluid_depth'],controls['fluid_boxes'],check);queries+=count;lower=lo if lower is None else min(lower,lo);completed+=1
                now=time.perf_counter()
                if progress is not None and now-last_progress>=1:
                    progress(dict(cells_completed=completed,region_queries=queries,total_cells=len(cells)));last_progress=now
            if progress is not None:progress(dict(cells_completed=completed,region_queries=queries,total_cells=len(cells)))
            a.verify()
            return dict(passed=True,cells=len(cells),region_queries=queries,fluid_boxes_scope='Per tetrahedron; whole-stage seconds remain a separate bound',minimum_real_field_lower_m=str(lower),geometry=parent._open()['origin'].provider.binding.geometry_identity,method='SameRealSource/ConservativeValueFilterWithExactFallback/StrictPositive/ExactLongestEdgeCover',scope='Continuous real-source field sign for transition cells; no claim of native floating-point arithmetic equivalence or inherited surface completeness.')
        yield fluid

@contextmanager
def produce(parent,ids,triangles,terminal,volumes,centres,controls,bounds,folder,resources):
    from rolling_fast_state import BackgroundInputs
    from background_topology_repair import key
    from background_mixed_geometry import batch_geometry
    validate(controls)
    if type(parent) is not BackgroundInputs:raise ValueError('LiveAcceptedTetPrismInputsRequired')
    parent.require(parent.view);run=parent.view;start=time.perf_counter();deadline=start+controls['seconds']
    def check():
        parent.require_origin(parent._open()['origin'])
        if time.perf_counter()>=deadline:raise ValueError('TetTransitionWholeStageTimeLimit')
    lease=None;s=None;p=None;stage='source_capability'
    phase_started={};timings={};phase_progress={}
    def trace(name,event,details=None):
        now=time.perf_counter()
        if event=='enter':phase_started[name]=now
        elif event=='exit':timings[name]=now-phase_started[name]
        if details is not None:phase_progress[name]=details
        durable_json(folder/'TRANSITION_TIMING.json',dict(schema='V4TransitionTiming/1',phase=name,event=event,
            elapsed_seconds=now-start,completed_seconds=dict(timings),progress=details,phase_progress=dict(phase_progress),
            diagnostic_only=True,scientific_acceptance=False,restart_supported=False))
    check.repair_progress=lambda values:trace('placement','progress',values)
    diagnostic_sink=None
    def retain_diagnostic(event,payload):
        nonlocal diagnostic_sink
        if diagnostic_sink is None:
            from background_core_repair import evidence_sink
            diagnostic_sink=evidence_sink(folder/'TRANSITION_REPAIR_EVENTS')
        diagnostic_sink(event,payload)
    check.repair_diagnostics=retain_diagnostic
    trace('source_scope','enter')
    with ExitStack() as held:
        try:
            with fluid_owner(parent,check,controls,progress=lambda values:trace('source_fluid','progress',values)) as fluid:
                trace('source_scope','exit')
                from standalone_tool import check_resources
                available=check_resources(folder.parent,resources)
                shape=variable_shape(controls);seed_rows=controls.get('seed_rows',controls.get('rows'))
                minimum_arrays=(seed_rows+1)*len(ids)*24+3*seed_rows*len(triangles)*32
                if resources['output_limit_bytes'] is not None and available['logical_bytes']+minimum_arrays>resources['output_limit_bytes']:raise ValueError('TetTransitionOutputResourceLimit')
                if available['free_bytes']-minimum_arrays<resources['minimum_free_bytes']:raise ValueError('TetTransitionFreeSpaceResourceLimit')
                stage='initialization';trace(stage,'enter')
                check.remaining_seconds=lambda: max(0.,deadline-time.perf_counter())
                parent.require(run)
                proposer=parent._open()['origin'].provider
                proposal=proposer.transition_proposal(run.points[ids],triangles,controls) if hasattr(proposer,'transition_proposal') else None
                if proposal is None:
                    s=initialize(run.points,parent.cells(),run.topology.owners,ids,triangles,terminal,controls,bounds,prism_volumes=frozen(volumes,'<f8'),prism_centres=frozen(centres,'<f8'),check=check,
                        _output=folder/'SHAPE_CONSTRUCTION' if shape else None,_resources=resources if shape else None)
                else:
                    fronts,construction,direction=proposal
                    s=initialize(run.points,parent.cells(),run.topology.owners,ids,triangles,terminal,controls,bounds,prism_volumes=frozen(volumes,'<f8'),prism_centres=frozen(centres,'<f8'),check=check,_fronts=fronts,_construction=construction,_direction=direction)
                if np.any(s['points']<=bounds[:,0]) or np.any(s['points']>=bounds[:,1]):raise ValueError('TetTransitionOutsideBackgroundDomain')
                trace('initialization','exit');stage='placement';trace(stage,'enter');p=place(s,check,parent);trace(stage,'exit')
                if np.any(s['points']<=bounds[:,0]) or np.any(s['points']>=bounds[:,1]):raise ValueError('TetTransitionOutsideBackgroundDomain')
                trace('array_publication','enter')
                compact=s['compact_ids'];inverse={int(v):i for i,v in enumerate(compact)};rows=np.asarray([[inverse[v] for v in c['vertices']] for c in s['additions']],dtype='<i8');outerids=np.asarray([inverse[int(v)] for v in s['outer'].ravel()],dtype='<i8').reshape(-1,3)
                if not shape and not np.array_equal(outerids,triangles+controls['rows']*len(ids)):raise ValueError('ChangedTetTransitionOuterAncestry')
                points=frozen(s['points'][compact],'<f8');rows=frozen(rows,'<i8')
                published_outer=(np.unique(outerids) if shape else np.arange(controls['rows']*len(ids),len(compact),dtype='<i8'))
                outer_lookup={int(v):i for i,v in enumerate(published_outer)}
                published_triangles=(np.asarray([[outer_lookup[int(v)] for v in face] for face in outerids],dtype='<i8') if shape else triangles)
                front=frozen(points[published_outer],'<f8')
                # Values are provisional until TRANSITION.json and the private
                # call-scoped lease exist. Never rewrite/copy them after proof.
                payload=dict(points=points,tetrahedra=rows,front=front,triangles=published_triangles,cap_ids=ids)
                if shape:payload.update(cap_triangles=triangles,outer_vertex_ids=published_outer,**{'vertex_envelope_'+k:v for k,v in s['vertex_envelope'].items()})
                if shape:
                    available=check_resources(folder.parent,resources);logical_minimum=sum(value.nbytes for value in payload.values())
                    if resources['output_limit_bytes'] is not None and available['logical_bytes']+logical_minimum>resources['output_limit_bytes']:raise ValueError('ShapeTransitionOutputResourceLimit')
                    if available['free_bytes']-logical_minimum<resources['minimum_free_bytes']:raise ValueError('ShapeTransitionFreeSpaceResourceLimit')
                np.savez(folder/'TRANSITION.npz',**payload)
                held.enter_context(guard(folder/'TRANSITION.npz'))
                with np.load(folder/'TRANSITION.npz',allow_pickle=False) as readback:
                    if set(readback.files)!=set(payload):raise ValueError('ChangedTetTransitionArraySchema')
                    for name,expected in payload.items():
                        value=readback[name]
                        if value.shape!=expected.shape or value.dtype!=expected.dtype or value.tobytes()!=expected.tobytes():raise ValueError('ChangedTetTransitionArrayReadback',name)
                durable_json(folder/'TRANSITION_PLACEMENT.json',p);trace('array_publication','exit')
                trace('post_placement_resources','enter');check_resources(folder.parent,resources);trace('post_placement_resources','exit')
                stage='independent_transition_admission';proof=admit(s,p,fluid,check,parent,trace=trace)
                if array_hash(s['points'])!=p['points_sha256']:raise ValueError('ChangedTetTransitionAdmissionCoordinates')
                check_resources(folder.parent,resources);parent.require(run)
            parent.require(run)
            outerowners=[s['owners'][key(f)][0][0] for f in s['outer']];byid={c['id']:c for c in s['additions']};g=batch_geometry(s['points'],[byid[i] for i in outerowners])
            volumes=frozen(np.asarray([g[i]['volume'] for i in outerowners]),'<f8');centres=frozen(np.asarray([g[i]['centre'] for i in outerowners]),'<f8')
            report=dict(schema='V4TetLayerAcceptance/2' if shape else 'V4TetLayerAcceptance/1',passed=True,controls=controls,parent_prism_mesh=run.live_acceptance.mesh,points_sha256=array_hash(points),tets_sha256=array_hash(rows),arrays_sha256=file_hash(folder/'TRANSITION.npz'),placement=p,proof=proof,seconds=time.perf_counter()-start,outer_bulk_neighbours_required=True,restart_supported=False)
            authored=identity(report);durable_json(folder/'TRANSITION.json',report);held.enter_context(guard(folder/'TRANSITION.json'))
            if identity(json.loads((folder/'TRANSITION.json').read_text(encoding='utf-8')))!=authored:raise ValueError('ChangedTetTransitionReportReadback')
            check_resources(folder.parent,resources);report['report_sha256']=file_hash(folder/'TRANSITION.json')
            names=([f"transition-volume:{report['arrays_sha256']}:vertex:{int(i)}" for i in published_outer] if shape else [f"transition:{controls['rows']}:parent:{int(i)}" for i in ids])
            meta=dict(schema='V4TetLayerOuterInterface/2' if shape else 'V4TetLayerOuterInterface/1',parent_prism_mesh=run.live_acceptance.mesh,vertex_ids=names,shell_owners=list(map(str,outerowners)),owner_kind='tetrahedron',points_sha256=array_hash(front),triangles_sha256=array_hash(published_triangles),transition_arrays_sha256=report['arrays_sha256'],transition_report_sha256=report['report_sha256']);meta['identity']=identity(meta)
            lease=object.__new__(TransitionInputs);_issued[lease]=_capture(parent=parent,points=points,tets=rows,front=front,triangles=frozen(published_triangles,'<i8'),outer_ids=frozen(published_outer,'<i8'),volumes=volumes,centres=centres,report_json=json.dumps(report),metadata_json=json.dumps(meta))
            stage='accepted_handoff';trace(stage,'enter')
            yield lease
        except BaseException as error:
            if lease is not None:
                import traceback
                durable_json(folder/'TRANSITION_CONSUMER_STOP.json',dict(stage=stage,error=repr(error),traceback=traceback.format_exc(),scientific_accepted=True,transition_report_sha256=file_hash(folder/'TRANSITION.json'),restart_supported=False))
                raise
            # A stopped stage never leaves a usable live credential. Preserve
            # its current private numerical state for diagnosis, not restart.
            if s is not None and not (folder/'TRANSITION.npz').exists():
                compact=s['compact_ids'];lookup={int(v):i for i,v in enumerate(compact)}
                stopped=dict(points=s['points'][compact],tetrahedra=np.asarray([[lookup[v] for v in c['vertices']] for c in s['additions']],dtype='<i8'),cap_ids=ids)
                if variable_shape(controls):stopped.update(cap_triangles=triangles,outer_triangles=np.asarray([[lookup[v] for v in f] for f in s['outer']],dtype='<i8'),**{'vertex_envelope_'+k:v for k,v in s['vertex_envelope'].items()})
                else:stopped['triangles']=triangles
                np.savez(folder/'TRANSITION.npz',**stopped)
            if not (folder/'TRANSITION_STOP.json').exists():
                import traceback
                durable_json(folder/'TRANSITION_STOP.json',dict(stage=stage,error=repr(error),traceback=traceback.format_exc(),scientific_accepted=False,restart_supported=False,seconds=time.perf_counter()-start,arrays_sha256=file_hash(folder/'TRANSITION.npz') if (folder/'TRANSITION.npz').exists() else None))
            raise
        finally:
            if lease is not None:_issued.pop(lease,None)
