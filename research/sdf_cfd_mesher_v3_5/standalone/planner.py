"""Configured V3 complete-horizon numerical planning, without case launchers.

Each instance has private loaded module dictionaries. Original files and other
planner instances retain their own parameters. The shape-specific NACA risk
screen is never called: exact all-face/all-row discovery is the existing
select_staged_schedule entry. Source/collision proof is a separate obligation.
"""
from dataclasses import dataclass
from pathlib import Path
import importlib.util, json, sys, uuid,ast
import numpy as np

ROOT=Path(__file__).resolve().parent

@dataclass(frozen=True)
class Request:
    first_height: float
    growth: float
    layers: int
    def __post_init__(self):
        if type(self.layers) is not int or not 3<=self.layers<=20:raise ValueError('SupportedHorizon3To20Required')
        if not np.isfinite([self.first_height,self.growth]).all() or self.first_height<=0 or self.growth<1:raise ValueError('InvalidPrismRequest')
        if not np.isfinite(self.nominal).all():raise ValueError('UnrepresentableSchedule')
    @property
    def nominal(self):return self.first_height*np.power(self.growth,np.arange(self.layers,dtype='<f8'))

def _load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module
    spec.loader.exec_module(module)
    return module

class Planner:
    def __init__(self,request):
        self.request=request
        root=ROOT/'v3'
        prefix='_v35_'+uuid.uuid4().hex
        # The original module import wiring executes no study driver or input read.
        before_path=sys.path[:];before_modules=dict(sys.modules)
        try:
            sys.modules.pop('run_weight_aware_planner',None)
            sys.modules.pop('foundation_geometry',None)
            sys.path.insert(0,str(root/'foundation_aware_prism_shell_qualification'))
            self.skew=_load(prefix,root/'foundation_skew_increment_qualification/run_skew_aware_planner.py')
            self.weight=self.skew.WEIGHT;self.base=self.weight.BASE;self.math=self.base.PM
            self.state_type=_load(prefix+'_state',root/'sdf_guided_advancing_front_qualification/accepted_front_state.py').AcceptedFrontState
            self.geometry=sys.modules['foundation_geometry']
        finally:
            sys.path[:]=before_path
            # Restore flat aliases; private function globals retain their objects.
            for name in ('foundation_geometry','run_weight_aware_planner','weight_base_staged_planner','v3_staged_prism_math'):
                if name in before_modules:sys.modules[name]=before_modules[name]
                else:sys.modules.pop(name,None)
        for module in (self.base,self.weight,self.skew):
            module.LAYERS=request.layers
        self.base.H1=self.skew.H1=request.first_height
        self.base.R_NOMINAL=self.skew.NOMINAL_R=request.growth
        original_loader=self.base.load_module
        def configured_dependency(name,path):
            if Path(path).name=='run_weight_aware_planner.py':return self.weight
            if Path(path).name=='foundation_geometry.py':return self.geometry
            return original_loader(name,path)
        self.base.load_module=configured_dependency
        source=root/'short_wing_displacement_contract_2026-09-06/evaluate.py'
        scope=dict(np=np,PM=self.math,FG=self.geometry,DET=self.base.FOUNDATION_QUAL,WEIGHT=self.weight.QUAL_WEIGHT,SKEW=self.skew.QUAL_SKEW)
        for node in ast.parse(source.read_text()).body:
            if isinstance(node,ast.FunctionDef):exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),scope)
        self.stack_audit=scope['stack_audit']
        self.role_determinant=scope['determinant']

    def _terminal_B_repair(self,state,h,result):
        """Cover the existing final-B requirement after ordinary C selection.

        One existing fallback, no changes to its search or admission criteria.
        Return every proposed schedule for capture even when full audit refuses.
        """
        rows=result['complete']['rows']
        terminal=rows[-1]['failures_B']
        eligible=(result['wedge']['pass'] and result['complete']['all_C_numerical_pass']
            and not any(result['weight'][k] for k in ('side_qualification_failures','horizontal_qualification_failures'))
            and not any(result['skew'][k] for k in ('qualification_skew_failures','qualification_weight_failures'))
            and all(not any(r['failures_B'].values()) for r in rows[:-1])
            and terminal['determinant']>0 and not any(v for k,v in terminal.items() if k!='determinant'))
        if not eligible:return h,result,None
        x=state.positions(h);faces=state.faces
        prisms=np.concatenate([x[-2][faces],x[-1][faces]],axis=1)
        bad=np.flatnonzero(self.role_determinant(prisms,True,False)<self.base.FOUNDATION_QUAL)
        if not len(bad):raise ValueError('TerminalBRepairAuditSeedMismatch')
        incident,_=self.base.build_incidence(faces,len(state.wall))
        tri=state.wall[faces];area=.5*np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)
        va=np.zeros(len(state.wall))
        for k in range(3):np.add.at(va,faces[:,k],area/3)
        start=max(3,self.request.layers-1)
        selected=self.base._select_independent_late_candidate(bad.tolist(),state.wall,faces,
            state.vectors[0],h,incident,va,start,state)
        detail=dict(trigger='C_valid_terminal_B_determinant_only',source_face_ids=bad.tolist(),
            threshold=self.base.FOUNDATION_QUAL,compression_start_layer=start,
            baseline_audit=result,baseline_schedule=h.copy(),
            fallback={k:v for k,v in selected.items() if k!='proposal'})
        if not selected['pass']:
            detail['outcome']='TerminalBRepairUnresolved';return h,result,detail
        proposal=np.asarray(selected['proposal'],dtype='<f8')
        upper=self.request.nominal.copy();upper[1]=upper[2]
        if (proposal.shape!=h.shape or not np.array_equal(proposal[:,:start-1],h[:,:start-1])
            or not np.isfinite(proposal).all() or np.any(proposal<=0) or np.any(proposal>h)
            or np.any(proposal>upper)):
            raise ValueError('TerminalBRepairScheduleContractViolation')
        final=self.audit(state,proposal)
        detail['outcome']='TerminalBRepairPassed' if final['numerical_feasible'] else 'TerminalBRepairFullAuditRefusal'
        detail['candidate_audit']=final
        return proposal,final,detail

    def audit(self,state,schedule):
        wall,faces=state.wall,state.faces
        directions=state.vectors[0]
        edges,ef=self.geometry.unique_manifold_edges(faces)
        wedge=self.base.audit_faces(wall,faces,directions,schedule,accepted_state=state)
        weight=self.weight.complete_weight_audit(wall,faces,directions,schedule,edges,ef,accepted_state=state)
        skew=self.skew.complete_face_audit(wall,faces,directions,schedule,edges,ef,accepted_state=state)
        complete=self.stack_audit(state.positions(schedule),faces)
        passed=wedge['pass'] and not any(weight[k] for k in ('side_qualification_failures','horizontal_qualification_failures')) and not any(skew[k] for k in ('qualification_skew_failures','qualification_weight_failures')) and complete['all_B_numerical_pass'] and complete['all_C_numerical_pass']
        return dict(numerical_feasible=bool(passed),wedge=wedge,weight=weight,skew=skew,complete=complete,ownership=dict(state=state.ownership,required=('B_final_isolated_shell','C_prospective_final_fluid'),actual_core_neighbour_weights='pending'))

    def plan(self,state,*,compress=True,observer=None):
        h=np.broadcast_to(self.request.nominal,(len(state.wall),self.request.layers)).copy()
        selected=self.base.select_staged_schedule(state.wall,state.faces,state.vectors[0],h,accepted_state=state,observer=observer) if compress else dict(pass_gate=self.base.audit_faces(state.wall,state.faces,state.vectors[0],h,accepted_state=state)['pass'],proposal=h,selections=[])
        if not selected['pass_gate']:return dict(numerical_feasible=False,reason='NoV3WedgeSchedule',selection=selected)
        h=selected['proposal']
        edges,ef=self.geometry.unique_manifold_edges(state.faces)
        def areas():
            _,neighbors=self.base.build_incidence(state.faces,len(state.wall))
            a=state.wall[state.faces];a=.5*np.linalg.norm(np.cross(a[:,1]-a[:,0],a[:,2]-a[:,0]),axis=1)
            va=np.zeros(len(state.wall))
            for k in range(3):np.add.at(va,state.faces[:,k],a/3)
            return neighbors,va
        weights=self.weight.complete_weight_audit(state.wall,state.faces,state.vectors[0],h,edges,ef,accepted_state=state)
        if compress and weights['side_qualification_failures']:
            incident,neighbors=self.base.build_incidence(state.faces,len(state.wall));_,va=areas()
            correction=self.weight.select_correction(state.wall,state.faces,state.vectors[0],h,edges,ef,weights['qualification_edges'],weights['first_qualification_layer'],incident,neighbors,va,accepted_state=state)
            h=correction['proposal']
        skew=self.skew.complete_face_audit(state.wall,state.faces,state.vectors[0],h,edges,ef,accepted_state=state)
        if compress and (skew['qualification_skew_failures'] or skew['qualification_weight_failures']):
            neighbors,va=areas()
            warning=min(r['layer'] for r in skew['by_layer'] if r['qualification_skew_failures'] or r['qualification_weight_failures'])
            starts=range(max(2,warning-9),min(self.request.layers,warning)+1)
            correction=self.skew.select_field(state.wall,state.faces,state.vectors[0],h,edges,ef,skew['qualification_edges'],neighbors,va,accepted_state=state,start_layers=starts)
            h=correction['proposal']
        # Frozen first height, nominal upper bound, positive increments and full
        # horizon remain explicit even if a selector returns malformed output.
        upper=self.request.nominal.copy()
        # Preserve V3's declared row-2 feasibility exception: it may grow only
        # within [nominal row2, nominal row3]. Other rows keep nominal caps.
        upper[1]=upper[2]
        if h.shape!=(len(state.wall),self.request.layers) or not np.all(h[:,0]==self.request.first_height) or np.any(h<=0) or np.any(h>upper):raise ValueError('SelectedScheduleRequestViolation')
        result=self.audit(state,h)
        repair=None
        if compress:h,result,repair=self._terminal_B_repair(state,h,result)
        if repair is not None:result=dict(result,selection_repair=repair)
        positions=state.positions(h)
        delta=np.subtract(positions[1:],positions[:-1],dtype='<f8')
        result.update(schedule=h,positions=positions,delta=delta,
            requested=dict(first_height_m=self.request.first_height,growth=self.request.growth,layers=self.request.layers,nominal_increments_m=self.request.nominal.tolist(),nominal_total_m=float(self.request.nominal.sum())),
            achieved=dict(layers=self.request.layers,scalar_total_min_m=float(h.sum(1).min()),scalar_total_max_m=float(h.sum(1).max()),compressed_vertices=int(np.any(h<self.request.nominal,axis=1).sum()),row2_increased_vertices=int((h[:,1]>self.request.nominal[1]).sum()),path_totals_m=np.linalg.norm(delta,axis=2).sum(0)),
            source_global_proof='pending',mesh_accepted=False,
            increment_guard=self.skew.increment_guard_audit(h,state.wall,state.faces,state.vectors[0],accepted_state=state))
        return result

def jsonable(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError(type(value).__name__)
