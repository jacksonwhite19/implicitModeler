"""Validated V3 physical-shell enrollment into the copied V4 background owner.

This module grants no acceptance from a caller receipt. It constructs live
caches only after source segments, full V4 quality/topology, independent face
coverage, exact volume/degree, and V3 native identity/incidence have passed.
"""
from pathlib import Path
from types import SimpleNamespace
import json,time
import numpy as np
from sdf_cfd_mesher_v4.common.authority import identity,array_hash,frozen,file_hash

class PrismShellGeometry:
    def __init__(self,binding,owner,positions,native_identity):
        self.binding=binding;self.owner=owner;self.positions=frozen(positions,'<f8')
        self.x=frozen(positions[0],'<f8');self.layer=len(positions)-1
        self.native_identity=native_identity;self._positions_hash=array_hash(self.positions)
    def check(self):
        self.binding.qualification.verify(self.binding.authority);self.owner.check()
        if array_hash(self.positions)!=self._positions_hash or self.owner.positions().tobytes()!=self.positions.tobytes():raise ValueError('ChangedV35ShellGeometry')
    def transition_proposal(self,cap,triangles,controls):
        self.check()
        return None  # Existing V4 generic finite-transition construction.

def prepare(initial,owner,folder,*,source_handoff=None,verification_mode='audit',_source_evidence_scope=None):
    """Call while the surface/source lifetime and native-file guards are held."""
    import generalize_local_remesh as g
    from accepted_trajectory_contract import _planned_quality
    from mixed_front_fixture_provider import Provider
    from live_spline_first_layer import SourceSheetFirstLayerProvider
    from production_fast_layer import FastLayer
    from sdf_cfd_mesher_v4.advancing_front.execution_profile import MemoryTransaction,ExecutionProfile
    from incremental_layer_validation import Topology,Quality,ExactVolume,key
    from incremental_degree import DegreeState
    from rolling_fast_state import LiveAcceptance,mesh_identity,numeric_identity,cache_identity,verify_fresh_coverage
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
    if type(initial) is not SourceSheetFirstLayerProvider or initial.verify_wall() is not True or owner.authority is not initial.source or initial.assigned.get('admitted') is not True:raise ValueError('V35LiveSourceFirstFrontRequired')
    owner.check();positions=owner.positions();N=len(positions)-1
    if initial.high.tobytes()!=positions[1].tobytes() or initial.x.tobytes()!=positions[0].tobytes():raise ValueError('V35FirstFrontMismatch')
    # Exact source-normal height and source progress on every actual segment.
    # No later path coefficient is mislabeled as a source-normal height.
    from source_handoff import consume,VERSION
    if verification_mode not in ('audit','production'):raise ValueError('UnknownSourceHandoffMode')
    proofs=consume(source_handoff,initial.source,initial.h1,positions,verification_mode)
    reused=proofs is not None
    if not reused:
        proofs=[]
        for k in range(1,N+1):
            print('shell.source_segments',k,N,flush=True)
            proof=initial.source.qualify_segments(positions[k-1],positions[k],first_height_m=initial.h1 if k==1 else None)
            if proof.get('passed') is not True:raise ValueError('V35SourceSegmentRefusal',k)
            proofs.append(proof)
    evaluation=dict(schema=VERSION,mode=verification_mode,
        method='live_final_population_authenticated' if reused else 'recomputed',
        independent_source_recomputation=not reused,layers=N,segments=N*len(initial.x),
        scope='Source progress and H1 only; independent face coverage and all quality/volume/degree gates remain mandatory')
    segments=dict(passed=True,layers=proofs,scope='Existing source-segment predicates; source surface retains its declared assurance')
    if verification_mode=='production':
        from source_evidence import retain
        retain(folder,evaluation,segments,positions,initial.source.geometry_identity,initial.h1,_source_evidence_scope)
    else:
        g.save(folder/'SOURCE_EVALUATION.json',evaluation);g.save(folder/'SOURCE_SEGMENTS.json',segments)
    points,cells,outer,owners,checked=_planned_quality(positions[0],initial.tri,positions,completed_front=True)
    g.save(folder/'RAW_QUALITY.json',checked)
    if not all(checked[k]['passed'] for k in ('quality','ownership','vertex_links')):raise ValueError('V35ShellQualityOrTopologyRefusal')
    authority=dict(initial.authority,v35_plan=owner.source_identity,v35_source_segments=identity(proofs))
    class CertificateProvider:
        pack=Provider.pack
        def phase(self,name,operation):
            print('shell.'+name,'start',flush=True);start=time.perf_counter()
            try:return operation()
            finally:self.times[name]=time.perf_counter()-start
    certifier=CertificateProvider();certifier.x=initial.x;certifier.tri=initial.tri;certifier.authority=authority
    certifier.meshes={};certifier.folder=folder;certifier.surface=initial.surface
    certifier.previous_certificate_state=None;certifier.times={}
    checked=SourceSheetFirstLayerProvider.certify_candidate(certifier,list(positions[1:]),np.full(len(initial.x),N),checked)
    if checked.get('passed') is not True:raise ValueError('V35FullShellAdmissionRefusal')
    ordered_faces=[owners[k][0][1] for k in sorted(owners)]
    methods=verify_fresh_coverage(points,ordered_faces,range(len(ordered_faces)),checked['certificate'])
    state=certifier.pack(list(positions[1:]),np.full(len(initial.x),N))
    state.update(quality={},certificate={},atlas={},forecast={},_production_unsealed=True)
    topology=Topology(cells,owners);wall=[tuple(reversed(f)) for f in initial.tri];topology.boundary(outer,wall)
    q=checked['quality'];prospective=checked['complete_incidence']
    quality=Quality({r['id']:r for r in q['cells']},{key(r['vertices']):{k:v for k,v in r.items() if k!='vertices'} for r in q['faces']},{r['id']:r for r in q['exact']},{r['id']:r for r in prospective['cells']})
    quality.bind_masks(topology,outer)
    volume=ExactVolume(points,cells,outer,initial.tri,quality.exact)
    if volume.total!=volume.boundary_total:raise ValueError('V35ExactVolumeMismatch')
    degree=DegreeState(points,wall+outer,volume.witness,checked['volume']['interior_degree'])
    run=object.__new__(FastLayer);run.rolling=True;run.profile=ExecutionProfile.ProductionFast
    run.folder=folder;run.tx=MemoryTransaction(state);run.points=points;run.outer=outer;run.wall=wall;run.topology=topology
    run.provider=PrismShellGeometry(initial.binding,owner,positions,None);run.provider.session=run.tx
    run.quality=quality;run.volume=volume;run.degree_state=degree
    prepared=PreparedShell()
    _prepared[prepared]=(initial,owner,run,checked,methods,array_hash(positions),mesh_identity(state),numeric_identity(points,topology,outer,wall),cache_identity(quality,volume,degree))
    g.save(folder/'PREPARED_SHELL.json',dict(passed=True,source_global_proof='passed_before_native_construction',layers=N,prisms=len(cells),mesh_accepted=False,scope='Prospective shell checks only; native construction pending'))
    return prepared

class PreparedShell:
    pass
from weakref import WeakKeyDictionary
_prepared=WeakKeyDictionary()

def enroll(prepared,native):
    import generalize_local_remesh as g
    from rolling_fast_state import LiveAcceptance,mesh_identity,numeric_identity,cache_identity
    if type(prepared) is not PreparedShell or prepared not in _prepared:raise ValueError('V35PrivatePreparedShellRequired')
    initial,owner,run,checked,methods,digest,planned_mesh,planned_numeric,planned_cache=_prepared.pop(prepared)
    owner.check();run.provider.check();positions=owner.positions();N=len(positions)-1
    if array_hash(positions)!=digest:raise ValueError('ChangedV35PreparedPositions')
    if mesh_identity(run.tx.current)!=planned_mesh or numeric_identity(run.points,run.topology,run.outer,run.wall)!=planned_numeric or cache_identity(run.quality,run.volume,run.degree_state)!=planned_cache:raise ValueError('ChangedV35PreparedShell')
    from native import readback
    raw=Path(native['folder']).resolve()
    native_case=raw.parents[2]
    sealed=json.loads((native_case/'SEALED_PLAN.json').read_text())
    if file_hash(native_case/'SEALED_PLAN.npz')!=sealed['sha256']:raise ValueError('ChangedV35NativePlan')
    with np.load(native_case/'SEALED_PLAN.npz',allow_pickle=False) as saved:
        if saved['positions'].tobytes()!=positions.tobytes() or saved['faces'].tobytes()!=initial.tri.tobytes():raise ValueError('V35NativePlanBindingMismatch')
    # Never import a caller-supplied passed flag or detached numeric dictionary.
    native=readback(raw,positions,initial.tri,sealed['arithmetic_tolerance_m'])
    if native['positions'].tobytes()!=positions.tobytes():raise ValueError('V35NativeArithmeticRequiresExactV4Binding')
    if initial.high.tobytes()!=positions[1].tobytes() or initial.x.tobytes()!=positions[0].tobytes():raise ValueError('V35FirstFrontMismatch')
    if len(run.topology.cells)!=len(native['volumes']) or len(run.points)!=len(native['points']):raise ValueError('V35NativeInventoryMismatch')
    native_identity=identity(dict(points=array_hash(native['points']),positions=array_hash(native['positions']),faces=identity(native['faces']),owners=array_hash(native['owner']),neighbours=array_hash(native['neighbour']),patches=native['boundary_patches']))
    state=run.tx.current
    state['authority']=dict(state['authority'],v35_native=native_identity)
    run.provider.native_identity=native_identity
    topology=run.topology;points=run.points;outer=run.outer;wall=run.wall
    quality=run.quality;volume=run.volume;degree=run.degree_state
    run.live_acceptance=LiveAcceptance(mesh_identity(state),numeric_identity(points,topology,outer,wall),cache_identity(quality,volume,degree),identity(state['authority']),checked['certificate']['required_pairs'],tuple(sorted(methods.items())),run.tx.generation)
    run.live_acceptance.check(run)
    g.save(run.folder/'ACCEPTED_SHELL.json',dict(passed=True,mesh=run.live_acceptance.mesh,prisms=len(topology.cells),layers=N,source=initial.binding.geometry_identity,native=native_identity,surface_assurance=initial.wall_authority['assurance'],certificate_relations=run.live_acceptance.count,methods=dict(run.live_acceptance.methods),scope='Live V3.5 prism shell only; no core/CFD qualification'))
    return run
