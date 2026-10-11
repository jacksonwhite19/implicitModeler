"""One ordinary production advance from sealed L1. No fixture initialization.

Uses the existing SDF predictor, mixed-cell validators, continuous certificate,
volume/degree audit, ordinary transaction runner and native checkpoint writer.
There is no later-front, repair, remesh, or core-ingestion operation here.
"""
import json,time,sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import psutil
import generalize_local_remesh as g
from run_geometry_family import AnalyticProvider
from mixed_front_fixture_provider import Provider
from production_surface_authority import load_production_wall
from production_forecast_admission import ProductionForecastAdapter
from geometry_family_authority import Source,sdf_record
from run_production_collision_localization import save
from capsule_layer2_preflight import V4,L1,SURFACE,REQUEST
from sdf_cfd_mesher_v4.common.authority import identity,file_hash,array_hash
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash
from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import MixedFrontRunner,preserves_committed_cells
from sdf_cfd_mesher_v4.advancing_front.mixed_front_journal import InMemoryMixedFrontJournal
from validation_row_stream import validation_digest

OUT=V4/'results/production_adaptive_capsule_layer2'
ADMISSION=V4/'results/production_layer2_readonly_admission'


def restore():
    for name,h in [('report.md','0a5d2e2204c31901a8f2b7d43ad0f8a22fac51b16432b05f29d69eeefdb81f10'),('AUTHORITY.json','9ab7734b79ed2fb2480c92e91eb6fd35336d992d29084684d7a6abe4f78e0ccd')]:
        if file_hash(ADMISSION/name)!=h:raise ValueError('AdmissionAuthorityMismatch')
    admission=json.loads((ADMISSION/'AUTHORITY.json').read_text())
    if admission['admission_identity']!='1453e6492cbb4bdf8bb5297871732415ce33775dc754ddd38d12ad790c9849e3' or admission['decision']['status']!='ProductionLayer2AdvanceAuthorized':raise ValueError('AdmissionNotAuthorized')
    for name,h in admission['evidence'].items():
        if file_hash(ADMISSION/name)!=h:raise ValueError('AdmissionEvidenceMismatch')
    wall,tri,surface=load_production_wall(SURFACE);spec=dict(components=[json.loads((SURFACE.parent/'GEOMETRY.json').read_text())['definition']])
    state=NativeCheckpointStore(L1/'A/checkpoints').restart();b=admission['binding']
    expected=dict(committed=b['committed'],geometry_identity=b['geometry'],surface_identity=b['surface'],request=b['request'],schedule=b['schedule'],request_file=b['request_file'],layer=1,front_coordinates=b['coordinates'])
    adapter=ProductionForecastAdapter(state,wall,tri,surface,REQUEST,Source(spec),expected)
    if adapter.binding!=b:raise ValueError('AdmissionFrontMismatch')
    if float(adapter.remaining_increments[0]).hex()!='0x1.6b844d56c388ep-13':raise ValueError('L2IncrementMismatch')
    return state,adapter,surface,spec,admission


class ProductionNextLayerProvider(AnalyticProvider):
    # Defaults preserve the sealed L2 adapter and its guard-only tests.
    base_layer=1
    next_layer=2
    def __init__(self,folder,state,adapter,spec):
        # Do not invoke either fixture constructor or the first-layer provider.
        self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=True)
        self.x=adapter.committed_prefix[0];self.tri=adapter.topology.triangles;self.spec=spec;self.source=Source(spec)
        self.schedule=adapter.schedule;self.authority=dict(state['authority']);self.base=scientific_hash(state)
        self.records=[];self.times={};self.checked={};self.meshes={};self.numerical={}
        self.reuse_certificates=True;self.previous_certificate_state=state;self.generated=0
        self.base_layer=adapter.binding['layer'];self.next_layer=adapter.binding['next_layer']
        if self.next_layer!=self.base_layer+1:raise ValueError('NextLayerMismatch')

    def phase(self,name,fn):
        print(name,'start',flush=True);t=time.perf_counter();value=fn();self.times[name]=time.perf_counter()-t
        print(name,'done',self.times[name],flush=True);return value

    def geometry(self,state,extra=0):
        layers={int(c['last_prism_layer']) for c in state['columns'].values()}
        if len(layers)!=1 or not layers<={self.base_layer,self.next_layer}:raise ValueError('UniformBoundPrefixRequired')
        layer=next(iter(layers));n=len(self.x)
        if any(int(k.split(':')[1])>layer for k in state['vertices']):raise ValueError('UnexpectedFutureState')
        fronts=[np.array([state['vertices'][f'layer:{k}:source:{i}'] for i in range(n)]) for k in range(1,layer+1)]
        if extra:
            if extra!=1 or layer!=self.base_layer or self.generated:raise ValueError(f'ExactlyOneL{self.next_layer}NoL{self.next_layer+1}')
            self.prediction=self.phase(f'L{self.next_layer}_construction',lambda:g.q.predict_next_front(fronts[-1],self.schedule.targets[self.next_layer-1],self.source,iteration_cap=256))
            fronts.append(self.prediction.coordinates);self.generated+=1
        return fronts,np.full(n,layer,int)

    def propose(self,state):
        if scientific_hash(state)!=self.base:raise ValueError(f'StaleL{self.base_layer}')
        fronts,levels=self.geometry(state,1);levels[:]=self.next_layer
        p,c,o,checked,times=self.phase('combined_numeric',lambda:g.validation(self.x,self.tri,fronts,levels,False));self.times.update(times)
        from final_shell_incidence import check
        incidence=self.phase('completed_neighbour_determinant',lambda:check(p,c,o))
        # Store the qualified actual-neighbour plus declared terminal-cap tensor.
        # Remove only the diagnostic exposed-cap cell-quality verdict.
        values={r['id']:r['determinant'] for r in incidence['cells']}
        q=checked['quality'];self.exposed=np.array([r['determinant'] for r in q['cells']])
        q['cells']=[dict(r,determinant=values[r['id']]) for r in q['cells']]
        q['failures']=[r for r in q['failures'] if r['kind']!='cell_quality']
        q['failures'] += [dict(kind='cell_quality',**r) for r in q['cells'] if not (r['volume']>0 and r['determinant']>=.0011 and r['aspect']<=1000)]
        q['passed']=not q['failures'];checked['complete_incidence']=incidence
        self.assigned=sdf_record(self.spec,fronts[-1],self.schedule.targets[self.next_layer-1])
        self.points,self.cells,self.outer,self.fronts=p,c,o,fronts
        np.savez_compressed(self.folder/'CANDIDATE.npz',front=fronts[-1],triangles=self.tri)
        save(self.folder/'RAW_QUALITY.json',q);save(self.folder/'INCIDENCE.json',incidence)
        if not (q['passed'] and incidence['passed'] and self.assigned['admitted']):raise ValueError(f'NominalLayer{self.next_layer}RejectedNoNewRepair')
        checked=self.phase('final_continuous_and_volume',lambda:self.certify_candidate(fronts,levels,checked))
        self.check=checked;save(self.folder/'QUALITY.json',q);save(self.folder/'VOLUME.json',checked['volume'])
        save(self.folder/'CERTIFICATE.json',{k:v for k,v in checked['certificate'].items() if k!='proofs'})
        if not checked['passed']:raise ValueError('FinalCertificationRejected')
        candidate=self.pack(fronts,levels,state)
        if not preserves_committed_cells(candidate,state):raise ValueError('CommittedPrefixMutation')
        self.candidate_geometry=identity({k:candidate[k] for k in ('wall','first_layer','vertices','cells','faces','outer','columns','prism_front')})
        return dict(state=candidate,fronts=fronts,levels=levels,check=checked,layer=self.next_layer)

    def evidence(self,state,proposal):return Provider.evidence(self,state,proposal)

    def bind(self,state,checked,previous=None):
        # The private journal restores a mutable ReceiptOverlay. Delta proof
        # binding requires its sealed immutable ReceiptTable base, not that
        # wrapper. Verify scientific equality before using the original base;
        # keep attach_validation's stale-base and geometry guards unchanged.
        if previous is not None:
            if scientific_hash(previous)!=self.base:raise ValueError(f'StaleL{self.base_layer}ReceiptBase')
            previous=self.previous_certificate_state
        return Provider.bind(self,state,checked,previous)

    def validate(self,state,proposal,cavity):
        if cavity is not None or not self.check['passed'] or not self.assigned['admitted']:raise ValueError(f'L{self.next_layer}FinalGateRejected')
        if identity({k:state[k] for k in ('wall','first_layer','vertices','cells','faces','outer','columns','prism_front')})!=self.candidate_geometry:raise ValueError('CandidateMutation')
        key=scientific_hash(state);self.checked[key]=self.check
        return dict(state=key,passed=True,proof=validation_digest(self.check))

    def correct(self,before,proposal,decision):return Provider.correct(self,before,proposal,decision)
    def discover(self,*args):raise ValueError('NoNewRepairOrRemeshInLayer2Task')


# Backward-compatible name for the sealed L2 orchestration below.
ProductionLayer2Provider=ProductionNextLayerProvider


def run(label):
    start=time.perf_counter();print('restore start',flush=True);state,adapter,surface,spec,admission=restore();restore_time=time.perf_counter()-start
    print('restore done',restore_time,flush=True);folder=OUT/label
    provider=ProductionLayer2Provider(folder,state,adapter,spec)
    runner=MixedFrontRunner(folder/'checkpoints',provider,profile='ProductionFast')
    runner.store=InMemoryMixedFrontJournal(state)
    try:
        transaction=provider.phase('production_transaction',runner.step)
        committed=runner.store.restart()
        if not preserves_committed_cells(committed,state) or committed['wall']!=state['wall'] or committed['first_layer']!=state['first_layer']:raise ValueError('CommittedPrefixMutation')
        if {c['last_prism_layer'] for c in committed['columns'].values()}!={2}:raise ValueError('WrongCommittedLayer')
        # The in-memory transaction is still private. Neither run may publish
        # its checkpoint until both complete scientific records match.
        ready=dict(passed=True,committed=transaction['state'],transaction=transaction,coordinates=array_hash(provider.fronts[-1]),
            topology=array_hash(provider.tri),new_connectivity=array_hash(np.array([c['vertices'] for c in provider.cells if c['layer']==2],np.int64)),
            combined_connectivity=array_hash(np.array([c['vertices'] for c in provider.cells],np.int64)),
            ownership=identity([(c['id'],c['column']) for c in provider.cells]),quality=file_hash(folder/'QUALITY.json'),
            incidence=file_hash(folder/'INCIDENCE.json'),certificate=provider.check['certificate']['proof_hash'],generated=provider.generated)
        save(folder/'READY.json',ready);print('READY_FOR_REPLAY',label,identity(ready),flush=True)
        barrier=OUT/'REPLAY_AUTHORIZATION.json';waiting=time.perf_counter()
        while not barrier.exists():
            if time.perf_counter()-waiting>3600:raise RuntimeError('ReplayAuthorizationNotAvailable')
            time.sleep(1)
        authorization=json.loads(barrier.read_text())
        if authorization.get('scientific_identity')!=identity(ready):raise ValueError('ReplayAuthorizationMismatch')
        for runname in ('A','B'):
            other=json.loads((OUT/runname/'READY.json').read_text())
            if other!=ready:raise ValueError('ReplayMismatchBeforePublication')
        store=NativeCheckpointStore(folder/'checkpoints')
        committed_id=provider.phase('checkpoint',lambda:store.publish(committed))
        if committed_id!=transaction['state']:raise ValueError('CheckpointIdentityMismatch')
        result=dict(**ready,checkpoint_pointer=file_hash(folder/'checkpoints/CURRENT.json'),publication=dict(L2=True,L3=False,core=False),
            times=provider.times,restore_seconds=restore_time,total_seconds=time.perf_counter()-start,
            peak_rss=psutil.Process().memory_info().peak_wset,retained_rss=psutil.Process().memory_info().rss)
        wall,tri,after=load_production_wall(SURFACE)
        if after!=surface or file_hash(REQUEST)!=admission['binding']['request_file']:raise ValueError('AuthorityMutation')
        save(folder/'FINAL.json',result);print(json.dumps(result),flush=True)
    except Exception as exc:
        save(folder/'FAILURE.json',dict(error=type(exc).__name__+':'+str(exc),times=provider.times,generated=provider.generated,committed_L2=False,total_seconds=time.perf_counter()-start))
        raise


if __name__=='__main__':run(sys.argv[1])
