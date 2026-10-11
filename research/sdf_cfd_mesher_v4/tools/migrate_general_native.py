"""Native historical rollback from sealed pre-remesh coordinates and targets.

Accepted remesh geometry is comparison/proof-reuse authority only. Discovery
and replacement connectivity are reconstructed from current intersecting caps.
"""
import json,time,subprocess
from copy import deepcopy
from pathlib import Path
import generalize_local_remesh as g
import verify_local_cavity_remesh as smoke
from probe_general_migration import stored_seed
from mixed_front_fixture_provider import Provider
from audit_production_migration import OUT
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash,rebuild_dependencies
from sdf_cfd_mesher_v4.advancing_front.mixed_schedule import bind_schedule
from sdf_cfd_mesher_v4.advancing_front.mixed_front_runner import MixedFrontRunner
from sdf_cfd_mesher_v4.common.authority import identity,file_hash


class HistoricalProvider(Provider):
    def __init__(self,folder):
        self.case,self.prefix,self.history,_,self.seed=stored_seed()
        self.x=self.case.x;self.tri=self.case.f;self.spec=self.case.spec;self.schedule=self.case.s
        self.folder=Path(folder);self.folder.mkdir(parents=True,exist_ok=True)
        path=g.s.ROOT/'results/residual_layer5_failure/baseline.npz'
        auth=json.loads((path.parent/'AUTHORITY.json').read_text())
        # The payload must be named by the preserved authority, not accepted on
        # the strength of an unverified local filename.
        hashes={v for table in auth.values() if isinstance(table,dict) for v in table.values() if isinstance(v,str)}
        if file_hash(path) not in hashes:raise ValueError('UnsealedHistoricalBaseline')
        with g.np.load(path) as z:self.high=z['high'];self.targets=z['targets'];low=z['low']
        assert g.np.array_equal(low,self.prefix[-1])
        expected=json.loads((path.parent/'REPRODUCER_IDENTITY.json').read_text())
        assert g.q.array_hash(self.high)==expected['coordinate_hash']
        self.fronts=self.prefix+[self.high]
        oldmap=json.loads((g.s.ROOT/'results/local_layer_termination_topology/candidate_1_map.json').read_text())
        self.base=g.np.full(len(self.x),5);self.base[oldmap['stopped_vertices']]=4
        self.authority=dict(schedule=g.q.array_hash(self.schedule.targets),quality=file_hash(Path(g.t.__file__)),certificate=file_hash(Path(g.s.cert.__file__)))
        self.records=[];self.times={};self.checked={};self.meshes={};self.numerical={}

    def initialize(self):
        eligible={i for i,tri in enumerate(self.tri) if all(self.base[v]==5 for v in tri)}
        state=self.pack(self.fronts,self.base,eligible=eligible)
        actual={str(v):[float(h[v]) for h in self.history]+[float(self.targets[v])]+
            [float(self.targets[v]+(target-self.schedule.targets[4])) for target in self.schedule.targets[5:]] for v in range(len(self.x))}
        bind_schedule(state,self.schedule.targets,actual)
        rebuild_dependencies(None,state,self.schedule.targets.tolist())
        return state

    def geometry(self,state,extra=0):
        depth=g.np.zeros(len(self.x),int)
        for key in state['vertices']:
            _,layer,_,source=key.split(':');depth[int(source)]=max(depth[int(source)],int(layer))
        fronts=[x.copy() for x in self.fronts[:max(depth)]]
        while len(fronts)<max(depth):fronts.append(fronts[-1].copy())
        for key,p in state['vertices'].items():
            _,layer,_,source=key.split(':')
            if int(layer):fronts[int(layer)-1][int(source)]=p
        return fronts,depth

    def propose(self,state):
        fronts,levels=self.geometry(state)
        if not state['events']:
            return dict(state=state,fronts=fronts,levels=levels,layer=len(fronts),historical_repair=True)
        result=smoke.continuation(self.case,self.prefix,self.high,self.targets,dict(vertex_depth=levels.tolist()))
        if not result['passed']:raise ValueError('NeighborContinuationFailure')
        selected=next(r for r in result['attempts'] if r['passed']);high=self.high.copy()
        owner=g.s.r.b.owner(self.x,self.spec)
        for i in selected['active']:
            high[i:i+1],_=g.s.r.b.sphere_project(self.high[i:i+1],float(self.targets[i]+selected['scale']*self.schedule.increments[5]),self.case.sdf,self.spec,owner[i:i+1])
        assert g.q.array_hash(high)==selected['front_hash']
        trial=levels.copy();trial[selected['active']]=6
        self.fronts=self.prefix+[self.high,high]
        candidate=self.pack(self.fronts,trial,state)
        for i in selected['active']:candidate['local_schedule']['actual'][str(i)][5]=float(self.targets[i]+selected['scale']*self.schedule.increments[5])
        self.records.append(dict(continuation=result))
        return dict(state=candidate,fronts=self.fronts,levels=trial,layer=6,historical_repair=False)

    def evidence(self,state,proposal):
        from sdf_cfd_mesher_v4.advancing_front.termination_policy import IMMEDIATE_CHECKS
        context=dict(base=scientific_hash(state),region=identity(state['prism_front']),layer=proposal['layer'],topology=identity(state['faces']),schedule=self.authority['schedule'])
        m=self.meshes[identity(proposal['state']['cells'])]
        cert=g.s.cert.certify(g.np.array(m['points']),m['outer'],range(len(m['outer'])))
        ref=dict(context=context,proof=identity(g.s.r.w.clean(cert)))
        vertices=sorted({int(v)%len(self.x) for bad in cert['failures'] for face in bad['pair'] for v in m['outer'][face]})
        columns=sorted({i for bad in cert['failures'] for i in bad['pair']})
        seed=dict(vertices=vertices,columns=columns,cells=[],faces=[])
        self.failure_layer=min((int(v)//len(self.x) for bad in cert['failures'] for f in bad['pair'] for v in m['outer'][f]),default=proposal['layer'])
        valid=cert['passed'];proposal['check']=dict(passed=valid)
        return dict(context=context,support=dict(ref,status='clean'),immediate=dict(ref,status='valid' if valid else 'invalid',seed=seed),
            warnings=[] if valid else [ref['proof']],attempts=[],
            witnesses=[] if valid else [dict(ref,scope='immediate',qualified=True,kind='ExactCollision',seed=seed)],
            accepted=dict(ref,stage='NominalAdvance',checks={k:True for k in IMMEDIATE_CHECKS}) if valid else None)

    def discover(self,before,proposal,decision):
        from coupled_shell_validation import check
        levels,expansion=g.discover(self.x,self.tri,proposal['fronts'],set(decision['seed']['vertices']),self.failure_layer,proposal['levels'],candidate_check=check)
        if levels is None:raise ValueError('CavityDiscoveryFailure')
        candidate=self.pack(proposal['fronts'],levels,before)
        removed=sorted(k for k,c in before['cells'].items() if candidate['cells'].get(k)!=c)
        result=dict(removed_cells=removed,levels=levels.tolist(),expansion=expansion,seed=decision['seed'],boundary=identity(before['outer']))
        g.save(self.folder/'ROLLBACK_MANIFEST.json',dict(cavity=result,removed=[before['cells'][k] for k in removed]))
        return result

    def certify_validation(self,state,proposal,cavity):
        key=scientific_hash(state)
        if key not in self.numerical:raise ValueError('MissingBoundNumericalValidation')
        p,c,o,checked,times,fronts,levels=self.numerical.pop(key)
        from coupled_shell_validation import qualify_closures
        qualify_closures(self.folder,proposal['layer'],p,c,checked)
        owners,_=g.t.ownership(c,o,[tuple(reversed(f)) for f in self.tri]);faces=[owners[k][0][1] for k in sorted(owners)]
        # Reuse the unchanged qualified proof only after exact full geometry
        # and connectivity equality. A changed continuation is certified fresh.
        old=g.s.ROOT/'results/local_termination_cavity_remesh'
        selected=json.loads((old/'SELECTED.json').read_text())
        same=g.np.array_equal(p,g.np.load(old/'mesh_points.npy')) and identity(c)==identity(selected['cells']) and identity(o)==identity(selected['outer'])
        if same:
            auth=json.loads((old/'AUTHORITY.json').read_text())
            for name in ('SELECTED.json','mesh_points.npy','FULL_FACE_CERTIFICATE.json'):
                assert file_hash(old/name)==auth['evidence_sha256']['results/local_termination_cavity_remesh/'+name]
            cert=json.loads((old/'FULL_FACE_CERTIFICATE.json').read_text())
        else:cert=g.s.cert.certify(p,faces,range(len(faces)))
        volume=g.volume.audit(p,c,o,[tuple(map(int,f)) for f in self.tri])
        checked.update(certificate=cert,volume=volume,passed=cert['passed'] and volume['interior_degree']['passed'] and volume['exact_volume_balance'] and volume['positive_cells'])
        if not checked['passed']:raise ValueError('ExactMixedValidationFailure')
        assigned=True
        for key,pnt in state['vertices'].items():
            _,layer,_,source=key.split(':')
            if int(layer):
                q=self.case.sdf.evaluate(g.np.array([pnt]));target=state['local_schedule']['actual'][source][int(layer)-1]
                assigned &= bool(g.np.all(abs(q.phi-target)<=q.source_error_bound))
        ids=sorted({v for face in o for v in face});lookup={v:i+1 for i,v in enumerate(ids)}
        g.save(self.folder/'CORE_INGEST_INPUT.json',dict(coordinates=p[ids].tolist(),triangles=[[lookup[v] for v in f] for f in o],outer_hash=identity(o)))
        def wsl(path):return '/mnt/'+path.drive[0].lower()+str(path)[2:].replace('\\','/')
        subprocess.run(['wsl','--','python3',wsl(g.s.ROOT/'tools/cavity_core_ingestion_smoke.py'),wsl((self.folder/'CORE_INGEST_INPUT.json').resolve())],check=True,capture_output=True)
        core=json.loads((self.folder/'CORE_INGESTION.json').read_text())
        continuation=smoke.continuation(self.case,self.prefix,self.high,self.targets,dict(vertex_depth=levels.tolist())) if cavity else dict(passed=True)
        checks=dict(hard_quality=True,exact_validity=True,ownership=checked['ownership']['passed'],continuous=cert['passed'],sdf_target=assigned,
            volume_conformity=checked['passed'],outer_interface=checked['vertex_links']['passed'],core_compatibility=core['passed'],neighbor_continuation=continuation['passed'],
            wall_first_layer=state['wall']['vertices']=={str(i):v.tolist() for i,v in enumerate(self.x)},outside_preserved=True)
        if not all(checks.values()):raise ValueError('GeneralMixedAcceptanceFailed:'+str(checks))
        g.save(self.folder/f'VALIDATION_{proposal["layer"]}.json',dict(state=scientific_hash(state),checks=checks,
            volume=volume,ownership=checked['ownership'],vertex_links=checked['vertex_links'],
            quality_passed=checked['quality']['passed'],quality_failures=checked['quality']['failures'],
            certificate=dict(passed=cert['passed'],required_pairs=cert['required_pairs'],failures=cert['failures'],proof_hash=cert['proof_hash']),
            validation_hash=identity(g.s.r.w.clean(checked))))
        self.checked[scientific_hash(state)]=checked
        self.records.append(dict(reference_geometry_equal=same,certificate_reused=same,relations=cert['required_pairs']))
        return dict(state=scientific_hash(state),passed=True,proof=identity(g.s.r.w.clean(checked)),checks=checks)


def run():
    import traceback
    folder=OUT/'general_native';provider=HistoricalProvider(folder);tick=time.perf_counter()
    try:
        initial=provider.initialize();runner=MixedFrontRunner(folder/'checkpoints',provider);runner.initialize(initial)
        operations=[]
        for i in range(2):
            operations.append(runner.step());g.save(folder/'OPERATIONS.json',operations);print('COMMITTED',i,flush=True)
        final=runner.store.restart()
        g.save(folder/'RESULT.json',dict(initial=scientific_hash(initial),final=scientific_hash(final),records=provider.records,
            wall_equal=initial['wall']==final['wall'],first_layer_equal=initial['first_layer']==final['first_layer'],cells=len(final['cells'])))
    except BaseException:
        g.save(folder/'FAILURE.json',dict(traceback=traceback.format_exc()));raise
    finally:g.save(folder/'RUNTIME.json',dict(seconds=time.perf_counter()-tick,peak_rss=g.q.psutil.Process().memory_info().peak_wset))

if __name__=='__main__':run()
