"""Real-mesh crash/restart tests with exact-input-bound validation reuse.

Fresh primary geometry/certificates must already exist. Replay reconstructs
the cavity from pre-event evidence. It may reuse validation only after exact
identity of the complete candidate mesh and authority has been established.
No production atlas/forecast integration claim is made.
"""
import json,sys,tempfile,time
from pathlib import Path
import exercise_mixed_remesh_transactions as e
from sdf_cfd_mesher_v4.advancing_front.remesh_transaction import (
    MixedTransactionStore,unpack,identity,flattened_inputs,retain_receipts,file_hash)
g=e.g; np=e.np; OUT=e.OUT

class Crash(BaseException): pass

def run(name):
    folder=OUT/name/'final'; z=json.loads((folder/'result.json').read_text())
    original=MixedTransactionStore(folder/'checkpoints').restart()
    base=unpack(folder/'checkpoints'/f"STATE_{original['previous']}.json")
    evidence=z['evidence']; spec=next(s for s in g.fixtures.definitions() if s['name']==name)
    x=np.array(base['mesh']['wall']['coordinates']);tri=np.array(base['mesh']['wall']['triangles']);n=len(x)
    fronts=[np.array(base['mesh']['points'])[i:i+n] for i in range(n,len(base['mesh']['points']),n)]
    fronts.append(g.fixtures.front(spec,g.q.load_schedule().targets[len(fronts)]))
    rows=[];start=time.perf_counter()
    for stage in ('before_dispatch','after_private_rollback','before_publication','after_publication','forced_failure'):
        with tempfile.TemporaryDirectory(prefix='v4-mixed-restart-') as temp:
            store=MixedTransactionStore(temp);store.initialize(base['mesh'],base['local'],base['authority']);calls=[]
            def backend(request,private):
                levels,expansion=g.discover(x,tri,fronts,set(request['seed']['vertices']),evidence['context']['layer'])
                if levels is None:raise ValueError('ReplayDiscoveryFailed')
                new=e.mesh(x,tri,fronts,levels)
                if identity(new)!=identity(original['mesh']):raise ValueError('ReplayMeshMismatch')
                manifest=g.s.removal_manifest(private['cells'],new['cells'])
                details=dict(removed_cells=[r['cell']['id'] for r in manifest['removed']],manifest=manifest,
                             expansion=expansion,seed=request['seed'],levels=levels.tolist())
                calls.append(identity(new));return new,details
            def rebuild(old,new,details):
                a=flattened_inputs(old['mesh'],old['authority']);b=flattened_inputs(new,old['authority'])
                kept={};invalid={}
                for family,value in old['local']['receipts'].items():
                    kept[family],invalid[family]=retain_receipts(value,a,b)
                return dict(receipts=kept),dict(invalidated={k:len(v) for k,v in invalid.items()},
                    retained={k:len(v) for k,v in kept.items()},keys=invalid)
            def validate(request,new,local,details):
                if identity(new)!=identity(original['mesh']) or base['authority']!=original['authority']:
                    raise ValueError('UnboundCertificateReuse')
                if stage=='forced_failure':raise ValueError('InjectedFailureAfterPrivateRemesh')
                local.update(original['local'])
                return z['transaction']['invocation']['receipt']
            def crash(point):
                if point==stage:raise Crash()
            pre=file_hash(Path(temp)/'CURRENT.json')
            if stage=='forced_failure':
                record=store.dispatch(evidence,backend,rebuild,validate)
                equal=store.restart()==base and pre==file_hash(Path(temp)/'CURRENT.json')
                assert equal and not record['committed']
            else:
                try:store.dispatch(evidence,backend,rebuild,validate,crash)
                except Crash:pass
                checkpoint=store.restart()
                assert checkpoint['transaction']==int(stage=='after_publication')
                record=store.dispatch(evidence,backend,rebuild,validate)
                equal=identity(store.restart())==identity(original)
                if not equal:
                    g.save(folder/'REPLAY_MISMATCH.json',dict(stage=stage,record=record,expected=z['transaction']))
                assert equal
                before_calls=len(calls);assert store.dispatch(evidence,backend,rebuild,validate)==record
                assert len(calls)==before_calls
            rows.append(dict(stage=stage,passed=equal,backend_calls=len(calls),final_state=identity(store.restart()),
                             exact_geometry_hashes=calls,certificate_reuse='Complete mesh and implementation authority identities checked'))
    g.save(folder/'RESTART_REPLAY.json',dict(passed=True,records=rows,production_runner=False))
    g.save(folder/'restart_runtime.json',dict(seconds=time.perf_counter()-start))
    print(name,'real-mesh restart/rollback passed',flush=True)

if __name__=='__main__':run(sys.argv[1])
