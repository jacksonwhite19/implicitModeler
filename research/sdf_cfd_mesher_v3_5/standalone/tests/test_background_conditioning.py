"""Normal handoff controls; no fresh/live authority is fabricated by tests."""
from pathlib import Path
from dataclasses import asdict
import sys,tempfile,unittest,json,copy
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]));import geometry
from test_core_connectivity import bipyramid
from background_materialization import build
from background_topology_repair import boundary,orient
import local_termination_topology as topology
from background_conditioning import apply,verify,from_joined,core_arrays
from constrained_local import repair as cavity_repair,Policy as CavityPolicy,population
from core_connectivity import Policy as VertexPolicy
from direct_connection import policy
from sdf_cfd_mesher_v4.common.authority import identity,file_hash

def fixture(thin=.03):
    x,raw,_,_=bipyramid(thin)
    # Stationary halo is a component fixture, not an admitted prism source.
    prefix=build(np.asarray([c['vertices'] for c in raw[2:]],dtype='<i8'),0)
    core=build(np.asarray([c['vertices'] for c in raw[:2]],dtype='<i8'),len(prefix))
    cells=prefix+core;far=np.asarray(boundary(cells)[1],dtype='<i8');owners,inc=topology.ownership(cells,list(map(tuple,far)),[]);assert inc['passed']
    cap=np.asarray([f[::-1] for f in boundary(core)[1]],dtype='<i8')
    return dict(points=x,bulk_tets=np.asarray([c['vertices'] for c in core],dtype='<i8'),cells=cells,owners=owners,
        farfield=far,wall=np.empty((0,3),dtype='<i8'),prefix_front=cap,prefix_ids=[c['id'] for c in prefix],prefix_points=len(x),
        provenance=dict(binding=dict(test_component_only=True),live_parent=False,generation_invocations=0),bounds=np.asarray([[-20.,20.]]*3))

def selected():return policy([[-20.,20.]]*3,.26236426446749106,1.)

def save_native(folder,original):
    native=folder/'native/generated';native.mkdir(parents=True)
    np.savez(native/'NETGEN.npz',points=original['points'][:5],tets=original['bulk_tets'])

def saved_arrays(joined):
    prefix=set(joined['prefix_ids']);core=[c for c in joined['cells'] if c['id'] not in prefix]
    return dict(points=joined['points'],background_tets=joined['bulk_tets'],tetrahedron_ids=np.asarray([c['id'] for c in core],dtype='<i8'),
        cap=joined['prefix_front'],farfield=joined['farfield'],wall=joined['wall'])

def refused_vertex(points,cells,owners,core_ids,far,folder,*,minimum,policy,check):
    # Fault-injected proposal exhaustion tests orchestration, while the actual
    # constrained driver and all mathematical admission predicates still run.
    from constrained_local import CavityTopology
    folder.mkdir();s=CavityTopology(points,cells,owners,core_ids,far);ids,rows,q,bad=population(s,minimum)
    a=core_arrays(s);np.savez_compressed(folder/'INPUT.npz',**a);np.savez_compressed(folder/'RESULT.npz',**a)
    r=dict(outcome='FiniteConnectivityFamilyRefusal',policy=asdict(policy),commits=[],history=[],initial_bad_cells=bad,initial_minimum=float(q.min()),initial_core_cells=len(ids))
    (folder/'REPORT.json').write_text(json.dumps(r));return s,r

def inserted_cavity(s,folder,*,minimum,policy,check,proposer):
    def synthetic(s,old,faces,seed,rule,trial,limits,check):
        trial.mkdir();x=s.points[:5].copy();rows=np.asarray(orient(x,[[3,4,0,1],[3,4,1,2],[3,4,2,0]]),dtype='<i8')
        vertex=len(x);centre=x[rows[0]].mean(0);x=np.vstack((x,centre+.05*(x[rows[0,0]]-centre)))
        sub=build(rows[:1],0)[0];rows=np.vstack((rows[1:],np.asarray(orient(x,[[vertex,*f] for f in sub['faces']]),dtype='<i8')))
        path=trial/'optimized.npz';np.savez_compressed(path,points=x,tetrahedra=rows)
        return dict(stages=[dict(stage='optimized',sha256=file_hash(path),intermediate_passed=True)])
    return cavity_repair(s,folder,minimum=minimum,policy=policy,check=check,proposer=synthetic)

class Handoff(unittest.TestCase):
    def test_scoped_replay_writes_only_separate_diagnostics_and_restores_context(self):
        from background_conditioning import replay_output_scope,_replay_output
        original=fixture(.5)
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);folder=root/'bundle';folder.mkdir();save_native(folder,original)
            joined,r=apply(original,folder/'conditioning',selected(),lambda:None)
            before={p.relative_to(folder).as_posix():file_hash(p) for p in folder.rglob('*') if p.is_file()}
            with replay_output_scope(root/'offline_conditioning'):
                points=verify(folder,dict(prefix_points=original['prefix_points']),saved_arrays(joined),joined,selected(),lambda:None)
            self.assertEqual(points.tobytes(),joined['points'].tobytes());self.assertIsNone(_replay_output.get())
            self.assertTrue((root/'offline_conditioning').is_dir())
            self.assertEqual(before,{p.relative_to(folder).as_posix():file_hash(p) for p in folder.rglob('*') if p.is_file()})

    def test_passing_mesh_is_true_noop(self):
        original=fixture(.5);before=(original['points'].tobytes(),identity(original['cells']),repr(original['owners']))
        def forbidden(*a,**k):raise AssertionError('No repair or native proposer on passing input')
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);save_native(folder,original)
            joined,r=apply(original,folder/'conditioning',selected(),lambda:None,vertex_constructor=forbidden,cavity_constructor=forbidden)
            points=verify(folder,dict(prefix_points=original['prefix_points']),saved_arrays(joined),joined,selected(),lambda:None)
            self.assertEqual(points.tobytes(),original['points'].tobytes())
        self.assertEqual(r['outcome'],'AlreadyAboveFloor');self.assertEqual(r['native_calls'],0);self.assertEqual(r['existing_vertex_commits'],0)
        self.assertEqual(before,(joined['points'].tobytes(),identity(joined['cells']),repr(joined['owners'])))
    def test_existing_vertex_success_and_native_bound_replay(self):
        original=fixture();before=identity(original['cells'])
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);save_native(folder,original);joined,r=apply(original,folder/'conditioning',selected(),lambda:None)
            self.assertEqual(r['existing_vertex_commits'],1);self.assertEqual(r['cavity_commits'],0)
            self.assertEqual(len(joined['bulk_tets']),3);self.assertEqual(identity(original['cells']),before)
            arrays=saved_arrays(joined);points=verify(folder,dict(prefix_points=original['prefix_points']),arrays,joined,selected(),lambda:None)
            self.assertEqual(points.tobytes(),joined['points'].tobytes())
    def test_inserted_point_and_actual_owners_propagate_and_replay(self):
        original=fixture()
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);save_native(folder,original)
            joined,r=apply(original,folder/'conditioning',selected(),lambda:None,vertex_constructor=refused_vertex,cavity_constructor=inserted_cavity)
            self.assertEqual(r['cavity_commits'],1);self.assertEqual(len(joined['points']),len(original['points'])+1)
            owners,inc=topology.ownership(joined['cells'],list(map(tuple,joined['farfield'])),[])
            self.assertTrue(inc['passed']);self.assertEqual(repr(sorted(owners.items())),repr(sorted(joined['owners'].items())))
            arrays=saved_arrays(joined);self.assertEqual(len(arrays['tetrahedron_ids']),len(joined['bulk_tets']))
            from finite_research.direct_readback import core_metadata
            rebuilt=core_metadata(joined['bulk_tets'],arrays['tetrahedron_ids'],conditioned=True,first_id=max(joined['prefix_ids'])+1)
            prefix=set(joined['prefix_ids']);self.assertEqual(rebuilt,[c for c in joined['cells'] if c['id'] not in prefix])
            self.assertGreater(min(arrays['tetrahedron_ids']),max(prefix)+1)
            points=verify(folder,dict(prefix_points=original['prefix_points']),arrays,joined,selected(),lambda:None)
            self.assertEqual(points.tobytes(),joined['points'].tobytes())
            # Exact V4 writer sees new point and complete owner-derived cells;
            # this serialization control does not claim a real live admission.
            from export_mesh import functions
            writer=functions()['write_msh'];target=folder/'transfer.msh'
            allrows=np.asarray([c['vertices'] for c in joined['cells']],dtype='<i8')
            writer(target,joined['points'],joined['wall'],joined['farfield'],np.empty((0,6),dtype='<i8'),allrows)
            text=target.read_text();self.assertIn('$Nodes\n'+str(len(joined['points']))+'\n',text)
    def test_refusal_keeps_original_state(self):
        original=fixture();before=(original['points'].tobytes(),identity(original['cells']),repr(original['owners']))
        def failed(*a,**kw):raise ValueError('InjectedConstructionRefusal')
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'conditioning'
            with self.assertRaisesRegex(ValueError,'InjectedConstructionRefusal'):apply(original,out,selected(),lambda:None,vertex_constructor=refused_vertex,cavity_constructor=failed)
            self.assertFalse((out/'CONDITIONED.npz').exists());self.assertFalse((out/'CONDITIONING.json').exists())
        self.assertEqual(before,(original['points'].tobytes(),identity(original['cells']),repr(original['owners'])))
    def test_reader_rejects_changed_conditioned_file(self):
        original=fixture()
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);save_native(folder,original);joined,_=apply(original,folder/'conditioning',selected(),lambda:None)
            with (folder/'conditioning/CONDITIONED.npz').open('ab') as f:f.write(b'corrupt')
            with self.assertRaisesRegex(ValueError,'ChangedConditioningArtifact'):verify(folder,dict(prefix_points=original['prefix_points']),saved_arrays(joined),joined,selected(),lambda:None)
    def test_reader_reconciles_provenance_mirror_counts(self):
        original=fixture()
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);save_native(folder,original);joined,_=apply(original,folder/'conditioning',selected(),lambda:None)
            joined['provenance']['conditioning']=dict(joined['provenance']['conditioning'],native_calls=99)
            with self.assertRaisesRegex(ValueError,'ChangedConditioningProvenanceMirrors'):verify(folder,dict(prefix_points=original['prefix_points']),saved_arrays(joined),joined,selected(),lambda:None)
    def test_partial_component_success_is_rolled_back_on_next_refusal(self):
        original=fixture();x=original['points'];nv=len(x);source=x[original['bulk_tets'][0]]+[10,0,0]
        original['points']=np.vstack((x,source));extra=build(np.asarray(orient(original['points'],[[nv,nv+1,nv+2,nv+3]]),dtype='<i8'),8)[0]
        original['cells']=original['cells']+[extra];original['bulk_tets']=np.vstack((original['bulk_tets'],extra['vertices']))
        original['farfield']=np.asarray(boundary(original['cells'])[1],dtype='<i8');original['owners'],_=topology.ownership(original['cells'],list(map(tuple,original['farfield'])),[])
        before=(original['points'].tobytes(),identity(original['cells']),repr(original['owners']))
        def failed(*a,**kw):raise ValueError('AfterPrivateCommitRefusal')
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'conditioning'
            with self.assertRaisesRegex(ValueError,'AfterPrivateCommitRefusal'):apply(original,out,selected(),lambda:None,cavity_constructor=failed)
            stopped=json.loads((out/'STOP.json').read_text());self.assertEqual(stopped['steps'][0]['commits'],1)
            self.assertFalse((out/'CONDITIONED.npz').exists())
        self.assertEqual(before,(original['points'].tobytes(),identity(original['cells']),repr(original['owners'])))
    def test_native_reservation_uses_outer_lease_and_shared_credit(self):
        from direct_connection import NativeBudget
        import constrained_local
        original=fixture();now=[0.];budget=NativeBudget(10,clock=lambda:now[0]);seen=[]
        def lease():pass
        lease.remaining_seconds=lambda:2.
        def native(*args):seen.append(args[-1].remaining_seconds());now[0]+=1;return {}
        def cavity(s,folder,*,minimum,policy,check,proposer):
            inner=lambda:None;inner.remaining_seconds=lambda:100.
            proposer(s,[],[],6,'existing',folder,policy,inner)
            raise ValueError('ReservationControlStop')
        with tempfile.TemporaryDirectory() as tmp,patch.object(constrained_local,'native_candidate',native):
            with self.assertRaisesRegex(ValueError,'ReservationControlStop'):apply(original,Path(tmp)/'conditioning',selected(),lease,native_budget=budget,vertex_constructor=refused_vertex,cavity_constructor=cavity)
        self.assertEqual(seen,[2.]);self.assertEqual(budget.used,1.)
    def test_apex_custody_uses_conditioned_coordinates_and_arbitrary_ids(self):
        from finite_research import direct_repair_custody
        original=fixture();prefix=set(original['prefix_ids']);prefix_state=dict(cells=[c for c in original['cells'] if c['id'] in prefix])
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);save_native(folder,original);joined,_=apply(original,folder/'conditioning',selected(),lambda:None)
            core=[c for c in joined['cells'] if c['id'] not in prefix]
            points,receipt=direct_repair_custody.apply(folder,joined,prefix_state,core,selected(),lambda:None)
            joined['points']=points;joined['provenance'].update(repair=receipt,construction_invocations=1)
            arrays=saved_arrays(joined);arrays['prisms']=np.asarray([c['vertices'] for c in prefix_state['cells']],dtype='<i8')
            arrays['prism_ids']=np.asarray(original['prefix_ids'],dtype='<i8')
            result=direct_repair_custody.verify(folder,dict(prefix_points=original['prefix_points']),arrays,joined,selected(),lambda:None)
            self.assertTrue(result['passed']);self.assertEqual(result['optimizer_replays'],0)
    def test_historical_unconditioned_apex_custody_stays_native_exact(self):
        from finite_research import direct_repair_custody
        original=fixture();prefix=set(original['prefix_ids']);prefix_state=dict(cells=[c for c in original['cells'] if c['id'] in prefix])
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);save_native(folder,original);joined,_=apply(original,folder/'conditioning',selected(),lambda:None)
            joined['provenance'].pop('conditioning');np.savez(folder/'native/generated/NETGEN.npz',points=joined['points'][:5],tets=joined['bulk_tets'])
            core=[c for c in joined['cells'] if c['id'] not in prefix]
            points,r=direct_repair_custody.apply(folder,joined,prefix_state,core,selected(),lambda:None)
            joined['points']=points;joined['provenance'].update(repair=r,construction_invocations=1)
            arrays=saved_arrays(joined);arrays['prisms']=np.asarray([c['vertices'] for c in prefix_state['cells']],dtype='<i8')
            result=direct_repair_custody.verify(folder,dict(prefix_points=original['prefix_points']),arrays,joined,selected(),lambda:None)
            self.assertTrue(result['passed'])
    def test_nonzero_atomic_apex_move_replays_after_conditioning(self):
        from finite_research import direct_repair_custody,direct_shared_apex as apex
        from finite_research.direct_fill_contract import binding
        from sdf_cfd_mesher_v4.common.authority import array_hash
        original=fixture();prefix=set(original['prefix_ids']);prefix_state=dict(cells=[c for c in original['cells'] if c['id'] in prefix])
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);save_native(folder,original)
            joined,_=apply(original,folder/'conditioning',selected(),lambda:None,vertex_constructor=refused_vertex,cavity_constructor=inserted_cavity)
            core=[c for c in joined['cells'] if c['id'] not in prefix];arrays=saved_arrays(joined)
            arrays['prisms']=np.asarray([c['vertices'] for c in prefix_state['cells']],dtype='<i8')
            # Exercise the unchanged atomic Star predicate/commit and receipt
            # replay directly. This is a schema control, not a solver campaign.
            context=apex.Context(arrays,joined,selected(),lambda:None);v=len(original['points']);star=context.star(v)
            initial=star.search(np.zeros(3));neighbours=sorted({int(w) for row in joined['bulk_tets'] if v in row for w in row if w!=v})
            target=joined['points'][neighbours].mean(0);proposal=star.final((target-joined['points'][v])/star.radius)
            self.assertTrue(proposal['passed']);self.assertGreater(proposal['batch']['summary']['minimum_weight'],float(initial['metrics'][:,0].min()))
            record=star.commit(proposal)
            record.update(status='COMMITTED',trials=[dict(iterations=1),dict(iterations=1)],evaluations=star.evaluations)
            result=dict(method=apex.METHOD,acceptance_contract=binding(selected()),repair_invocations=1,parent_native_sha256=file_hash(folder/'native/generated/NETGEN.npz'),
                conditioning_receipt_sha256=joined['provenance']['conditioning']['receipt_sha256'],records=[record],selected_vertices=[v],changed_vertices=[v],changed_count=1,
                union_affected_tets=len(context.affected_union),original_coordinates_sha256=array_hash(joined['points']),coordinates_sha256=array_hash(context.points))
            target=folder/'SHARED_APEX_REPAIR.json';target.write_text(json.dumps(result))
            receipt=dict(method_identity=identity(apex.METHOD),receipt_sha256=file_hash(target),repair_invocations=1,acceptance_contract=binding(selected()),
                original_coordinate_identity=array_hash(joined['points']),coordinate_identity=array_hash(context.points),changed_vertices=[v],changed_count=1)
            joined['points']=context.points;joined['provenance'].update(repair=receipt,construction_invocations=1);arrays['points']=context.points
            verified=direct_repair_custody.verify(folder,dict(prefix_points=original['prefix_points']),arrays,joined,selected(),lambda:None)
            self.assertTrue(verified['passed']);self.assertEqual(verified['changed_count'],1)

if __name__=='__main__':unittest.main()
