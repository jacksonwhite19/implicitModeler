"""Small real-kernel append tests; fixtures are not production authorities."""
import sys
from pathlib import Path
V4 = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(V4/'tools'), str(V4/'src')]
import copy, tempfile, unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import generalize_local_remesh as g
import geometry_family_authority as geom
from sdf_cfd_mesher_v4.common.authority import identity, file_hash
from rolling_fast_engine import from_initial
from rolling_fast_state import mesh_identity, export_mesh, inherit


def fixture(folder):
    """Existing analytic fixture producer; no alternate geometry mathematics."""
    from production_first_layer_run import ProductionFirstLayerProvider
    from native_mixed_face_certificate import certify
    from first_layer_metric_state import apply
    from native_mixed_face_certificate import SOURCE
    spec = next(x for x in geom.definitions() if x['name'] == 'capsule_control')
    # Compact validator fixture, not the production control or a sizing policy.
    # The coarse unscaled fixture is also rejected by the unchanged determinant
    # gate; its large edges cannot support the SF2 thin height.
    spec = dict(geom.scale_variant(spec, .02), level=0)
    x, tri = geom.build(spec); source = geom.Source(spec)
    h = float.fromhex('0x1.5d3b8a168f007p-13')
    high = x+h*source.evaluate(x).normal
    points, cells, outer, checked, _ = g.validation(x, tri, [high], np.ones(len(x), int), False)
    checked = apply(checked, points, cells, outer)
    faces = [r[0][1] for _, r in sorted(g.t.ownership(cells, outer, [tuple(reversed(f)) for f in tri])[0].items())]
    checked['certificate'] = certify(points, faces, range(len(faces)))
    checked['volume'] = g.volume.audit(points, cells, outer, [tuple(f) for f in tri])
    checked['passed'] = (checked['quality']['passed'] and checked['certificate']['passed']
        and checked['volume']['positive_cells'] and checked['volume']['exact_volume_balance']
        and checked['volume']['interior_degree']['passed'])
    p = object.__new__(ProductionFirstLayerProvider)
    from historical_capsule_surface import CapsuleGeometry
    authority = CapsuleGeometry(spec['components'][0])
    # Fixture only: exercise initialization and predicates without forging a
    # production surface seal or asking the real loader to accept a fake one.
    p.binding = SimpleNamespace(authority=authority, geometry_identity=authority.geometry_identity,
        qualification=SimpleNamespace(verify=lambda a: a is authority))
    p.x=x; p.tri=tri; p.h1=h; p.high=high; p.source=source; p.spec=spec
    p.points=points; p.cells=cells; p.outer=outer; p.check=checked
    p.assigned=geom.sdf_record(spec, high, h); p.verify_wall=lambda:True
    p.meshes={}; p.authority=dict(schedule=identity([h,h,h,h]),
        quality=file_hash(Path(g.t.__file__)), certificate=file_hash(Path(g.s.cert.__file__)),
        certificate_filter=file_hash(SOURCE), sdf=source.source_identity)
    return p


class RollingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='v4-rolling-test-')
        self.folder = Path(self.temp.name)
        self.p = fixture(self.folder)
        self.assertTrue(self.p.check['passed'], self.p.check['quality']['failures'])
        self.run = from_initial(self.p, [self.p.h1]*4, self.folder)

    def tearDown(self): self.temp.cleanup()

    def test_real_multilayer_predicates_and_eviction(self):
        from incremental_layer_validation import key
        for k in (2,3,4):
            result = self.run.step()
            self.assertEqual(k, result['layer'])
            self.assertTrue(result['degree_passed']); self.assertTrue(result['exact_volume_balance'])
            self.assertIsNone(self.run.proofs); self.assertIsNone(self.run.face_keys)
            self.assertEqual({}, self.run.tx.current['certificate'])
            self.assertEqual({}, self.run.tx.current['quality'])
            self.assertEqual(len(self.p.tri), len(self.run.quality.cells))
            self.assertEqual({k}, {self.run.topology.cells[i]['layer'] for i in self.run.quality.cells})
            self.assertEqual(k*len(self.p.tri), len(self.run.topology.cells))
            # Existing full independent predicates vs retained newest cache.
            q = g.t.quality(self.run.points, list(self.run.topology.cells.values()), self.run.topology.owners, 1)
            self.assertTrue(q['passed'], q['failures'])
            for r in q['cells']:
                if r['id'] in self.run.quality.cells: self.assertEqual(r, self.run.quality.cells[r['id']])
            for r in q['exact']:
                if r['id'] in self.run.quality.exact: self.assertEqual(r, self.run.quality.exact[r['id']])
            from final_shell_incidence import check
            incidence = check(self.run.points, list(self.run.topology.cells.values()), self.run.outer)
            self.assertTrue(incidence['passed'])
            for r in incidence['cells']:
                if r['id'] in self.run.quality.prospective: self.assertEqual(r, self.run.quality.prospective[r['id']])
            fresh = __import__('native_mixed_face_certificate').certify(self.run.points,
                [v[0][1] for _,v in sorted(self.run.topology.owners.items())], range(len(self.run.topology.owners)))
            self.assertTrue(fresh['passed']); self.assertEqual(fresh['required_pairs'], self.run.live_acceptance.count)
        exported = export_mesh(self.run, self.folder/'MESH.npz')
        self.assertFalse(exported['restart_supported']); self.assertFalse(exported['published_scientific_authority'])
        self.assertFalse(any(self.folder.rglob('*.sqlite')))
        with self.assertRaisesRegex(ValueError,'HistoricalReceiptRenderer'): self.run.binding_state()
        with self.assertRaisesRegex(ValueError,'NoPendingIncrement'): self.run.step()

    def test_failure_leaves_current_and_caches(self):
        before = self.run.tx.current; generation = self.run.tx.generation
        quality = self.run.quality; volume = self.run.volume; accepted = self.run.live_acceptance
        with patch('native_mixed_face_certificate.certify', return_value={'passed':False}):
            with self.assertRaisesRegex(ValueError,'ContinuousGeometryFailure'): self.run.step()
        self.assertIs(before,self.run.tx.current); self.assertEqual(generation,self.run.tx.generation)
        self.assertIs(quality,self.run.quality); self.assertIs(volume,self.run.volume)
        self.assertIs(accepted,self.run.live_acceptance)

    def test_mutated_parent_rejected(self):
        self.run.tx.current['vertices']['layer:0:source:0'][0] += 1
        with self.assertRaisesRegex(ValueError,'ChangedRollingAcceptedParent'): self.run.step()

    def test_numeric_topology_and_cache_mutations_rejected(self):
        old = self.run.topology.owners.copy()
        face = next(k for k,v in old.items() if len(v)==2)
        self.run.topology.owners.pop(face)
        with self.assertRaisesRegex(ValueError,'ChangedRollingAcceptedParent'):
            export_mesh(self.run,self.folder/'wrong.npz')
        self.run.topology.owners=old
        i=next(iter(self.run.quality.cells));self.run.quality.cells[i]=dict(self.run.quality.cells[i],volume=-1)
        with self.assertRaisesRegex(ValueError,'ChangedRollingAcceptedParent'):self.run.step()

    def test_no_archived_reader_after_initialization(self):
        self.p.check={};self.p.meshes={}
        with patch('native_proof_segments.LoadedSegment.__init__',side_effect=AssertionError('old proof read')), \
                patch('composable_state.CommittedState.__init__',side_effect=AssertionError('old checkpoint read')), \
                patch('incremental_pair_storage.PairIncidence.__init__',side_effect=AssertionError('old pair index read')):
            self.assertEqual(2,self.run.step()['layer'])
            self.assertEqual(3,self.run.step()['layer'])

    def test_mesh_writer_keeps_one_export(self):
        from rolling_fast_engine import write_mesh
        first=write_mesh(self.run,self.folder)
        self.run.step();second=write_mesh(self.run,self.folder)
        self.assertNotEqual(first['coordinate_hash'],second['coordinate_hash'])
        self.assertEqual([self.folder/'MESH.npz'],list(self.folder.glob('MESH*.npz')))
        self.assertEqual(0,len(list(self.folder.rglob('*.sqlite'))))

    def test_initial_failure_is_not_a_live_credential(self):
        self.p.check['passed']=False
        with self.assertRaisesRegex(ValueError,'UnqualifiedRollingInitialGeometry'):
            from_initial(self.p,[self.p.h1]*2,self.folder)

    def test_public_dispatch_and_restart_rejection(self):
        import standalone_tool as cli
        with patch('rolling_fast_engine.execute',return_value='rolling') as route:
            self.assertEqual('rolling',cli._execute_guarded('case',self.folder,0,3,'fastprod'))
            route.assert_called_once_with('case',self.folder,3)
        for values in ((1,None,None,None),(0,'restart',None,None),
                       (0,None,'initial',None),(0,None,None,'transition')):
            with self.assertRaisesRegex(ValueError,'RollingFastProdRestartNotSupported'):
                cli._execute_guarded('case',self.folder,values[0],3,'fastprod',*values[1:])

    def test_worker_geometry_host_is_owned_by_rolling_session(self):
        import run_live_sf2_first_layer as worker
        host=SimpleNamespace(close=__import__('unittest.mock',fromlist=['Mock']).Mock())
        self.p.folder=self.folder;self.p.times={}
        self.p.generate_candidate=lambda:True
        self.p.phase=lambda name,action:action()
        (self.folder/'QUALITY.json').write_text('{}',encoding='utf-8')
        pre=dict(resources={'minimum_free_bytes':1},surface={'surface_identity':'fixture'},
                 request_identity='request',source={})
        with patch.object(worker,'preflight',return_value=pre),patch.object(worker,'OUT',self.folder), \
                patch.object(worker,'diagnostics',return_value={}), \
                patch('production_first_layer_run.ProductionFirstLayerProvider',return_value=self.p), \
                patch('standalone_tool.geometry',return_value=(host,self.p.binding)):
            worker.standalone_worker('candidate',True,commit_action=lambda *args:'accepted',
                geometry_session=(host,self.p.binding))
            host.close.assert_not_called()
            worker.standalone_worker('candidate',True,commit_action=lambda *args:'accepted')
            host.close.assert_called_once()

    def test_live_smooth_target_binding_and_host_lifetime(self):
        from standalone_tool import geometry, first_request
        from production_surface_authority import load_production_wall
        from standalone_case import load
        from live_spline_first_layer import existing_compatibility
        from accepted_trajectory_contract import source_directions
        from owned_runtime import SOURCE
        from rolling_fast_engine import RollingGeometry
        from sdf_cfd_mesher_v4.advancing_front.execution_profile import MemoryTransaction
        case=V4/'cases/standalone_smooth_development.json'
        surface=V4/'results/live_sf2_surface/replay_20260929/authority'
        x,faces,sealed=load_production_wall(surface)
        with np.load(surface/'REFERENCES.npz',allow_pickle=False) as z:refs=z['face_refs']
        chosen=np.concatenate([np.flatnonzero(refs==r)[:4] for r in sorted(set(refs))])
        vertices=np.unique(faces[chosen]); local=np.full(len(x),-1,int);local[vertices]=np.arange(len(vertices))
        x=x[vertices];faces=local[faces[chosen]];refs=refs[chosen]
        request=first_request(self.folder,surface,case)
        _,increments,_=load(case);host,binding=geometry(case)
        try:
            normals,incidents,later,_=source_directions(binding.authority,x,faces,refs,
                tuple(f'source:{i}' for i in range(len(x))),SOURCE/'bind_planner.py')
            compatible=existing_compatibility()
            direction=np.asarray([compatible(normals[sorted(s)])['displacement_per_height'] for s in incidents])
            first=x+increments[0]*direction
            p=SimpleNamespace(binding=binding,x=x,tri=faces,high=first,h1=increments[0],source=binding.authority,
                refs={'face_refs':refs},request=self.folder/'REQUEST.json',wall_authority=sealed)
            state={k:{} for k in ('wall','first_layer','cells','faces','columns','prism_front','versions','authority','events')}
            state.update(outer=[],vertices={f'layer:{k}:source:{i}':v.tolist()
                for k,front in enumerate((x,first)) for i,v in enumerate(front)})
            owner=RollingGeometry(p,increments,state);owner.session=MemoryTransaction(state)
            fronts,levels=owner.geometry(state,1)
            direct=owner.owner.state.positions(owner.owner.schedule.increments.T)
            self.assertEqual(direct[2].tobytes(),fronts[-1].tobytes())
            target=owner.validate_fast_target(state,fronts[-1],2)
            self.assertTrue(target['admitted']);self.assertEqual('path_length_m',target['quantity'])
            self.assertEqual(mesh_identity(state),target['premises']['parent'])
            self.assertEqual(mesh_identity(state),target['premises']['target']['parent'])
            self.assertEqual(set(range(1,5)),set(map(int,refs)))
            # A source owned by this session must fail after its host closes.
            host.close()
            with self.assertRaises(Exception):owner.check()
        finally:host.close()

    def test_unsupported_change_and_stale_generation(self):
        before = self.run.tx.current; after = copy.deepcopy(before)
        after['authority']['quality'] = 'altered'
        with self.assertRaisesRegex(ValueError,'PredicateAuthorityChanged'): inherit(before,after,[])
        after=copy.deepcopy(before); after['cells'].pop(next(iter(after['cells'])))
        with self.assertRaisesRegex(ValueError,'CommittedCellChanged'): inherit(before,after,[])
        with self.assertRaisesRegex(ValueError,'StaleTransactionGeneration'):
            self.run.tx.commit({},99,lambda _:True)

    def test_source_schedule_and_coverage_rejection(self):
        self.run.provider.spec['cavity']=True
        with self.assertRaisesRegex(ValueError,'ChangedRollingCaseInputs'): self.run.step()
        certificate=dict(self.p.check['certificate'],required_pairs=1)
        with self.assertRaisesRegex(ValueError,'UnqualifiedRollingCertificate'):
            from rolling_fast_state import verify_fresh_coverage
            verify_fresh_coverage(self.p.points,[],[],certificate)


if __name__=='__main__':unittest.main()
