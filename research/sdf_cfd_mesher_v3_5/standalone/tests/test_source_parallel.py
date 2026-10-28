"""Actual local workers and existing source/handoff/retained evidence gates."""
from pathlib import Path
from contextlib import ExitStack
import copy,json,sys,tempfile,time,unittest
from unittest.mock import patch
import numpy as np
S=Path(__file__).resolve().parents[1];sys.path.insert(0,str(S))
import geometry,standalone_tool as cli,collision_policy as collision
import source_parallel as parallel
from source_handoff import live_handoff,consume
from source_evidence import validate_execution,retain,validate
from sdf_cfd_mesher_v4.common.authority import identity

class ParallelSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from planner import Request
        cls.temp=tempfile.TemporaryDirectory();cls.case=Path(cls.temp.name)/'CASE.json'
        req=Request(2e-5,1.3,5)
        geometry.write_case(S/'fixtures/v4_case_template.json',S/'fixtures/sphere.rhai',cls.case,req)
        token=cli._surface_assurance.set('numerical')
        try:cls.host,cls.binding=cli.geometry(cls.case)
        finally:cli._surface_assurance.reset(token)
        cls.source=cls.binding.authority;cls.h=2e-5
        # Source-only population, not a surface/mesh acceptance fixture.
        direction=np.tile(np.eye(3),(74,1))[:220]
        cls.x=np.asarray([direction*(.001+h) for h in np.r_[0,np.cumsum(req.nominal)]])
    @classmethod
    def tearDownClass(cls):cls.host.close();cls.temp.cleanup()
    def handoff(self,workers=4):return live_handoff(self.source,self.h,'production',source_workers=workers,deadline=time.perf_counter()+120)
    def pool(self,height=None):return parallel.Pool(self.source,self.h if height is None else height,4,time.perf_counter()+120)
    def test_01_actual_proofs_consumer_single_use_and_retained_record(self):
        expected=collision._segment_population(self.source,self.h,self.x,True)[1]
        from types import SimpleNamespace
        from source_evidence import validate_completion,validate_runtime
        import v35_shell
        class Initial:
            def verify_wall(self):return True # Explicit synthetic first-front premise only.
        initial=Initial();initial.source=self.source;initial.h1=self.h;initial.x=self.x[0];initial.high=self.x[1];initial.assigned=dict(admitted=True);initial.tri=np.empty((0,3),dtype=np.int64)
        trajectory=SimpleNamespace(authority=self.source,check=lambda:None,positions=lambda:self.x)
        class StopBeforeMesh(Exception):pass
        with self.handoff() as owner,tempfile.TemporaryDirectory() as folder:
            root=Path(folder);shell=root/'shell'
            owner.qualify(self.source,self.h,self.x);self.assertTrue(owner.accept(self.x))
            with ExitStack() as scope,patch('live_spline_first_layer.SourceSheetFirstLayerProvider',Initial),patch('accepted_trajectory_contract._planned_quality',side_effect=StopBeforeMesh),self.assertRaises(StopBeforeMesh):
                v35_shell.prepare(initial,trajectory,shell,source_handoff=owner,verification_mode='production',_source_evidence_scope=scope)
            actual=json.loads((shell/'SOURCE_SEGMENTS.json').read_text())['layers'];self.assertEqual(actual,expected)
            evaluation=json.loads((shell/'SOURCE_EVALUATION.json').read_text());record=evaluation['execution']
            validate_execution(record,self.x,self.source.geometry_identity,self.h,identity(actual))
            self.assertEqual(record['effective_workers'],4);self.assertEqual(record['contained_active_processes'],0)
            self.assertIsNone(consume(owner,self.source,self.h,self.x,'production'))
            (root/'CASE.json').write_bytes(self.case.read_bytes());(root/'native').mkdir();np.savez(root/'native/SEALED_PLAN.npz',positions=self.x)
            from sdf_cfd_mesher_v4.common.authority import array_hash
            parent=dict(actual_positions_sha256=array_hash(self.x),source_identity=self.source.geometry_identity,source_segments_identity=identity(actual))
            validate_completion(root,json.loads(self.case.read_text()),parent);validate_runtime(root,record['runtime_identity'])
            changed=copy.deepcopy(evaluation);changed['execution']['runtime_identity']='0'*64
            for row in changed['execution']['workers']:row['runtime']='0'*64
            # Structurally consistent forgery still fails the held producer anchor.
            (shell/'SOURCE_EVALUATION.json').write_text(json.dumps(changed))
            with self.assertRaisesRegex(ValueError,'ProducerRuntimeMismatch'):validate_runtime(root,record['runtime_identity'])
            (shell/'SOURCE_EVALUATION.json').write_text(json.dumps(evaluation))
            (root/'CASE.json').write_text('{}')
            with self.assertRaises(ValueError):validate_completion(root,json.loads(self.case.read_text()),parent)
            self.assertFalse((shell/'PREPARED_SHELL.json').exists())
            for key,value in [('state','held'),('proof_identity','0'*64),('runtime_identity','0'*64),('positions_sha256','0'*64),('effective_workers',2)]:
                changed=copy.deepcopy(record);changed[key]=value
                with self.subTest(key=key),self.assertRaises(ValueError):validate_execution(changed,self.x,self.source.geometry_identity,self.h,identity(actual))
    def test_02_known_refusal_order_and_healthy_new_population(self):
        bad=self.x.copy();bad[1,80:160]=bad[0,80:160]+2*(bad[1,80:160]-bad[0,80:160])
        with self.assertRaises(collision.SourceSegmentRefusal) as original:collision._segment_population(self.source,self.h,bad,True)
        with self.handoff() as owner:
            with self.assertRaises(collision.SourceSegmentRefusal) as actual:owner.qualify(self.source,self.h,bad)
            self.assertEqual(actual.exception.detail,original.exception.detail)
            self.assertFalse(owner.accept(bad))
            owner.qualify(self.source,self.h,self.x);self.assertTrue(owner.accept(self.x))
            self.assertIsNotNone(consume(owner,self.source,self.h,self.x,'production'))
            record=owner.execution_record();self.assertTrue(record['refusal_diagnostics'])
    def test_03_corrupt_coverage_context_and_crash_fail_without_serial_retry(self):
        original=parallel.Pool._receive
        for change in ('missing','duplicate','context','crash'):
            with self.subTest(change=change):
                def receive(pool,state):
                    if change=='crash' and state=='layer':pool.processes[0].kill()
                    rows=original(pool,state)
                    if state=='layer':
                        if change=='missing':rows.pop()
                        if change=='duplicate':rows[1]=copy.deepcopy(rows[0])
                    if change=='context' and state=='ready':rows[0]['context']='changed'
                    return rows
                with self.handoff() as owner,patch.object(parallel.Pool,'_receive',receive),patch.object(collision,'_segment_population',side_effect=AssertionError('UnsafeSerialRetry')):
                    with self.assertRaises((ValueError,ChildProcessError,IndexError)):owner.qualify(self.source,self.h,self.x)
                    self.assertFalse(owner.accept(self.x));self.assertIsNone(consume(owner,self.source,self.h,self.x,'production'))
    def test_04_capability_owner_expiry_and_duplicate_use(self):
        pool=self.pool()
        try:
            cap=pool.qualify(self.x)
            with self.assertRaisesRegex(ValueError,'Capability'):pool.take(dict(passed=True),self.source,self.h,self.x)
            self.assertTrue(pool.closed)
        finally:
            if not pool.closed:pool.abort()
    def test_05_changed_consumer_closes_owned_workers_and_rejects_evidence(self):
        for change in ('coordinates','numpy'):
            with self.subTest(change=change),self.handoff() as owner:
                owner.qualify(self.source,self.h,self.x);self.assertTrue(owner.accept(self.x))
                if change=='coordinates':
                    x=self.x.copy();x[-1,0,0]=np.nextafter(x[-1,0,0],np.inf)
                    self.assertIsNone(consume(owner,self.source,self.h,x,'production'))
                else:
                    with np.errstate(over='ignore'):self.assertIsNone(consume(owner,self.source,self.h,self.x,'production'))
                self.assertEqual(owner.execution_record()['effective_workers'],1)
    def test_06_small_audit_custom_callback_and_depth_select_before_start(self):
        self.assertEqual(parallel.eligible(self.source,self.x[:,:3],'production',4),'small_population')
        self.assertEqual(parallel.eligible(self.source,self.x,'audit',4),'audit_serial_recompute')
        old=np.geterrcall()
        try:
            np.seterrcall(lambda *args:None)
            self.assertEqual(parallel.eligible(self.source,self.x,'production',4),'unsupported_source_context')
        finally:np.seterrcall(old)
        scope=self.source.queries._scope;old=scope._budgets
        try:
            object.__setattr__(scope,'_budgets',(7,*old[1:]))
            self.assertEqual(parallel.eligible(self.source,self.x,'production',4),'proof_validator_depth_limit')
        finally:object.__setattr__(scope,'_budgets',old)
    def test_07_cancel_deadline_retires_job(self):
        pool=self.pool()
        try:
            pool.deadline=time.perf_counter()-1
            with self.assertRaises(TimeoutError):pool.qualify(self.x)
        finally:
            if not pool.closed:pool.abort()
        self.assertTrue(all(not p.is_alive() for p in pool.processes))
    def test_08_upward_rounded_height_summary_keeps_original_reserve(self):
        x=self.x[:,:4];ranges=[(0,2),(2,4)];parts=[]
        for lo,hi in ranges:
            proof=self.source.qualify_segments(x[0,lo:hi],x[1,lo:hi],first_height_m=self.h)
            proof['maximum_first_height_error_m']=1e-12
            parts.append(dict(range=[lo,hi],proof=proof))
        import hashlib
        expression=hashlib.sha256(repr(self.source.queries._scope._batch_inputs()[0]).encode()).hexdigest()
        result=parallel.validate_layer(parts,x,1,self.h,self.source.geometry_identity,expression,ranges)
        self.assertEqual(result['maximum_first_height_error_m'],1e-12)
    def test_09_changed_operand_during_blocking_closure_is_fatal(self):
        x=self.x.copy();original=parallel.Pool.finish
        def finish(pool,positions,proofs):
            record=original(pool,positions,proofs)
            x[-1,0,0]=np.nextafter(x[-1,0,0],np.inf)
            return record
        with self.handoff() as owner:
            owner.qualify(self.source,self.h,x);self.assertTrue(owner.accept(x))
            with patch.object(parallel.Pool,'finish',finish),self.assertRaisesRegex(ValueError,'DuringWorkerClosure'):
                consume(owner,self.source,self.h,x,'production')

if __name__=='__main__':unittest.main()
