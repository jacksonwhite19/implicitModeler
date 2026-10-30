"""Actual eight-process lifecycle/proof controls plus the resource boundary."""
from pathlib import Path
import copy,sys,time,unittest
from types import SimpleNamespace
from unittest.mock import patch
S=Path(__file__).resolve().parents[1];sys.path[:0]=[str(S),str(Path(__file__).parent)]
import geometry,psutil
import test_source_parallel as baseline
import source_parallel as parallel
import test_source_process_owner as ownership

class EightProcessOwnerTests(ownership.ProcessOwnerTests):
    WORKERS=8

class EightSourceTests(baseline.ParallelSourceTests):
    WORKERS=8
    def test_10_affinity_is_explicit_and_insufficient_or_unknown_fails_before_job(self):
        with patch.object(psutil,'Process',return_value=SimpleNamespace(cpu_affinity=lambda:list(range(4)))),patch('source_process_owner.Job') as job:
            with self.assertRaisesRegex(ValueError,'EightAvailableCPUs'):
                parallel.Pool(self.source,self.h,8,time.perf_counter()+20)
            job.assert_not_called()
        with patch.object(psutil,'Process',side_effect=psutil.AccessDenied()),patch('source_process_owner.Job') as job:
            with self.assertRaises(psutil.AccessDenied):parallel.Pool(self.source,self.h,8,time.perf_counter()+20)
            job.assert_not_called()
    def test_11_eight_record_requires_real_cardinality_and_affinity(self):
        from source_handoff import consume
        from source_evidence import validate_execution
        from sdf_cfd_mesher_v4.common.authority import identity
        with self.handoff() as owner:
            owner.qualify(self.source,self.h,self.x);self.assertTrue(owner.accept(self.x))
            actual=consume(owner,self.source,self.h,self.x,'production');record=owner.execution_record()
            self.assertEqual(record['parent_cpu_affinity'],sorted(psutil.Process().cpu_affinity()))
            validate_execution(record,self.x,self.source.geometry_identity,self.h,identity(actual))
            for change in ('missing_affinity','short_affinity','duplicate_affinity','missing_worker','duplicate_worker','effective_four','wrong_check_stage','active_worker'):
                changed=copy.deepcopy(record)
                if change=='missing_affinity':changed.pop('parent_cpu_affinity')
                if change=='short_affinity':changed['parent_cpu_affinity']=list(range(4))
                if change=='duplicate_affinity':changed['parent_cpu_affinity']=[0]*8
                if change=='missing_worker':changed['workers'].pop()
                if change=='duplicate_worker':changed['workers'][-1]=copy.deepcopy(changed['workers'][0])
                if change=='effective_four':changed['effective_workers']=4
                if change=='wrong_check_stage':changed['parent_cpu_affinity_check']='guessed'
                if change=='active_worker':changed['contained_active_processes']=1
                with self.subTest(change=change),self.assertRaises(ValueError):validate_execution(changed,self.x,self.source.geometry_identity,self.h,identity(actual))
    def test_12_small_and_audit_eight_selection_never_spawn(self):
        from source_handoff import live_handoff,consume
        with patch.object(parallel,'Pool',side_effect=AssertionError('IneligiblePoolStarted')):
            for mode,x,reason in [('production',self.x[:,:3],'small_population'),('audit',self.x,'audit_serial_recompute')]:
                with live_handoff(self.source,self.h,mode,source_workers=8,deadline=time.perf_counter()+120) as owner:
                    owner.qualify(self.source,self.h,x)
                    if mode=='production':
                        self.assertTrue(owner.accept(x));self.assertIsNotNone(consume(owner,self.source,self.h,x,mode))
                    else:
                        self.assertFalse(owner.accept(x));self.assertIsNone(consume(owner,self.source,self.h,x,mode))
                    record=owner.execution_record();self.assertEqual(record['requested_workers'],8);self.assertEqual(record['effective_workers'],1)
    def test_13_cli_checks_only_explicit_eight_production_before_launch(self):
        import mesh
        argv=['mesh.py','--case','unused.json','--geometry','unused.rhai','--metres-per-model-unit','0.001',
            '--prism-first-height','0.00002','--prism-growth','1.3','--prism-layers','5','--output','unused']
        for workers,mode in [(1,'production'),(4,'production'),(8,'audit')]:
            with patch.object(sys,'argv',argv+['--source-workers',str(workers),'--verification',mode]),patch.object(mesh,'supervise',return_value=17),patch.object(parallel,'_eight_worker_affinity',side_effect=AssertionError('ChangedOldPath')):
                self.assertEqual(mesh.main(),17)
        with patch.object(sys,'argv',argv+['--source-workers','8','--verification','production']),patch.object(mesh,'supervise') as launch,patch.object(parallel,'_eight_worker_affinity',side_effect=ValueError('EightAvailableCPUs')):
            with self.assertRaisesRegex(ValueError,'EightAvailableCPUs'):mesh.main()
            launch.assert_not_called()

if __name__=='__main__':unittest.main()
