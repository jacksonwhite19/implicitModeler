"""Run all retained tests; report inherited assertions without changing them."""
from pathlib import Path
import sys,unittest,json,time
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tests')];sys.dont_write_bytecode=True
OUT=ROOT/'results/task4'
HISTORICAL=(
    'test_forensics.ForensicTests.test_current_task_preservation_and_sealed_task2',
    'test_setup.SetupTests.test_branch_and_unrelated_git_status_preserved',
    'test_setup.SetupTests.test_fork_hashes_and_bounded_preservation',
    'test_task1.FoundationTests.test_18_v3_preservation',
    'test_task2.DetailedTests.test_original_sources_unchanged',
    'test_task3.RelaxationTests.test_current_task_unrelated_state_preserved',
    'test_task3_repair.CertificateRepairTests.test_followup_preserves_unrelated_state',
)

if __name__=='__main__':
    started=time.perf_counter()
    with (OUT/'TESTS.txt').open('w',encoding='utf-8') as stream:
        result=unittest.TextTestRunner(stream=stream,verbosity=2).run(unittest.defaultTestLoader.discover(str(ROOT/'tests')))
    inherited=[];unexpected=[]
    for test,trace in result.failures+result.errors:
        row=dict(test=test.id(),traceback=trace)
        (inherited if any(test.id().startswith(x) for x in HISTORICAL) else unexpected).append(row)
    summary=dict(test_count=result.testsRun,passed=result.testsRun-len(result.failures)-len(result.errors)-len(result.skipped),
        failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),inherited_state_failures=inherited,
        unexpected_failures=unexpected,all_scientific_and_task4_tests_passed=not unexpected,seconds=time.perf_counter()-started)
    (OUT/'TEST_SUMMARY.json').write_text(json.dumps(summary,indent=2,sort_keys=True)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('inherited_state_failures','unexpected_failures')}))
    # The retained suite still returns failure when historical assertions fail.
    sys.exit(not result.wasSuccessful())
