"""Retain old test contracts; write this run only into Task-4C."""
from task4_tests import *
OUT=ROOT/'results/task4c'
if __name__=='__main__':
    started=time.perf_counter()
    with (OUT/'TESTS.txt').open('w',encoding='utf-8') as stream:
        result=unittest.TextTestRunner(stream=stream,verbosity=2).run(unittest.defaultTestLoader.discover(str(ROOT/'tests')))
    inherited=[];unexpected=[]
    for test,trace in result.failures+result.errors:
        (inherited if any(test.id().startswith(name) for name in HISTORICAL) else unexpected).append(dict(test=test.id(),traceback=trace))
    summary=dict(test_count=result.testsRun,passed=result.testsRun-len(result.failures)-len(result.errors)-len(result.skipped),
        failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),inherited_state_failures=inherited,
        unexpected_failures=unexpected,all_scientific_tests_passed=not unexpected,seconds=time.perf_counter()-started)
    (OUT/'TEST_SUMMARY.json').write_text(json.dumps(summary,indent=2,sort_keys=True)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('inherited_state_failures','unexpected_failures')}))
    sys.exit(not result.wasSuccessful())
