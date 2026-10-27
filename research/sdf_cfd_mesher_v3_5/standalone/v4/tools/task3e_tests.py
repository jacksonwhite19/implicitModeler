"""Retain prior test contracts; separate existing outside-scope assertions."""
from pathlib import Path
import unittest,sys,io,json,time,psutil
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')];sys.dont_write_bytecode=True
started=time.perf_counter();stream=io.StringIO()
suite=unittest.defaultTestLoader.discover(str(ROOT/'tests'))
result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
failed=[t.id() for t,_ in result.failures];errors=[t.id() for t,_ in result.errors]
expected=set(json.loads((ROOT/'results/task3d/TESTS.json').read_text())['failed_tests'])
r=dict(tests=result.testsRun,failures=len(failed),errors=len(errors),skipped=len(result.skipped),
    failed_tests=failed,error_tests=errors,new_failures=sorted(set(failed)-expected),
    historical_assertions_unchanged=set(failed)==expected,seconds=time.perf_counter()-started,
    peak_rss=psutil.Process().memory_info().peak_wset,log=stream.getvalue())
(ROOT/'results/task3e/TESTS.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps({k:v for k,v in r.items() if k!='log'},indent=2))
sys.exit(1 if r['new_failures'] or errors else 0)
