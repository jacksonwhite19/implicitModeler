"""Retain all prior contracts; persist exact test failures without exemptions."""
from pathlib import Path
import unittest,sys,io,json,time
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')];sys.dont_write_bytecode=True
started=time.perf_counter();stream=io.StringIO();suite=unittest.defaultTestLoader.discover(str(ROOT/'tests'))
result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
r=dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),passed=result.wasSuccessful(),
    failed_tests=[t.id() for t,_ in result.failures],error_tests=[t.id() for t,_ in result.errors],seconds=time.perf_counter()-started,log=stream.getvalue())
(ROOT/'results/task3d/TESTS.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps({k:v for k,v in r.items() if k!='log'},indent=2));sys.exit(0 if result.wasSuccessful() else 1)
