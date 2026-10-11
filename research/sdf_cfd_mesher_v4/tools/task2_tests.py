"""All retained setup/Task-1 tests plus focused Task-2 tests; new evidence only."""
from pathlib import Path
import sys,io,json,time,unittest,os
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.dont_write_bytecode=True
start=time.perf_counter();stream=io.StringIO()
suite=unittest.defaultTestLoader.discover(str(ROOT/'tests'),pattern='test_*.py')
result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
record=dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),passed=result.wasSuccessful(),seconds=time.perf_counter()-start,log=stream.getvalue())
(ROOT/'results/task2/TESTS.json').write_text(json.dumps(record,indent=2)+'\n');print(record['log']);raise SystemExit(0 if result.wasSuccessful() else 1)
