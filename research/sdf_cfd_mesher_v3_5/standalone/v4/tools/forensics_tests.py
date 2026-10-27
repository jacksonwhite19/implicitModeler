"""Run every retained test without changing historical test assertions."""
from pathlib import Path
import sys,os,io,json,time,unittest
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.dont_write_bytecode=True
t=time.perf_counter();stream=io.StringIO();suite=unittest.defaultTestLoader.discover(str(ROOT/'tests'),pattern='test_*.py')
r=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
out=dict(tests=r.testsRun,failures=len(r.failures),errors=len(r.errors),skipped=len(r.skipped),passed=r.wasSuccessful(),seconds=time.perf_counter()-t,log=stream.getvalue())
(ROOT/'results/nominal_forecast_forensics/TESTS.json').write_text(json.dumps(out,indent=2)+'\n');print(out['log']);raise SystemExit(0 if r.wasSuccessful() else 1)
