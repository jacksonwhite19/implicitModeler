from pathlib import Path
import sys,io,json,time,unittest
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.dont_write_bytecode=True
t=time.perf_counter();stream=io.StringIO();suite=unittest.defaultTestLoader.discover(str(ROOT/'tests'),pattern='test_*.py');result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
text=stream.getvalue();print(text)
j=dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),passed=result.wasSuccessful(),seconds=time.perf_counter()-t,log=text)
(ROOT/'results/task1/TESTS.json').write_text(json.dumps(j,indent=2)+'\n');raise SystemExit(0 if result.wasSuccessful() else 1)
