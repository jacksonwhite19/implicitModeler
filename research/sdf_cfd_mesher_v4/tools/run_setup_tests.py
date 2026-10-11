"""Run only V4 setup tests, never copied scientific test suites."""
from pathlib import Path
import io, json, sys, time, unittest
ROOT=Path(__file__).resolve().parents[1]
sys.dont_write_bytecode=True
sys.path.insert(0,str(ROOT/'src'))
started=time.perf_counter()
stream=io.StringIO()
suite=unittest.defaultTestLoader.discover(str(ROOT/'tests'),pattern='test_setup.py')
result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
text=stream.getvalue();print(text)
record=dict(command='python -B research/sdf_cfd_mesher_v4/tools/run_setup_tests.py',tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),passed=result.wasSuccessful(),seconds=time.perf_counter()-started,scope='imports, source identity, reference paths, branch and bounded preservation only',log=text)
(ROOT/'authorities/SETUP_TEST_RESULTS.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
raise SystemExit(0 if result.wasSuccessful() else 1)
