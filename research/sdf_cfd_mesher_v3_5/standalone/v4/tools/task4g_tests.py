"""One focused regression run; no full research-study execution."""
from task4g_common import *
import unittest
def run():
    patterns=('test_task4g_evaluation.py','test_task1.py','test_task2.py','test_task3.py','test_private_rolling.py',
        'test_rolling.py','test_persistent_planning.py','test_dynamic_risk_atlas.py','test_production_certificate_adapter.py','test_task4e_governance.py')
    suite=unittest.TestSuite()
    for pattern in patterns:suite.addTests(unittest.defaultTestLoader.discover(str(ROOT/'tests'),pattern=pattern))
    with (OUT/'REGRESSION_TESTS.txt').open('w',newline='\n') as f:r=unittest.TextTestRunner(stream=f,verbosity=2).run(suite)
    save('TEST_SUMMARY.json',dict(tests=r.testsRun,failures=[(str(t),e) for t,e in r.failures],errors=[(str(t),e) for t,e in r.errors],skipped=[(str(t),e) for t,e in r.skipped],patterns=patterns))
    print('TESTS',r.testsRun,len(r.failures),len(r.errors),flush=True)
if __name__=='__main__':run()
