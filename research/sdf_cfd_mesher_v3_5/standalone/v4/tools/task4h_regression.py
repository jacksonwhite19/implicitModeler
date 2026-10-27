from task4h_common import *
import unittest
def run():
    patterns=('test_task4h_exact_geometry.py','test_task4g_evaluation.py','test_forensics.py','test_task3_repair.py',
              'test_rolling_spots.py','test_persistent_planning.py','test_production_certificate_adapter.py','test_efficient_certificate.py','test_private_rolling.py','test_rolling.py')
    suite=unittest.TestSuite()
    for pattern in patterns:suite.addTests(unittest.defaultTestLoader.discover(str(ROOT/'tests'),pattern=pattern))
    with (OUT/'TESTS.txt').open('w',newline='\n') as f:r=unittest.TextTestRunner(stream=f,verbosity=2).run(suite)
    save('TEST_SUMMARY.json',dict(tests=r.testsRun,failures=[(str(t),e) for t,e in r.failures],errors=[(str(t),e) for t,e in r.errors],skipped=[(str(t),e) for t,e in r.skipped],patterns=patterns))
    print('TESTS',r.testsRun,len(r.failures),len(r.errors),flush=True)
if __name__=='__main__':run()
