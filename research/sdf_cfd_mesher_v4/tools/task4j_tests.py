from task4j_common import *
import unittest,io
def run():
    sys.path.insert(0,str(ROOT/'tests'))
    names=['test_task4j_certification','test_efficient_certificate','test_production_certificate_adapter','test_certificate_replay_equivalence','test_incremental_certificate']
    suite=unittest.defaultTestLoader.loadTestsFromNames(names);stream=io.StringIO();s=time.perf_counter()
    result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    (OUT/'TESTS.txt').write_text(stream.getvalue(),encoding='utf-8')
    save('TESTS.json',dict(modules=names,run=result.testsRun,failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),seconds=time.perf_counter()-s,successful=result.wasSuccessful(),full_development_suite_run=False,historical_repository_state_assertions_run=False))
    print(stream.getvalue());assert result.wasSuccessful()
if __name__=='__main__':run()
