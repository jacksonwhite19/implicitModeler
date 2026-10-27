import sys,unittest,uuid,concurrent.futures,time,subprocess
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from native import operation
from build_native import ROOT,linux
class Operations(unittest.TestCase):
    def test_linux_limit_and_executing_utility_cancellation(self):
        folder=ROOT/'results'/('owned_operation_'+uuid.uuid4().hex[:8]);folder.mkdir(parents=True)
        result=operation('/usr/bin/python3',['-c','import resource;print(resource.getrlimit(resource.RLIMIT_AS))'],folder,10,name='limit')
        self.assertEqual(result['linux_address_space_limit_bytes'],16*2**30)
        self.assertIn(str(16*2**30),(folder/'limit.stdout').read_text())
        with concurrent.futures.ThreadPoolExecutor() as pool:
            future=pool.submit(operation,'/bin/sleep',['30'],folder,40,name='inert_export')
            scope=folder/'inert_export.linux_scope.json';deadline=time.monotonic()+10
            while not scope.exists() and time.monotonic()<deadline:time.sleep(.05)
            self.assertTrue(scope.exists());time.sleep(.3)
            cancel=subprocess.run(['wsl','-d','Ubuntu-22.04','--','python3',linux(ROOT/'cancel_native.py'),linux(scope),linux(folder),'unused'],timeout=10)
            self.assertEqual(cancel.returncode,0)
            with self.assertRaisesRegex(RuntimeError,'NativeConsumerFailed'):future.result(timeout=10)
