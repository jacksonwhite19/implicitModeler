from pathlib import Path
import unittest,sys,uuid,json,subprocess
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from source_custody import held
from geometry import owned_runtime

class SourceCustody(unittest.TestCase):
    def test_source_write_is_denied_during_worker_scope(self):
        target=ROOT/'fixtures/custody_control.json'
        if not target.exists():target.write_text('{"control":"source_write_denial"}\n')
        folder=ROOT/'results'/('custody_control_'+uuid.uuid4().hex[:8]);folder.mkdir(parents=True)
        with held(folder,folder/'SOURCE_CUSTODY.json'):
            with self.assertRaises(PermissionError):target.write_text('must not replace source')
        result=json.loads((folder/'SOURCE_CUSTODY.json').read_text())
        self.assertTrue(result['passed']);self.assertFalse(result['changed'])

    def test_versioned_runtime_and_extensionless_binary_custody(self):
        from source_custody import selected
        from build_native import linux
        folder=ROOT/'results'/('runtime_custody_'+uuid.uuid4().hex[:8]);folder.mkdir(parents=True)
        paths=[ROOT/'v4/runtime/lib/gmsh/libgmsh.so.4.15',ROOT/'v4/runtime/bin/mmgs_metric_contract_runner']
        inventory=selected(folder)
        self.assertTrue(all(p in inventory for p in paths))
        with held(folder,folder/'SOURCE_CUSTODY.json'):
            for p in paths:
                with self.assertRaises(PermissionError):
                    with p.open('r+b'):pass
            command=['wsl','-d','Ubuntu-22.04','--','python3','-c','import ctypes;ctypes.CDLL('+repr(linux(paths[0]))+');print("pinned_library_loaded")']
            result=subprocess.run(command,capture_output=True,text=True,timeout=30)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('pinned_library_loaded',result.stdout)
        self.assertTrue(json.loads((folder/'SOURCE_CUSTODY.json').read_text())['passed'])

if __name__=='__main__':unittest.main()
