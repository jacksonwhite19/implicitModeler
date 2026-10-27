from pathlib import Path
import unittest,sys,subprocess
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from build_native import linux

class Independence(unittest.TestCase):
    def test_python_source_and_subprocess_denial(self):
        original='/mnt/c/Users/Jackson/Desktop/02_Projects/09b_Implicit_CAD_claude/src/lib.rs'
        code="import sitecustomize\nprint(sitecustomize.__file__)\ntry:\n open("+repr(original)+")\nexcept PermissionError as e:\n print(str(e))\nelse:\n raise RuntimeError('Original source was readable')"
        run=subprocess.run(['wsl','-d','Ubuntu-22.04','--','env','PYTHONPATH='+linux(ROOT/'safety'),'python3','-B','-c',code],capture_output=True,text=True,timeout=20)
        self.assertEqual(run.returncode,0,run.stderr);self.assertIn(linux(ROOT/'safety'),run.stdout);self.assertIn('OriginalSourceAccessForbidden',run.stdout)
        from mesh import source_independence_guard
        source_independence_guard()
        with self.assertRaisesRegex(PermissionError,'OriginalSourceAccessForbidden'):
            open(r'C:\Users\Jackson\Desktop\02_Projects\09b_Implicit_CAD_claude\src\lib.rs')
        with self.assertRaisesRegex(PermissionError,'OriginalSourceSubprocessForbidden'):
            subprocess.run(['wsl','-d','Ubuntu-22.04','--','cat',original])

if __name__=='__main__':unittest.main()
