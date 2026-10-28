"""Scope JSON data escaping and fail-before-exec; no mesher/library loading."""
from pathlib import Path
from types import SimpleNamespace
import json,os,shlex,subprocess,sys,tempfile,unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import geometry,native,runtime_environment
from build_native import linux

@unittest.skipUnless(os.name=='nt','Transport writes a Windows workspace and checks Bash serialization')
class NativeScopeSerialization(unittest.TestCase):
    def test_exact_argv_data_and_scope_failure_precedes_exec(self):
        (ROOT/'results').mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='scope json ',dir=ROOT/'results') as tmp:
            case=Path(tmp);program='never-executed';args=['first\nsecond',r'\n',r'\literal\path','%d%s%n',"quote'\";$(false)",'SDF \u2192 prism']
            with patch.object(runtime_environment,'launch_prefix',return_value=''),patch.object(native,'subprocess',SimpleNamespace(run=lambda *a,**k:SimpleNamespace(returncode=0))):
                native.operation(program,args,case,seconds=1,name='serializer')
            script=(case/'serializer.launch.sh').read_text();prefix=script.split('exec prlimit --as=',1)[0]
            marker=case/'executed';tail='printf executed > '+shlex.quote(linux(marker))+'\n'
            command=['wsl','-d','Ubuntu-22.04','--','bash','--noprofile','--norc','-s']
            done=subprocess.run(command,input=(prefix+tail).encode(),capture_output=True,timeout=15);self.assertEqual(done.returncode,0,done.stderr.decode())
            self.assertTrue((case/'serializer.linux_scope.json').exists(),(done.stdout+done.stderr).decode()+prefix)
            value=json.loads((case/'serializer.linux_scope.json').read_text());self.assertEqual(value['argv'],[program,*args]);self.assertIsInstance(value['pid'],int);self.assertGreater(value['pid'],0);self.assertIsInstance(value['start_ticks'],int);self.assertGreater(value['start_ticks'],0);self.assertTrue(marker.exists())
            marker.unlink();bad=prefix.replace(shlex.quote(linux(case/'serializer.linux_scope.json')),shlex.quote(linux(case/'missing/scope.json')))
            refused=subprocess.run(command,input=(bad+tail).encode(),capture_output=True,timeout=15);self.assertNotEqual(refused.returncode,0);self.assertFalse(marker.exists())

if __name__=='__main__':unittest.main()
