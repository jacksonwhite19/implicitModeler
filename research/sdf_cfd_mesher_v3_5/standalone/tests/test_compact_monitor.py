"""Bounded Windows checkpoint/cleanup fixtures; no geometry or mesh campaign."""
from pathlib import Path
from types import SimpleNamespace
import json,os,subprocess,sys,tempfile,threading,time,unittest
from unittest.mock import patch
HERE=Path(__file__).resolve().parents[1];sys.path.insert(0,str(HERE))
import geometry
import compact_lifecycle as lifecycle
from compact_paths import checked


class CompactMonitor(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.addCleanup(self.tmp.cleanup)

    def test_exact_recursive_bytes_and_lexical_forms(self):
        (self.root/'nested').mkdir();(self.root/'nested/a').write_bytes(b'x'*19)
        (self.root/'b').write_bytes(b'x'*7)
        self.assertEqual(lifecycle.live_bytes(self.root),26)
        relative=Path(os.path.relpath(self.root))
        # Relative paths can contain '..'; the strict existing boundary refuses.
        if '..' in relative.parts:
            with self.assertRaisesRegex(ValueError,'CompactPathTraversal'):lifecycle.live_bytes(relative)
        self.assertEqual(lifecycle.live_bytes(Path(str(self.root).upper())),26)

    def test_real_checkpoint_create_replace_delete(self):
        stop=threading.Event();errors=[];writes=[]
        def writer():
            try:
                while not stop.is_set():
                    pending=self.root/'checkpoint.pending';pending.write_bytes(b'x'*64)
                    os.replace(pending,self.root/'checkpoint.json');writes.append(1)
                    (self.root/'checkpoint.json').unlink()
            except BaseException as error:errors.append(repr(error))
        thread=threading.Thread(target=writer);thread.start();samples=0
        try:
            end=time.monotonic()+2
            while time.monotonic()<end:
                self.assertGreaterEqual(lifecycle.live_bytes(self.root),0);samples+=1
        finally:stop.set();thread.join()
        self.assertFalse(errors);self.assertGreater(len(writes),100);self.assertGreater(samples,100)

    def test_real_directory_cleanup_overlap_does_not_block_removal(self):
        parent=self.root/'delivery-parent';parent.mkdir();scratch=parent/'scratch';scratch.mkdir()
        for i in range(8):
            d=scratch/str(i);d.mkdir()
            for j in range(12):(d/str(j)).write_bytes(b'x'*32)
        start=threading.Event();errors=[]
        def cleanup():
            start.wait()
            try:
                for d in scratch.iterdir():
                    for p in d.iterdir():p.unlink();time.sleep(.0001)
                    d.rmdir()
                scratch.rmdir()
            except BaseException as error:errors.append(repr(error))
        thread=threading.Thread(target=cleanup);thread.start();start.set();samples=0
        try:
            while thread.is_alive():lifecycle.live_bytes(parent);samples+=1
        finally:thread.join()
        self.assertFalse(errors);self.assertFalse(scratch.exists());self.assertGreater(samples,0)
        self.assertEqual(lifecycle.live_bytes(parent),0)

    def junction(self,target):
        link=self.root/'junction'
        result=subprocess.run(['cmd','/c','mklink','/J',str(link),str(target)],capture_output=True)
        self.assertEqual(result.returncode,0,result.stderr)
        self.addCleanup(lambda:os.rmdir(link) if link.exists() else None)
        return link

    def test_real_outside_junction_refuses_root_and_nested(self):
        with tempfile.TemporaryDirectory() as outside:
            target=Path(outside);(target/'outside').write_bytes(b'x'*31)
            link=self.junction(target)
            with self.assertRaisesRegex(ValueError,'CompactReparsePoint'):lifecycle.live_bytes(link)
            with self.assertRaisesRegex(ValueError,'CompactReparsePoint'):lifecycle.live_bytes(self.root)
            with self.assertRaisesRegex(ValueError,'CompactReparsePoint'):checked(link)
            self.assertEqual((target/'outside').stat().st_size,31)
            os.rmdir(link)

    def test_real_file_symlink_refuses_without_following(self):
        target=self.root/'payload';target.write_bytes(b'x'*10);link=self.root/'link'
        try:os.symlink(target,link)
        except OSError as error:
            if getattr(error,'winerror',None)!=1314:raise
            # DrvFs creates a real LX_SYMLINK reparse point without changing
            # Windows privileges. Both tags must be refused by attributes.
            from build_native import linux
            result=subprocess.run(['wsl','-d','Ubuntu-22.04','--','python3','-c',
                'import os,sys;os.symlink(sys.argv[1],sys.argv[2])',linux(target),linux(link)],capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
        self.assertTrue(link.lstat().st_file_attributes&0x400)
        with self.assertRaisesRegex(ValueError,'CompactReparsePoint'):lifecycle.live_bytes(self.root)
        link.unlink();self.assertEqual(target.stat().st_size,10)

    def test_persistent_io_error_is_not_ignored(self):
        leaf=self.root/'denied';leaf.write_bytes(b'x');original=Path.lstat
        def denied(path,*args,**kwargs):
            if path==leaf:raise PermissionError('persistent fixture denial')
            return original(path,*args,**kwargs)
        with patch.object(Path,'lstat',denied):
            with self.assertRaises(PermissionError):lifecycle.live_bytes(self.root)

    def test_persistent_directory_redirect_is_not_ignored(self):
        with patch.object(lifecycle,'checked',side_effect=ValueError('CompactPathRedirect')):
            with self.assertRaisesRegex(ValueError,'CompactPathRedirect'):lifecycle.live_bytes(self.root)

    def test_output_limit_still_refuses_actual_payload(self):
        (self.root/'payload').write_bytes(b'x'*11)
        args=SimpleNamespace(memory_limit_gib=16,output_limit_bytes=10)
        with self.assertRaisesRegex(ValueError,'CompactInvocationOutputLimit'):
            lifecycle.resource_check(SimpleNamespace(root=self.root),args,self.root/'absent',time.monotonic()+30,rss=1)


if __name__=='__main__':unittest.main(verbosity=2)
