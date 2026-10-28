"""Linux read-only profile/byte/loader controls; no Gmsh import or meshing."""
from pathlib import Path
import copy,json,os,shutil,subprocess,sys,tempfile,unittest
from unittest.mock import patch
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import runtime_environment as runtime

@unittest.skipUnless(os.name=='posix','Linux path and /proc mapping semantics')
class RuntimeEnvironment(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='v35 profile ');self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        for name in (*runtime.ORIGINALS,runtime.MANIFEST,'v4/runtime/MANIFEST.json',*runtime.RELOCATIONS.values()):
            target=self.root/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/name,target)
        self.original=runtime.manifest(self.root);self.pins=runtime.effective_pins(self.root)
        self.restored={str((self.root/r['relative_path']).resolve()) for r in self.original['libraries']}
    def save(self,v):(self.root/runtime.MANIFEST).write_text(json.dumps(v))
    def reader(self,p):return runtime.sha(p) if str(p) in self.restored else self.pins[str(p)]
    def maps(self):
        rows=[]
        for name in sorted(self.restored):
            st=Path(name).stat();device='%x:%x'%(os.major(st.st_dev),os.minor(st.st_dev));rows.append('1000-2000 r--p 00000000 %s %d %s'%(device,st.st_ino,name.replace(' ','\\040')))
        return '\n'.join(rows)
    def test_only_two_paths_relocated_same_hashes_full_population(self):
        base=runtime.baseline(self.root);self.assertEqual(len(self.pins),1916)
        for old,pin in base.items():
            new=str((self.root/runtime.RELOCATIONS[old]).resolve()) if old in runtime.RELOCATIONS else str(Path(old).resolve())
            self.assertEqual(self.pins[new],pin)
        self.assertTrue(set(runtime.RELOCATIONS).isdisjoint(self.pins));self.assertEqual(runtime.profile(self.root)['version'],runtime.VERSION)
    def test_complete_audit_and_restored_missing_changed_or_unrelated_change(self):
        with patch.object(runtime,'loader_check',return_value=dict(passed=True)) as loader:
            good=runtime.audit(self.root,reader=self.reader);self.assertTrue(good['passed']);self.assertEqual(good['checked_files'],1916);loader.assert_called_once()
        for name in [*sorted(self.restored),next(n for n in self.pins if n not in self.restored)]:
            def changed(p):return '0'*64 if str(p)==name else self.reader(p)
            with patch.object(runtime,'loader_check') as loader:
                bad=runtime.audit(self.root,reader=changed);self.assertFalse(bad['passed']);self.assertEqual(len(bad['mismatches']),1);loader.assert_not_called()
        target=Path(next(iter(self.restored)));original=target.read_bytes();changed=bytearray(original);changed[-1]^=1;target.write_bytes(changed)
        with patch.object(runtime,'loader_check') as loader:
            self.assertFalse(runtime.audit(self.root,reader=self.reader)['passed']);loader.assert_not_called()
        target.write_bytes(original);target.unlink()
        self.assertFalse(runtime.audit(self.root,reader=self.reader)['passed'])
    def test_profile_source_hash_role_scope_and_path_mutations_refuse(self):
        cases=[]
        for key,value in [('schema','unknown'),('baseline_identity','0'*64),('baseline_files',1915),('historical_environment_qualification_inherited',True),('original_manifests',{})]:
            v=copy.deepcopy(self.original);v[key]=value;cases.append(v)
        for key,value in [('relative_path','../escape.so'),('relative_path','v4/runtime/isolated_brotli/libbrotlienc.so.1'),('sha256','0'*64)]:
            v=copy.deepcopy(self.original);v['libraries'][0][key]=value;cases.append(v)
        for rows in [self.original['libraries'][:1],self.original['libraries']*2,[self.original['libraries'][0]]*2]:
            v=copy.deepcopy(self.original);v['libraries']=rows;cases.append(v)
        for i,v in enumerate(cases):
            self.save(v)
            with self.subTest(case=i),self.assertRaises((ValueError,KeyError)):runtime.effective_pins(self.root)
        self.save(self.original);p=self.root/self.original['libraries'][0]['relative_path'];p.unlink();p.symlink_to(self.root/self.original['libraries'][1]['relative_path'])
        with self.assertRaises(ValueError):runtime.manifest(self.root)
    def test_canonical_collision_refuses(self):
        first=self.root/'external';first.write_text('synthetic');alias=self.root/'alias';alias.symlink_to(first)
        base={k:runtime.baseline(self.root)[k] for k in runtime.RELOCATIONS};base[str(first)]=runtime.sha(first);base[str(alias)]=runtime.sha(first)
        v=copy.deepcopy(self.original);v.update(baseline_identity=runtime.identity(base),baseline_files=len(base));self.save(v)
        with patch.object(runtime,'baseline',return_value=base),self.assertRaisesRegex(ValueError,'CanonicalPathCollision'):runtime.effective_pins(self.root)
    def test_actual_map_identity_missing_foreign_deleted_encoder_and_preload(self):
        text=self.maps();self.assertTrue(runtime.require_mappings(self.root,maps=text+'\n'+text)['passed'])
        for bad in [text.splitlines()[0],text+' (deleted)',text+'\n1000-2000 r--p 0 00:00 0 /foreign/libbrotlidec.so.1',text+'\n1000-2000 r--p 0 00:00 0 /foreign/libbrotlienc.so.1',text.replace('r--p','r--p').replace(' 1000 ',' 0 ')]:
            if bad==text:continue
            with self.subTest(maps=bad),self.assertRaises(ValueError):runtime.require_mappings(self.root,maps=bad)
        line=text.splitlines()[0];fields=line.split(maxsplit=5);fields[4]='0';bad=' '.join(fields)+'\n'+text.splitlines()[1]
        with self.assertRaisesRegex(ValueError,'MappingFileIdentity'):runtime.require_mappings(self.root,maps=bad)
        with patch.dict(os.environ,{'LD_PRELOAD':'untrusted.so'}):
            with self.assertRaisesRegex(ValueError,'PreloadConflict'):runtime.require_mappings(self.root,maps=text)
            with self.assertRaisesRegex(ValueError,'PreloadConflict'):runtime.audit(self.root,reader=self.reader)
    def test_loader_resolution_and_extra_or_wrong_dependency(self):
        expected=next(r['sha256'] for r in runtime.read(self.root/'v4/runtime/MANIFEST.json')['files'] if r['path']=='runtime/lib/gmsh/libgmsh.so.4.15')
        def reader(p):return expected if str(p).endswith('libgmsh.so.4.15') else self.reader(p)
        text='\n'.join('x => '+name+' (0x0)' for name in sorted(self.restored))
        with patch.object(runtime.subprocess,'run',return_value=SimpleNamespace(returncode=0,stdout=text,stderr='')):self.assertTrue(runtime.loader_check(self.root,reader=reader)['passed'])
        extra=self.root/'extra.so';extra.write_text('synthetic')
        for bad in [text+'\nx => '+str(extra)+' (0x0)',text.splitlines()[0],'x => not found']:
            with patch.object(runtime.subprocess,'run',return_value=SimpleNamespace(returncode=0,stdout=bad,stderr='')),self.assertRaises(ValueError):runtime.loader_check(self.root,reader=reader)
    def test_preflight_profile_population_binding_and_launch_quoting(self):
        value=dict(schema='V35ExternalRuntimePreflight/1',passed=True,profile=runtime.profile(self.root),checked_files=1916,baseline_identity=runtime.identity(runtime.baseline(self.root)),effective_identity=runtime.identity(self.pins),restored_files=2,mismatches=[],loader=dict(passed=True))
        with patch.object(runtime.subprocess,'run',return_value=SimpleNamespace(returncode=0,stdout=json.dumps(value),stderr='')) as call:
            self.assertTrue(runtime.preflight(self.root,seconds=5)['passed']);self.assertEqual(call.call_args.kwargs['timeout'],5)
            script=call.call_args.args[0][-1];self.assertIn('LD_PRELOAD',script);self.assertIn("export LD_LIBRARY_PATH='",script);self.assertIn('${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}',script)
        for key,bad in [('effective_identity','bad'),('profile',{}),('checked_files',1915),('passed',1),('loader',dict(passed=False))]:
            v=dict(value,**{key:bad})
            with patch.object(runtime.subprocess,'run',return_value=SimpleNamespace(returncode=0,stdout=json.dumps(v),stderr='')),self.assertRaises(ValueError):runtime.preflight(self.root)

if __name__=='__main__':unittest.main()
