"""Relocated V3.5 runtime closure; original external byte pins stay mandatory.

Historical package pyc files were deliberately not distributed. Execution uses
the copied source files and -B. Current local source pins come from the held
V3.5 source closure, not from a claimed historical release qualification.
"""
from pathlib import Path
import sys,json,subprocess,importlib
ROOT=Path(__file__).resolve().parent


class RuntimeIdentity:
    def __init__(self,source_files):
        from sdf_cfd_mesher_v4.common.authority import file_hash
        self.sha=file_hash;self.allowed={};self.binding=json.loads((ROOT/'v4/runtime/FINITE_NATIVE_BINDING.json').read_text())
        if str(Path(sys.executable).resolve())!=self.binding['executable']:raise ValueError('ChangedNativeRuntimeInterpreter')
        toolchain=json.loads((ROOT/'v4/tools/finite_research/direct_evidence_modules/LINUX_TOOLCHAIN.json').read_text())
        def add(path,pin):
            path=str(Path(path).resolve())
            if path in self.allowed and self.allowed[path]!=pin:raise ValueError('ConflictingDirectRuntimePin',path)
            self.allowed[path]=pin
        # Original absolute repository origins are provenance, never imports.
        # Their relocated local replacements are pinned below by the parent.
        old='/mnt/c/Users/Jackson/Desktop/02_Projects/09b_Implicit_CAD_claude/'
        for name,pin in toolchain['files'].items():
            if not name.startswith(old):add(name,pin)
        for row in self.binding['files']:
            if 'external_path' in row:add(row['external_path'],row['sha256'])
        for name,pin in source_files.items():
            path=(ROOT/name).resolve()
            if not path.is_relative_to(ROOT):raise ValueError('DirectSourceOutsidePackage')
            add(path,pin)
        self.expected_modules={name:dict(path=str(((ROOT/'v4'/row['package_relative']) if 'package_relative' in row else Path(row['external_path'])).resolve()),version=row['version']) for name,row in self.binding['modules'].items()}

    def verify(self):
        for path,pin in self.allowed.items():
            if self.sha(path)!=pin:raise ValueError('ChangedDirectRuntimeFile',path)
        for name,expected in self.expected_modules.items():
            module=importlib.import_module(name)
            actual=dict(path=str(Path(module.__file__).resolve()),version=getattr(module,'__version__',None))
            if actual!=expected:raise ValueError('ChangedResolvedDirectModule',name,actual,expected)
        loaded={str(Path(m.__file__).resolve()) for m in tuple(sys.modules.values()) if getattr(m,'__file__',None) and Path(m.__file__).is_file()}
        for path in sorted(loaded):
            if self.allowed.get(path)!=self.sha(path):raise ValueError('UnpinnedDirectLoadedModule',path)
        import gmsh
        from background_query_build import record
        binaries=[str(Path(sys.executable).resolve()),str(Path(gmsh.lib._name).resolve()),*[str(record(kind)[1].resolve()) for kind in ('query','direct')]]
        dependencies={}
        for binary in binaries:
            result=subprocess.run(['/usr/bin/ldd',binary],capture_output=True,text=True,check=True,timeout=30)
            if 'not found' in result.stdout:raise ValueError('MissingDirectNativeDependency')
            paths=sorted({str(Path(word).resolve()) for line in result.stdout.splitlines() for word in line.split() if word.startswith('/') and Path(word).is_file()})
            for path in paths:
                if self.allowed.get(path)!=self.sha(path):raise ValueError('UnpinnedDirectSharedLibrary',path)
            dependencies[binary]=paths
        # Imports above must not change a pinned dependency during the check.
        for path,pin in self.allowed.items():
            if self.sha(path)!=pin:raise ValueError('ChangedDirectRuntimeAfterImports',path)
        loaded={str(Path(m.__file__).resolve()) for m in tuple(sys.modules.values()) if getattr(m,'__file__',None) and Path(m.__file__).is_file()}
        for path in sorted(loaded):
            if self.allowed.get(path)!=self.sha(path):raise ValueError('UnpinnedDirectLoadedModule',path)
        return dict(passed=True,pinned_files=len(self.allowed),loaded_modules=len(loaded),dependencies=dependencies,
            execution='V3.5 relocated source closure plus unchanged external byte pins; historical qualification not asserted')
