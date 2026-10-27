"""One fixed private native-owner selection; no reload, registry or authority."""
from contextlib import contextmanager, ExitStack
from pathlib import Path
import hashlib,sys
HERE=Path(__file__).resolve().parent
BUNDLE=HERE/'owner'

def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()

@contextmanager
def selected_native(pins):
    from immutable_state_access import guard
    before=sys.path[:]
    with ExitStack() as held:
        for path in [HERE/'owner_selection.py',*[BUNDLE/n for n in ('native_scope.py','wire.py','sitecustomize.py','linux_scope.py','windows_atomic_process.py','windows_job_capability.py','delegated_scope.py','delegation_pipe.py')]]:
            held.enter_context(guard(path))
            if pins.get(str(path))!=sha(path):raise ValueError('UnboundFixedNativeOwner',str(path))
        for name in ('wire','native_scope','windows_atomic_process','windows_job_capability','delegated_scope','delegation_pipe'):
            module=sys.modules.get(name)
            if module is not None and Path(getattr(module,'__file__','')).resolve()!=BUNDLE/(name+'.py'):
                raise ValueError('WrongCachedNativeOwner',name)
        sys.path.insert(0,str(BUNDLE))
        try:
            import wire,native_scope
            pair=(wire,native_scope)
            def attest():
                if (sys.modules.get('wire') is not pair[0] or sys.modules.get('native_scope') is not pair[1]
                        or Path(wire.__file__).resolve()!=BUNDLE/'wire.py'
                        or Path(native_scope.__file__).resolve()!=BUNDLE/'native_scope.py'
                        or native_scope.HERE!=BUNDLE or native_scope.receive is not wire.receive
                        or native_scope.send is not wire.send
                        or native_scope.Scope.control.__globals__ is not vars(native_scope)):
                    raise ValueError('ChangedFixedNativeOwnerIdentity')
                for name in ('windows_atomic_process','windows_job_capability','delegated_scope','delegation_pipe'):
                    module=sys.modules.get(name)
                    if module is not None and Path(getattr(module,'__file__','')).resolve()!=BUNDLE/(name+'.py'):
                        raise ValueError('ChangedAtomicHelperOrigin',name)
                for path in (BUNDLE/'native_scope.py',BUNDLE/'wire.py',BUNDLE/'windows_atomic_process.py',BUNDLE/'windows_job_capability.py',BUNDLE/'delegated_scope.py',BUNDLE/'delegation_pipe.py'):
                    if pins.get(str(path))!=sha(path):raise ValueError('ChangedFixedNativeOwnerSource',str(path))
                return dict(native_scope=str(BUNDLE/'native_scope.py'),wire=str(BUNDLE/'wire.py'),here=str(native_scope.HERE),native_sha256=sha(BUNDLE/'native_scope.py'),wire_sha256=sha(BUNDLE/'wire.py'),canonical_objects=True)
            attest()
            try:yield native_scope,attest
            except BaseException as primary:
                try:attest()
                except BaseException as secondary:primary.add_note('Native owner exit: '+repr(secondary))
                raise
            else:attest()
        finally:sys.path[:]=before
