"""Owned background binary admission using existing deny-write file guards."""
from contextlib import contextmanager,ExitStack
import ctypes,json
import build_provenance as provenance
from immutable_state_access import guard
from path_preserving_deduplication import file_hash

@contextmanager
def held_library(path,sources):
    record=path.with_name(path.name+'.BUILD.json') if path.suffix=='.so' else path.with_suffix('.BUILD.json')
    with ExitStack() as held:
        provenance.hold(held)
        held.enter_context(guard(record))
        metadata=provenance.receipt(record)
        if metadata['exit_code']!=0 or metadata['sources']!=sources:raise ValueError('UnqualifiedCandidateBuild')
        command=metadata['command']
        if set(sources)=={'independent_walk.rs'}:names=['independent_walk.rs']
        elif set(sources)=={'screen_candidate.rs','efficient_candidate.rs'}:
            names=['screen_candidate.rs','efficient_candidate.rs']
        else:raise ValueError('UnsupportedCandidateSourceClosure')
        for name,pin in sources.items():
            held.enter_context(guard(path.parent/name))
            if file_hash(path.parent/name)!=pin:raise ValueError('ChangedCandidateSource')
        # Authenticate the entire reviewed command, not just a panic flag.
        expected=['rustc','--edition=2021','--crate-type=cdylib','-O',
                  '-C','debuginfo=0','-C','panic=unwind',provenance.original(path.parent/names[0]),'-o',provenance.original(path)]
        digest=__import__('hashlib').sha256(b''.join((path.parent/n).read_bytes() for n in names)).hexdigest()
        if command!=expected or metadata.get('source_digest')!=digest:
            raise ValueError('UnsupportedCandidateBuild')
        held.enter_context(guard(path))
        if file_hash(path)!=metadata['binary_sha256']:raise ValueError('ChangedCandidateBinary')
        yield ctypes.CDLL(str(path))
