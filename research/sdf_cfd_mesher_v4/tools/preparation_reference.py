"""Cold authentication of newly produced preparation dependencies.

Equivalent to attest(path, file_hash(path)), with the digest calculated inside
the existing deny-write guard rather than calculated twice. No trust cache,
caller assertion, persisted witness or change to reference/authentication format.
Existing references with an expected digest still use access.attest unchanged.
"""
import immutable_state_access as access
from path_preserving_deduplication import file_hash


def attest_content(path):
    with access.guard(path):
        before=access.token(path)
        digest=file_hash(path)
        if access.token(path)!=before:
            raise ValueError('MutationDuringAuthentication')
        return dict(sha256=digest,token=before)
