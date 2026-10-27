"""Forward cap selection using the existing source guard and module loader.

Select in each fresh process after fixture imports and before resource imports.
The caller keeps its full input custody through native_scope.bounded retirement.
No mesh dispatch, scientific change or modification of a historical owner occurs.
"""
from contextlib import contextmanager, ExitStack
from pathlib import Path
import hashlib
import json
import sys

HERE = Path(__file__).resolve().parent
NAMES = ('resource_limits', 'sdf_cfd_mesher_v4._frozen_resource_limits')


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


@contextmanager
def selected(pins):
    from immutable_state_access import guard
    from sdf_cfd_mesher_v4.runtime import frozen_imports
    primary = None
    owner = None
    with frozen_imports._lock, ExitStack() as held:
        if any(name in sys.modules for name in NAMES):
            raise ValueError('FreshResourceOwnerRequired')
        for path in (HERE/'PROFILE.json', HERE/'selection.py', HERE/'resource_limits.py'):
            held.enter_context(guard(path))
            if pins.get(str(path)) != sha(path):
                raise ValueError('ChangedResourceProfileSource', str(path))
        profile = json.loads((HERE/'PROFILE.json').read_text(encoding='utf-8'))
        if (profile['schema'] != 'ForwardResourceProfile/1'
                or profile['resource_owner'] != 'resource_limits.py'
                or profile['resource_owner_sha256'] != sha(HERE/'resource_limits.py')):
            raise ValueError('InvalidResourceProfile')
        try:
            owner = frozen_imports._load('resource_limits', HERE/'resource_limits.py')
            if (owner.APPROVED != profile['limits']
                    or Path(owner.__file__).resolve() != HERE/'resource_limits.py'):
                raise ValueError('ChangedSelectedResourceOwner')
            owner.validate(profile['limits'])
            yield owner, profile
        except BaseException as error:
            primary = error
            raise
        finally:
            try:
                if owner is not None and any(sys.modules.get(name) is not owner for name in NAMES):
                    raise ValueError('ChangedSelectedResourceOwner')
            except BaseException as error:
                if primary is None:
                    raise
                primary.add_note('Resource profile cleanup: '+repr(error))
            finally:
                for name in NAMES:
                    sys.modules.pop(name, None)
