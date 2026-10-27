"""Opt-in measured critical-path controls; sealed implementations stay intact."""
from contextlib import contextmanager
from unittest.mock import patch
import numpy as np


def contains(index, key):
    from persistent_numeric_incidence import PackedClosure
    if isinstance(index, PackedClosure):
        return contains(index.delta, key) or contains(index.parent, key)
    index.live()
    if not isinstance(key, str) or len(key) != 64:
        return False
    try:
        needle = key.encode('ascii')
    except UnicodeEncodeError:
        return False
    hashes = index.lookup.hashes
    i = int(np.searchsorted(hashes, needle))
    return i < len(hashes) and hashes[i] == needle


def install():
    import os
    from current_front_dependencies import install_reader
    install_reader()
    # Encoding is a writer choice, never a prerequisite for authenticated reads.
    from compact_front_atlas import install_reader as install_atlas_reader
    install_atlas_reader()
    from compact_initial_atlas import install_reader as install_cold_atlas_reader
    install_cold_atlas_reader()
    if os.environ.get('V4_QUALITY_BATCH') == '1':
        import composable_assembly as assembly
        from critical_assembly import assemble
        assembly.assemble = assemble
    import persistent_prism_transition as transition
    import critical_prism_transition as improved
    transition.produce = improved.produce
    import persistent_numeric_incidence as numeric
    from critical_geometry import ndarray_views
    original_open = numeric.open_geometry
    numeric.open_geometry = lambda *a, **kw: ndarray_views(original_open(*a, **kw))
    import prism_source_lookup as source
    original_views = source.point_views

    @contextmanager
    def views(base, index, adapter):
        with original_views(base, index, adapter):
            face_type = type(base.numeric_point_views['faces'])
            def member(self, key):
                base.check_live()
                return contains(index, key)
            with patch.object(face_type, '__contains__', member):
                yield
    source.point_views = views
