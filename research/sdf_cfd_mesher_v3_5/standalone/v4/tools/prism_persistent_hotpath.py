"""Opt-in persistent prism path; bootstrap anchors are explicit inputs."""
_INSTALLED=False


def install():
    global _INSTALLED
    if _INSTALLED:return
    import prism_hotpath_v2
    if prism_hotpath_v2._INSTALLED:
        raise ValueError('InstallPersistentPathBeforeLegacyHotpathWrapper')
    import composable_transition
    from persistent_prism_transition import produce
    composable_transition.produce=produce
    from prism_persistent_views import install as views
    from prism_sealed_file_cache import install as publication
    from prism_receipt_witness import install as receipt
    views();publication();receipt()
    _INSTALLED=True


def bind_bootstrap(state,directory,index_root,version_path,version_root,lookup=None):
    """Caller pins independently deep-qualified bootstrap roots, never guesses."""
    from persistent_numeric_incidence import PackedSegment
    import immutable_state_access as access
    from sdf_cfd_mesher_v4.common.authority import identity
    from pathlib import Path
    state.check()
    with PackedSegment(directory,index_root,state.content_root),access.guard(version_path):
        body=access.read(version_path)
        if identity(body)!=version_root or body['parent_content']!=state.content_root or body['numeric_root']!=index_root or not body['closed']:
            raise ValueError('WrongVersionClosure')
    state.numeric_incidence=dict(directory=str(Path(directory).resolve()),root=index_root,content=state.content_root)
    if lookup is not None:state.numeric_incidence['lookup']=lookup
    state.numeric_version_closure=dict(path=str(Path(version_path).resolve()),root=version_root)


from contextlib import contextmanager

@contextmanager
def geometry_session(state,adapter):
    from persistent_numeric_incidence import open_geometry
    from prism_source_lookup import point_views
    reference=getattr(state,'numeric_incidence',None) or state.body['plan'].get('persistent_geometry')
    if reference is None:raise ValueError('MissingPersistentGeometryReference')
    with open_geometry(reference,state.content_root) as packed,point_views(state,packed,adapter):
        yield packed


def bind_committed_geometry(state,binding,expected_identity):
    """Bind an independently qualified immutable index to its scientific state."""
    from sdf_cfd_mesher_v4.common.authority import identity
    from persistent_numeric_incidence import open_geometry
    if identity(binding)!=expected_identity or binding['child_content']!=state.content_root:
        raise ValueError('WrongCommittedGeometryBinding')
    state.check()
    with open_geometry(binding,state.content_root):pass
    state.numeric_incidence=binding
