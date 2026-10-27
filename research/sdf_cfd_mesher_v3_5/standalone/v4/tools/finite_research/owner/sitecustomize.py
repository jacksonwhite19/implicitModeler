"""Only active inside the isolated fixture's explicitly scoped child tree."""
import os
if os.name == 'nt' and 'V4_FIXTURE_NATIVE_SCOPE' in os.environ:
    try:
        import native_scope
        native_scope.install()
    except BaseException as error:
        raise SystemExit('RequiredFixtureNativeScopeUnavailable: '+repr(error))
