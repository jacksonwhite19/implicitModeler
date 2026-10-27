"""Uninstalled command routing preparation; full outer source custody required.

The original command owns all flags, case binding, scientific functions and
release checks. Only its child entry prefix is substituted for one call.
The standalone entry is disabled. This module confers no execution authority.
"""
from contextlib import contextmanager, ExitStack, nullcontext
from pathlib import Path
from types import FunctionType, GeneratorType
import hashlib
import importlib
import json
import sys

HERE = Path(__file__).resolve().parent
V4 = HERE.parents[1]
COMMAND_ALIASES = ('standalone_tool', 'sdf_cfd_mesher_v4._frozen_standalone_tool')
PRIVATE_NAMES = ('finite_collar_stage_adapter', 'finite_collar_common_routing')
PRIVATE_ALIASES = tuple(a for n in PRIVATE_NAMES for a in (n, 'sdf_cfd_mesher_v4._frozen_'+n))
PREFIX_SLOT = '_finite_collar_worker_prefix'
_MISSING = object()
_used = False


def _sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def _empty_command():
    if any(n in sys.modules for n in COMMAND_ALIASES):
        raise ValueError('FreshCommonCommandHostRequired')



def _first_mismatch(cli, replacement, prefix, binding, routing, case_owner, case_scope, functions, contexts):
    # Same predicates and short-circuit order as the original cleanup expression.
    if cli._supervise_guarded is not replacement: return ('supervisor', '')
    if cli.__dict__.get(PREFIX_SLOT) is not prefix: return ('prefix', '')
    if sys.modules.get('standalone_tool') is not cli: return ('canonical-command', '')
    if COMMAND_ALIASES[1] in sys.modules: return ('frozen-command-alias', '')
    for name, module in zip(PRIVATE_NAMES, (binding, routing)):
        for n in (name, 'sdf_cfd_mesher_v4._frozen_'+name):
            if sys.modules.get(n) is not module: return ('helper', n)
    if sys.modules.get('standalone_case') is not case_owner: return ('canonical-case', '')
    if case_owner._import_scopes is not case_scope: return ('case-context-object', '')
    for n, value in functions.items():
        if getattr(cli, n, _MISSING) is not value: return ('function', n)
    for n, value in contexts.items():
        if getattr(cli, n, _MISSING) is not value: return ('context', n)
    return None


def _enforce_identity(cli, replacement, prefix, binding, routing, case_owner, case_scope, functions, contexts, observer):
    mismatch = _first_mismatch(cli, replacement, prefix, binding, routing,
                               case_owner, case_scope, functions, contexts)
    if mismatch is not None:
        try:
            if observer is not None: observer(mismatch)
        except BaseException:
            pass  # Observation cannot replace or suppress required rejection.
        raise ValueError('ChangedCommonCommandDuringConsumer')


@contextmanager
def _canonical_writer_initialization(cli, functions, check, pins):
    """Authenticate only the normal first writer transition, at its original call."""
    from immutable_state_access import guard
    with ExitStack() as held:
        names = ('optimized_admission', 'optimized_durable_json', 'path_preserving_deduplication')
        modules = {}
        for name in names:
            path = V4/'tools'/(name+'.py')
            held.enter_context(guard(path))
            if pins.get(str(path)) != _sha(path):
                raise ValueError('ChangedWriterInitializationSource', name)
            module = importlib.import_module(name)
            if (sys.modules.get(name) is not module or Path(module.__file__).resolve() != path
                    or Path(module.__spec__.origin).resolve() != path):
                raise ValueError('ChangedWriterInitializationModule', name)
            modules[name] = module
        installer = modules['optimized_admission']
        original_install = installer.install
        original_writer = modules['path_preserving_deduplication'].durable_json
        optimized_writer = modules['optimized_durable_json'].durable_json
        for value, name in ((original_install, 'optimized_admission'),
                            (original_writer, 'path_preserving_deduplication'),
                            (optimized_writer, 'optimized_durable_json')):
            if (not isinstance(value, FunctionType) or value.__globals__ is not modules[name].__dict__
                    or Path(value.__code__.co_filename).resolve() != V4/'tools'/(name+'.py')):
                raise ValueError('ChangedWriterInitializationFunction', name)
        if cli.durable_json is not original_writer or functions.get('durable_json') is not original_writer:
            raise ValueError('OriginalWriterBeforeCanonicalInitializationRequired')
        expected = dict(functions)
        expected['durable_json'] = optimized_writer  # Predeclared; never resnapshot changed bindings.
        attempted = False
        primary = None
        def unchanged_modules():
            if any(sys.modules.get(name) is not module for name, module in modules.items()):
                raise ValueError('ChangedWriterInitializationModule')
            if modules['optimized_durable_json'].durable_json is not optimized_writer:
                raise ValueError('ChangedOptimizedWriter')
        def initialize():
            nonlocal attempted
            if attempted or installer.install is not initialize:
                raise ValueError('UnexpectedWriterInitializationEntry')
            attempted = True
            try:
                unchanged_modules()
                check(functions)
                if cli.durable_json is not original_writer:
                    raise ValueError('OriginalWriterBeforeCanonicalInitializationRequired')
                result = original_install()
                unchanged_modules()
                if installer.install is not initialize:
                    raise ValueError('ChangedCanonicalInstallerDuringInitialization')
                check(expected)
                functions['durable_json'] = optimized_writer
                return result
            finally:
                # The qualified stack legitimately calls install a second time.
                # Restore immediately so that canonical call remains unchanged.
                installer.install = original_install
        installer.install = initialize
        try:
            yield
        except BaseException as error:
            primary = error
            raise
        finally:
            try:
                unchanged_modules()
                if installer.install is not (original_install if attempted else initialize):
                    raise ValueError('ChangedCanonicalInstallerEntry')
            except BaseException as error:
                if primary is None: raise
                primary.add_note('Writer initialization cleanup: '+repr(error))
            finally:
                installer.install = original_install


@contextmanager
def selected_command(*, request_root, selector, nonmeshing_control=False, observer=None, terminal_scope=None):
    """Keep tested selection alive through the actual main/worker consumer."""
    request_root = Path(request_root).resolve()
    global _used
    if _used:
        raise ValueError('FreshCommonCommandHostRequired')
    _empty_command()
    if any(n in sys.modules for n in PRIVATE_ALIASES):
        raise ValueError('ConflictingCommonCommandHelper')
    _used = True
    original_path = sys.path[:]
    primary = None
    try:
        sys.path[:0] = [str(V4/'src'), str(V4/'tools')]
        from sdf_cfd_mesher_v4.runtime import frozen_imports
        from immutable_state_access import guard
        with ExitStack() as held, frozen_imports._lock:
            manifest = request_root/'PINNED_SOURCES.json'
            held.enter_context(guard(manifest))
            pins = json.loads(manifest.read_text())
            local = [HERE/'connection.py', HERE/'routing_body.py', HERE/'run.py',
                     HERE/'stage.py', request_root/'STAGE_REQUEST.json',
                     V4/'tools/standalone_tool.py', V4/'tools/standalone_case.py',
                     Path(frozen_imports.__file__).resolve(), V4/'tools/immutable_state_access.py']
            for path in local:
                held.enter_context(guard(path))
                if pins.get(str(path)) != _sha(path):
                    raise ValueError('ChangedCommonCommandSource', str(path))
            if _sha(request_root/'STAGE_REQUEST.json') != selector:
                raise ValueError('ChangedCandidateSelection')
            _empty_command()
            binding = frozen_imports._load(PRIVATE_NAMES[0], HERE/'stage.py')
            with nullcontext({}) as modules:  # Upstream uses only canonical modules.
                _empty_command()
                cli = importlib.import_module('standalone_tool')
                if (Path(cli.__file__).resolve() != V4/'tools/standalone_tool.py'
                        or Path(cli.__spec__.origin).resolve() != V4/'tools/standalone_tool.py'
                        or sys.modules.get(COMMAND_ALIASES[0]) is not cli
                        or COMMAND_ALIASES[1] in sys.modules):
                    raise ValueError('ChangedCommonCommandIdentity')
                case_owner = sys.modules['standalone_case']
                case_scope = case_owner._import_scopes
                if Path(case_owner.__file__).resolve() != V4/'tools/standalone_case.py':
                    raise ValueError('ChangedCaseOwner')
                original = cli._supervise_guarded
                if (original.__globals__ is not cli.__dict__ or original.__closure__
                        or original.__defaults__ or original.__kwdefaults__
                        or Path(original.__code__.co_filename).resolve() != V4/'tools/standalone_tool.py'):
                    raise ValueError('ChangedSupervisorIdentity')
                functions = {n: v for n, v in vars(cli).items()
                             if isinstance(v, FunctionType) and n != '_supervise_guarded'}
                contexts = {n: v for n, v in vars(cli).items() if isinstance(v, cli.ContextVar)}
                routing = frozen_imports._load(PRIVATE_NAMES[1], HERE/'routing_body.py')
                code = routing._supervise_guarded.__code__
                if code.co_freevars or routing._supervise_guarded.__defaults__ or routing._supervise_guarded.__kwdefaults__:
                    raise ValueError('UnsupportedRoutingClosure')
                replacement = FunctionType(code, cli.__dict__, original.__name__)
                previous_prefix = cli.__dict__.get(PREFIX_SLOT, _MISSING)
                executable_record = request_root/'PYTHON_IDENTITY.json'
                held.enter_context(guard(executable_record))
                execution_pins = json.loads((request_root/('controls01_INPUT_PINS.json' if nonmeshing_control else 'live01_INPUT_PINS.json')).read_text())
                if execution_pins.get(str(executable_record)) != _sha(executable_record):
                    raise ValueError('ChangedOwnedExecutableRecord')
                executable = json.loads(executable_record.read_text())['child']['executable']
                worker_entry = HERE/'run.py'
                if nonmeshing_control:
                    held.enter_context(guard(worker_entry))
                    if execution_pins.get(str(worker_entry)) != _sha(worker_entry):
                        raise ValueError('ChangedNonmeshingWorkerEntry')
                prefix = (executable, '-B', str(worker_entry), '--request-dir', str(request_root), '--finite-collar-selection', selector)
                if cli._supervise_guarded is not original:
                    raise ValueError('SupervisorChangedBeforeRouting')
                cli.__dict__[PREFIX_SLOT] = prefix
                cli._supervise_guarded = replacement
                try:
                    def check(expected):
                        # The unchanged terminal owner temporarily wraps surface_stage.
                        # Authenticate its actual suspended generator, never the current
                        # arbitrary CLI binding, and change only this scoped expectation.
                        current = expected
                        if terminal_scope:
                            if type(terminal_scope) is not list or len(terminal_scope) != 1:
                                raise ValueError('ExactTerminalOwnerRequired')
                            generator = getattr(terminal_scope[0], 'gen', None)
                            if (type(generator) is not GeneratorType or not generator.gi_suspended
                                    or generator.gi_code is not binding.terminal_dispatch.__wrapped__.__code__
                                    or generator.gi_frame is None
                                    or generator.gi_frame.f_globals is not binding.__dict__):
                                raise ValueError('ActiveTerminalOwnerRequired')
                            values = generator.gi_frame.f_locals
                            surface_owner=binding
                            surface_path=HERE/'stage.py'
                            if 'direct_scope' in values:
                                from finite_research import direct_runtime
                                direct_path=HERE/'direct_runtime.py'
                                if pins.get(str(direct_path))!=_sha(direct_path) or Path(direct_runtime.__file__).resolve()!=direct_path:
                                    raise ValueError('UnboundDirectTerminalOwner')
                                nested=getattr(values['direct_scope'],'gen',None)
                                if (type(nested) is not GeneratorType or not nested.gi_suspended
                                        or nested.gi_code is not direct_runtime.terminal_dispatch.__wrapped__.__code__
                                        or nested.gi_frame is None or nested.gi_frame.f_globals is not direct_runtime.__dict__
                                        or values.get('direct_dispatch') is not direct_runtime.terminal_dispatch):
                                    raise ValueError('ActiveDirectTerminalOwnerRequired')
                                generator=nested
                                values=nested.gi_frame.f_locals
                                surface_owner=direct_runtime
                                surface_path=direct_path
                            surface = values.get('surface')
                            if (values.get('cli') is not cli
                                    or values.get('original_surface') is not functions['surface_stage']
                                    or not isinstance(surface, FunctionType)
                                    or cli.surface_stage is not surface
                                    or surface.__globals__ is not surface_owner.__dict__
                                    or surface.__code__ not in generator.gi_code.co_consts
                                    or Path(surface.__code__.co_filename).resolve() != surface_path):
                                raise ValueError('ChangedTerminalSurfaceBinding')
                            cells = dict(zip(surface.__code__.co_freevars, surface.__closure__ or ()))
                            if set(cells) != {'cli','load_production_wall','original_surface','outer_check','value'}:
                                raise ValueError('ChangedTerminalSurfaceClosure')
                            if any(cells[n].cell_contents is not values[n] for n in cells):
                                raise ValueError('ChangedTerminalSurfaceClosure')
                            current = dict(expected)
                            current['surface_stage'] = surface
                        _enforce_identity(cli, replacement, prefix, binding, routing,
                                          case_owner, case_scope, current, contexts, observer)
                    with _canonical_writer_initialization(cli, functions, check, pins):
                        yield cli, binding
                except BaseException as error:
                    primary = error
                    raise
                finally:
                    try:
                        _enforce_identity(cli, replacement, prefix, binding, routing,
                                          case_owner, case_scope, functions, contexts, observer)
                    except BaseException as error:
                        if primary is None:
                            raise
                        primary.add_note('Common command cleanup: '+repr(error))
                    finally:
                        cli._supervise_guarded = original
                        if previous_prefix is _MISSING:
                            cli.__dict__.pop(PREFIX_SLOT, None)
                        else:
                            cli.__dict__[PREFIX_SLOT] = previous_prefix
                # Canonical command/case owners remain installed through binding cleanup.
    finally:
        for name in PRIVATE_ALIASES:
            sys.modules.pop(name, None)
        sys.path[:] = original_path


def prepared_main(argv):
    raise ValueError('UseOwnedFiniteResearchCommand')
