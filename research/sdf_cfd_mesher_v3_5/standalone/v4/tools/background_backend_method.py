"""Explicit backend methods; no geometry, mesh acceptance or automatic tuning."""
import stage_observations as obs
from pathlib import Path

from owned_runtime import V4
from capsule_sizing_control import digest

LEGACY='DelaunayScalarLeaf/1'
HXT='HXT4DirectSizing095/1'

def validate(name):
    if type(name) is not str or name not in (LEGACY,HXT):raise ValueError('UnsupportedBackgroundBackendMethod')
    return name

def definition(name):
    validate(name)
    return dict(schema='V4BackgroundBackendMethod/1',name=name,
        algorithm3d=1 if name==LEGACY else 10,threads3d=1 if name==LEGACY else 4,
        callback='existing_scalar_leaf_python_bridge' if name==LEGACY else 'owned_direct_c_query',
        generation_factor_hex=(1. if name==LEGACY else .95).hex(),generation_dimensions=[3],
        effective_sizing='Existing pinned callback target; factor 1.' if name==LEGACY else
            '3D target = 0.95 * pinned callback target after Gmsh clamping; 2D target unchanged. Effective 3D cap and growth scale change.')

def request_method(request):
    if request.get('schema')=='RetainedBackgroundDiagnosticRequest/1' or request.get('context')=='RETAINED_BACKGROUND_DIAGNOSTIC':
        raise ValueError('DiagnosticBackgroundCannotEnterProduction')
    if request.get('schema') in ('V4GradedBackground/4','V4GradedBackground/5','V4GradedBackground/6'):
        if request['schema'] in ('V4GradedBackground/5','V4GradedBackground/6'):
            from background_completion_controls import validate_completion
            validate_completion(request.get('interface_completion'))
            if request['schema']=='V4GradedBackground/6':
                from tet_layer_controls import validate as validate_transition
                validate_transition(request.get('tet_transition'))
        return validate(request.get('backend_method'))
    if 'backend_method' in request or 'backend_method_identity' in request:
        raise ValueError('BackgroundMethodRequiresExplicitPolicyVersion')
    return None

@obs.observed('method.selection','source_lock')
def selection_record(name):
    chosen=definition(name)
    names=('background_backend_method.py','core_backend.py','gmsh_core_setup.py','background_completion_controls.py','background_interface_completion.py')
    if name==HXT:names+=('background_query_build.py','background_native_query.py','background_direct_callback.py',
        'background_sizing.py','optimized_core_sizing.py','generic_core_sizing.py')
    chosen['source_pins']={str((V4/'tools'/n).relative_to(V4).as_posix()):digest(V4/'tools'/n) for n in names}
    if name==HXT:
        from background_query_build import record,ROOT
        builds={}
        for kind in ('query','direct'):
            metadata,library=record(kind);path=ROOT/('QUERY_BUILD.json' if kind=='query' else 'DIRECT_BUILD.json')
            builds[kind]=dict(record=path.relative_to(V4).as_posix(),record_sha256=digest(path),
                library=library.relative_to(V4).as_posix(),binary_sha256=metadata['binary_sha256'])
        chosen['native_builds']=builds
    return chosen

def validate_readback(backend,expected):
    if backend.get('schema')=='RetainedBackgroundDiagnosticReceipt/1' or backend.get('context')=='RETAINED_BACKGROUND_DIAGNOSTIC':
        raise ValueError('DiagnosticBackgroundCannotAdmitProduction')
    selection=expected.get('backend_method')
    if selection is None:
        if 'backend_method' in backend:raise ValueError('UnexpectedBackgroundMethodReadback')
        return
    validate(selection.get('name'))
    report=backend.get('backend_method')
    if not isinstance(report,dict) or report.get('selection')!=selection or report.get('passed') is not True:
        raise ValueError('BackgroundMethodReadbackMismatch')
    if selection['name']==LEGACY:
        if report.get('execution')!='existing_worker':raise ValueError('BackgroundLegacyReadbackMismatch')
        return
    sessions=[dict(dimension=2,before=1.,factor=1.,passed=True),dict(dimension=3,before=1.,factor=.95,passed=True)]
    options={'Mesh.Algorithm3D':10,'General.NumThreads':4,'Mesh.MaxNumThreads1D':1,
        'Mesh.MaxNumThreads2D':1,'Mesh.MaxNumThreads3D':4,'Mesh.OptimizeThreshold':.3,
        'Mesh.Reproducible':1,'Mesh.RandomSeed':1}
    c=report.get('callback',{})
    if (report.get('generation_sessions')!=sessions or report.get('settings')!=[options]*4
        or report.get('closed') is not True
        or c.get('registrations')!=2 or c.get('removals')!=2 or c.get('quarantined') is not False
        or c.get('python_failure') is not False or c.get('native_failure')!=0
        or c.get('cache_capacity')!=0 or c.get('exact_coordinate_cache_hits')!=0
        or type(c.get('evaluations')) is not int or c['evaluations']<=0
        or c.get('evaluations')!=c.get('native_calls',-1)+c.get('fallback_calls',-1)):
        raise ValueError('BackgroundDirectReadbackMismatch')

class DirectExecution:
    """Existing direct lifetime owner, with explicit setup/generation calls."""
    def __init__(self,gmsh,triangles,seeds,*,g,h_max,selection):
        from background_native_query import NativeQueryField
        from background_direct_callback import DirectCallback,SessionBridge
        if selection!=selection_record(HXT):raise ValueError('ChangedOwnedDirectSelection')
        self.gmsh=gmsh;self.selection=selection;self.sessions=[];self.settings=[];self.closed=False
        self.field=NativeQueryField(triangles,seeds,g=g,h_max=h_max);self.callback=None
        try:
            if not self.field.native_report()['native_enabled']:raise ValueError('RequiredOwnedDirectQueryUnavailable')
            self.callback=DirectCallback(self.field)
            self.bridge=SessionBridge(gmsh,self._generate,gmsh.finalize);self.bridge.owner=self.callback
        except BaseException:
            if self.callback is not None:self.callback.close()
            self.field.close();raise

    def setup(self,gmsh):
        if gmsh is not self.gmsh or self.closed:raise ValueError('ChangedDirectExecutionModel')
        from gmsh_core_setup import setup_profile
        self.settings.append(setup_profile(gmsh,algorithm=10,threads3d=4))

    def install(self,callback):self.bridge.install(callback)
    def generate(self,dimension):return self.bridge.generate(dimension)
    def finalize(self):return self.bridge.finalize()

    def _generate(self,dimension):
        # SessionBridge invokes this operation inside the existing owner lease.
        if type(dimension) is not int or dimension not in (2,3):raise ValueError('InvalidDirectGenerationDimension')
        option=self.gmsh.option;before=option.getNumber('Mesh.MeshSizeFactor')
        if before!=1.:raise ValueError('ChangedPreGenerationSizingFactor')
        factor=.95 if dimension==3 else 1.
        if dimension==3:option.setNumber('Mesh.MeshSizeFactor',factor)
        if option.getNumber('Mesh.MeshSizeFactor')!=factor:raise ValueError('UnappliedGenerationSizingFactor')
        result=self.gmsh.model.mesh.generate(dimension)
        if option.getNumber('Mesh.MeshSizeFactor')!=factor:raise ValueError('ChangedGenerationSizingFactor')
        self.sessions.append(dict(dimension=dimension,before=before,factor=factor,passed=True))
        return result

    def finish(self):
        self.callback.require_passed()
        if (self.closed or len(self.settings)!=4 or self.callback.registered
            or self.callback.registrations!=2 or self.callback.removals!=2
            or selection_record(HXT)!=self.selection):raise ValueError('IncompleteOwnedDirectExecution')
        report=dict(passed=True,selection=self.selection,generation_sessions=list(self.sessions),
            settings=list(self.settings),callback=self.callback.report(),closed=True)
        self.close()
        validate_readback({'backend_method':report},{'backend_method':self.selection})
        return report

    def close(self):
        if self.closed:return
        failure=None
        try:
            if self.callback is not None:
                self.callback.close()
                if self.callback.pointer or self.callback.quarantined:raise ValueError('OwnedDirectExecutionQuarantined')
        except BaseException as error:failure=error
        try:self.field.close()
        except BaseException as error:
            if failure is None:failure=error
            else:failure.add_note('DirectFieldCleanupFailure: '+repr(error))
        self.closed=True
        if failure is not None:raise failure
