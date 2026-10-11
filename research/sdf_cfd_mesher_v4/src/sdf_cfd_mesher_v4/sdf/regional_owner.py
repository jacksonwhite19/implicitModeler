"""Call-scoped custody of an actual canonical SDF expression.

The caller supplies authenticated executable and kernel-source identities from
its reviewed runtime/build closure. A caller's arbitrary hash is not a release
approval. This owner grants mathematical face-bound queries only; it does not
grant surface coverage, topology, native-error acceptance or prism admission.
"""
from contextlib import contextmanager
from contextvars import ContextVar
from copy import deepcopy
import threading
import json
import math
from pathlib import Path
import re
import psutil
from .canonical import CanonicalGeometry,CanonicalKernel,GeometryFailure
from .regional import Expression,certify_triangle,query_enclosure


_ISSUER=object()
_batch=ContextVar('canonical_verified_input_batch',default=None)


def fail(code,detail,identity=None):
    raise GeometryFailure(code,'regional_bounds',detail,identity)


def digest(value):
    if type(value) is not str or re.fullmatch('[0-9a-f]{64}',value) is None:
        fail('AuthenticatedIdentityRequired','lowercase SHA-256 required')
    return value


def descriptor(value):
    return json.dumps(value,allow_nan=False,sort_keys=True,separators=(',',':'))


class CanonicalRegionScope:
    def __setattr__(self,name,value):
        if getattr(self,'_sealed',False):fail('ProtectedRegionalState','held proof state is immutable')
        object.__setattr__(self,name,value)

    def __init__(self,issuer,geometry,packet,*,root_ref,maximum_nodes,maximum_depth,maximum_patches,precision_bits):
        if issuer is not _ISSUER:fail('RegionalScopeIssuerRequired','use canonical_region_scope')
        if type(root_ref) is not int or root_ref<=0:fail('PositiveRootReferenceRequired','one explicit root reference')
        if (type(maximum_depth) is not int or maximum_depth<0 or type(maximum_patches) is not int or maximum_patches<1
                or type(precision_bits) is not int or not 1<=precision_bits<=256):
            fail('ExplicitProofBudgetRequired','depth, patch and precision budgets')
        data=packet.get('data',{})
        if (data.get('schema')!='V4SourceExpression/1' or data.get('continuous_acceptance') is not False
                or packet.get('metres_per_model_unit')!=geometry.description['metres_per_model_unit']):
            fail('SourceExpressionEnvelopeMismatch','schema, units or acceptance scope',geometry.identity)
        self._geometry=geometry;self._host=geometry.owner;self._handle=geometry.handle
        self._transport=geometry.owner.transport;self._process=self._transport.process;self._lock=geometry.owner.lock
        self._holds=geometry.owner._protected_handles
        self._identity=packet['geometry_identity'];self._source=packet['kernel_source_sha256']
        self._description=descriptor(geometry.description)
        self._expression=Expression.from_packet(data['expression'],maximum_nodes=maximum_nodes)
        self._scale=packet['metres_per_model_unit'];self._root_ref=root_ref
        self._budgets=(maximum_depth,maximum_patches,precision_bits);self._active=False
        self._sealed=True

    def verify(self,authority=None):
        held=_batch.get()
        if held is not None:
            try:
                if held[0] is not self or (authority is not None and authority is not held[1]):
                    fail('RegionalAuthorityMismatch','wrong verified batch owner',self._identity)
                held[2]();return self
            except BaseException:
                held[4]();raise
        g=self._geometry;host=self._host
        if not self._active:fail('ExpiredRegionalScope','call-scoped result cannot be reopened',self._identity)
        if (g.owner is not host or host.transport is not self._transport or
                self._transport.process is not self._process or host.lock is not self._lock or
                host._protected_handles is not self._holds or
                type(g.handle) is not int or g.handle!=self._handle or g.identity!=self._identity
                or g.description.get('kernel_source_sha256')!=self._source
                or g.description.get('metres_per_model_unit')!=self._scale
                or host.transport.closed.is_set() or host.transport.process.poll() is not None
                or not host._protected_handles.get(self._handle)):
            fail('ChangedRegionalScope','native object, units or live custody changed',self._identity)
        if authority is not None and getattr(authority,'geometry',None) is not g:
            fail('RegionalAuthorityMismatch','face owner is not the captured canonical geometry',self._identity)
        return self

    @contextmanager
    def verified_input_batch(self,authority,live,invalidate):
        """Reuse this source owner; detached expression never becomes a receipt."""
        if _batch.get() is not None:raise ValueError('NestedRegionalInputBatch')
        self.verify(authority);thread=threading.get_ident();active=True
        captured=(deepcopy(self._expression),self._scale,self._budgets,self._root_ref)
        pin=(repr(self._expression),self._scale,self._budgets,self._root_ref)
        if (repr(captured[0]),*captured[1:])!=pin:raise ValueError('ChangedRegionalCapture')
        def check():
            if not active or not self._active:raise ValueError('ExpiredRegionalInputBatch')
            if threading.get_ident()!=thread:raise ValueError('RegionalInputBatchThreadMismatch')
            if (self._transport.closed.is_set() or self._process.poll() is not None
                    or not self._holds.get(self._handle)):
                fail('ChangedRegionalScope','source lifetime or handle custody ended',self._identity)
            live()
        token=_batch.set((self,authority,check,captured,invalidate))
        try:yield
        finally:
            active=False;_batch.reset(token);self.verify(authority)
            if (repr(captured[0]),*captured[1:])!=pin:raise ValueError('ChangedPrivateRegionalInputs')
            if (repr(self._expression),self._scale,self._budgets,self._root_ref)!=pin:
                raise ValueError('ChangedRegionalBatchInputs')

    def _batch_inputs(self):
        held=_batch.get()
        if held is not None:
            if held[0] is not self:raise ValueError('DetachedRegionalInputBatch')
            held[2]();return held[3]
        self.verify();return self._expression,self._scale,self._budgets,self._root_ref

    def reject_batch(self):
        held=_batch.get()
        if held is not None:
            if held[0] is not self:raise ValueError('DetachedRegionalInputBatch')
            held[4]()

    def certify_face(self,authority,ref,triangle,contract):
        try:return self._certify_face(authority,ref,triangle,contract)
        except BaseException:
            self.reject_batch();raise

    def _certify_face(self,authority,ref,triangle,contract):
        from surface_binding import QualificationContract
        if not isinstance(contract,QualificationContract):fail('QualificationContractRequired','existing surface contract required')
        numeric=getattr(authority,'numerical_acceptance_active',None)
        if getattr(contract,'assurance','qualification')!='qualification' or callable(numeric) and numeric():
            self.reject_batch();raise ValueError('FormalQualificationRequired')
        self.verify(authority);contract.validate()
        expression,scale,budgets,root=self._batch_inputs()
        if type(ref) is not int or ref!=root or set(contract.sheet_names)!={root}:
            fail('RegionalReferenceMismatch','root field proof does not attest multiple feature branches',self._identity)
        limits=contract.batch_contract() if callable(getattr(contract,'batch_contract',None)) else contract
        depth,patches,bits=budgets
        result=certify_triangle(expression,triangle,metres_per_model_unit=scale,
            position_limit_m=limits.position_limit_m,normal_limit_rad=limits.normal_limit_rad,
            maximum_depth=depth,maximum_patches=patches,precision_bits=bits)
        self.verify(authority)
        return (result['status'],result.get('position_bound_m',math.inf),
                result.get('normal_bound_rad',math.pi),result.get('method','regional_unresolved'))

    def query_enclosure(self,point):
        self.verify()
        expression,scale,budgets,_=self._batch_inputs()
        result=query_enclosure(expression,point,scale,precision_bits=budgets[2])
        self.verify()
        return result


@contextmanager
def canonical_region_scope(geometry,*,authenticated_executable_sha256,authenticated_source_sha256,
                           root_ref,maximum_nodes,maximum_depth,maximum_patches,precision_bits):
    """Reuse immutable NTFS custody and the native owner's request lock.

    Authenticate once, detach the immutable expression once and hold the handle
    for all face queries. No per-face source hashing or native-parent replay.
    Native-query source identity must be supplied from the caller's authenticated
    build/release, not copied from an untrusted geometry description.
    """
    from immutable_state_access import guard
    from path_preserving_deduplication import file_hash
    if type(geometry) is not CanonicalGeometry or type(geometry.owner) is not CanonicalKernel:
        fail('CanonicalHandleRequired','actual canonical object and process owner required')
    binary_pin=digest(authenticated_executable_sha256);source_pin=digest(authenticated_source_sha256)
    host=geometry.owner
    command=host.transport.process.args
    if type(command) not in (tuple,list) or not command:fail('NativeExecutableRequired','direct native process command')
    binary=Path(command[0]).resolve(strict=True)
    with host.protect_handle(geometry.handle),guard(binary):
        if Path(psutil.Process(host.transport.process.pid).exe()).resolve()!=binary:
            fail('NativeExecutableMismatch','process executable differs from command')
        if file_hash(binary)!=binary_pin:fail('NativeExecutableMismatch','executable authentication failed')
        if (host.ready.get('kernel_source_sha256')!=source_pin or
                geometry.description.get('kernel_source_sha256')!=source_pin):
            fail('KernelSourceMismatch','authenticated source differs from live process')
        packet=geometry.regional_expression()
        scope=CanonicalRegionScope(_ISSUER,geometry,packet,root_ref=root_ref,maximum_nodes=maximum_nodes,
            maximum_depth=maximum_depth,maximum_patches=maximum_patches,precision_bits=precision_bits)
        object.__setattr__(scope,'_active',True)
        try:
            yield scope
            scope.verify()
            if (descriptor(geometry.description)!=scope._description or
                    descriptor(host.request(op='describe',handle=scope._handle))!=scope._description):
                fail('GeometryDescriptionMismatch','held descriptor changed',scope._identity)
        finally:object.__setattr__(scope,'_active',False)
