"""Unselected general-SDF connection to existing provider operations.

Manual sizing is a user request. Continuous face accuracy and final cell quality
remain separate acceptance obligations. This adapter grants no full admission.
"""
from dataclasses import dataclass
from contextlib import contextmanager,nullcontext
from contextvars import ContextVar
from collections import Counter
import math
import threading,time,ctypes,os
from functools import wraps
from copy import deepcopy
import types
import numpy as np
from historical_capsule_surface import Authority, provider
from prism_host_metric import PrismMetricPolicy
from surface_metric_arithmetic import intersect_tangent_metric
from sdf_cfd_mesher_v4.sdf.regional import Interval, region, upper_float, filtered_regions, numerical_point
from sdf_cfd_mesher_v4.sdf.operations import ProtectedQueries
from procedural_authorities import curvature_size, tangent_metric, unit
from surface_metric_arithmetic import preserve_geometry as _preserve_geometry
from shape_aware_metric import ShapeMetricPolicy, apply_shape_metric


# Retained V3 normal-map construction policy, in SI coordinates. These are
# proposal controls, not a continuous curvature or final-normal certificate.
NORMAL_MAP_STEP_M = 1.0e-5
NORMAL_MAP_CURVATURE_RESERVE = 2.50


def _normal_map_basis(normal):
    u = unit(np.cross(normal, np.array([0., 0., 1.])) if abs(normal[2]) < .9
             else np.cross(normal, np.array([0., 1., 0.])))
    return u, unit(np.cross(normal, u))


def _normal_variation_data(normal, u, v, probe_normals, requirements):
    du = (probe_normals[0] - probe_normals[1]) / (2 * NORMAL_MAP_STEP_M)
    dv = (probe_normals[2] - probe_normals[3]) / (2 * NORMAL_MAP_STEP_M)
    shape = np.array([[np.dot(u, du), np.dot(u, dv)],
                      [np.dot(v, du), np.dot(v, dv)]])
    values, vectors = np.linalg.eigh(.5 * (shape + shape.T))
    directions = [unit(vectors[0, j]*u + vectors[1, j]*v) for j in range(2)]
    lengths = [min(requirements.size_m,
                   curvature_size(NORMAL_MAP_CURVATURE_RESERVE*abs(float(k)), 1))
               for k in values]
    if min(lengths) < requirements.minimum_size_m:
        raise ValueError('ManualNormalVariationSizeConflict')
    metric = tangent_metric(directions[0], lengths[0], directions[1], lengths[1],
                            normal, min(lengths))
    # Preserve the explicit user metric in exact binary64 arithmetic, as the
    # existing prism composition preserves its required geometry metric.
    metric, _ = _preserve_geometry(metric, np.eye(3)/requirements.size_m**2)
    principal=np.concatenate((*directions,
        NORMAL_MAP_CURVATURE_RESERVE*np.abs(values),normal))
    return metric,principal


def _normal_variation_tensor(normal,u,v,probe_normals,requirements):
    return _normal_variation_data(normal,u,v,probe_normals,requirements)[0]


def _principal_record(data):
    return dict(directions=(data[:3].copy(),data[3:6].copy()),
                curvatures=tuple(float(v) for v in data[6:8]),normal=data[8:11].copy())


def validate_shape_policy(policy):
    if (type(policy) is not ShapeMetricPolicy or
            type(policy.normal_qualification_fraction) not in (float,int) or
            type(policy.mmgs_response_reserve) not in (float,int) or
            not math.isfinite(policy.normal_qualification_fraction) or
            not math.isfinite(policy.mmgs_response_reserve) or
            not 0<policy.normal_qualification_fraction<=1 or
            policy.mmgs_response_reserve<=0 or policy.curvature_reserve_is_embedded is not True):
        raise ValueError('InvalidExplicitShapeMetricPolicy')

_surface_queries=ContextVar('general_surface_queries',default=None)
_construction=ContextVar('general_surface_construction',default=None)
_verified=ContextVar('general_verified_operation',default=None)
_acceptance=ContextVar('general_final_acceptance',default=None)
_prism_queries=ContextVar('general_prism_queries',default=None)


def verified_query(method):
    """One custody owner around an operation and all its nested consumers."""
    @wraps(method)
    def call(self,*args,**kwargs):
        kind={'project_sheet':'projection','exact_normal':'normal',
              'implicit_value':'residual','composed_metric':'metric',
              'project_sheet_batch':'projection','composed_metrics_batch':'metric',
              'construction_normals_batch':'normal'}.get(method.__name__)
        with self.verified_operation(kind):return method(self,*args,**kwargs)
    return call

def validate_prism_policy(policy):
    # These are the existing metric policy domains, not new mesh gates.
    def finite(value):
        return type(value) in (float,int) and math.isfinite(value)
    height=policy.first_height_m
    thickness=policy.minimum_owned_layer_thickness_m
    target=policy.foundation_qualification_target
    response=policy.metric_response_factor
    if (not finite(height) or height<=0 or
            thickness is not None and (not finite(thickness) or thickness<=0) or
            not finite(target) or not 0<target<1 or
            not finite(response) or not 0<response<=1 or
            type(policy.reference_shape) is not str or
            policy.reference_shape!='right_isosceles_parallel_trajectory'):
        raise ValueError('InvalidExplicitPrismMetricPolicy')


@dataclass(frozen=True)
class ManualRequirements:
    size_m: float
    minimum_size_m: float
    maximum_size_m: float
    projection_tolerance_m: float
    maximum_projection_displacement_m: float
    maximum_projection_iterations: int
    normal_estimate_bound_rad: float

    def validate(self):
        values=(self.size_m,self.minimum_size_m,self.maximum_size_m,self.projection_tolerance_m,
                self.maximum_projection_displacement_m,self.normal_estimate_bound_rad)
        if any(type(v) not in (float,int) or not math.isfinite(v) or v<=0 for v in values):
            raise ValueError('ExplicitFiniteManualRequirementsRequired')
        if not self.minimum_size_m<=self.size_m<=self.maximum_size_m:
            raise ValueError('ManualSurfaceSizeConflict')
        if (self.projection_tolerance_m!=2e-12 or self.maximum_projection_displacement_m<2e-12
                or type(self.maximum_projection_iterations) is not int or self.maximum_projection_iterations<1
                or self.normal_estimate_bound_rad>=2):
            raise ValueError('UnsupportedProjectionOrNormalRequirements')


class GeneralSdfAuthority(Authority):
    """Source-bound query/metric roles; missing ownership stays explicit.

    exact_normal is the retained provider's method name. The returned vector is
    a binary64 estimate with a source-branch cone proof at the supplied bound;
    it is not relabelled as an exact floating-point derivative or native query.
    """
    name='canonical_general_sdf_candidate'

    def __setattr__(self,name,value):
        if getattr(self,'_sealed',False):raise ValueError('ProtectedGeneralSdfAuthority')
        object.__setattr__(self,name,value)

    def __init__(self,scope,requirements,prism_policy,shape_policy=None):
        if type(requirements) is not ManualRequirements or type(prism_policy) is not PrismMetricPolicy:
            raise ValueError('ExplicitManualAndPrismPoliciesRequired')
        requirements.validate()
        validate_prism_policy(prism_policy)
        shape_policy=ShapeMetricPolicy() if shape_policy is None else shape_policy
        validate_shape_policy(shape_policy)
        self.queries=ProtectedQueries(scope)
        self.geometry=scope._geometry
        self.geometry_identity=scope._identity
        self._root_ref=scope._root_ref
        self._requirements=requirements
        self._prism_policy=prism_policy
        self._shape_policy=shape_policy
        self._sealed=True

    @property
    def sheet_names(self):return {self._root_ref:'canonical.root_zero_field'}
    @property
    def curve_names(self):return {}
    @property
    def corner_names(self):return {}

    def verify(self):
        held=_verified.get()
        if held is not None and held[0] is self:
            held[1]();return True
        self.queries._scope.verify(self)
        self._requirements.validate()
        validate_prism_policy(self._prism_policy)
        validate_shape_policy(self._shape_policy)
        return True

    @contextmanager
    def verified_operation(self,kind=None):
        query=_surface_queries.get();held=_verified.get()
        if _prism_queries.get() is self and (_construction.get() is not None or _acceptance.get() is not None):
            raise ValueError('SurfaceQueriesCannotQualifyPrisms')
        if held is not None:
            try:
                if held[0] is not self:raise ValueError('DetachedVerifiedOperation')
                held[1]()
                yield
            except BaseException:
                held[5]();raise
            return
        if query is None or query[0] is not self or (_construction.get() is None and _prism_queries.get() is not self):
            yield;return  # No protected input scope, no verification reuse.
        query[3]();thread=threading.get_ident();active=True
        def live():
            if not active:raise ValueError('ExpiredVerifiedOperation')
            if threading.get_ident()!=thread:raise ValueError('VerifiedOperationThreadMismatch')
            query[6]()  # Query-scope liveness; no duplicate full authentication.
            if _prism_queries.get() is self:
                query[8]()  # Retain per-call arithmetic liveness.
                if not self.stock_prism_queries():raise ValueError('ChangedPrismQueryMethods')
            if _prism_queries.get() is self and (_construction.get() is not None or _acceptance.get() is not None):
                raise ValueError('SurfaceQueriesCannotQualifyPrisms')
        # Canonical captured state contains only immutable native scalars.
        # Returned policies are fresh objects, never these canonical values.
        def invalidate():
            query[5]()
            construction=_construction.get()
            if construction is not None:
                if construction[0] is not self:raise ValueError('DetachedConstructionScope')
                construction[3]()
        token=_verified.set((self,live,*query[7],self._root_ref,invalidate))
        completed=False
        try:
            # Reuse the existing native/source batch owner for prism predicates.
            # Private expression capture and live native custody stay mandatory.
            batch=(self.queries._scope.verified_input_batch(self,live,invalidate)
                   if _prism_queries.get() is self else nullcontext())
            with batch:
                yield
            completed=True
        except BaseException:
            invalidate();raise  # Neither cache may retain rejected values.
        finally:
            active=False;_verified.reset(token)
            try:query[3]()
            except BaseException:
                if completed and kind is not None:query[4](kind)
                invalidate();raise

    def manual_controls(self):
        held=_verified.get()
        if held is not None and held[0] is self:held[1]();return ManualRequirements(*held[2])
        return self._requirements

    def prism_controls(self):
        held=_verified.get()
        if held is not None and held[0] is self:held[1]();return PrismMetricPolicy(*held[3])
        return self._prism_policy

    def shape_controls(self):
        held=_verified.get();query=_surface_queries.get()
        if query is not None and query[0] is self:
            if held is not None and held[0] is self:held[1]()
            else:query[3]()
            return ShapeMetricPolicy(*query[10])
        self.verify();policy=self._shape_policy
        return ShapeMetricPolicy(policy.normal_qualification_fraction,
            policy.mmgs_response_reserve,policy.curvature_reserve_is_embedded)

    def acceptance_active(self,contract=None):
        held=_acceptance.get()
        if held is None:return False
        if held[0] is not self or contract is not None and held[1] is not contract:
            raise ValueError('DetachedFinalAcceptance')
        self.verify();return True

    def numerical_acceptance_active(self):
        return self.acceptance_active() and _acceptance.get()[2]=='numerical'

    def acceptance_contract(self,contract):
        if not self.acceptance_active(contract):return None
        # Fresh nested containers. Consumers cannot mutate the canonical pin.
        import json
        return json.loads(_acceptance.get()[3])

    def acceptance_source(self,contract):
        if not self.acceptance_active(contract):return None
        import json
        target,topology,budget=_acceptance.get()[5]
        return json.loads(target),json.loads(topology),budget,_acceptance.get()[2]

    def reject_acceptance(self):
        held=_verified.get()
        if held is not None:
            if held[0] is not self:raise ValueError('DetachedFinalAcceptance')
            held[5]()

    @contextmanager
    def acceptance_scope(self,contract,arrays):
        """One final owner; custody reuse never upgrades numerical evidence."""
        from general_source_contract import GeneralSourceQualification,array_binding,snapshot
        from dataclasses import asdict
        from sdf_cfd_mesher_v4.common.authority import frozen
        from sdf_cfd_mesher_v4.sdf.regional import invalidate_filtered_cache
        if type(contract) is not GeneralSourceQualification:raise ValueError('GeneralSourceContractRequired')
        query=_surface_queries.get()
        if query is None or query[0] is not self:raise ValueError('ProtectedQueryScopeRequired')
        if _acceptance.get() is not None or _verified.get() is not None or self.construction_active():
            raise ValueError('NestedOrConstructionFinalAcceptance')
        query[3]();contract.verify(self)
        original=tuple(np.asarray(v) for v in arrays)
        pins=tuple(array_binding(v) for v in original)
        values=tuple(frozen(v,v.dtype.str) for v in original)
        if tuple(array_binding(v) for v in values)!=pins:raise ValueError('ChangedFinalAcceptanceInputs')
        policy=snapshot(asdict(contract));mode=contract.assurance
        contract_pin=contract.custody_identity()
        source=(contract._target,contract._topology,tuple(contract._coverage_budget))
        cache={};thread=threading.get_ident();active=True
        def live():
            if not active:raise ValueError('ExpiredFinalAcceptance')
            if threading.get_ident()!=thread:raise ValueError('FinalAcceptanceThreadMismatch')
            scope=self.queries._scope
            if (not scope._active or scope._transport.closed.is_set() or scope._process.poll() is not None
                    or not scope._holds.get(scope._handle)):
                raise ValueError('ChangedFinalAcceptanceSourceLifetime')
            query[8]()  # Arithmetic + source-query lifetime, also on cache hits.
        def invalidate():
            query[5]();cache.clear();invalidate_filtered_cache()
        try:
            with self.queries._scope.verified_input_batch(self,live,invalidate):
                verified=_verified.set((self,live,*query[7],self._root_ref,invalidate))
                accepted=_acceptance.set((self,contract,mode,policy,cache,source))
                try:
                    yield values
                    live()
                    if tuple(array_binding(v) for v in values)!=pins:
                        raise ValueError('ChangedPrivateFinalAcceptanceInputs')
                except BaseException:
                    invalidate();raise
                finally:
                    _acceptance.reset(accepted);_verified.reset(verified)
            query[3]();contract.verify(self)
            if contract.custody_identity()!=contract_pin or snapshot(asdict(contract))!=policy:
                raise ValueError('ChangedFinalAcceptanceContract')
            if tuple(array_binding(v) for v in original)!=pins:
                raise ValueError('ChangedFinalAcceptanceInputs')
        except BaseException:
            invalidate();raise
        finally:active=False;cache.clear()

    def _point(self,ref,p):
        self.verify()
        held=_verified.get();root=held[4] if held is not None and held[0] is self else self._root_ref
        if type(ref) not in (int,np.int64,np.int32) or int(ref)!=root:
            raise ValueError('UnknownSourceRootReference')
        value=np.asarray(p,dtype=float)
        if value.shape!=(3,) or not np.isfinite(value).all():raise ValueError('FiniteSourcePointRequired')
        return tuple(float(v) for v in value)

    @contextmanager
    def query_scope(self):
        """Reuse phase-separated query values only within live source custody.

        The cache is closure-local. It stores immutable bytes/scalars, never
        caller arrays or acceptance receipts. Every hit checks source liveness
        and unchanged controls. No native/real-target equivalence is assumed.
        """
        if _surface_queries.get() is not None:raise ValueError('NestedGeneralQueryScope')
        self.verify();scope=self.queries._scope;queries=self.queries
        def controls():
            # All policy members are scalar and validated before each check.
            # Capture them together without recursive dataclass deep copies.
            # Keep every current member and the same tuple equality semantics.
            r=self._requirements;p=self._prism_policy
            return ((r.size_m,r.minimum_size_m,r.maximum_size_m,
                     r.projection_tolerance_m,r.maximum_projection_displacement_m,
                     r.maximum_projection_iterations,r.normal_estimate_bound_rad),
                    (p.first_height_m,p.foundation_qualification_target,
                     p.minimum_owned_layer_thickness_m,p.metric_response_factor,
                     p.reference_shape),
                    scope._identity,scope._source,scope._root_ref,scope._scale,
                    scope._budgets,repr(scope._expression),
                    (self._shape_policy.normal_qualification_fraction,
                     self._shape_policy.mmgs_response_reserve,
                     self._shape_policy.curvature_reserve_is_embedded))
        captured=controls();thread=threading.get_ident();active=True
        # Reuse requires the same materialization/error environment as compute.
        # Existing custody is Windows-only; qualify this read for Windows x64.
        if os.name!='nt' or ctypes.sizeof(ctypes.c_void_p)!=8:
            raise ValueError('UnqualifiedGeneralQueryArithmeticCustody')
        crt=ctypes.CDLL('ucrtbase.dll');read_control=crt._controlfp_s
        read_control.argtypes=(ctypes.POINTER(ctypes.c_uint),ctypes.c_uint,ctypes.c_uint)
        read_control.restype=ctypes.c_int
        def arithmetic():
            word=ctypes.c_uint()
            if read_control(ctypes.byref(word),0,0)!=0:
                raise ValueError('GeneralQueryArithmeticReadFailure')
            return word.value,tuple(sorted(np.geterr().items())),np.geterrcall()
        captured_arithmetic=arithmetic()
        cache={};counts=Counter();seconds=Counter();payload=0
        def record_batch(kind,points,evaluations,elapsed):
            arithmetic_live()
            counts['construction_'+kind+'_batch_calls']+=1
            counts['construction_'+kind+'_batch_points']+=points
            counts['construction_'+kind+'_batch_evaluations']+=evaluations
            seconds['construction_'+kind+'_batch']+=elapsed
        def invalidate():
            nonlocal payload
            from sdf_cfd_mesher_v4.sdf.regional import invalidate_filtered_cache
            cache.clear();payload=0;invalidate_filtered_cache()
        def boundary_failure(kind):
            phase='construction' if _construction.get() is not None else 'acceptance'
            counts[kind+'_failures']+=1;counts[phase+'_'+kind+'_failures']+=1
        def live():
            if not active:raise ValueError('ExpiredGeneralQueryScope')
            if threading.get_ident()!=thread:raise ValueError('GeneralQueryThreadMismatch')
        def arithmetic_live():
            live();now=arithmetic()
            if now[:2]!=captured_arithmetic[:2] or now[2] is not captured_arithmetic[2]:
                raise ValueError('ChangedGeneralQueryArithmeticContext')
        def check():
            if _prism_queries.get() is self and not self.stock_prism_queries():
                invalidate();raise ValueError('ChangedPrismQueryMethods')
            nested=_verified.get()
            if nested is not None and nested[0] is self:
                nested[1]();counts['nested_checks_reused']+=1;return
            counts['full_checks']+=1
            live();self.verify()
            now=arithmetic()
            if now[:2]!=captured_arithmetic[:2] or now[2] is not captured_arithmetic[2]:
                raise ValueError('ChangedGeneralQueryArithmeticContext')
            if self.queries is not queries or self.queries._scope is not scope or controls()!=captured:
                raise ValueError('ChangedGeneralQueryControls')
        def query(kind,ref,value,compute):
            nonlocal payload
            check();phase=('construction' if self.construction_active() else
                'numerical_acceptance' if self.numerical_acceptance_active() else 'acceptance')
            key=(kind,int(ref),np.asarray(value,dtype='<f8').tobytes(),phase)
            counts[kind+'_requests']+=1;counts[phase+'_'+kind+'_requests']+=1
            if key in cache:
                counts[kind+'_hits']+=1;counts[phase+'_'+kind+'_hits']+=1;stored=cache[key]
            else:
                if compute is None:
                    check()
                    return None  # Protected lookup only; caller supplies a cold batch.
                counts[kind+'_computations']+=1;counts[phase+'_'+kind+'_computations']+=1;start=time.perf_counter()
                try:
                    result=compute();check()
                    stored=float(result) if kind in ('residual','position_bound') else np.asarray(result,dtype='<f8').tobytes()
                except BaseException:
                    counts[kind+'_failures']+=1;counts[phase+'_'+kind+'_failures']+=1;raise
                finally:
                    elapsed=time.perf_counter()-start;seconds[kind]+=elapsed;seconds[phase+'_'+kind]+=elapsed
                cache[key]=stored;payload+=len(key[2])+(8 if kind in ('residual','position_bound') else len(stored))
            # Materialize private arrays for each caller. Mutation cannot reach cache.
            result=stored if kind in ('residual','position_bound') else np.frombuffer(stored,dtype='<f8').copy()
            if kind in ('metric','geometry_metric','prism_metric'):result=result.reshape(3,3)
            check()
            return result
        def snapshot():
            live()
            return dict(schema='V4GeneralSurfaceQueryReuse/1',counts=dict(counts),
                        computation_seconds=dict(seconds),entries=len(cache),logical_payload_bytes=payload,
                        role='Protected phase-separated query values; construction values are proposals only; no cross-phase receipt reuse',
                        native_equivalence=False,arithmetic_control_word=captured_arithmetic[0],
                        numpy_error_modes=dict(captured_arithmetic[1]))
        try:
            with filtered_regions(check) as filter_stats:
                old_snapshot=snapshot
                def snapshot():return dict(old_snapshot(),regional_filter=filter_stats())
                token=_surface_queries.set((self,query,snapshot,check,boundary_failure,invalidate,live,captured[:2],arithmetic_live,record_batch,captured[-1]))
                try:
                    yield snapshot
                    check()
                finally:cache.clear();_surface_queries.reset(token)
        finally:active=False

    def stock_prism_queries(self):
        # Cross-handoff reuse must not suppress custom/stateful callback calls.
        normal=self.exact_normal;numerical=self._numerical_normal
        return (type(self) is GeneralSdfAuthority and
                type(normal) is types.MethodType and normal.__self__ is self and
                normal.__func__ is _STANDARD_EXACT_NORMAL and
                type(numerical) is types.MethodType and numerical.__self__ is self and
                numerical.__func__ is _STANDARD_NUMERICAL_NORMAL)

    @contextmanager
    def prism_query_scope(self):
        """Share existing qualified queries inside one live prism operation.

        Construction and numerical-surface proposals cannot enter this scope.
        Nested calls join only the same protected authority and arithmetic owner.
        No cache or acceptance receipt survives the outer context.
        """
        if type(self) is not GeneralSdfAuthority:
            raise ValueError('PrismQueryOwnerRequired')
        if self.construction_active() or self.acceptance_active():
            raise ValueError('SurfaceQueriesCannotQualifyPrisms')
        if not self.stock_prism_queries():raise ValueError('StockPrismQueryMethodsRequired')
        held=_surface_queries.get();prism=_prism_queries.get()
        if held is not None and held[0] is not self:
            raise ValueError('DetachedPrismQueryScope')
        if prism is not None and prism is not self:
            raise ValueError('DetachedPrismQueryOwner')
        with (self.query_scope() if held is None else nullcontext(held[2])) as statistics:
            query=_surface_queries.get();query[3]()
            token=_prism_queries.set(self)
            try:
                yield statistics
                query[3]()
            except BaseException:
                query[5]();raise
            finally:_prism_queries.reset(token)

    def _query(self,kind,ref,value,compute):
        held=_surface_queries.get()
        return compute() if held is None or held[0] is not self else held[1](kind,ref,value,compute)

    def _cached_query(self,kind,ref,value):
        held=_surface_queries.get()
        return None if held is None or held[0] is not self else held[1](kind,ref,value,None)

    def construction_active(self):
        held=_construction.get()
        if held is None:return False
        if held[0] is not self:raise ValueError('DetachedConstructionScope')
        held[1]();return True

    @contextmanager
    def construction_scope(self):
        """Temporary point proposals under the existing protected source scope."""
        held=_surface_queries.get()
        if held is None or held[0] is not self:raise ValueError('ProtectedQueryScopeRequired')
        if _construction.get() is not None:raise ValueError('NestedConstructionScope')
        self.verify();thread=threading.get_ident();active=True;cache={}
        expression=deepcopy(self.queries._scope._expression);scale=self.queries._scope._scale
        def check():
            if not active:raise ValueError('ExpiredConstructionScope')
            if threading.get_ident()!=thread:raise ValueError('ConstructionThreadMismatch')
            self.verify()
            held[3]()
        def sample(p):
            check();key=np.asarray(p,dtype='<f8').tobytes()
            if key not in cache:
                result=numerical_point(expression,p,scale)
                if len(cache)>=8192:cache.clear()
                cache[key]=(result.value,tuple(result.gradient))
            check();return cache[key]
        def sample_many(points):
            from sdf_cfd_mesher_v4.sdf.regional import numerical_points
            check();held[8]()
            result=numerical_points(expression,points,scale)
            held[8]();check();return result
        def live():
            if not active:raise ValueError('ExpiredConstructionScope')
            if threading.get_ident()!=thread:raise ValueError('ConstructionThreadMismatch')
            held[3]()
        def invalidate():
            if not active:raise ValueError('ExpiredConstructionScope')
            if threading.get_ident()!=thread:raise ValueError('ConstructionThreadMismatch')
            cache.clear()
        check()
        token=_construction.set((self,live,sample,invalidate,sample_many))
        try:
            yield
            check()
        finally:
            active=False;cache.clear();_construction.reset(token)

    def _numerical_sample(self,value):
        held=_construction.get()
        if held is not None:
            if held[0] is not self:raise ValueError('DetachedConstructionScope')
            return held[2](value)
        final=_acceptance.get()
        if final is not None:
            if final[0] is not self:raise ValueError('DetachedFinalAcceptance')
            self.verify();key=np.asarray(value,dtype='<f8').tobytes();cache=final[4]
            if key not in cache:
                expression,scale,_,_=self.queries._scope._batch_inputs()
                result=numerical_point(expression,value,scale)
                cache[key]=(result.value,tuple(result.gradient))
            self.verify();return cache[key]
        result=numerical_point(self.queries._scope._expression,value,self.queries._scope._scale)
        return result.value,result.gradient

    def _numerical_normal(self,value):
        _,gradient=self._numerical_sample(value)
        length=math.sqrt(sum(v*v for v in gradient))
        if not math.isfinite(length) or length<=0:raise ValueError('ConstructionNormalSingular')
        result=np.asarray([v/length for v in gradient],dtype=float)
        result[result==0]=0.  # Match existing rational materialization of zero.
        return result

    def _numerical_project(self,value):
        r=self.manual_controls();start=np.asarray(value,dtype=float);p=start.copy()
        for _ in range(r.maximum_projection_iterations):
            field,gradient=self._numerical_sample(p);g=np.asarray(gradient);g2=float(g@g)
            if not math.isfinite(g2) or g2<=0:raise ValueError('ConstructionProjectionGradientSingular')
            if abs(field)/math.sqrt(g2)<=r.projection_tolerance_m:
                p[p==0]=0.  # Same zero materialization as the proved projector.
                return p
            p=p-field*g/g2
            if not np.isfinite(p).all() or np.linalg.norm(p-start)>r.maximum_projection_displacement_m:
                raise ValueError('ConstructionProjectionDisplacementExceeded')
        raise ValueError('ConstructionProjectionIterationBudgetExceeded')

    def _construction_batch_inputs(self,points,refs):
        """Private, bit-bound proposals inside the existing construction owner."""
        from sdf_cfd_mesher_v4.common.authority import frozen
        from general_source_contract import array_binding
        if not self.construction_active():raise ValueError('ProtectedConstructionBatchRequired')
        originals=(np.asarray(points),np.asarray(refs))
        pins=tuple(array_binding(v) for v in originals)
        private=tuple(frozen(v,v.dtype.str) for v in originals)
        if tuple(array_binding(v) for v in private)!=pins:raise ValueError('ChangedConstructionBatchInputs')
        x,v=private
        if (x.dtype!=np.dtype('float64') or x.ndim!=2 or x.shape[1]!=3 or not len(x)
                or not np.isfinite(x).all() or v.dtype.kind not in 'iu' or v.shape!=(len(x),)
                or set(v)!={self._root_ref}):raise ValueError('ConstructionPointReferencesRequired')
        def check():
            if not self.construction_active():raise ValueError('ProtectedConstructionBatchRequired')
            _surface_queries.get()[8]()
            if tuple(array_binding(w) for w in originals)!=pins:raise ValueError('ChangedConstructionBatchInputs')
        return x,v,check

    @verified_query
    def project_sheet_batch(self,ref,points):
        """Batch the same scalar Newton projector; never a final certificate."""
        self._point(ref,(0.,0.,0.))
        original=np.asarray(points)
        refs=np.full(len(original),int(ref),dtype=np.int64) if original.ndim else np.empty(0,dtype=np.int64)
        x,_,check=self._construction_batch_inputs(original,refs)
        sample=_construction.get()[4];r=self.manual_controls()
        output=np.empty_like(x);evaluations=0;start_time=time.perf_counter()
        for lo in range(0,len(x),4096):
            check();initial=x[lo:lo+4096];p=initial.copy();active=np.arange(len(p))
            for iteration in range(r.maximum_projection_iterations):
                jet=sample(p[active]);evaluations+=len(active);gradient=np.column_stack(jet.gradient)
                # Preserve the scalar projector's fresh-array dot reduction.
                def scalar_dot(g):
                    g=g.copy();return float(g@g)
                g2=np.array([scalar_dot(g) for g in gradient])
                if not np.isfinite(g2).all() or np.any(g2<=0):raise ValueError('ConstructionProjectionGradientSingular')
                done=np.abs(jet.value)/np.array([math.sqrt(v) for v in g2])<=r.projection_tolerance_m
                p[active[done]]=np.where(p[active[done]]==0,0.,p[active[done]])
                remaining=~done
                if not np.any(remaining):break
                active=active[remaining]
                updated=p[active]-jet.value[remaining,None]*gradient[remaining]/g2[remaining,None]
                if (not np.isfinite(updated).all() or
                        any(np.linalg.norm(q-initial[i])>r.maximum_projection_displacement_m for q,i in zip(updated,active))):
                    raise ValueError('ConstructionProjectionDisplacementExceeded')
                p[active]=updated
                if iteration+1==r.maximum_projection_iterations:raise ValueError('ConstructionProjectionIterationBudgetExceeded')
            output[lo:lo+len(p)]=p;check()
        check();_surface_queries.get()[9]('projection',len(x),evaluations,time.perf_counter()-start_time)
        return output

    @verified_query
    def construction_normals_batch(self,points,refs):
        """Same numerical normal proposals in protected construction only."""
        x,_,check=self._construction_batch_inputs(points,refs)
        sample=_construction.get()[4];output=np.empty_like(x);start=time.perf_counter()
        for lo in range(0,len(x),4096):
            check();jet=sample(x[lo:lo+4096]);gradient=np.column_stack(jet.gradient)
            for i,g in enumerate(gradient,lo):
                # Preserve _numerical_normal's Python scalar reduction and zeros.
                length=math.sqrt(sum(float(v)*float(v) for v in g))
                if not math.isfinite(length) or length<=0:raise ValueError('ConstructionNormalSingular')
                normal=np.array([float(v)/length for v in g]);normal[normal==0]=0.
                output[i]=normal
            check()
        check();_surface_queries.get()[9]('normal',len(x),len(x),time.perf_counter()-start)
        return output

    @verified_query
    def geometry_metrics_batch(self,points,refs):
        """Batch the retained projected normal-map construction; no proof reuse."""
        x,ref,check=self._construction_batch_inputs(points,refs)
        normals=self.construction_normals_batch(x,ref)
        for i in range(len(x)):
            normals[i]=self._query('normal',ref[i],x[i],lambda i=i:normals[i])
        r=self.manual_controls();output=np.empty((len(x),3,3));start=time.perf_counter()
        # Four projected probes per vertex, bounded by the existing4096 batch.
        for lo in range(0,len(x),1024):
            check();p=x[lo:lo+1024];n=normals[lo:lo+len(p)]
            bases=[_normal_map_basis(value) for value in n]
            probes=np.array([q + NORMAL_MAP_STEP_M*d for q,(u,v) in zip(p,bases)
                             for d in (u,-u,v,-v)])
            probe_refs=np.repeat(ref[lo:lo+len(p)],4)
            projected=self.project_sheet_batch(int(ref[lo]),probes)
            sampled=self.construction_normals_batch(projected,probe_refs).reshape(-1,4,3)
            for i,(normal,(u,v),neighbors) in enumerate(zip(n,bases,sampled),lo):
                metric,principal=_normal_variation_data(normal,u,v,neighbors,r)
                self.validate_composed_metric(metric)
                output[i]=self._query('geometry_metric',ref[i],x[i],lambda metric=metric:metric)
                self._query('geometry_principal',ref[i],x[i],lambda principal=principal:principal)
            check()
        check();_surface_queries.get()[9]('normal_variation',len(x),4*len(x),time.perf_counter()-start)
        return output,normals

    @verified_query
    def composed_metrics_batch(self,points,refs):
        """Geometry, prism majorant and retained generic shape conditioning."""
        x,_,check=self._construction_batch_inputs(points,refs)
        bases,normals=self.geometry_metrics_batch(x,refs);policy=self.prism_controls()
        output=np.empty((len(x),3,3));start_time=time.perf_counter()
        for i,(base,normal) in enumerate(zip(bases,normals)):
            result,_=intersect_tangent_metric(base,normal,policy,
                preserve_required_geometry=True,include_reference_diagnostics=False)
            self.validate_composed_metric(result)
            result=self._query('prism_metric',refs[i],x[i],lambda result=result:result)
            principal=self.principal_data(x[i],refs[i])
            result,_=self.finish_shape(result,principal,self.shape_controls())
            self.validate_composed_metric(result)
            output[i]=self._query('metric',refs[i],x[i],lambda result=result:result)
        check();_surface_queries.get()[9]('metric',len(x),0,time.perf_counter()-start_time)
        return output

    @verified_query
    def numerical_surface_samples(self,x,f,fr,vr,centres):
        """One final-owner batch of the unchanged shared surface sample sites."""
        from sdf_cfd_mesher_v4.sdf.regional import numerical_points
        from sdf_cfd_mesher_v4.common.authority import frozen
        from general_source_contract import array_binding
        if not self.numerical_acceptance_active():raise ValueError('NumericalFinalAcceptanceRequired')
        originals=tuple(np.asarray(v) for v in (x,f,fr,vr,centres))
        pins=tuple(array_binding(v) for v in originals)
        private=tuple(frozen(v,v.dtype.str) for v in originals)
        if tuple(array_binding(v) for v in private)!=pins:raise ValueError('ChangedNumericalSampleInputs')
        x,f,fr,vr,centres=private
        if (x.dtype!=np.dtype('float64') or x.ndim!=2 or x.shape[1]!=3 or not len(x) or
                f.ndim!=2 or f.shape[1]!=3 or f.dtype.kind not in 'iu' or not len(f) or
                np.any(f<0) or np.any(f>=len(x)) or not np.isfinite(x).all() or
                fr.shape!=(len(f),) or vr.shape!=(len(x),) or fr.dtype.kind not in 'iu' or vr.dtype.kind not in 'iu' or
                set(fr)!={self._root_ref} or set(vr)!={self._root_ref} or
                centres.dtype!=np.dtype('float64') or centres.shape!=(len(f),3) or not np.isfinite(centres).all() or
                not np.array_equal(centres,x[f].mean(1))):
            raise ValueError('GeneralRootReferenceCoverageRequired')
        edges,inverse=np.unique(np.sort(np.concatenate((f[:,[0,1]],f[:,[1,2]],f[:,[2,0]])),axis=1),axis=0,return_inverse=True)
        sites=np.concatenate((x,x[edges].mean(1),centres))
        sample_faces=np.column_stack((f,inverse.reshape(3,len(f)).T+len(x),np.arange(len(f))+len(x)+len(edges)))
        position=np.empty(len(sites));normals=np.empty_like(sites);field=np.empty(len(sites))
        requirements=self.manual_controls()
        expression,scale,_,_=self.queries._scope._batch_inputs()
        start_time=time.perf_counter();evaluated=0;iterations=0
        for lo in range(0,len(sites),4096):
            self.verify();initial=sites[lo:lo+4096];p=initial.copy()
            jet=numerical_points(expression,p,scale);evaluated+=len(p)
            gradient=np.column_stack(jet.gradient);field[lo:lo+len(p)]=jet.value
            # Keep the scalar summation order used by _numerical_normal.
            length=np.array([math.sqrt(sum(float(v)*float(v) for v in g)) for g in gradient])
            if not np.isfinite(length).all() or np.any(length<=0):raise ValueError('ConstructionNormalSingular')
            normal=np.array([[float(v)/float(size) for v in g] for g,size in zip(gradient,length)]);normal[normal==0]=0.
            normals[lo:lo+len(p)]=normal
            active=np.arange(len(p))
            for iteration in range(requirements.maximum_projection_iterations):
                self.verify();iterations+=1
                # Keep the existing scalar dot/sqrt/norm reductions. Batch the
                # expression algebra without moving projection/gate boundaries.
                def scalar_dot(g):
                    g=g.copy();return float(g@g)
                g2=np.array([scalar_dot(g) for g in gradient])
                if not np.isfinite(g2).all() or np.any(g2<=0):raise ValueError('ConstructionProjectionGradientSingular')
                done=np.abs(jet.value)/np.array([math.sqrt(v) for v in g2])<=requirements.projection_tolerance_m
                p[active[done]]=np.where(p[active[done]]==0,0.,p[active[done]])
                remaining=~done
                if not np.any(remaining):break
                active=active[remaining]
                updated=p[active]-jet.value[remaining,None]*gradient[remaining]/g2[remaining,None]
                if (not np.isfinite(updated).all() or
                        any(np.linalg.norm(q-initial[i])>requirements.maximum_projection_displacement_m for q,i in zip(updated,active))):
                    raise ValueError('ConstructionProjectionDisplacementExceeded')
                p[active]=updated
                if iteration+1==requirements.maximum_projection_iterations:
                    raise ValueError('ConstructionProjectionIterationBudgetExceeded')
                jet=numerical_points(expression,updated,scale);evaluated+=len(updated)
                gradient=np.column_stack(jet.gradient)
            position[lo:lo+len(p)]=[np.linalg.norm(q-origin) for q,origin in zip(p,initial)]
            self.verify()
        self.verify()
        if tuple(array_binding(v) for v in originals)!=pins:raise ValueError('ChangedNumericalSampleInputs')
        return dict(position=position,normals=normals,field=field,sample_faces=sample_faces,
            vertex_count=len(x),edge_count=len(edges),sample_count=len(sites),
            batch_statistics=dict(sites=len(sites),point_evaluations=evaluated,projection_batch_iterations=iterations,
                maximum_batch_points=4096,seconds=time.perf_counter()-start_time,
                role='Final numerical samples only; no proof or cross-phase reuse'))

    @verified_query
    def qualify_final_points(self,coordinates,refs):
        """Existing final owner proves actual bits; never moves final points."""
        if self.construction_active():raise ValueError('ConstructionCannotQualifyMesh')
        if self.numerical_acceptance_active():raise ValueError('FormalQualificationRequired')
        from sdf_cfd_mesher_v4.common.authority import array_hash,frozen
        from general_source_contract import array_binding
        original_x=np.asarray(coordinates);original_v=np.asarray(refs)
        if (original_x.dtype!=np.dtype('float64') or original_x.ndim!=2 or original_x.shape[1]!=3
                or not len(original_x) or not np.isfinite(original_x).all()
                or original_v.dtype not in (np.dtype('int32'),np.dtype('int64'))
                or original_v.shape!=(len(original_x),)):
            raise ValueError('FinalPointArraysRequired')
        captured=(array_binding(original_x),array_binding(original_v))
        x=frozen(original_x,original_x.dtype.str);v=frozen(original_v,original_v.dtype.str)
        if (array_binding(x),array_binding(v))!=captured:raise ValueError('ChangedFinalPointInputs')
        start=time.perf_counter();bounds=[]
        for p,ref in zip(x,v):
            value=self._point(ref,p);r=self.manual_controls()
            def proof():
                result=self.queries.project(value,projection_tolerance_m=r.projection_tolerance_m,
                    maximum_displacement_m=r.maximum_projection_displacement_m,maximum_iterations=1)
                if np.asarray(result['point_m'],dtype='<f8').tobytes()!=np.asarray(value,dtype='<f8').tobytes():
                    raise ValueError('FinalPointProofMovedInput')
                return result['position_bound_m']
            bounds.append(self._query('position_bound',ref,value,proof))
            self.exact_normal(ref,p)
        self.verify()
        if (array_binding(np.asarray(coordinates)),array_binding(np.asarray(refs)))!=captured:
            raise ValueError('ChangedFinalPointInputs')
        return dict(passed=True,vertices=len(x),coordinate_hash=array_hash(x),reference_hash=array_hash(v),
            materialized_inputs=dict(coordinates=captured[0],references=captured[1]),
            maximum_position_bound_m=max(bounds),normal_bound_rad=self.manual_controls().normal_estimate_bound_rad,
            seconds=time.perf_counter()-start,method='Existing exact normal-fibre and branch-cone predicates on final bits',
            construction_values_accepted=False)

    def branch_name(self,ref):
        self._point(ref,(0,0,0))
        return 'canonical.root_zero_field'

    @verified_query
    def project_sheet(self,ref,p):
        value=self._point(ref,p);r=self.manual_controls()
        def compute():
            if self.construction_active() or self.numerical_acceptance_active():return self._numerical_project(value)
            result=self.queries.project(value,projection_tolerance_m=r.projection_tolerance_m,
                maximum_displacement_m=r.maximum_projection_displacement_m,
                maximum_iterations=r.maximum_projection_iterations)
            self.verify()
            return np.asarray(result['point_m'],dtype=float)
        return self._query('projection',ref,value,compute)

    @verified_query
    def exact_normal(self,ref,p):
        value=self._point(ref,p)
        def compute():
            proposed=self._numerical_normal(value)
            if self.construction_active() or self.numerical_acceptance_active():return proposed
            arguments=dict(normal_limit_rad=self.manual_controls().normal_estimate_bound_rad,
                           proposed_direction=proposed)
            if _prism_queries.get() is self:
                # Preserve the existing point jet until the first-height proof
                # reads it. Source/arithmetic checks and every predicate remain.
                arguments['retain_point_region']=True
            result=self.queries.normal(value,**arguments)
            self.verify()
            return np.asarray(result['direction'],dtype=float)
        return self._query('normal',ref,value,compute)

    @verified_query
    def implicit_value(self,ref,p):
        value=self._point(ref,p);scope=self.queries._scope
        def compute():
            if self.construction_active() or self.numerical_acceptance_active():return self._numerical_sample(value)[0]
            expression,scale,budgets,_=scope._batch_inputs()
            field=region(expression,tuple(Interval.point(v) for v in value),scale,
                         precision_bits=budgets[2]).value
            # Unchanged upper field residual; not an asserted Euclidean distance.
            result=upper_float(field.magnitude)*(-1 if field.hi<0 else 1)
            self.verify()
            return result
        return self._query('residual',ref,value,compute)

    def competing_component_sheet_refs(self,ref):
        self._point(ref,(0,0,0))
        # This candidate has one root FIELD reference, not a proved partition
        # of its connected sheets. Full topology/ownership is still unbound.
        return ()

    @verified_query
    def metric(self,vertex,curves,multiplier):
        value=self._point(vertex.sheet_ref,vertex.point)
        if curves or multiplier!=1:raise ValueError('UnsupportedManualMetricFeatureOrMultiplier')
        r=self.manual_controls()
        if self.construction_active():
            cached=self._cached_query('geometry_metric',vertex.sheet_ref,value)
            if cached is not None:return cached
            return self.geometry_metrics_batch(np.array([vertex.point]),
                                               np.array([vertex.sheet_ref]))[0][0]
        normal=self.exact_normal(vertex.sheet_ref,vertex.point);u,v=_normal_map_basis(normal)
        neighbors=np.array([self.exact_normal(vertex.sheet_ref,
                    self.project_sheet(vertex.sheet_ref,vertex.point+NORMAL_MAP_STEP_M*d))
                    for d in (u,-u,v,-v)])
        result,principal=_normal_variation_data(normal,u,v,neighbors,r)
        self._query('geometry_principal',vertex.sheet_ref,value,lambda:principal)
        self.validate_composed_metric(result);self.verify()
        return result

    @verified_query
    def principal_data(self,p,ref):
        value=self._point(ref,p)
        data=self._cached_query('geometry_principal',ref,value)
        if data is None:
            from historical_capsule_surface import SeedVertex
            # Same normal-map operation populates both values in this phase.
            self.metric(SeedVertex(np.asarray(value),int(ref),0,0),set(),1)
            data=self._cached_query('geometry_principal',ref,value)
            if data is None:
                normal=self.exact_normal(ref,value);u,v=_normal_map_basis(normal)
                neighbors=np.array([self.exact_normal(ref,self.project_sheet(ref,
                    np.asarray(value)+NORMAL_MAP_STEP_M*d)) for d in (u,-u,v,-v)])
                _,data=_normal_variation_data(normal,u,v,neighbors,self.manual_controls())
        return _principal_record(data)

    @verified_query
    def finish_shape(self,matrix,principal,policy):
        captured=self.shape_controls()
        admitted=ShapeMetricPolicy(captured.normal_qualification_fraction,
            captured.mmgs_response_reserve,captured.curvature_reserve_is_embedded)
        if policy!=admitted:raise ValueError('GeneralShapePolicyMismatch')
        result,stats=apply_shape_metric(matrix,principal,admitted)
        # The retained construction majorant handles binary64 rank-one rounding.
        # All existing independent serialization/majorant predicates remain.
        result,preserved=_preserve_geometry(result,matrix)
        self.validate_composed_metric(result)
        return result,dict(stats,**preserved)

    @verified_query
    def prism_metric(self,p,ref):
        from historical_capsule_surface import SeedVertex
        value=self._point(ref,p)
        def compute():
            base=self.metric(SeedVertex(np.asarray(value),int(ref),0,0),set(),1)
            result,_=intersect_tangent_metric(base,self.exact_normal(ref,value),self.prism_controls(),
                preserve_required_geometry=True,include_reference_diagnostics=False)
            self.validate_composed_metric(result);return result
        return self._query('prism_metric',ref,value,compute)

    @verified_query
    def composed_metric(self,p,ref):
        value=self._point(ref,p)
        return self._query('metric',ref,value,lambda:self._composed_metric(value,ref))

    def _composed_metric(self,p,ref):
        from historical_capsule_surface import SeedVertex
        value=self._point(ref,p)
        if self.construction_active():
            return self.composed_metrics_batch(np.array([value]),np.array([ref]))[0]
        result,_=self.finish_shape(self.prism_metric(value,ref),
                                 self.principal_data(value,ref),self.shape_controls())
        self.verify()
        return result

    def validate_composed_metric(self,result):
        eigenvalues=np.linalg.eigvalsh(result)
        r=self.manual_controls()
        if not np.isfinite(eigenvalues).all() or eigenvalues[0]<=0:
            raise ValueError('NonSPDManualPrismMetric')
        if eigenvalues[-1]>1/r.minimum_size_m**2 or eigenvalues[0]<1/r.maximum_size_m**2:
            raise ValueError('ManualPrismMetricSizeConflict')

    @verified_query
    def write_contract(self,path,vertices,faces,edges):
        self.verify()
        # The stock provider computes these tensors before serializing them.
        # Share only construction values with its immediate serialization check.
        # Final acceptance uses a separate phase key and cannot consume them.
        from historical_capsule_surface import SeedVertex
        reuse=self.construction_active()
        def vertex_pin(items):
            rows=[]
            for v in items:
                if any(type(n) not in (int,np.int32,np.int64)
                       for n in (v.sheet_ref,v.chart,v.corner_ref)):
                    raise ValueError('NativeContractVertexMetadataRequired')
                value=self._point(v.sheet_ref,v.point)
                rows.append((np.asarray(value,dtype='<f8').tobytes(),
                             int(v.sheet_ref),int(v.chart),int(v.corner_ref)))
            return tuple(rows)
        captured=vertex_pin(vertices) if reuse else None
        private=tuple(SeedVertex(np.frombuffer(p,dtype='<f8'),ref,chart,corner)
                      for p,ref,chart,corner in captured) if reuse else vertices
        authority=self
        if reuse:
            bases,normals=self.geometry_metrics_batch(
                np.array([v.point for v in private]),np.array([v.sheet_ref for v in private]))
            # Only this immediate stock writer consumes the call-local proposals.
            # No construction payload crosses into final acceptance.
            owner=self
            class WriterQueries:
                def __getattr__(self,name):return getattr(owner,name)
                def row(self,ref,point):
                    owner.verify()
                    if composed>=len(private):raise ValueError('ChangedContractCompositionInventory')
                    expected=private[composed]
                    if (ref!=expected.sheet_ref or
                        np.asarray(point,dtype='<f8').tobytes()!=expected.point.tobytes()):
                        raise ValueError('ChangedContractCompositionInputs')
                    return composed
                def metric(self,vertex,curves,multiplier):
                    if curves or multiplier!=1:raise ValueError('UnsupportedManualMetricFeatureOrMultiplier')
                    return bases[self.row(vertex.sheet_ref,vertex.point)].copy()
                def exact_normal(self,ref,point):return normals[self.row(ref,point)].copy()
            authority=WriterQueries()
        composed=0
        def compose(*args,**kwargs):
            nonlocal composed
            result,stats=intersect_tangent_metric(*args,**kwargs,
                preserve_required_geometry=True,include_reference_diagnostics=False)
            self.validate_composed_metric(result)
            vertex=private[composed]
            value=self._point(vertex.sheet_ref,vertex.point)
            result=self._query('prism_metric',vertex.sheet_ref,value,lambda:result)
            result,shape=self.finish_shape(result,self.principal_data(value,vertex.sheet_ref),self.shape_controls())
            shape_rows.append(shape)
            if reuse:
                if composed>=len(private):raise ValueError('ChangedContractCompositionInventory')
                vertex=private[composed]
                value=self._point(vertex.sheet_ref,vertex.point)
                cached=self._query('metric',vertex.sheet_ref,value,lambda:result)
                if cached.tobytes()!=np.asarray(result,dtype='<f8').tobytes():
                    raise ValueError('ChangedContractCompositionValue')
            composed+=1
            return result,stats
        # Same owned provider code, composition and binary contract. The pinned
        # provider composes exactly once per vertex, in its serialization order.
        shape_rows=[]
        ns=dict(provider.write_contract.__globals__)
        ns['intersect_tangent_metric']=compose
        writer=types.FunctionType(provider.write_contract.__code__,ns,
                                  provider.write_contract.__name__,provider.write_contract.__defaults__)
        result=writer(path,private,faces,edges,authority,1,
                      prism_policy=self.prism_controls(),shape_policy=None)
        if reuse and (composed!=len(private) or vertex_pin(vertices)!=captured):
            raise ValueError('ChangedContractCompositionInputs')
        result['shape_aware_tangent_conditioning']=dict(enabled=True,
            active_vertices=sum(int(s['active']) for s in shape_rows),
            normal_qualification_fraction=self.shape_controls().normal_qualification_fraction,
            mmgs_response_reserve=self.shape_controls().mmgs_response_reserve,
            computed_metric_preservation='ComputedGeometryMajorant/1')
        self.verify()
        return result

    def qualify_segments(self,start,end,*,first_height_m=None):
        """Retain no call-scoped payload from a rejected qualification."""
        try:
            if not self.stock_prism_queries():
                if _prism_queries.get() is self:raise ValueError('ChangedPrismQueryMethods')
                return self._qualify_segments(start,end,first_height_m=first_height_m)
            with self.prism_query_scope(),self.verified_operation('segments'):
                return self._qualify_segments(start,end,first_height_m=first_height_m)
        except BaseException:
            from sdf_cfd_mesher_v4.sdf.regional import invalidate_filtered_cache
            query=_surface_queries.get()
            if query is not None and query[0] is self:query[5]()
            invalidate_filtered_cache()
            self.queries._scope.reject_batch()
            raise

    def _qualify_segments(self,start,end,*,first_height_m=None):
        """Prove actual vertex segments and existing full first-normal height.

        Interval normalization encloses the true real-source unit normal at
        the materialized wall point. No floating estimate is used as a proof.
        """
        from sdf_cfd_mesher_v4.common.authority import frozen,array_hash
        from sdf_cfd_mesher_v4.sdf.regional import exact, exact_fallback, UnresolvedRegion, _filtered
        if self.construction_active():raise ValueError('ConstructionCannotQualifyMesh')
        if self.acceptance_active():
            self.reject_acceptance();raise ValueError('SurfaceAcceptanceCannotQualifySegments')
        self.verify();s=self.queries._scope
        expression,scale,budgets,_=s._batch_inputs()
        x=frozen(start,'<f8');y=frozen(end,'<f8')
        if x.shape!=y.shape or x.ndim!=2 or x.shape[1]!=3 or not len(x) or not np.isfinite([x,y]).all():
            raise ValueError('FiniteSourceSegmentsRequired')
        if first_height_m is not None and (type(first_height_m) not in (float,int) or not math.isfinite(first_height_m) or first_height_m<=0):
            raise ValueError('PositiveExplicitFirstHeightRequired')
        def height_error(p,q):
            # Replay this complete predicate, including C1 and normalization,
            # when conservative filtered bounds cannot resolve it.
            from sdf_cfd_mesher_v4.sdf.source_topology import c1_region
            if not c1_region(expression,tuple(Interval.point(float(v))/scale for v in p),budgets[2]):
                raise ValueError('UniqueSourceWallNormalUnresolved')
            jet=region(expression,tuple(Interval.point(float(v)) for v in p),scale,precision_bits=budgets[2])
            length=sum((g.square() for g in jet.gradient),Interval.point(0)).sqrt(budgets[2])
            height=sum((g*(exact(float(b))-exact(float(a))) for g,a,b in zip(jet.gradient,p,q)),Interval.point(0))/length
            error=(height-exact(float(first_height_m))).magnitude
            if height.lo<=0 or error>=exact(1e-12):raise ValueError('TrueSourceFullNormalHeightUnresolved')
            return upper_float(error)
        errors=[]
        for p,q in zip(x,y):
            self.queries.outward_segment(p,q)
            if first_height_m is not None:
                try:error=height_error(p,q)
                except ValueError as failure:
                    state=_filtered.get()
                    if (state is None or state['exact'] or
                            not (isinstance(failure,UnresolvedRegion) or failure.args in (
                                ('UniqueSourceWallNormalUnresolved',),('TrueSourceFullNormalHeightUnresolved',)))):
                        raise
                    state['counts']['height_query_fallbacks']+=1
                    error=exact_fallback(lambda:height_error(p,q))
                errors.append(error)
        self.verify()
        if (np.shape(start)!=x.shape or np.shape(end)!=y.shape or
                array_hash(np.asarray(start,dtype='<f8'))!=array_hash(x) or
                array_hash(np.asarray(end,dtype='<f8'))!=array_hash(y)):
            raise ValueError('ChangedSourceSegmentInputs')
        return dict(passed=True,segments=len(x),start=array_hash(x),end=array_hash(y),
            exact_field_progress=True,positive_endpoints=True,first_source_normal_height_m=first_height_m,
            maximum_first_height_error_m=max(errors,default=None),height_reserve_m=1e-12 if first_height_m is not None else None,
            scope='Actual vertex segments and source-normal height only; cell/contact/volume/degree obligations remain separate.')

    def inspect(self):
        self.verify()
        return dict(geometry_identity=self.geometry_identity,units='m',
            projection='source-normal-fibre proof on materialized binary64 coordinates',
            normals='binary64 estimates with explicit real-source branch-cone bounds',
            sizing='explicit SI size cap + retained projected normal-map curvature metric + existing prism tangent intersection + retained generic shape conditioning',
            metric_composition='ComputedGeometryMajorant/1; unchanged independent exact_majorant acceptance',
            source_distance_coverage='whole-zero-set interval/triangle proximity proof available',
            execution_admission_owner='SurfaceBinding.inspect',
            scope='Query/provider capabilities only. The bound source contract derives topology; the existing surface/prism/background owners must accept each generated mesh.',
            native_equivalence=False)

# Defining-module owners remain stable if consumers supply earlier custom methods.
_STANDARD_EXACT_NORMAL=GeneralSdfAuthority.exact_normal
_STANDARD_NUMERICAL_SAMPLES=GeneralSdfAuthority.numerical_surface_samples

_STANDARD_NUMERICAL_NORMAL=GeneralSdfAuthority._numerical_normal
