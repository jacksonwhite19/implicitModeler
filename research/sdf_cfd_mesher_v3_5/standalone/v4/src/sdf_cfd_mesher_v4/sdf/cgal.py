"""Same canonical handle services CGAL diagnostics and extraction callbacks."""
import json
import math
import time
import struct
from collections import Counter, OrderedDict
from dataclasses import dataclass
from .canonical import LineProcess, GeometryFailure, require_capabilities


@dataclass(frozen=True)
class CgalInvocation:
    executable_prefix: tuple[str, ...]
    mode: str
    io_path: str
    relative_error: float
    repetitions: int | None = None
    extraction_criteria: tuple[float, ...] | None = None
    seed: int | None = None

    def command(self, region):
        if not math.isfinite(self.relative_error) or not 0 < self.relative_error < 1:
            raise ValueError('CGAL relative error must lie strictly between zero and one')
        args = [*self.executable_prefix, self.mode, self.io_path,
                *map(str,region.center_m),str(region.radius_m),str(self.relative_error)]
        if self.mode=='--probe' and isinstance(self.repetitions,int) and 1<=self.repetitions<=100000:
            return [*args,str(self.repetitions)]
        if (self.mode in ('--extract','--extract-surface') and self.extraction_criteria is not None
                and len(self.extraction_criteria)==6 and self.seed is not None and self.seed>=0
                and all(math.isfinite(x) and x>0 for x in self.extraction_criteria)):
            return [*args,*map(str,self.extraction_criteria),str(self.seed)]
        raise ValueError('explicit diagnostic repetition count or all six extraction criteria and seed required')


class QueryDiagnostics:
    """Bounded passive statistics. None of these values controls a query.

    Repetition counts use the last 4096 distinct keys, not global uniqueness.
    The binary32 key reproduces host input conversion only; it grants no real-
    source or native-zero-set equivalence. Timings are instrumented-run costs.
    """
    def __init__(self, scale):
        if not math.isfinite(scale) or scale<=0:raise ValueError('PositiveDiagnosticUnitScale')
        self.scale=scale;self.completed=0;self.signs=Counter();self.exponents=Counter()
        self.recent64=OrderedDict();self.recent32=OrderedDict()
        self.repeats64=0;self.repeats32=0
        self.minimum=[math.inf]*3;self.maximum=[-math.inf]*3
        self.seconds=Counter();self.native_query_handler_seconds=0.

    def observe(self,p,value,native_seconds):
        start=time.perf_counter()
        if not math.isfinite(value) or not math.isfinite(native_seconds) or native_seconds<0:
            raise ValueError('InvalidNativeQueryDiagnostic')
        self.completed+=1;self.native_query_handler_seconds+=native_seconds
        self.signs['negative' if value<0 else 'positive' if value>0 else 'zero']+=1
        self.exponents['zero' if value==0 else str(math.floor(math.log10(abs(value))))]+=1
        # Exact binary64 coordinates and the host's binary32 model-unit input.
        # No caching: even a repeated query is still sent to the native owner.
        key64=struct.pack('<ddd',*p)
        key32=struct.pack('<fff',*(v/self.scale for v in p))
        for key,recent,name in ((key64,self.recent64,'repeats64'),(key32,self.recent32,'repeats32')):
            if key in recent:setattr(self,name,getattr(self,name)+1);recent.move_to_end(key)
            else:
                recent[key]=None
                if len(recent)>4096:recent.popitem(last=False)
        for i,v in enumerate(p):self.minimum[i]=min(v,self.minimum[i]);self.maximum[i]=max(v,self.maximum[i])
        self.seconds['statistics']+=time.perf_counter()-start

    def snapshot(self,requested,wall,status,error=None):
        exclusive=dict(self.seconds)
        return dict(schema='V4CgalPassiveQueryDiagnostics/1',status=status,error=error,
            requested_queries=requested,completed_queries=self.completed,wall_seconds=wall,
            exclusive_seconds=exclusive,other_seconds=wall-sum(exclusive.values()),
            native_query_handler_seconds_nested_in_canonical_roundtrip=self.native_query_handler_seconds,
            timing_scope=dict(native_query_handler='Handle lookup, validation, SI conversion, allocation, field evaluation and output rescaling; excludes native serialization. Nested in canonical_roundtrip.',
                native_wait_and_transport='CGAL work/wait plus stdout transport; no native refinement-stage attribution.'),
            signs=dict(self.signs),absolute_native_value_log10_bins=dict(self.exponents),
            repetition_window_distinct_keys=4096,repeated_binary64_points=self.repeats64,
            repeated_binary32_model_points=self.repeats32,
            point_bounds_m=[self.minimum,self.maximum] if self.completed else None,
            model_unit_scale=self.scale,passive_only=True,
            does_not_prove='No global unique count, native refinement-stage attribution, real-target proximity, coverage, topology or mesh acceptance.')


def run_cgal(geometry, invocation, region, *, timeout_seconds, maximum_queries,
             progress=None, diagnostic_unit_scale=None):
    require_capabilities(geometry, 'cgal_callback')
    region.validate(geometry)
    if timeout_seconds<=0 or maximum_queries<=0:
        raise ValueError('explicit positive resource limits required')
    diagnostics=QueryDiagnostics(diagnostic_unit_scale) if progress is not None else None
    command = invocation.command(region)
    start = time.perf_counter()
    pipe = LineProcess(command, timeout_seconds)
    calls = 0
    values, labels = [], []
    failure = None
    status='failed';error=None;primary_exception=None
    try:
        while True:
            if time.perf_counter()-start > timeout_seconds:
                raise GeometryFailure('CgalDeadlineExceeded','cgal_callback',timeout_seconds,geometry.identity)
            t=time.perf_counter() if diagnostics is not None else None
            line = pipe.read(timeout_seconds-(time.perf_counter()-start))
            if diagnostics is not None:diagnostics.seconds['native_wait_and_transport']+=time.perf_counter()-t
            if line.startswith('Q '):
                calls += 1
                try:
                    if calls > maximum_queries:
                        raise GeometryFailure('QueryBudgetExceeded', 'cgal_callback', str(maximum_queries), geometry.identity)
                    p = list(map(float, line.split()[1:]))
                    if len(p)!=3 or not all(map(math.isfinite,p)):
                        raise GeometryFailure('InvalidCallbackPoint', 'cgal_callback', line, geometry.identity)
                    if diagnostics is None:value = geometry.query([p])[0]
                    else:
                        t=time.perf_counter();report=geometry.query_report([p])
                        diagnostics.seconds['canonical_roundtrip']+=time.perf_counter()-t
                        value=report['values_m'][0]
                        diagnostics.observe(p,value,report['query_seconds'])
                    t=time.perf_counter() if diagnostics is not None else None
                    pipe.write('OK '+format(value,'.17g'))
                    if diagnostics is not None:
                        diagnostics.seconds['reply_write']+=time.perf_counter()-t
                        if calls%10000==0:
                            t=time.perf_counter();progress(diagnostics.snapshot(calls,t-start,'running'))
                            diagnostics.seconds['progress_output']+=time.perf_counter()-t
                except GeometryFailure as e:
                    failure = e
                    pipe.write('ERR '+e.record['code'])
            elif line.startswith('V '):
                _, value, label = line.split()
                values.append(float(value)); labels.append(int(label))
            elif line.startswith('FAIL '):
                if failure is not None:
                    raise failure
                raise GeometryFailure('CgalNativeFailure', 'cgal_callback', line[5:], geometry.identity)
            elif line.startswith('DONE '):
                if failure is not None:
                    raise failure
                result = json.loads(line[5:])
                if result['calls'] != calls:
                    raise GeometryFailure('CallbackCountMismatch', 'cgal_callback', result, geometry.identity)
                code = pipe.process.wait(timeout=max(.001,timeout_seconds-(time.perf_counter()-start)))
                if code:
                    raise GeometryFailure('CgalExitFailure','cgal_callback',code,geometry.identity)
                status='completed'
                return dict(geometry_identity=geometry.identity, native=result, callback_queries=calls,
                            command=command, enclosing_region=dict(center_m=region.center_m,radius_m=region.radius_m,authority=region.authority),
                            values_m=values, labels=labels, wall_seconds=time.perf_counter()-start,
                            stderr=''.join(pipe.errors),
                            query_diagnostics=diagnostics.snapshot(calls,time.perf_counter()-start,status) if diagnostics is not None else None)
            else:
                raise GeometryFailure('CgalProtocolCorruption', 'cgal_callback', line, geometry.identity)
    except BaseException as exc:
        error=str(exc);primary_exception=exc
        raise
    finally:
        pipe.close()
        if diagnostics is not None:
            try:progress(diagnostics.snapshot(calls,time.perf_counter()-start,status,error))
            except BaseException as exc:
                if primary_exception is None:raise
                primary_exception.add_note('Final CGAL diagnostic output also failed: '+str(exc))
