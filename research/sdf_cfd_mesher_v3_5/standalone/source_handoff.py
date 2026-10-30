"""Run-local whole-population source evidence, never an admission or disk cache.

Production may authenticate completed planner source checks at final prepare.
Audit always recomputes. Actual final geometry, coverage and all other gates
remain the consumer's responsibility. No individual query scope is extended.
"""
from contextlib import contextmanager
from dataclasses import asdict
from pathlib import Path
import ctypes,hashlib,json,os,threading,time
import numpy as np
from general_sdf_authority import GeneralSdfAuthority,_STANDARD_SEGMENT_HANDOFF_METHODS
from sdf_cfd_mesher_v4.sdf.operations import ProtectedQueries,_STANDARD_OUTWARD_SEGMENT
from sdf_cfd_mesher_v4.common.authority import frozen,array_hash

VERSION='V35LiveFinalSourcePopulation/1'
_METHODS=dict(_STANDARD_SEGMENT_HANDOFF_METHODS)
_OUTWARD=_STANDARD_OUTWARD_SEGMENT

def _context(source):
    """Fresh custody; no claim that a prior verified_operation is still live."""
    if type(source) is not GeneralSdfAuthority or os.name!='nt':return None
    import general_sdf_authority as owner
    from sdf_cfd_mesher_v4.sdf import regional,regional_owner
    # Only ordinary producer/prepare boundaries are eligible. Nested source or
    # exact-filter modes retain the original evaluator and its own scope rules.
    if any(scope.get() is not None for scope in
            (owner._surface_queries,owner._verified,owner._prism_queries,
             owner._construction,owner._acceptance,regional._filtered,regional_owner._batch)):
        return None
    if any(getattr(getattr(source,k),'__func__',None) is not v for k,v in _METHODS.items()):return None
    if type(source.queries) is not ProtectedQueries or getattr(source.queries.outward_segment,'__func__',None) is not _OUTWARD:return None
    if not source.stock_prism_queries():return None
    source.verify()
    if source.construction_active() or source.acceptance_active():return None
    scope=source.queries._scope;expression,scale,budgets,root=scope._batch_inputs()
    word=ctypes.c_uint();read=ctypes.CDLL('ucrtbase.dll')._controlfp_s
    read.argtypes=(ctypes.POINTER(ctypes.c_uint),ctypes.c_uint,ctypes.c_uint);read.restype=ctypes.c_int
    if read(ctypes.byref(word),0,0):raise ValueError('SourceHandoffArithmeticReadFailure')
    # File custody is normally held by source_custody; also bind exact evaluator
    # source bytes here. Native liveness and handle custody come from verify().
    import inspect
    import sdf_cfd_mesher_v4.sdf.regional as regional
    import sdf_cfd_mesher_v4.sdf.operations as operations
    import sdf_cfd_mesher_v4.sdf.regional_owner as regional_owner
    files=(inspect.getfile(GeneralSdfAuthority),regional.__file__,operations.__file__,regional_owner.__file__)
    values=dict(geometry=source.geometry_identity,expression=repr(expression),scale=float(scale).hex(),budgets=budgets,root=root,
        manual=asdict(source.manual_controls()),prism=asdict(source.prism_controls()),shape=asdict(source.shape_controls()),
        sources=[(str(Path(p).resolve()),hashlib.sha256(Path(p).read_bytes()).hexdigest()) for p in files],
        arithmetic_control=word.value,numpy_errors=np.geterr())
    # repr preserves signed zero/type differences in validated scalar policies.
    return (id(source.queries),id(scope),repr(values),np.geterrcall())

def _same_context(left,right):
    # Error callbacks may implement value equality; arithmetic custody requires
    # the exact callback object, as the original source query scope does.
    return (left is not None and right is not None and
            left[:3]==right[:3] and left[3] is right[3])

class _HandoffState:
    def __init__(self,source,height,mode,source_workers=1,deadline=None):
        self.source=source;self.height=height;self.mode=mode
        self.active=True;self.thread=threading.get_ident();self.pending=None;self.accepted=None
        self.context=_context(source) if mode=='production' else None
        if type(source_workers) is not int or source_workers not in (1,2,4,8):raise ValueError('InvalidSourceWorkerSelection')
        self.workers=source_workers;self.deadline=deadline;self.pool=None;self.execution=None
        if source_workers>1 and (deadline is None or not np.isfinite(deadline) or deadline<=time.perf_counter()):raise ValueError('SourceWorkerDeadlineRequired')
        self.events=[]
    def _live(self):
        if not self.active or threading.get_ident()!=self.thread:raise ValueError('ExpiredOrDetachedSourceHandoff')
    def discard(self,reason):
        self._live();self.pending=None;self.accepted=None
        self.events.append(dict(event='discard',reason=reason))
    def qualify(self,source,height,positions):
        from collision_policy import _segment_population
        self._live();self.discard('new_population')
        if source is not self.source or type(height) is not type(self.height) or repr(height)!=repr(self.height):
            if self.pool is not None:
                self.pool.abort();raise ValueError('ChangedParallelProducerSourceOrHeight')
            return _segment_population(source,height,positions,False)[0]
        before=_context(source) if self.context is not None else None
        if not _same_context(before,self.context):
            if self.pool is not None:
                self.pool.abort();raise ValueError('ChangedParallelProducerContext')
            return _segment_population(source,height,positions,False)[0]
        private=frozen(positions,'<f8')
        try:
            from source_parallel import eligible,Pool
            reason=eligible(source,private,self.mode,self.workers)
            if reason is None:
                if self.pool is None:self.pool=Pool(source,height,self.workers,self.deadline)
                capability=self.pool.qualify(private)
                proofs=self.pool.take(capability,source,height,private)
                summary=dict(rows=[dict(layer=k,passed=True,segments=len(private[0])) for k in range(1,len(private))],scope='Existing source progress/height predicates; no pair separation or distance bound')
                self.execution=None
            else:
                if self.pool is not None:raise ValueError('ChangedParallelEligibilityAfterStart')
                summary,proofs=_segment_population(source,height,private,True)
                self.execution=dict(schema='V35SourceExecutionSelection/1',requested_workers=self.workers,effective_workers=1,reason=reason)
            self.events.append(dict(event='execution_selection',requested_workers=self.workers,effective_workers=self.workers if reason is None else 1,reason=reason))
            if not _same_context(_context(source),before):raise ValueError('ChangedSourceHandoffContext')
            if np.asarray(positions).shape!=private.shape or np.asarray(positions,dtype='<f8').tobytes()!=private.tobytes():raise ValueError('ChangedSourceHandoffOperands')
            if len(proofs)!=len(private)-1:raise ValueError('IncompleteSourceHandoffPopulation')
            for k,proof in enumerate(proofs,1):
                if (proof.get('passed') is not True or proof.get('segments')!=len(private[0]) or
                    proof.get('start')!=array_hash(private[k-1]) or proof.get('end')!=array_hash(private[k]) or
                    proof.get('geometry_identity')!=source.geometry_identity or
                    proof.get('first_source_normal_height_m')!=(height if k==1 else None) or
                    len(proof.get('segment_proofs',()))!=len(private[0])):
                    raise ValueError('IncompleteSourceHandoffPopulation')
            payload=json.dumps(proofs,allow_nan=False,separators=(',',':')).encode()
            self.pending=(private,payload,before)
            self.events.append(dict(event='qualified',layers=len(proofs),segments=(len(private)-1)*len(private[0])))
            return summary
        except BaseException as error:
            from collision_policy import SourceSegmentRefusal
            if self.pool is not None and not self.pool.closed and type(error) is not SourceSegmentRefusal:self.pool.abort()
            self.discard('source_failure');raise
    def accept(self,positions):
        """Called only after construct's actual selected collision outcome passes."""
        self._live();self.accepted=None
        pending=self.pending;self.pending=None
        if pending is None:return False
        private,payload,context=pending
        if (np.asarray(positions).shape!=private.shape or np.asarray(positions,dtype='<f8').tobytes()!=private.tobytes() or
                not _same_context(_context(self.source),context)):
            self.events.append(dict(event='not_accepted',reason='changed_final_population_or_context'));return False
        self.accepted=pending;self.events.append(dict(event='accepted_final_collision_population'));return True
    def consume(self,source,height,positions,mode):
        self._live();accepted=self.accepted;self.accepted=None;self.pending=None
        if accepted is None:
            if self.pool is not None and not self.pool.closed:self.pool.finish(None,None)
            self.execution=None
            return None
        private,payload,context=accepted
        if (mode!='production' or mode!=self.mode or source is not self.source or
                type(height) is not type(self.height) or repr(height)!=repr(self.height) or
                np.asarray(positions).shape!=private.shape or np.asarray(positions,dtype='<f8').tobytes()!=private.tobytes() or
                not _same_context(_context(source),context)):
            if self.pool is not None and not self.pool.closed:self.pool.finish(None,None)
            self.execution=None
            self.events.append(dict(event='fallback',reason='changed_consumer_population_or_context'));return None
        result=json.loads(payload)
        if self.pool is not None:
            self.execution=self.pool.finish(private,result)
            if (not _same_context(_context(source),context) or np.asarray(positions).shape!=private.shape or np.asarray(positions,dtype='<f8').tobytes()!=private.tobytes()):
                self.execution=None;raise ValueError('ChangedSourceConsumerDuringWorkerClosure')
        self.events.append(dict(event='consumed',layers=len(result),segments=(len(private)-1)*len(private[0])))
        return result
    def execution_record(self):
        self._live()
        return json.loads(json.dumps(self.execution)) if self.execution is not None else dict(schema='V35SourceExecutionSelection/1',requested_workers=self.workers,effective_workers=1,reason='original_consumer_recomputation')
    def close(self):
        self.pending=None;self.accepted=None;self.active=False
        if self.pool is not None and not self.pool.closed:self.pool.abort()

class _Handoff:
    """Read-only callable facade; evidence stays in private run-local closures."""
    __slots__=('qualify','discard','accept','consume','close','events','execution_record')
    def __setattr__(self,name,value):raise ValueError('ProtectedSourceHandoff')
    def __init__(self,source,height,mode,source_workers=1,deadline=None):
        state=_HandoffState(source,height,mode,source_workers,deadline)
        for name in ('qualify','discard','accept','consume','close','execution_record'):
            method=getattr(state,name)
            def call(*args,_method=method,**kwargs):return _method(*args,**kwargs)
            object.__setattr__(self,name,call)
        # Diagnostics are detached and never consulted for authority.
        object.__setattr__(self,'events',lambda:json.loads(json.dumps(state.events)))

@contextmanager
def live_handoff(source,height,mode,*,source_workers=1,deadline=None):
    if mode not in ('audit','production'):raise ValueError('UnknownSourceHandoffMode')
    owner=_Handoff(source,height,mode,source_workers,deadline)
    try:yield owner
    finally:owner.close()

def consume(owner,source,height,positions,mode):
    if type(owner) is not _Handoff:return None
    return owner.consume(source,height,positions,mode)

def execution_record(owner):
    if type(owner) is not _Handoff:
        return dict(schema='V35SourceExecutionSelection/1',requested_workers=1,effective_workers=1,reason='original_consumer_recomputation')
    return owner.execution_record()
