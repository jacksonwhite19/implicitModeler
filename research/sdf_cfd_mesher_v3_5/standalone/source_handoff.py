"""Run-local whole-population source evidence, never an admission or disk cache.

Production may authenticate completed planner source checks at final prepare.
Audit always recomputes. Actual final geometry, coverage and all other gates
remain the consumer's responsibility. No individual query scope is extended.
"""
from contextlib import contextmanager
from dataclasses import asdict
from pathlib import Path
import ctypes,hashlib,json,os,threading
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
    def __init__(self,source,height,mode):
        self.source=source;self.height=height;self.mode=mode
        self.active=True;self.thread=threading.get_ident();self.pending=None;self.accepted=None
        self.context=_context(source) if mode=='production' else None
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
            return _segment_population(source,height,positions,False)[0]
        before=_context(source) if self.context is not None else None
        if not _same_context(before,self.context):return _segment_population(source,height,positions,False)[0]
        private=frozen(positions,'<f8')
        try:
            summary,proofs=_segment_population(source,height,private,True)
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
        except BaseException:
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
        if accepted is None:return None
        private,payload,context=accepted
        if (mode!='production' or mode!=self.mode or source is not self.source or
                type(height) is not type(self.height) or repr(height)!=repr(self.height) or
                np.asarray(positions).shape!=private.shape or np.asarray(positions,dtype='<f8').tobytes()!=private.tobytes() or
                not _same_context(_context(source),context)):
            self.events.append(dict(event='fallback',reason='changed_consumer_population_or_context'));return None
        result=json.loads(payload)
        self.events.append(dict(event='consumed',layers=len(result),segments=(len(private)-1)*len(private[0])))
        return result
    def close(self):
        self.pending=None;self.accepted=None;self.active=False

class _Handoff:
    """Read-only callable facade; evidence stays in private run-local closures."""
    __slots__=('qualify','discard','accept','consume','close','events')
    def __setattr__(self,name,value):raise ValueError('ProtectedSourceHandoff')
    def __init__(self,source,height,mode):
        state=_HandoffState(source,height,mode)
        for name in ('qualify','discard','accept','consume','close'):
            method=getattr(state,name)
            def call(*args,_method=method,**kwargs):return _method(*args,**kwargs)
            object.__setattr__(self,name,call)
        # Diagnostics are detached and never consulted for authority.
        object.__setattr__(self,'events',lambda:json.loads(json.dumps(state.events)))

@contextmanager
def live_handoff(source,height,mode):
    if mode not in ('audit','production'):raise ValueError('UnknownSourceHandoffMode')
    owner=_Handoff(source,height,mode)
    try:yield owner
    finally:owner.close()

def consume(owner,source,height,positions,mode):
    if type(owner) is not _Handoff:return None
    return owner.consume(source,height,positions,mode)
