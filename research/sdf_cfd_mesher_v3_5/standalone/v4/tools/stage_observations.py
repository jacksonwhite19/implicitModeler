"""Opt-in bounded scalar observations. Never scientific acceptance or a cache."""
from contextlib import contextmanager
from functools import wraps
from pathlib import Path
import atexit, json, math, os, sys, threading, time

CHOICE='bounded_operation_caller_v2'
SCHEMA='BoundedStageObservations/2'
LINK_SCHEMA='BoundedStageObservationLink/2'
MAX_RECORDS=256
SNAPSHOT_BYTES=122880  # final + pending + <=512B failure marker <256KiB/role
MAX_ROLES=32
_active=None
_child_index=0
_BUDGETS={0:range(4,8),1:range(8,12),2:range(12,24),24:range(25,32)}

def choice(value):
    selected=value.get('stage_timing')
    if selected is not None and (type(selected) is not str or selected!=CHOICE):
        raise ValueError('InvalidStageTimingSelection')
    return selected

def binding(assignment,planned,stage=None):
    expected=choice(assignment)
    if choice(planned)!=expected or (stage is not None and choice(stage)!=expected):
        raise ValueError('ChangedAuthenticatedStageTimingSelection')
    return expected


class _NullSpan:
    def __enter__(self):return None
    def __exit__(self,*unused):return False
_NULL=_NullSpan()

def safe(default=None):
    """Operational diagnostics never change the wrapped scientific outcome."""
    def decorate(call):
        @wraps(call)
        def protected(*args,**kwargs):
            try:return call(*args,**kwargs)
            except BaseException as error:
                try:
                    target=args[0] if args and isinstance(args[0],Collector) else _active
                    if target is not None and call.__name__!='problem':target.problem('Diagnostic '+call.__name__+': '+type(error).__name__)
                    else:print('STAGE_OBSERVATIONS_INCOMPLETE diagnostic='+call.__name__,file=sys.stderr,flush=True)
                except BaseException:pass
                return default
        return protected
    return decorate

def _totals():
    return dict(count=0,wall=0.,maximum_wall=0.,cpu=0.,maximum_cpu=0.,
        children_wall=0.,children_cpu=0.,exclusive_wall=0.,exclusive_cpu=0.,failures=0,units={})


class _Span:
    def __init__(self,c,name,kind):
        self.c=c;self.name=name;self.kind=kind;self.frame=None;self.deep=False;self.outer_deep=False
    def __enter__(self):
        c=self.c
        if not c.same_thread('span'):return None
        if c.finished:c.problem('Span after finish');return None
        try:
            if c.suppressed_depth or len(c.stack)>=32:
                self.deep=True;self.outer_deep=c.suppressed_depth==0
                if self.outer_deep:self.deep_started=c.now()
                c.suppressed_depth+=1;c.suppressed_calls+=1;c.problem('Span depth limit exceeded')
                return None
            started=c.now();operation=edge=None
            if c.stack and c.stack[-1]['operation'] is None:
                c.suppressed_calls+=1
            elif type(self.name) is not str or len(self.name)>96 or type(self.kind) is not str or len(self.kind)>32:
                c.suppressed_calls+=1;c.problem('Invalid span label')
            else:
                caller=c.stack[-1]['operation'] if c.stack else None
                try:operation,edge=c.admit(self.name,self.kind,caller)
                except BaseException as error:c.suppressed_calls+=1;c.problem('Span setup: '+type(error).__name__)
            self.frame=dict(operation=operation,edge=edge,started=started,children_wall=0.,children_cpu=0.)
            c.stack.append(self.frame)
        except BaseException as error:
            c.problem('Span setup: '+type(error).__name__)
            # A failed clock/setup must not attribute the body to its caller.
            self.deep=True;self.outer_deep=False;c.suppressed_depth+=1;c.suppressed_calls+=1
        return None
    def __exit__(self,kind,value,tb):
        c=self.c
        if not c.same_thread('span exit'):return False
        try:
            if self.deep:
                c.suppressed_depth-=1
                if self.outer_deep:
                    end=c.now();wall=end[0]-self.deep_started[0];used=end[1]-self.deep_started[1]
                    if c.stack:c.stack[-1]['children_wall']+=wall;c.stack[-1]['children_cpu']+=used
                return False
            if self.frame is None:return False
            if not c.stack or c.stack[-1] is not self.frame:raise ValueError('ChangedDiagnosticStack')
            c.stack.pop();frame=self.frame;end=c.now()
            wall=end[0]-frame['started'][0];used=end[1]-frame['started'][1]
            exclusive=wall-frame['children_wall'];exclusive_cpu=used-frame['children_cpu']
            if min(wall,used,exclusive,exclusive_cpu)<0:c.problem('Nonmonotonic invocation accounting')
            if frame['operation'] is not None:
                for row in (c.operations[frame['operation']],c.edges[frame['edge']]):
                    c.account(row,wall,used,frame['children_wall'],frame['children_cpu'],exclusive,exclusive_cpu,kind is not None)
            if c.stack:c.stack[-1]['children_wall']+=wall;c.stack[-1]['children_cpu']+=used
        except BaseException as error:c.problem('Span accounting: '+type(error).__name__)
        return False

class Collector:
    """Operation and immediate-caller summaries; invocation frames own accounting.

    This graph can contain recursion. It does not reconstruct full call paths.
    The private clock seam is instance-local and never selected by a request.
    """
    def __init__(self,path,role,slot,context,parent_call=None,*,_clock=None):
        self.path=Path(path);self.role=str(role)[:64];self.slot=slot
        self.context=str(context)[:128];self.parent_call=None if parent_call is None else str(parent_call)[:128]
        self.pid=os.getpid();self.parent_pid=os.getppid();self.thread=threading.get_ident()
        self._clock=_clock
        self.started,self.cpu_started=self.now()
        self.operations=[];self.operation_keys={};self.edges=[];self.edge_keys={};self.stack=[];self.links=[]
        self.suppressed_depth=0;self.suppressed_calls=0;self.adopted=False
        self.incomplete=[];self.overflow_calls=0;self.finished=False;self.marker_written=False
        self.outcome='running';self.flushes=0;self.writer_seconds=0.

    def now(self):
        value=(time.perf_counter(),time.process_time()) if self._clock is None else self._clock()
        if (type(value) not in (tuple,list) or len(value)!=2 or
                any(type(x) not in (int,float) or not math.isfinite(x) for x in value)):
            raise ValueError('InvalidLocalDiagnosticClock')
        return value

    def same_thread(self,operation):
        if threading.get_ident()==self.thread:return True
        self.problem('Cross-thread '+operation+' not covered');return False

    def admit(self,name,kind,caller):
        key=(name,kind);operation=self.operation_keys.get(key)
        new_operation=operation is None
        if new_operation:operation=len(self.operations)
        edge_key=(caller,operation);edge=self.edge_keys.get(edge_key)
        required=int(new_operation)+int(edge is None)
        if len(self.operations)+len(self.edges)+required>MAX_RECORDS:
            self.overflow_calls+=1;self.suppressed_calls+=1;self.problem('Combined operation/edge record limit exceeded');return None,None
        if new_operation:
            self.operations.append(dict(id=operation,name=name,kind=kind,**_totals()));self.operation_keys[key]=operation
        if edge is None:
            edge=len(self.edges);self.edges.append(dict(id=edge,caller=caller,operation=operation,**_totals()));self.edge_keys[edge_key]=edge
        return operation,edge

    @staticmethod
    def account(row,wall,cpu,children,children_cpu,exclusive,exclusive_cpu,failed):
        row['count']+=1;row['wall']+=wall;row['cpu']+=cpu
        row['maximum_wall']=max(row['maximum_wall'],wall);row['maximum_cpu']=max(row['maximum_cpu'],cpu)
        row['children_wall']+=children;row['children_cpu']+=children_cpu
        row['exclusive_wall']+=exclusive;row['exclusive_cpu']+=exclusive_cpu;row['failures']+=int(failed)

    @safe(False)
    def problem(self,reason):
        reason=str(reason)[:192]
        if reason not in self.incomplete and len(self.incomplete)<8:self.incomplete.append(reason)

    @safe(False)
    def adopt_start(self,wall,cpu,label):
        """Account for the local pre-activation admission as one opaque interval."""
        if not self.same_thread('adopt') or self.stack or self.suppressed_depth or self.adopted or self.finished:
            self.problem('Invalid startup adoption lifetime');return False
        if type(label) is not str or len(label)>96:raise ValueError('InvalidStartupLabel')
        end,used=self.now()
        if not all(type(x) in (int,float) and math.isfinite(x) for x in (wall,cpu)) or wall>end or cpu>used:
            raise ValueError('InvalidStartupClock')
        operation,edge=self.admit(label,'startup',None)
        if operation is None:return False
        self.started=wall;self.cpu_started=cpu
        for row in (self.operations[operation],self.edges[edge]):self.account(row,end-wall,used-cpu,0.,0.,end-wall,used-cpu,False)
        self.adopted=True;return True

    @safe(_NULL)
    def span(self,name,kind='other'):
        return _Span(self,name,kind)

    @safe(False)
    def units(self,name,value):
        try:
            if not self.same_thread('units') or self.suppressed_depth or not self.stack or self.finished:return
            frame=self.stack[-1]
            if frame['operation'] is None:return
            rows=[self.operations[frame['operation']]['units'],self.edges[frame['edge']]['units']]
            if (type(value) is not int or value<0 or type(name) is not str or len(name)>48
                    or any(name not in row and len(row)>=8 for row in rows)):
                self.problem('Work-unit limit exceeded');return
            for row in rows:row[name]=row.get(name,0)+value
        except BaseException as error:self.problem('Work accounting: '+type(error).__name__)

    @safe(False)
    def link(self,reference):
        if not self.same_thread('link') or self.finished:return False
        if len(self.links)>=32:self.problem('Child reference limit exceeded');return
        self.links.append(dict(reference))

    def snapshot(self):
        if not self.same_thread('snapshot'):raise ValueError('UnsupportedDiagnosticThread')
        end,cpu=self.now()
        return dict(schema=SCHEMA,choice=CHOICE,representation='operation_plus_immediate_caller',role=self.role,slot=self.slot,
            pid=self.pid,parent_pid=self.parent_pid,context=self.context,parent_call=self.parent_call,
            clock='local_perf_counter',cpu_clock='process_time_threads_included_children_excluded',
            start=self.started,end=end,wall=end-self.started,cpu=cpu-self.cpu_started,outcome=self.outcome,
            status='INCOMPLETE' if self.incomplete or self.stack or self.suppressed_depth else ('COMPLETE' if self.finished else 'PARTIAL'),
            status_scope='role_local_only; closed diagnostic-tree reconciliation required',
            reasons=self.incomplete[:],open_spans=len(self.stack)+self.suppressed_depth,overflow_calls=self.overflow_calls,suppressed_calls=self.suppressed_calls,
            operations=[dict(row,units=dict(row['units'])) for row in self.operations],
            edges=[dict(row,units=dict(row['units'])) for row in self.edges],combined_records=len(self.operations)+len(self.edges),
            child_references=[dict(link) for link in self.links],flushes=self.flushes,prior_writer_seconds=self.writer_seconds,
            limits=dict(roles=MAX_ROLES,records=MAX_RECORDS,snapshot_bytes=SNAPSHOT_BYTES,
                role_bytes=262144,total_bytes=8388608),
            exclusions=['Time before collector activation unless explicitly adopted',
                'Final sidecar writing and interpreter teardown after this snapshot',
                'Child CPU; unrelated process clocks cannot be subtracted',
                'Full call paths are not reconstructed; recursive inclusive totals overlap',
                'Operations and edges summarize the same invocations and must not be added together'],
            scientific_acceptance=False)

    @safe(False)
    def flush(self):
        if not self.same_thread('flush'):return False
        started=time.perf_counter()
        try:
            value=self.snapshot();raw=(json.dumps(value,sort_keys=True,allow_nan=False)+'\n').encode()
            if len(raw)>SNAPSHOT_BYTES:
                self.problem('Serialized byte limit exceeded');value['status']='INCOMPLETE';value['reasons']=self.incomplete[:]
                value['omitted_export_operations']=0;value['omitted_export_edges']=0
                while len(raw)>SNAPSHOT_BYTES and (value['edges'] or value['operations']):
                    # Remove edges before operations, so no exported edge dangles.
                    field='edges' if value['edges'] else 'operations';value[field].pop()
                    value['omitted_export_'+field]+=1
                    value['combined_records']=len(value['operations'])+len(value['edges'])
                    raw=(json.dumps(value,sort_keys=True,allow_nan=False)+'\n').encode()
            if len(raw)>SNAPSHOT_BYTES:raise ValueError('StageObservationEnvelopeOverflow')
            pending=self.path.with_name(self.path.name+'.pending')
            # Derived role directory must already be created by its existing owner.
            with pending.open('wb') as stream:
                written=stream.write(raw)
                if written!=len(raw):raise OSError('ShortStageObservationWrite')
                stream.flush();os.fsync(stream.fileno())
            os.replace(pending,self.path);self.flushes+=1
            return True
        except BaseException as error:
            self.problem('Writer failure: '+type(error).__name__)
            if not self.marker_written:
                self.marker_written=True
                try:print(('STAGE_OBSERVATIONS_INCOMPLETE role='+self.role+' '+self.incomplete[-1])[:480],file=sys.stderr,flush=True)
                except BaseException:pass
            return False
        finally:self.writer_seconds+=time.perf_counter()-started

    @safe(False)
    def finish(self,outcome='returned'):
        if not self.same_thread('finish'):return False
        self.outcome=str(outcome)[:96];self.finished=True
        if self.stack or self.suppressed_depth:self.problem('Unclosed diagnostic spans')
        return self.flush()

def enabled():return _active is not None

def activate(path,role,slot,context,parent_call=None):
    global _active,_child_index
    if _active is not None:raise ValueError('OneStageObservationRolePerProcess')
    if type(slot) is not int or not 0<=slot<MAX_ROLES:raise ValueError('InvalidStageObservationRoleSlot')
    try:_active=Collector(path,role,slot,context,parent_call);_child_index=0
    except BaseException as error:
        try:print(('STAGE_OBSERVATIONS_INCOMPLETE activation='+type(error).__name__)[:480],file=sys.stderr,flush=True)
        except BaseException:pass
        return None
    def final_flush():
        if _active is not None and not _active.finished:
            _active.problem('Role did not reach its selected finalizer');_active.finish('incomplete_exit')
    try:atexit.register(final_flush)
    except BaseException as error:_active.problem('Exit registration: '+type(error).__name__)
    return _active

@safe(_NULL)
def span(name,kind='other'):
    return _NULL if _active is None else _active.span(name,kind)

def observed(name,kind='other'):
    def decorate(call):
        @wraps(call)
        def wrapped(*args,**kwargs):
            if _active is None:return call(*args,**kwargs)
            with span(name,kind):return call(*args,**kwargs)
        return wrapped
    return decorate

def units(name,value):
    if _active is not None:_active.units(name,value)

def byte_value(value):
    units('hash_bytes',len(value));return value

def resolved(path):
    with span('path_resolution','source_path'):
        units('resolve_calls',1)
        return path.resolve()

def flush():
    if _active is not None:return _active.flush()

def finish(outcome='returned'):
    if _active is not None:return _active.finish(outcome)

@safe(None)
def child_binding(reference, *, finite=False):
    global _child_index
    if _active is None:return None
    if not _active.same_thread('child binding') or _active.finished:return None
    if finite and _active.slot!=2:
        _active.problem('Finite diagnostic child outside worker role');return None
    slots=range(3,4) if finite and _active.slot==2 else _BUDGETS.get(_active.slot,range(0))
    # finite has a disjoint fixed slot and does not consume outer-operation slots.
    if finite:
        key='finite_child_claimed'
        if getattr(_active,key,False):_active.problem('Finite diagnostic role slot exhausted');return None
        setattr(_active,key,True);index=0
    else:index=_child_index;_child_index+=1
    if index>=len(slots):_active.problem('Diagnostic child role budget exhausted');return None
    value=dict(schema=LINK_SCHEMA,choice=CHOICE,slot=slots[index],parent_slot=_active.slot,
                parent_call=self_reference(),reference=str(reference)[-1024:])
    _active.link(value);return value

def self_reference():
    return None if _active is None else str(_active.pid)+':'+str(_active.slot)+':'+_active.context

def activate_child(value,path,role,context, *, finite=False):
    if value is None:return None
    if (type(value) is not dict or set(value)!={'schema','choice','slot','parent_slot','parent_call','reference'}
            or value.get('schema')!=LINK_SCHEMA or value.get('choice')!=CHOICE):raise ValueError('InvalidBoundStageObservationChoice')
    slot=value.get('slot');parent=value.get('parent_slot')
    if type(slot) is not int or type(parent) is not int:raise ValueError('InvalidBoundStageObservationSlot')
    valid=slot==3 and parent==2 if finite else slot in _BUDGETS.get(parent,range(0))
    if not valid:raise ValueError('InvalidBoundStageObservationSlot')
    return activate(path,role,slot,context,value.get('parent_call'))
