from contextlib import contextmanager,ExitStack
from contextvars import ContextVar
from datetime import datetime,timezone,timedelta
from pathlib import Path
import hashlib,json,os,threading,time

_active=ContextVar('direct_native_budget',default=None)
STAGES=('farfield','candidate','export')
LIMIT=7200.
STAGE_LIMIT=3600.

def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()

def write(path,value):
    with path.open('x',encoding='utf8') as stream:
        json.dump(value,stream,sort_keys=True,allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())

class Budget:
    def __init__(self,invocation,binding,check):
        self.invocation=Path(invocation)
        self.binding=binding
        self.check=check
        self.pid=os.getpid()
        self.thread=threading.get_ident()
        self.lock=threading.Lock()
        self.custody=ExitStack()
        self.folder=self.invocation.parent/'NATIVE_BUDGET'
        self.folder.mkdir()
        self.used=0.
        self.rows=[]
        self.failed=False
        self.finished=False
        self.invocation_sha256=sha(self.invocation)
        write(self.folder/'BUDGET.json',dict(schema='DirectNativeBudget/1',limit_seconds=LIMIT,
            invocation_path=str(self.invocation),invocation_sha256=self.invocation_sha256,
            binding=binding,pid=self.pid,stages=STAGES,stage_limit_seconds=STAGE_LIMIT,
            accounting='Sequential stage wall time including preparation, failure and cleanup; no independent credit resets.'))
        self.identity=sha(self.folder/'BUDGET.json')
        from immutable_state_access import guard
        try:
            self.custody.enter_context(guard(self.invocation))
            self.custody.enter_context(guard(self.folder/'BUDGET.json'))
        except BaseException:
            self.custody.close()
            raise

    def require(self):
        if (os.getpid()!=self.pid or threading.get_ident()!=self.thread or self.failed or self.finished
                or sha(self.invocation)!=self.invocation_sha256 or sha(self.folder/'BUDGET.json')!=self.identity):
            raise ValueError('InactiveOrChangedDirectNativeBudget')
        self.check()

    @contextmanager
    def stage(self,name):
        self.require()
        if not self.lock.acquire(blocking=False):
            raise ValueError('ConcurrentNativeCreditAllocationRejected')
        started=time.perf_counter()
        primary=None
        issued=False
        try:
            if len(self.rows)>=len(STAGES) or name!=STAGES[len(self.rows)]:
                raise ValueError('OrderedSingleNativeStageRequired')
            allowance=min(STAGE_LIMIT,LIMIT-self.used,self.check.remaining_seconds()-15)
            if allowance<=30:
                raise TimeoutError('SharedDirectNativeBudgetExhausted')
            reservation=self.folder/(name+'_RESERVATION.json')
            write(reservation,dict(schema='DirectNativeReservation/1',stage=name,
                budget_path=str(self.folder/'BUDGET.json'),budget_sha256=self.identity,
                invocation_sha256=self.invocation_sha256,binding=self.binding,
                prior_debits=list(self.rows),used_seconds=self.used,seconds=allowance,
                expires_utc=(datetime.now(timezone.utc)+timedelta(seconds=allowance)).isoformat(),
                publication_enabled=False))
            ticket=dict(path=str(reservation),sha256=sha(reservation),seconds=allowance,
                deadline=started+allowance,stage=name)
            issued=True
            from immutable_state_access import guard
            self.custody.enter_context(guard(reservation))
            yield ticket
            self.require()
        except BaseException as error:
            primary=error
            self.failed=True
            raise
        finally:
            elapsed=time.perf_counter()-started
            self.used+=elapsed
            stage_overrun=issued and elapsed>allowance
            debit_row=None
            try:
                debit=self.folder/(name+'_DEBIT.json')
                write(debit,dict(schema='DirectNativeDebit/1',stage=name,seconds=elapsed,
                    used_seconds=self.used,limit_seconds=LIMIT,issued=issued,
                    reservation_sha256=sha(reservation) if issued else None,
                    allocated_seconds=allowance if issued else None,
                    passed=primary is None and self.used<=LIMIT and not stage_overrun,
                    error=repr(primary) if primary else ('NativeStageAllowanceExceeded' if stage_overrun else None)))
                debit_row=dict(path=str(debit),sha256=sha(debit))
                self.rows.append(debit_row)
                from immutable_state_access import guard
                self.custody.enter_context(guard(debit))
                if stage_overrun:
                    self.failed=True
                    raise TimeoutError('NativeStageAllowanceExceeded')
                if self.used>LIMIT:
                    self.failed=True
                    raise TimeoutError('AggregateDirectNativeBudgetExceeded')
            except BaseException as error:
                self.failed=True
                if primary is None:
                    raise
                primary.add_note('Native budget closure: '+repr(error))
            finally:
                self.lock.release()
                cleanup_seconds=time.perf_counter()-started-elapsed
                self.used+=cleanup_seconds
                if debit_row is not None:
                    debit_row['cleanup_seconds']=cleanup_seconds
                if issued and elapsed+cleanup_seconds>allowance:
                    self.failed=True
                    if primary is None:
                        raise TimeoutError('NativeStageCleanupAllowanceExceeded')
                    primary.add_note('Native stage including cleanup exceeded its original allowance')
                if self.used>LIMIT:
                    self.failed=True
                    if primary is None:
                        raise TimeoutError('AggregateNativeLedgerCleanupExceeded')
                    primary.add_note('Aggregate native ledger cleanup exceeded7200s')

    def finish(self):
        self.require()
        if len(self.rows)!=len(STAGES) or self.lock.locked():
            raise ValueError('IncompleteDirectNativeBudget')
        write(self.folder/'COMPLETION.json',dict(schema='DirectNativeBudgetCompletion/1',
            budget_sha256=self.identity,debits=self.rows,used_seconds=self.used,
            limit_seconds=LIMIT,passed=True,scientific_acceptance=False,
            final_ledger_and_source_custody='Non-native whole-payload work; native process retirement and stage debit cleanup are charged above.'))
        self.finished=True

@contextmanager
def scope(invocation,binding,check):
    if _active.get() is not None:
        raise ValueError('NativeBudgetResetRejected')
    budget=Budget(invocation,binding,check)
    token=_active.set(budget)
    primary=None
    try:
        yield budget
        if not budget.finished:
            raise ValueError('NativeBudgetCompletionRequired')
    except BaseException as error:
        primary=error
        raise
    finally:
        _active.reset(token)
        try:
            budget.custody.close()
            check()
        except BaseException as error:
            if primary is None:
                raise
            primary.add_note('Native budget custody closure: '+repr(error))

def stage(name):
    budget=_active.get()
    if budget is None:
        raise ValueError('AuthenticatedSharedNativeBudgetRequired')
    return budget.stage(name)

def remaining(ticket):
    allowance=ticket['deadline']-time.perf_counter()-2
    if allowance<=1:
        raise TimeoutError('NativeStagePreparationConsumedCredit')
    return allowance
