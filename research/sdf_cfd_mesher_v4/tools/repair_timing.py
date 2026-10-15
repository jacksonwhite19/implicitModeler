"""Bounded diagnostic counters and nested timers; never mesh acceptance."""
from collections import defaultdict
from contextlib import contextmanager
import time
import sys


class RepairProgress:
    def __init__(self,emit=None,*,clock=time.perf_counter,interval=1.0):
        self.emit=emit;self.clock=clock;self.interval=interval
        self.started=clock();self.last=self.started;self.stack=[]
        self.inclusive=defaultdict(float);self.exclusive=defaultdict(float)
        self.calls=defaultdict(int);self.counters=defaultdict(int);self.state={}
        self.reporting_seconds=0.;self.reports=0;self.error=None;self.closed=False
        self.reporting_error=None
        self.last_span_exception=None;self.outcome='running';self.terminal_error=None;self.numeric_passed=None

    @classmethod
    def from_check(cls,check):
        owner=getattr(check,'repair_progress',None)
        return owner if isinstance(owner,cls) else cls(owner)

    @contextmanager
    def span(self,name,*,publish=False):
        frame=[name,self.clock(),0.];self.stack.append(frame);self.calls[name]+=1
        primary=None
        try:
            self.poll(force=publish)
            yield
        except BaseException as error:
            primary=error;self.error=repr(error)
            self.last_span_exception=dict(span=name,error=repr(error))
            raise
        finally:
            elapsed=self.clock()-frame[1]
            assert self.stack.pop() is frame
            self.inclusive[name]+=elapsed;self.exclusive[name]+=elapsed-frame[2]
            if self.stack:self.stack[-1][2]+=elapsed
            self.poll(force=publish,primary=primary)

    def set(self,**values):
        self.state.update(values);self.poll()

    def count(self,name,amount=1):
        self.counters[name]+=amount;self.poll()

    def snapshot(self):
        now=self.clock();inc=dict(self.inclusive);exc=dict(self.exclusive)
        for j,(name,start,children) in enumerate(self.stack):
            elapsed=now-start
            active_child=now-self.stack[j+1][1] if j+1<len(self.stack) else 0.
            inc[name]=inc.get(name,0.)+elapsed
            exc[name]=exc.get(name,0.)+elapsed-children-active_child
        return dict(schema='V4RepairTiming/1',diagnostic_only=True,scientific_acceptance=False,
            elapsed_seconds=now-self.started,active=[row[0] for row in self.stack],
            inclusive_seconds=inc,exclusive_seconds=exc,calls=dict(self.calls),
            counters=dict(self.counters),state=dict(self.state),reporting_seconds=self.reporting_seconds,
            reports=self.reports,error=self.error,closed=self.closed,reporting_error=self.reporting_error,
            error_semantics='Last observed span exception, including handled candidate rejections.',
            last_span_exception=self.last_span_exception,outcome=self.outcome,terminal_error=self.terminal_error,
            numeric_passed=self.numeric_passed,
            observed_wall_includes_reporting=True,reporting_seconds_nested=True)

    def poll(self,*,force=False,primary=None):
        if self.emit is None:return
        now=self.clock()
        if not force and now-self.last<self.interval:return
        start=now
        try:self.emit(self.snapshot())
        except BaseException as failure:
            self.reporting_error=repr(failure)
            if primary is not None:
                primary.add_note('Diagnostic reporting also failed: '+repr(failure))
                return
            raise
        finally:self.reporting_seconds+=self.clock()-start
        self.reports+=1;self.last=self.clock()

    def iterate(self,source,name):
        iterator=iter(source);missing=object()
        while True:
            with self.span(name):item=next(iterator,missing)
            if item is missing:return
            yield item

    def close(self):
        self.closed=True;self.poll(force=True,primary=sys.exception())
