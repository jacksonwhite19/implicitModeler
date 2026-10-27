"""Execution obligations; these never select different numerical thresholds."""
from enum import Enum
from dataclasses import dataclass
from contextlib import contextmanager
import threading


class ExecutionProfile(str, Enum):
    ProductionFast = 'ProductionFast'
    Qualification = 'Qualification'
    Diagnostic = 'Diagnostic'

    @property
    def durable(self): return self is not self.ProductionFast

    @property
    def diagnostic(self): return self is self.Diagnostic


@dataclass
class MemoryTransaction:
    """Private candidate publication with stale-generation protection.

    The owner must give the candidate independent mutable containers. Failed
    validation never publishes. Durable replay is not implied by this receipt.
    """
    current: object
    generation: int = 0

    def __setattr__(self,name,value):
        if name in ('current','generation') and getattr(self,'_exclusive_owner',None) is not None:
            raise ValueError('MemoryTransactionExclusivelyLeased')
        object.__setattr__(self,name,value)

    @contextmanager
    def exclusive(self):
        if getattr(self,'_exclusive_owner',None) is not None:
            raise ValueError('NestedMemoryTransactionLease')
        owner=object()
        thread=threading.get_ident()
        self._exclusive_owner=owner
        def require():
            if self._exclusive_owner is not owner or threading.get_ident()!=thread:
                raise ValueError('ChangedMemoryTransactionLease')
        primary=None
        try:
            require()
            yield require
            require()
        except BaseException as error:
            primary=error
            raise
        finally:
            changed=self._exclusive_owner is not owner
            self._exclusive_owner=None
            if changed:
                error=ValueError('ChangedMemoryTransactionLease')
                if primary is None:raise error
                primary.add_note(str(error))

    def commit(self, candidate, expected_generation, validate):
        if getattr(self,'_exclusive_owner',None) is not None:raise ValueError('MemoryTransactionExclusivelyLeased')
        if expected_generation != self.generation: raise ValueError('StaleTransactionGeneration')
        if validate(candidate) is not True: raise ValueError('RejectedMemoryTransaction')
        if expected_generation != self.generation: raise ValueError('ConcurrentMemoryTransaction')
        self.current=candidate;self.generation+=1
        return self.current
