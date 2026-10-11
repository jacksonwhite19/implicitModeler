"""Execution obligations; these never select different numerical thresholds."""
from enum import Enum
from dataclasses import dataclass


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

    def commit(self, candidate, expected_generation, validate):
        if expected_generation != self.generation: raise ValueError('StaleTransactionGeneration')
        if validate(candidate) is not True: raise ValueError('RejectedMemoryTransaction')
        if expected_generation != self.generation: raise ValueError('ConcurrentMemoryTransaction')
        self.current=candidate;self.generation+=1
        return self.current
