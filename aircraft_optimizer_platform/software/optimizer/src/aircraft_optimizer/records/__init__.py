"""Typed record helpers for the platform skeleton."""

from aircraft_optimizer.records.candidate import CandidateSeed
from aircraft_optimizer.records.common import VariableValue
from aircraft_optimizer.records.failure import FailureDraft
from aircraft_optimizer.records.module import MetricValue

__all__ = ["CandidateSeed", "FailureDraft", "MetricValue", "VariableValue"]
