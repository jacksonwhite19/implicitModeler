from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MetricValue:
    value: object
    unit: str | None
    confidence: float | None
    source: str

    def to_dict(self) -> dict[str, object]:
        return {
            "value": self.value,
            "unit": self.unit,
            "confidence": self.confidence,
            "source": self.source,
        }
