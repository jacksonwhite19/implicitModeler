from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class VariableValue:
    value: float | int | str | bool
    unit: str | None = None

    def to_dict(self) -> dict[str, object]:
        return {"value": self.value, "unit": self.unit}
