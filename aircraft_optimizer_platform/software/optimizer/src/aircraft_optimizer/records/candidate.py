from __future__ import annotations

from dataclasses import dataclass

from aircraft_optimizer.records.common import VariableValue


@dataclass(frozen=True)
class CandidateSeed:
    aircraft_family: str
    design_variables: dict[str, VariableValue]
    normalized_design_vector: dict[str, float]
    created_by: str = "seed"
    generation: int = 0
    notes: str | None = None

    def design_variables_dict(self) -> dict[str, dict[str, object]]:
        return {key: value.to_dict() for key, value in self.design_variables.items()}

    def validate(self) -> None:
        if not self.aircraft_family:
            raise ValueError("aircraft_family is required")
        if not self.design_variables:
            raise ValueError("at least one design variable is required")
        missing_normalized = set(self.design_variables) - set(self.normalized_design_vector)
        if missing_normalized:
            joined = ", ".join(sorted(missing_normalized))
            raise ValueError(f"missing normalized values for: {joined}")
