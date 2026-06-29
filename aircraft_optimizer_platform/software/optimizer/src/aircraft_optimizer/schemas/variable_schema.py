from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from aircraft_optimizer.records import CandidateSeed


@dataclass(frozen=True)
class VariableSchema:
    schema_id: str
    aircraft_family: str
    schema_version: str
    variables: dict[str, dict[str, Any]]
    normalized_policy: dict[str, Any]
    raw: dict[str, Any]

    def validate_candidate(self, candidate: CandidateSeed) -> None:
        if candidate.aircraft_family != self.aircraft_family:
            raise ValueError(
                f"candidate aircraft_family {candidate.aircraft_family!r} does not match schema {self.aircraft_family!r}"
            )
        for name, spec in self.variables.items():
            if not spec.get("required", False):
                continue
            if name not in candidate.design_variables:
                raise ValueError(f"missing required variable {name}")

        for name, variable in candidate.design_variables.items():
            if name not in self.variables:
                raise ValueError(f"unknown design variable {name}")
            spec = self.variables[name]
            if variable.unit != spec.get("unit"):
                raise ValueError(
                    f"variable {name} unit {variable.unit!r} does not match schema unit {spec.get('unit')!r}"
                )
            value = float(variable.value)
            minimum = spec.get("min")
            maximum = spec.get("max")
            if minimum is not None and value < float(minimum):
                raise ValueError(f"variable {name} value {value} is below minimum {minimum}")
            if maximum is not None and value > float(maximum):
                raise ValueError(f"variable {name} value {value} exceeds maximum {maximum}")

        if self.normalized_policy.get("required", False):
            required_normalized = {
                name for name, spec in self.variables.items() if spec.get("required", False)
            } | set(candidate.design_variables)
            for name in required_normalized:
                if name not in candidate.normalized_design_vector:
                    raise ValueError(f"missing normalized value for {name}")

        norm_min = float(self.normalized_policy.get("min", 0.0))
        norm_max = float(self.normalized_policy.get("max", 1.0))
        for name, value in candidate.normalized_design_vector.items():
            if name not in self.variables:
                raise ValueError(f"unknown normalized variable {name}")
            normalized = float(value)
            if normalized < norm_min or normalized > norm_max:
                raise ValueError(
                    f"normalized variable {name} value {normalized} is outside [{norm_min}, {norm_max}]"
                )


def load_variable_schema(path: Path) -> VariableSchema:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("variable schema must be a JSON object")
    variables = data.get("variables")
    if not isinstance(variables, dict) or not variables:
        raise ValueError("variable schema must define variables")
    normalized_policy = data.get("normalized_policy", {})
    if not isinstance(normalized_policy, dict):
        raise ValueError("normalized_policy must be an object")
    return VariableSchema(
        schema_id=str(data["schema_id"]),
        aircraft_family=str(data["aircraft_family"]),
        schema_version=str(data["schema_version"]),
        variables=variables,
        normalized_policy=normalized_policy,
        raw=data,
    )
