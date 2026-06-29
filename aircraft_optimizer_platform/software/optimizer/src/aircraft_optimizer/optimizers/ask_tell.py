from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from aircraft_optimizer.records import CandidateSeed, VariableValue


@dataclass(frozen=True)
class OptimizerObservation:
    candidate_id: str
    status: str
    score: float | None
    failure_category: str | None


@dataclass(frozen=True)
class OptimizerProposal:
    name: str
    candidate: CandidateSeed
    parent_candidate_ids: list[str]
    rationale: str
    optimizer_state: dict[str, Any]


class HaltonAskTellPolicy:
    """Minimal deterministic ask/tell policy for early optimizer plumbing."""

    def __init__(self, schema: dict[str, Any]) -> None:
        self._schema = schema
        self._variables = list(schema["variables"].keys())
        self._observations: list[OptimizerObservation] = []

    def ask(self, iteration_index: int) -> OptimizerProposal:
        normalized = self._normalized_vector(iteration_index)
        candidate = CandidateSeed(
            aircraft_family=self._schema["aircraft_family"],
            design_variables={
                name: VariableValue(
                    self._denormalize(name, normalized[name]),
                    self._schema["variables"][name].get("unit"),
                )
                for name in self._variables
            },
            normalized_design_vector=normalized,
            created_by="halton_ask_tell_policy",
            generation=iteration_index,
            notes=f"Ask/tell Halton proposal {iteration_index}.",
        )
        best = self.best_observation()
        parent_candidate_ids = [best.candidate_id] if best and iteration_index > 0 else []
        return OptimizerProposal(
            name=f"halton_proposal_{iteration_index}",
            candidate=candidate,
            parent_candidate_ids=parent_candidate_ids,
            rationale=self._rationale(iteration_index, best),
            optimizer_state={
                "policy": "halton_ask_tell",
                "iteration_index": iteration_index,
                "normalized_design_vector": normalized,
                "best_candidate_id_before_iteration": best.candidate_id if best else None,
                "observation_count": len(self._observations),
            },
        )

    def tell(self, observation: OptimizerObservation) -> None:
        self._observations.append(observation)

    def best_observation(self) -> OptimizerObservation | None:
        scored = [
            observation
            for observation in self._observations
            if observation.status == "complete" and observation.score is not None
        ]
        if not scored:
            return None
        return max(scored, key=lambda observation: float(observation.score))

    @property
    def observations(self) -> list[OptimizerObservation]:
        return list(self._observations)

    def _normalized_vector(self, iteration_index: int) -> dict[str, float]:
        if iteration_index == 0:
            return {name: 0.5 for name in self._variables}
        bases = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41]
        if len(self._variables) > len(bases):
            raise ValueError(
                f"HaltonAskTellPolicy supports {len(bases)} variables; schema requested {len(self._variables)}"
            )
        return {
            name: round(_halton(iteration_index, bases[index]), 6)
            for index, name in enumerate(self._variables)
        }

    def _denormalize(self, variable_name: str, normalized: float) -> float:
        variable = self._schema["variables"][variable_name]
        minimum = float(variable["min"])
        maximum = float(variable["max"])
        return round(minimum + normalized * (maximum - minimum), 6)

    def _rationale(
        self,
        iteration_index: int,
        best: OptimizerObservation | None,
    ) -> str:
        if iteration_index == 0:
            return "Start from the center of the fixed-wing variable bounds."
        if best is None:
            return "Continue deterministic space-filling exploration; no successful best candidate exists yet."
        return f"Continue deterministic space-filling exploration using {best.candidate_id} as current best parent."


def _halton(index: int, base: int) -> float:
    result = 0.0
    fraction = 1.0 / base
    value = index
    while value > 0:
        result += fraction * (value % base)
        value //= base
        fraction /= base
    return result
