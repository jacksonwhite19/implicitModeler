from __future__ import annotations

from aircraft_optimizer.records import CandidateSeed, MetricValue


def fixture_scoring_metrics(candidate: CandidateSeed) -> dict[str, dict[str, object]]:
    variables = candidate.design_variables
    span = float(variables["wing.span_mm"].value)
    root = float(variables["wing.root_chord_mm"].value)
    tip = float(variables["wing.tip_chord_mm"].value)
    sweep = float(variables["wing.sweep_deg"].value)

    aspect_proxy = span / ((root + tip) / 2.0)
    sweep_penalty = abs(sweep - 15.0) * 0.02
    total = aspect_proxy - sweep_penalty
    return {
        "score.fixture_total": MetricValue(
            value=round(total, 6),
            unit=None,
            confidence=0.2,
            source="fixture_scoring",
        ).to_dict(),
        "score.aspect_proxy": MetricValue(
            value=round(aspect_proxy, 6),
            unit=None,
            confidence=0.2,
            source="fixture_scoring",
        ).to_dict(),
        "score.sweep_penalty": MetricValue(
            value=round(sweep_penalty, 6),
            unit=None,
            confidence=0.2,
            source="fixture_scoring",
        ).to_dict(),
    }
