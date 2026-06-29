from __future__ import annotations

import csv
from pathlib import Path

from aircraft_optimizer.records import MetricValue


def parse_su2_history_metrics(path: Path) -> dict[str, dict[str, object]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError("SU2 history file has no data rows")
    final = rows[-1]
    lift = _float_field(final, "CL")
    drag = _float_field(final, "CD")
    if drag == 0.0:
        raise ValueError("SU2 history drag coefficient is zero")
    metrics = {
        "cfd.lift_coefficient": MetricValue(
            value=lift,
            unit=None,
            confidence=0.5,
            source="su2_history_fixture",
        ).to_dict(),
        "cfd.drag_coefficient": MetricValue(
            value=drag,
            unit=None,
            confidence=0.5,
            source="su2_history_fixture",
        ).to_dict(),
        "cfd.lift_to_drag": MetricValue(
            value=lift / drag,
            unit=None,
            confidence=0.5,
            source="su2_history_fixture",
        ).to_dict(),
        "cfd.iterations": MetricValue(
            value=int(float(final.get("Inner_Iter", len(rows)))),
            unit="count",
            confidence=1.0,
            source="su2_history_fixture",
        ).to_dict(),
    }
    if "CMz" in final:
        metrics["cfd.moment_coefficient_pitch"] = MetricValue(
            value=_float_field(final, "CMz"),
            unit=None,
            confidence=0.5,
            source="su2_history_fixture",
        ).to_dict()
    if "rms[Rho]" in final:
        metrics["cfd.final_residual"] = MetricValue(
            value=_float_field(final, "rms[Rho]"),
            unit=None,
            confidence=1.0,
            source="su2_history_fixture",
        ).to_dict()
    return metrics


def _float_field(row: dict[str, str], key: str) -> float:
    if key not in row:
        raise ValueError(f"SU2 history missing required field {key}")
    return float(row[key])
