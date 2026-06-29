from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--force-coeffs", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--tail-window", type=int, default=10)
    parser.add_argument("--reference-window-start", type=float, default=30.0)
    parser.add_argument("--reference-window-end", type=float, default=48.0)
    parser.add_argument("--max-tail-relative-drift", type=float, default=0.25)
    parser.add_argument(
        "--max-tail-absolute-drift",
        type=float,
        default=0.01,
        help="Allowed absolute tail/reference mean drift before relative drift is considered a failure.",
    )
    parser.add_argument("--max-abs-coefficient", type=float, default=5.0)
    args = parser.parse_args()

    rows = read_force_coeffs(args.force_coeffs)
    report = summarize(
        rows,
        args.force_coeffs,
        tail_window=args.tail_window,
        reference_window_start=args.reference_window_start,
        reference_window_end=args.reference_window_end,
        max_tail_relative_drift=args.max_tail_relative_drift,
        max_tail_absolute_drift=args.max_tail_absolute_drift,
        max_abs_coefficient=args.max_abs_coefficient,
    )
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(
        json.dumps(
            {
                "stable_for_scoring": report["stable_for_scoring"],
                "reason": report["reason"],
                "last": report["last"],
                "tail": report["tail"],
                "report": str(args.report),
            },
            indent=2,
            sort_keys=True,
        )
    )


def read_force_coeffs(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        parts = stripped.split()
        if len(parts) < 6:
            continue
        time, cm, cd, cl, clf, clr = (float(value) for value in parts[:6])
        rows.append(
            {
                "time": time,
                "Cm": cm,
                "Cd": cd,
                "Cl": cl,
                "Cl_f": clf,
                "Cl_r": clr,
            }
        )
    if not rows:
        raise ValueError(f"No force coefficient rows found in {path}")
    return rows


def summarize(
    rows: list[dict[str, float]],
    path: Path,
    *,
    tail_window: int,
    reference_window_start: float,
    reference_window_end: float,
    max_tail_relative_drift: float,
    max_tail_absolute_drift: float,
    max_abs_coefficient: float,
) -> dict[str, object]:
    tail_rows = rows[-tail_window:]
    reference_rows = [
        row for row in rows if reference_window_start <= row["time"] <= reference_window_end
    ]
    if not reference_rows:
        reference_rows = rows[: max(1, len(rows) - tail_window)]

    last = rows[-1]
    tail = coefficient_stats(tail_rows)
    reference = coefficient_stats(reference_rows)
    all_stats = coefficient_stats(rows)

    drift = {}
    absolute_drift = {}
    reasons = []
    for key in ("Cd", "Cl", "Cm"):
        absolute_drift[key] = tail[key]["mean"] - reference[key]["mean"]
        drift[key] = relative_delta(tail[key]["mean"], reference[key]["mean"])
        if (
            abs(absolute_drift[key]) > max_tail_absolute_drift
            and abs(drift[key]) > max_tail_relative_drift
        ):
            reasons.append(
                f"{key} tail mean drift {drift[key]:.3g} exceeds {max_tail_relative_drift} "
                f"and absolute drift {absolute_drift[key]:.3g} exceeds {max_tail_absolute_drift}"
            )
        if abs(last[key]) > max_abs_coefficient:
            reasons.append(f"{key} final magnitude {last[key]:.3g} exceeds {max_abs_coefficient}")
        if abs(all_stats[key]["max_abs"]) > max_abs_coefficient:
            reasons.append(
                f"{key} max magnitude {all_stats[key]['max_abs']:.3g} exceeds {max_abs_coefficient}"
            )

    return {
        "force_coeffs": str(path),
        "row_count": len(rows),
        "time_start": rows[0]["time"],
        "time_end": rows[-1]["time"],
        "last": compact_coefficients(last),
        "tail_window": tail_window,
        "tail": tail,
        "reference_window": {
            "start": reference_window_start,
            "end": reference_window_end,
            "row_count": len(reference_rows),
            "stats": reference,
        },
        "all": all_stats,
        "tail_relative_drift_vs_reference": drift,
        "tail_absolute_drift_vs_reference": absolute_drift,
        "max_tail_relative_drift_allowed": max_tail_relative_drift,
        "max_tail_absolute_drift_allowed": max_tail_absolute_drift,
        "max_abs_coefficient_allowed": max_abs_coefficient,
        "stable_for_scoring": not reasons,
        "reason": "pass" if not reasons else "; ".join(reasons),
    }


def coefficient_stats(rows: list[dict[str, float]]) -> dict[str, dict[str, float]]:
    stats = {}
    for key in ("Cm", "Cd", "Cl", "Cl_f", "Cl_r"):
        values = [row[key] for row in rows]
        stats[key] = {
            "first": values[0],
            "last": values[-1],
            "min": min(values),
            "max": max(values),
            "max_abs": max(abs(value) for value in values),
            "mean": statistics.fmean(values),
            "stdev": statistics.pstdev(values) if len(values) > 1 else 0.0,
            "delta_last_minus_first": values[-1] - values[0],
        }
    return stats


def compact_coefficients(row: dict[str, float]) -> dict[str, float]:
    return {key: row[key] for key in ("time", "Cm", "Cd", "Cl", "Cl_f", "Cl_r")}


def relative_delta(value: float, reference: float) -> float:
    denominator = max(abs(reference), 1e-12)
    return (value - reference) / denominator


if __name__ == "__main__":
    main()
