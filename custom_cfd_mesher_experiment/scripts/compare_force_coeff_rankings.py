from __future__ import annotations

import argparse
import csv
import json
import statistics
from pathlib import Path
from typing import Any


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--confirmation-run-root", type=Path, default=None)
    parser.add_argument("--case-glob", default="*/kOmegaSST_laminarStart_k002_om50_p01_u02_100")
    parser.add_argument(
        "--ranking-case-glob",
        default="*/kOmegaSST_laminarStart_k002_om50_p01_u02_60",
    )
    parser.add_argument(
        "--confirmation-case-glob",
        default="*/kOmegaSST_laminarStart_k002_om50_p01_u02_100",
    )
    parser.add_argument("--ranking-time", type=float, default=60.0)
    parser.add_argument("--confirmation-time", type=float, default=100.0)
    parser.add_argument("--window", type=int, default=5)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows = (
        compare_between_roots(args)
        if args.confirmation_run_root is not None
        else compare_within_root(args)
    )

    rankings = {
        "Cd_low_is_better": rank_correlation(rows, "ranking_cd_mean", "confirmation_cd_mean", reverse=False),
        "Cl_high_is_better": rank_correlation(rows, "ranking_cl_mean", "confirmation_cl_mean", reverse=True),
        "abs_Cm_low_is_better": rank_correlation_abs(rows, "ranking_cm_mean", "confirmation_cm_mean"),
    }
    method_note = (
        "Separate-root study: ranking values come from the ranking run root and "
        "confirmation values come from the confirmation run root."
        if args.confirmation_run_root is not None
        else (
            "Proxy study: ranking values are sampled from the same completed 100-step "
            "kOmegaSST runs at time 60, not from separate reruns."
        )
    )
    summary = {
        "run_root": str(args.run_root),
        "confirmation_run_root": str(args.confirmation_run_root)
        if args.confirmation_run_root is not None
        else None,
        "case_glob": args.case_glob,
        "ranking_case_glob": args.ranking_case_glob,
        "confirmation_case_glob": args.confirmation_case_glob,
        "candidate_count": len(rows),
        "ranking_time_requested": args.ranking_time,
        "confirmation_time_requested": args.confirmation_time,
        "window": args.window,
        "method_note": method_note,
        "rank_correlations": rankings,
        "rows": rows,
    }

    csv_path = args.output_dir / "ranking_vs_confirmation_coefficients.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = list(rows[0].keys()) if rows else ["variant_id"]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    json_path = args.output_dir / "ranking_vs_confirmation_summary.json"
    json_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")

    md_path = args.output_dir / "ranking_vs_confirmation_summary.md"
    md_path.write_text(render_markdown(summary, csv_path, json_path), encoding="utf-8")
    print(json.dumps({"summary": str(json_path), "csv": str(csv_path), "markdown": str(md_path)}, indent=2))


def compare_within_root(args: argparse.Namespace) -> list[dict[str, Any]]:
    rows = []
    for case_dir in sorted(args.run_root.glob(args.case_glob)):
        force_path = (
            case_dir / "postProcessing" / "aircraftForceCoeffs" / "0" / "forceCoeffs.dat"
        )
        if not force_path.exists():
            continue
        force_rows = read_force_coeffs(force_path)
        if not force_rows:
            continue
        ranking = summarize_at_time(force_rows, args.ranking_time, args.window)
        confirmation = summarize_at_time(force_rows, args.confirmation_time, args.window)
        rows.append(
            {
                "variant_id": case_dir.parent.name,
                "case_dir": str(case_dir),
                "force_coeffs": str(force_path),
                "ranking_time": ranking["time"],
                "confirmation_time": confirmation["time"],
                "ranking_cd_mean": ranking["Cd_mean"],
                "ranking_cl_mean": ranking["Cl_mean"],
                "ranking_cm_mean": ranking["Cm_mean"],
                "confirmation_cd_mean": confirmation["Cd_mean"],
                "confirmation_cl_mean": confirmation["Cl_mean"],
                "confirmation_cm_mean": confirmation["Cm_mean"],
                "delta_cd": confirmation["Cd_mean"] - ranking["Cd_mean"],
                "delta_cl": confirmation["Cl_mean"] - ranking["Cl_mean"],
                "delta_cm": confirmation["Cm_mean"] - ranking["Cm_mean"],
            }
        )
    return rows


def compare_between_roots(args: argparse.Namespace) -> list[dict[str, Any]]:
    confirmation_cases = {
        case_dir.parent.name: case_dir
        for case_dir in sorted(args.confirmation_run_root.glob(args.confirmation_case_glob))
    }
    rows = []
    for ranking_case_dir in sorted(args.run_root.glob(args.ranking_case_glob)):
        variant_id = ranking_case_dir.parent.name
        confirmation_case_dir = confirmation_cases.get(variant_id)
        if confirmation_case_dir is None:
            continue
        ranking_force_path = (
            ranking_case_dir
            / "postProcessing"
            / "aircraftForceCoeffs"
            / "0"
            / "forceCoeffs.dat"
        )
        confirmation_force_path = (
            confirmation_case_dir
            / "postProcessing"
            / "aircraftForceCoeffs"
            / "0"
            / "forceCoeffs.dat"
        )
        if not ranking_force_path.exists() or not confirmation_force_path.exists():
            continue
        ranking_rows = read_force_coeffs(ranking_force_path)
        confirmation_rows = read_force_coeffs(confirmation_force_path)
        if not ranking_rows or not confirmation_rows:
            continue
        ranking = summarize_at_time(ranking_rows, args.ranking_time, args.window)
        confirmation = summarize_at_time(
            confirmation_rows,
            args.confirmation_time,
            args.window,
        )
        rows.append(
            {
                "variant_id": variant_id,
                "case_dir": str(ranking_case_dir),
                "confirmation_case_dir": str(confirmation_case_dir),
                "force_coeffs": str(ranking_force_path),
                "confirmation_force_coeffs": str(confirmation_force_path),
                "ranking_time": ranking["time"],
                "confirmation_time": confirmation["time"],
                "ranking_cd_mean": ranking["Cd_mean"],
                "ranking_cl_mean": ranking["Cl_mean"],
                "ranking_cm_mean": ranking["Cm_mean"],
                "confirmation_cd_mean": confirmation["Cd_mean"],
                "confirmation_cl_mean": confirmation["Cl_mean"],
                "confirmation_cm_mean": confirmation["Cm_mean"],
                "delta_cd": confirmation["Cd_mean"] - ranking["Cd_mean"],
                "delta_cl": confirmation["Cl_mean"] - ranking["Cl_mean"],
                "delta_cm": confirmation["Cm_mean"] - ranking["Cm_mean"],
            }
        )
    return rows


def read_force_coeffs(path: Path) -> list[dict[str, float]]:
    rows = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        parts = stripped.split()
        if len(parts) < 6:
            continue
        rows.append(
            {
                "time": float(parts[0]),
                "Cm": float(parts[1]),
                "Cd": float(parts[2]),
                "Cl": float(parts[3]),
                "Cl_f": float(parts[4]),
                "Cl_r": float(parts[5]),
            }
        )
    return rows


def summarize_at_time(rows: list[dict[str, float]], target_time: float, window: int) -> dict[str, float]:
    closest_index = min(range(len(rows)), key=lambda index: abs(rows[index]["time"] - target_time))
    start = max(0, closest_index - window + 1)
    selected = rows[start : closest_index + 1]
    return {
        "time": rows[closest_index]["time"],
        "Cd_mean": statistics.fmean(row["Cd"] for row in selected),
        "Cl_mean": statistics.fmean(row["Cl"] for row in selected),
        "Cm_mean": statistics.fmean(row["Cm"] for row in selected),
    }


def rank_correlation(
    rows: list[dict[str, Any]],
    ranking_key: str,
    confirmation_key: str,
    *,
    reverse: bool,
) -> dict[str, Any]:
    ranking = ranks(rows, ranking_key, reverse=reverse)
    confirmation = ranks(rows, confirmation_key, reverse=reverse)
    return summarize_rank_match(ranking, confirmation)


def rank_correlation_abs(
    rows: list[dict[str, Any]],
    ranking_key: str,
    confirmation_key: str,
) -> dict[str, Any]:
    ranking_values = {row["variant_id"]: abs(float(row[ranking_key])) for row in rows}
    confirmation_values = {row["variant_id"]: abs(float(row[confirmation_key])) for row in rows}
    ranking = ranks_from_values(ranking_values, reverse=False)
    confirmation = ranks_from_values(confirmation_values, reverse=False)
    return summarize_rank_match(ranking, confirmation)


def ranks(rows: list[dict[str, Any]], key: str, *, reverse: bool) -> dict[str, int]:
    values = {row["variant_id"]: float(row[key]) for row in rows}
    return ranks_from_values(values, reverse=reverse)


def ranks_from_values(values: dict[str, float], *, reverse: bool) -> dict[str, int]:
    ordered = sorted(values.items(), key=lambda item: item[1], reverse=reverse)
    return {variant_id: index + 1 for index, (variant_id, _) in enumerate(ordered)}


def summarize_rank_match(ranking: dict[str, int], confirmation: dict[str, int]) -> dict[str, Any]:
    variant_ids = sorted(set(ranking) & set(confirmation))
    n = len(variant_ids)
    if n < 2:
        return {"candidate_count": n, "spearman": None, "exact_order_match": None}
    diffs = [ranking[variant_id] - confirmation[variant_id] for variant_id in variant_ids]
    spearman = 1.0 - (6.0 * sum(diff * diff for diff in diffs)) / (n * (n * n - 1))
    return {
        "candidate_count": n,
        "spearman": spearman,
        "exact_order_match": all(diff == 0 for diff in diffs),
        "max_rank_delta": max(abs(diff) for diff in diffs),
        "ranking_order": order_from_ranks(ranking),
        "confirmation_order": order_from_ranks(confirmation),
    }


def order_from_ranks(rank_map: dict[str, int]) -> list[str]:
    return [variant_id for variant_id, _rank in sorted(rank_map.items(), key=lambda item: item[1])]


def render_markdown(summary: dict[str, Any], csv_path: Path, json_path: Path) -> str:
    lines = [
        "# Ranking CFD Proxy Correlation Study",
        "",
        f"Run root: `{summary['run_root']}`",
        "",
        summary["method_note"],
        "",
        "## Rank Correlations",
        "",
        "| Metric | Spearman | Max Rank Delta | Exact Order Match |",
        "|---|---:|---:|---|",
    ]
    for name, result in summary["rank_correlations"].items():
        spearman = result["spearman"]
        lines.append(
            "| "
            + name
            + " | "
            + ("n/a" if spearman is None else f"{spearman:.3f}")
            + " | "
            + str(result.get("max_rank_delta", "n/a"))
            + " | "
            + str(result.get("exact_order_match"))
            + " |"
        )
    lines.extend(
        [
            "",
            "## Coefficients",
            "",
            "| Variant | Cd @ ranking | Cd @ confirmation | Cl @ ranking | Cl @ confirmation | Cm @ ranking | Cm @ confirmation |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in summary["rows"]:
        lines.append(
            f"| {row['variant_id']} | "
            f"{row['ranking_cd_mean']:.6f} | {row['confirmation_cd_mean']:.6f} | "
            f"{row['ranking_cl_mean']:.6f} | {row['confirmation_cl_mean']:.6f} | "
            f"{row['ranking_cm_mean']:.6f} | {row['confirmation_cm_mean']:.6f} |"
        )
    lines.extend(["", f"CSV: `{csv_path}`", f"JSON: `{json_path}`", ""])
    return "\n".join(lines)


if __name__ == "__main__":
    main()
