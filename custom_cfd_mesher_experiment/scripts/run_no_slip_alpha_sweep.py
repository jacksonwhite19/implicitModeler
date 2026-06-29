from __future__ import annotations

import argparse
import json
import math
import shutil
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from run_no_slip_laminar_start_rans import (
    add_yplus_function,
    run_openfoam,
    setup_case,
    summarize_case,
    summarize_yplus,
    find_latest_time_field,
    set_turbulence_relaxation,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--variant-ids", default="")
    parser.add_argument("--alphas-deg", default="-2,0,4,8,10")
    parser.add_argument("--stage-name", default="full_survivor_sweep")
    parser.add_argument("--stage-role", default="survivor_expanded_rough_scoring")
    parser.add_argument("--speed-mps", type=float, default=22.352)
    parser.add_argument("--end-time", type=int, default=60)
    parser.add_argument("--write-interval", type=int, default=10)
    parser.add_argument("--rho-inf", type=float, default=1.225)
    parser.add_argument("--l-ref", type=float, default=0.1325)
    parser.add_argument("--a-ref", type=float, default=0.1007)
    parser.add_argument("--cofr", default="0.35,0,0.04")
    parser.add_argument(
        "--vertical-axis",
        choices=["y", "z"],
        default="z",
        help="Aircraft vertical/lift axis. Use y for current real no-inlet exporter STLs where z is span.",
    )
    parser.add_argument("--k", type=float, default=0.02)
    parser.add_argument("--omega", type=float, default=50.0)
    parser.add_argument("--parallel-procs", type=int, default=8)
    parser.add_argument("--timeout-s", type=int, default=2400)
    parser.add_argument(
        "--wall-treatment",
        choices=["wall_function", "low_re"],
        default="wall_function",
    )
    parser.add_argument(
        "--summary-name",
        default="alpha_sweep_summary.json",
    )
    parser.add_argument(
        "--reuse-existing",
        action="store_true",
        help="Summarize existing per-alpha case folders instead of rerunning OpenFOAM.",
    )
    parser.add_argument(
        "--early-prune",
        action="store_true",
        help="Skip remaining alpha cases only after multiple clearly bad alpha results.",
    )
    parser.add_argument("--early-prune-min-completed-alphas", type=int, default=2)
    parser.add_argument("--early-prune-no-lift-cl-max", type=float, default=0.02)
    parser.add_argument("--early-prune-weak-lift-cl-max", type=float, default=0.08)
    parser.add_argument("--early-prune-min-best-ld", type=float, default=0.5)
    parser.add_argument("--early-prune-max-abs-coefficient", type=float, default=10.0)
    args = parser.parse_args()

    selected = {item.strip() for item in args.variant_ids.split(",") if item.strip()}
    alphas = [float(item.strip()) for item in args.alphas_deg.split(",") if item.strip()]
    reports = []
    started = time.perf_counter()
    for variant_dir in sorted(args.run_root.iterdir()):
        if not variant_dir.is_dir() or variant_dir.name.startswith("."):
            continue
        if selected and variant_dir.name not in selected:
            continue
        if not (variant_dir / "openfoam_case" / "constant" / "polyMesh").exists():
            continue
        reports.append(run_variant_alpha_sweep(variant_dir, alphas, args))

    summary = {
        "schema": "no_slip_alpha_sweep.v0_1",
        "stage_name": args.stage_name,
        "stage_role": args.stage_role,
        "reuse_existing": args.reuse_existing,
        "run_root": str(args.run_root),
        "alphas_deg": alphas,
        "runtime_s": time.perf_counter() - started,
        "variant_count": len(reports),
        "early_prune_enabled": args.early_prune,
        "early_pruned_variant_count": sum(
            1 for report in reports if report.get("early_pruning", {}).get("pruned")
        ),
        "reports": reports,
    }
    (args.run_root / args.summary_name).write_text(
        json.dumps(summary, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True))


def run_variant_alpha_sweep(
    variant_dir: Path,
    alphas: list[float],
    args: argparse.Namespace,
) -> dict[str, Any]:
    started = time.perf_counter()
    alpha_reports = []
    prune_decision: dict[str, Any] | None = None
    for index, alpha in enumerate(alphas):
        alpha_reports.append(run_alpha_case(variant_dir, alpha, args))
        remaining = alphas[index + 1 :]
        prune_decision = early_prune_decision(alpha_reports, args)
        if prune_decision.get("prune") and remaining:
            skipped_reports = [
                skipped_alpha_report(
                    alpha_deg=skipped_alpha,
                    variant_dir=variant_dir,
                    args=args,
                    decision=prune_decision,
                )
                for skipped_alpha in remaining
            ]
            alpha_reports.extend(skipped_reports)
            prune_decision["skipped_alphas_deg"] = remaining
            break
    aggregate = aggregate_polar(alpha_reports)
    report = {
        "variant_id": variant_dir.name,
        "stage_name": args.stage_name,
        "stage_role": args.stage_role,
        "reuse_existing": args.reuse_existing,
        "source_case": str(variant_dir / "openfoam_case"),
        "alphas_deg": alphas,
        "alpha_reports": alpha_reports,
        "aggregate": aggregate,
        "early_pruning": {
            "enabled": bool(args.early_prune),
            "pruned": bool(prune_decision and prune_decision.get("prune")),
            "decision": prune_decision if prune_decision and prune_decision.get("prune") else None,
        },
        "runtime_s": time.perf_counter() - started,
    }
    (variant_dir / args.summary_name).write_text(
        json.dumps(report, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return report


def skipped_alpha_report(
    *,
    alpha_deg: float,
    variant_dir: Path,
    args: argparse.Namespace,
    decision: dict[str, Any],
) -> dict[str, Any]:
    slug = alpha_slug(alpha_deg)
    velocity = alpha_velocity(args.speed_mps, alpha_deg, vertical_axis=args.vertical_axis)
    drag_dir = normalize(velocity)
    lift_dir = alpha_lift_dir(alpha_deg, vertical_axis=args.vertical_axis)
    return {
        "alpha_deg": alpha_deg,
        "stage_name": args.stage_name,
        "stage_role": args.stage_role,
        "velocity": velocity,
        "drag_dir": drag_dir,
        "lift_dir": lift_dir,
        "laminar_case": str(variant_dir / f"alpha_{slug}_steady_laminar_{args.end_time}"),
        "rans_case": str(variant_dir / f"alpha_{slug}_kOmegaSST_laminarStart_{args.end_time}"),
        "commands": [],
        "skipped_by_early_prune": True,
        "early_prune_reason": decision.get("reason"),
        "laminar": {"completed": False, "skipped": "early alpha prune"},
        "rans": {"completed": False, "skipped": "early alpha prune"},
    }


def run_alpha_case(
    variant_dir: Path,
    alpha_deg: float,
    args: argparse.Namespace,
) -> dict[str, Any]:
    slug = alpha_slug(alpha_deg)
    velocity = alpha_velocity(args.speed_mps, alpha_deg, vertical_axis=args.vertical_axis)
    drag_dir = normalize(velocity)
    lift_dir = alpha_lift_dir(alpha_deg, vertical_axis=args.vertical_axis)
    pitch_axis = "0,0,1" if args.vertical_axis == "y" else "0,1,0"
    alpha_args = SimpleNamespace(
        velocity=vector_arg(velocity),
        write_interval=args.write_interval,
        rho_inf=args.rho_inf,
        mag_u_inf=args.speed_mps,
        l_ref=args.l_ref,
        a_ref=args.a_ref,
        cofr=args.cofr,
        lift_dir=vector_arg(lift_dir),
        drag_dir=vector_arg(drag_dir),
        pitch_axis=pitch_axis,
        k=args.k,
        omega=args.omega,
        wall_treatment=args.wall_treatment,
        timeout_s=args.timeout_s,
        parallel_procs=args.parallel_procs,
        yplus_target_min_p50=30.0,
        yplus_target_max_p50=100.0,
        yplus_target_max_p95=150.0,
        yplus_target_max_max=300.0,
    )
    source_case = variant_dir / "openfoam_case"
    laminar_case = variant_dir / f"alpha_{slug}_steady_laminar_{args.end_time}"
    rans_case = variant_dir / f"alpha_{slug}_kOmegaSST_laminarStart_{args.end_time}"
    report: dict[str, Any] = {
        "alpha_deg": alpha_deg,
        "stage_name": args.stage_name,
        "stage_role": args.stage_role,
        "velocity": velocity,
        "drag_dir": drag_dir,
        "lift_dir": lift_dir,
        "laminar_case": str(laminar_case),
        "rans_case": str(rans_case),
        "commands": [],
    }
    if args.reuse_existing:
        report["reused_existing"] = True
        try:
            if (laminar_case / "log.foamRun").exists():
                summarize_case(
                    laminar_case,
                    report,
                    "laminar",
                    tail_window=20,
                    end_time=args.end_time,
                )
        except Exception as exc:
            report["laminar"] = {"completed": False, "error": str(exc)}
        try:
            summarize_case(
                rans_case,
                report,
                "rans",
                tail_window=20,
                end_time=args.end_time,
            )
            yplus_path = find_latest_time_field(
                rans_case,
                "yPlus",
                preferred_time=args.end_time,
            )
            report["rans"]["yplus"] = (
                summarize_yplus(yplus_path) if yplus_path else {"available": 0}
            )
        except Exception as exc:
            report["rans"] = {"completed": False, "error": str(exc)}
        return report

    try:
        setup_case(
            source_case,
            laminar_case,
            alpha_args,
            turbulence_model="laminar",
            end_time=args.end_time,
            p_relax=0.2,
            u_relax=0.3,
        )
        run_openfoam(
            laminar_case,
            args.timeout_s,
            report,
            "laminar",
            args.parallel_procs,
        )
        summarize_case(
            laminar_case,
            report,
            "laminar",
            tail_window=20,
            end_time=args.end_time,
        )
    except Exception as exc:
        report["laminar"] = {"completed": False, "error": str(exc)}

    laminar_ok = (
        bool(report.get("laminar", {}).get("completed"))
        and (laminar_case / str(args.end_time) / "U").exists()
    )
    if not laminar_ok:
        report["rans"] = {"completed": False, "skipped": "laminar did not complete"}
        return report

    try:
        setup_case(
            source_case,
            rans_case,
            alpha_args,
            turbulence_model="kOmegaSST",
            end_time=args.end_time,
            p_relax=0.1,
            u_relax=0.2,
        )
        shutil.copy2(laminar_case / str(args.end_time) / "U", rans_case / "0" / "U")
        shutil.copy2(laminar_case / str(args.end_time) / "p", rans_case / "0" / "p")
        set_turbulence_relaxation(rans_case, 0.2)
        add_yplus_function(rans_case)
        run_openfoam(
            rans_case,
            args.timeout_s,
            report,
            "rans",
            args.parallel_procs,
        )
        summarize_case(
            rans_case,
            report,
            "rans",
            tail_window=20,
            end_time=args.end_time,
        )
        yplus_path = find_latest_time_field(rans_case, "yPlus", preferred_time=args.end_time)
        report["rans"]["yplus"] = summarize_yplus(yplus_path) if yplus_path else {"available": 0}
    except Exception as exc:
        report["rans"] = {"completed": False, "error": str(exc)}
    return report


def early_prune_decision(
    alpha_reports: list[dict[str, Any]],
    args: argparse.Namespace,
) -> dict[str, Any]:
    if not args.early_prune or args.reuse_existing:
        return {"prune": False, "reason": "disabled"}

    attempted = [
        report
        for report in alpha_reports
        if not report.get("skipped_by_early_prune")
    ]
    min_completed = max(2, int(args.early_prune_min_completed_alphas))
    if len(attempted) < min_completed:
        return {
            "prune": False,
            "reason": "need_multiple_alpha_results",
            "attempted_alpha_count": len(attempted),
            "minimum_attempted_alpha_count": min_completed,
        }

    aggregate = aggregate_polar(attempted)
    points = aggregate["points"]
    usable = [point for point in points if point["ld"] is not None and point["cl"] > 0.0]
    rans_reports = [report.get("rans", {}) for report in attempted]
    solver_bad_count = sum(
        1
        for rans in rans_reports
        if (
            not rans.get("completed")
            or rans.get("floating_point_exception")
            or rans.get("fatal_error")
        )
    )
    unstable_force_count = sum(
        1
        for rans in rans_reports
        if rans.get("force_stable_for_scoring_gate") is False
    )
    max_abs_coefficient = max(
        (
            abs(float(value))
            for point in points
            for value in (point.get("cd"), point.get("cl"), point.get("cm"))
            if value is not None
        ),
        default=0.0,
    )
    max_cl = max((float(point["cl"]) for point in points), default=None)
    best_ld = max((float(point["ld"]) for point in usable), default=None)
    base = {
        "attempted_alpha_count": len(attempted),
        "completed_alpha_count": len(points),
        "usable_alpha_count": len(usable),
        "solver_bad_count": solver_bad_count,
        "unstable_force_count": unstable_force_count,
        "max_cl": max_cl,
        "best_ld": best_ld,
        "max_abs_coefficient": max_abs_coefficient,
        "policy": {
            "min_completed_alphas": min_completed,
            "no_lift_cl_max": args.early_prune_no_lift_cl_max,
            "weak_lift_cl_max": args.early_prune_weak_lift_cl_max,
            "min_best_ld": args.early_prune_min_best_ld,
            "max_abs_coefficient": args.early_prune_max_abs_coefficient,
        },
    }

    if solver_bad_count >= min_completed:
        return {**base, "prune": True, "reason": "multiple_solver_failures"}
    if points and max_abs_coefficient > float(args.early_prune_max_abs_coefficient):
        return {**base, "prune": True, "reason": "multiple_alpha_coefficient_blowup"}
    if len(points) >= min_completed and not usable and (
        max_cl is None or max_cl < float(args.early_prune_no_lift_cl_max)
    ):
        return {**base, "prune": True, "reason": "no_meaningful_positive_lift_after_multiple_alphas"}
    if (
        len(points) >= min_completed
        and best_ld is not None
        and best_ld < float(args.early_prune_min_best_ld)
        and max_cl is not None
        and max_cl < float(args.early_prune_weak_lift_cl_max)
    ):
        return {**base, "prune": True, "reason": "weak_lift_and_low_ld_after_multiple_alphas"}
    if len(points) >= min_completed and unstable_force_count >= min_completed and not usable:
        return {**base, "prune": True, "reason": "multiple_unstable_force_tails_without_usable_lift"}
    return {**base, "prune": False, "reason": "not_clearly_bad"}


def aggregate_polar(alpha_reports: list[dict[str, Any]]) -> dict[str, Any]:
    points = []
    attempted_count = 0
    skipped_count = 0
    for report in alpha_reports:
        if report.get("skipped_by_early_prune"):
            skipped_count += 1
            continue
        attempted_count += 1
        rans = report.get("rans", {})
        force_last = rans.get("force_last") or {}
        cd = force_last.get("cd", force_last.get("Cd"))
        cl = force_last.get("cl", force_last.get("Cl"))
        cm = force_last.get("cm", force_last.get("Cm"))
        if not rans.get("completed") or cd is None or cl is None or cm is None:
            continue
        ld = cl / cd if cd > 0 else None
        points.append(
            {
                "alpha_deg": report["alpha_deg"],
                "cd": cd,
                "cl": cl,
                "cm": cm,
                "ld": ld,
                "rans_case": report.get("rans_case"),
            }
        )
    usable = [point for point in points if point["ld"] is not None and point["cl"] > 0.0]
    best = max(usable, key=lambda item: item["ld"]) if usable else None
    return {
        "stage_name": (
            alpha_reports[0].get("stage_name") if alpha_reports else None
        ),
        "stage_role": (
            alpha_reports[0].get("stage_role") if alpha_reports else None
        ),
        "completed_alpha_count": len(points),
        "attempted_alpha_count": attempted_count,
        "skipped_alpha_count": skipped_count,
        "early_pruned": skipped_count > 0,
        "usable_alpha_count": len(usable),
        "points": points,
        "best_usable": best,
        "best_usable_ld": best["ld"] if best else None,
        "best_usable_alpha_deg": best["alpha_deg"] if best else None,
        "cl_alpha_per_deg": linear_slope(points, "cl"),
        "cm_alpha_per_deg": linear_slope(points, "cm"),
        "cd_min": min((point["cd"] for point in points), default=None),
        "cl_max": max((point["cl"] for point in points), default=None),
    }


def linear_slope(points: list[dict[str, Any]], key: str) -> float | None:
    if len(points) < 2:
        return None
    xs = [float(point["alpha_deg"]) for point in points]
    ys = [float(point[key]) for point in points]
    x_mean = sum(xs) / len(xs)
    y_mean = sum(ys) / len(ys)
    denom = sum((x - x_mean) ** 2 for x in xs)
    if denom == 0.0:
        return None
    return sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denom


def alpha_velocity(
    speed: float,
    alpha_deg: float,
    *,
    vertical_axis: str = "z",
) -> tuple[float, float, float]:
    alpha_rad = math.radians(alpha_deg)
    if vertical_axis == "y":
        return (speed * math.cos(alpha_rad), speed * math.sin(alpha_rad), 0.0)
    return (speed * math.cos(alpha_rad), 0.0, speed * math.sin(alpha_rad))


def alpha_lift_dir(
    alpha_deg: float,
    *,
    vertical_axis: str = "z",
) -> tuple[float, float, float]:
    alpha_rad = math.radians(alpha_deg)
    if vertical_axis == "y":
        return normalize((-math.sin(alpha_rad), math.cos(alpha_rad), 0.0))
    return normalize((-math.sin(alpha_rad), 0.0, math.cos(alpha_rad)))


def normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    mag = math.sqrt(sum(component * component for component in vector))
    return tuple(component / mag for component in vector)


def vector_arg(vector: tuple[float, float, float]) -> str:
    return ",".join(f"{component:.12g}" for component in vector)


def alpha_slug(alpha_deg: float) -> str:
    prefix = "p" if alpha_deg >= 0 else "m"
    value = abs(alpha_deg)
    text = f"{value:g}".replace(".", "p")
    return f"{prefix}{text}"


if __name__ == "__main__":
    main()
