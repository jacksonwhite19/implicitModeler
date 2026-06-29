from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


SCRIPTS_DIR = Path(__file__).resolve().parent


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--variant-ids", default="")
    parser.add_argument("--velocity", default="22.352,0,0")
    parser.add_argument("--end-time", type=int, default=100)
    parser.add_argument("--write-interval", type=int, default=10)
    parser.add_argument("--rho-inf", type=float, default=1.225)
    parser.add_argument("--mag-u-inf", type=float, default=22.352)
    parser.add_argument("--l-ref", type=float, default=0.1325)
    parser.add_argument("--a-ref", type=float, default=0.1007)
    parser.add_argument("--cofr", default="0.35,0,0.04")
    parser.add_argument("--lift-dir", default="0,0,1")
    parser.add_argument("--drag-dir", default="1,0,0")
    parser.add_argument("--pitch-axis", default="0,1,0")
    parser.add_argument("--k", type=float, default=0.02)
    parser.add_argument("--omega", type=float, default=50.0)
    parser.add_argument(
        "--wall-treatment",
        choices=["wall_function", "low_re"],
        default="wall_function",
        help="Aircraft-wall turbulence boundary condition set for the kOmegaSST leg.",
    )
    parser.add_argument("--yplus-target-min-p50", type=float, default=30.0)
    parser.add_argument("--yplus-target-max-p50", type=float, default=100.0)
    parser.add_argument("--yplus-target-max-p95", type=float, default=150.0)
    parser.add_argument("--yplus-target-max-max", type=float, default=300.0)
    parser.add_argument(
        "--parallel-procs",
        type=int,
        default=1,
        help="Run foamRun in parallel with this many MPI ranks after decomposePar.",
    )
    parser.add_argument(
        "--summary-name",
        default="",
        help="Optional run-root summary filename. Short or selected probes get a unique default.",
    )
    parser.add_argument(
        "--variant-summary-name",
        default="",
        help="Optional per-variant summary filename. Short or selected probes get a unique default.",
    )
    parser.add_argument("--timeout-s", type=int, default=600)
    args = parser.parse_args()

    selected = {item.strip() for item in args.variant_ids.split(",") if item.strip()}
    reports = []
    started = time.perf_counter()
    for variant_dir in sorted(args.run_root.iterdir()):
        if not variant_dir.is_dir() or variant_dir.name.startswith("."):
            continue
        if selected and variant_dir.name not in selected:
            continue
        if not (variant_dir / "openfoam_case" / "constant" / "polyMesh").exists():
            continue
        reports.append(run_variant(variant_dir, args))

    summary = {
        "run_root": str(args.run_root),
        "runtime_s": time.perf_counter() - started,
        "variant_count": len(reports),
        "laminar_completed": sum(1 for report in reports if report.get("laminar", {}).get("completed")),
        "rans_completed": sum(1 for report in reports if report.get("rans", {}).get("completed")),
        "reports": reports,
    }
    (args.run_root / run_summary_name(args)).write_text(
        json.dumps(summary, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True))


def run_variant(variant_dir: Path, args: argparse.Namespace) -> dict[str, Any]:
    started = time.perf_counter()
    source_case = variant_dir / "openfoam_case"
    laminar_case = variant_dir / f"steady_laminar_p02_u03_{args.end_time}"
    rans_case = variant_dir / f"kOmegaSST_laminarStart_k002_om50_p01_u02_{args.end_time}"
    report: dict[str, Any] = {
        "variant_id": variant_dir.name,
        "source_case": str(source_case),
        "laminar_case": str(laminar_case),
        "rans_case": str(rans_case),
        "commands": [],
    }

    try:
        setup_case(
            source_case,
            laminar_case,
            args,
            turbulence_model="laminar",
            end_time=args.end_time,
            p_relax=0.2,
            u_relax=0.3,
        )
        run_openfoam(laminar_case, args.timeout_s, report, "laminar", args.parallel_procs)
        summarize_case(laminar_case, report, "laminar", tail_window=20, end_time=args.end_time)
    except Exception as exc:
        report["laminar"] = {"completed": False, "error": str(exc)}

    laminar_ok = bool(report.get("laminar", {}).get("completed")) and (laminar_case / str(args.end_time) / "U").exists()
    if laminar_ok:
        try:
            setup_case(
                source_case,
                rans_case,
                args,
                turbulence_model="kOmegaSST",
                end_time=args.end_time,
                p_relax=0.1,
                u_relax=0.2,
            )
            shutil.copy2(laminar_case / str(args.end_time) / "U", rans_case / "0" / "U")
            shutil.copy2(laminar_case / str(args.end_time) / "p", rans_case / "0" / "p")
            set_turbulence_relaxation(rans_case, 0.2)
            add_yplus_function(rans_case)
            run_openfoam(rans_case, args.timeout_s, report, "rans", args.parallel_procs)
            summarize_case(rans_case, report, "rans", tail_window=20, end_time=args.end_time)
            yplus_path = find_latest_time_field(rans_case, "yPlus", preferred_time=args.end_time)
            report["rans"]["yplus"] = summarize_yplus(yplus_path) if yplus_path else {"available": 0}
            report["rans"]["yplus_wall_function_gate"] = yplus_wall_function_gate(
                report["rans"]["yplus"],
                min_p50=args.yplus_target_min_p50,
                max_p50=args.yplus_target_max_p50,
                max_p95=args.yplus_target_max_p95,
                max_max=args.yplus_target_max_max,
            )
        except Exception as exc:
            report["rans"] = {"completed": False, "error": str(exc)}
    else:
        report["rans"] = {"completed": False, "skipped": "laminar did not complete"}

    report["runtime_s"] = time.perf_counter() - started
    (variant_dir / variant_summary_name(args)).write_text(
        json.dumps(report, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return report


def run_summary_name(args: argparse.Namespace) -> str:
    if args.summary_name:
        return args.summary_name
    if args.variant_ids or args.end_time != 100 or args.parallel_procs > 1:
        return f"no_slip_ladder_summary_end{args.end_time}_np{args.parallel_procs}.json"
    return "no_slip_ladder_summary.json"


def variant_summary_name(args: argparse.Namespace) -> str:
    if args.variant_summary_name:
        return args.variant_summary_name
    if args.variant_ids or args.end_time != 100 or args.parallel_procs > 1:
        return f"no_slip_ladder_summary_end{args.end_time}_np{args.parallel_procs}.json"
    return "no_slip_ladder_summary.json"


def setup_case(
    source_case: Path,
    case_dir: Path,
    args: argparse.Namespace,
    *,
    turbulence_model: str,
    end_time: int,
    p_relax: float,
    u_relax: float,
) -> None:
    command = [
        sys.executable,
        str(SCRIPTS_DIR / "setup_incompressible_fluid_smoke.py"),
        "--source-case-dir",
        str(source_case),
        "--case-dir",
        str(case_dir),
        "--velocity",
        args.velocity,
        "--turbulence-model",
        turbulence_model,
        "--turbulent-k",
        str(args.k),
        "--turbulent-omega",
        str(args.omega),
        "--wall-treatment",
        args.wall_treatment if turbulence_model == "kOmegaSST" else "wall_function",
        "--end-time",
        str(end_time),
        "--write-interval",
        str(args.write_interval),
        "--farfield-mode",
        "freestream",
        "--p-relax",
        str(p_relax),
        "--u-relax",
        str(u_relax),
        "--n-correctors",
        "2",
        "--n-non-orthogonal-correctors",
        "5",
        "--force-coeffs",
        "--rho-inf",
        str(args.rho_inf),
        "--mag-u-inf",
        str(args.mag_u_inf),
        "--l-ref",
        str(args.l_ref),
        "--a-ref",
        str(args.a_ref),
        f"--cofr={args.cofr}",
        f"--lift-dir={args.lift_dir}",
        f"--drag-dir={args.drag_dir}",
        f"--pitch-axis={args.pitch_axis}",
        "--force-write-interval",
        "1",
    ]
    subprocess.run(command, check=True)


def run_openfoam(
    case_dir: Path,
    timeout_s: int,
    report: dict[str, Any],
    label: str,
    parallel_procs: int,
) -> None:
    case_wsl = to_wsl_path(case_dir.resolve())
    if parallel_procs > 1:
        write_decompose_par_dict(case_dir, parallel_procs)
        script = (
            "source /opt/openfoam13/etc/bashrc && "
            f"cd '{case_wsl}' && "
            "checkMesh -skewThreshold 12 > log.checkMesh 2>&1 || exit $?; "
            "rm -rf processor*; "
            "decomposePar -force > log.decomposePar 2>&1 || exit $?; "
            f"mpirun -np {parallel_procs} foamRun -parallel -solver incompressibleFluid > log.foamRun 2>&1; "
            "foam_rc=$?; "
            "reconstructPar -latestTime > log.reconstructPar 2>&1; "
            "exit $foam_rc"
        )
    else:
        script = (
            "source /opt/openfoam13/etc/bashrc && "
            f"cd '{case_wsl}' && "
            "checkMesh -skewThreshold 12 > log.checkMesh 2>&1 && "
            "foamRun -solver incompressibleFluid > log.foamRun 2>&1"
        )
    started = time.perf_counter()
    result = subprocess.run(
        ["wsl", "bash", "-lc", script],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout_s,
    )
    report["commands"].append(
        {
            "label": label,
            "command": script,
            "returncode": result.returncode,
            "runtime_s": time.perf_counter() - started,
            "output": result.stdout[-4000:],
        }
    )


def summarize_case(
    case_dir: Path,
    report: dict[str, Any],
    label: str,
    *,
    tail_window: int,
    end_time: int,
) -> None:
    incompressible_report = case_dir / "incompressible_summary.json"
    subprocess.run(
        [
            sys.executable,
            str(SCRIPTS_DIR / "summarize_incompressible_fluid_smoke.py"),
            "--case-dir",
            str(case_dir),
            "--log",
            str(case_dir / "log.foamRun"),
            "--report",
            str(incompressible_report),
            "--quiet",
        ],
        check=True,
    )
    force_report = case_dir / "force_coeffs_summary.json"
    force_coeffs = find_force_coeffs(case_dir)
    if force_coeffs is None:
        raise FileNotFoundError(f"No forceCoeffs.dat found under {case_dir}")
    reference_start = max(1, int(end_time * 0.5))
    reference_end = max(reference_start + 1, int(end_time * 0.8))
    subprocess.run(
        [
            sys.executable,
            str(SCRIPTS_DIR / "summarize_force_coeffs.py"),
            "--force-coeffs",
            str(force_coeffs),
            "--report",
            str(force_report),
            "--tail-window",
            str(tail_window),
            "--reference-window-start",
            str(reference_start),
            "--reference-window-end",
            str(reference_end),
            "--max-tail-relative-drift",
            "0.1",
            "--max-tail-absolute-drift",
            "0.01",
            "--max-abs-coefficient",
            "10.0",
        ],
        check=True,
    )
    incompressible = json.loads(incompressible_report.read_text(encoding="utf-8"))
    force = json.loads(force_report.read_text(encoding="utf-8"))
    report[label] = {
        "completed": bool(incompressible.get("completed")),
        "fatal_error": bool(incompressible.get("fatal_error")),
        "floating_point_exception": bool(incompressible.get("floating_point_exception")),
        "last_time": incompressible.get("last_time"),
        "execution_time_s": incompressible.get("execution_time_s"),
        "clock_time_s": incompressible.get("clock_time_s"),
        "last_p_final_residual": incompressible.get("last_p_final_residual"),
        "last_velocity_final_residual": incompressible.get("last_velocity_final_residual"),
        "force_last": force.get("last"),
        "force_tail": force.get("tail"),
        "force_stable_for_scoring_gate": force.get("stable_for_scoring"),
        "force_gate_reason": force.get("reason"),
        "incompressible_summary": str(incompressible_report),
        "force_coeffs_summary": str(force_report),
        "force_coeffs_path": str(force_coeffs),
    }


def write_decompose_par_dict(case_dir: Path, parallel_procs: int) -> None:
    system_dir = case_dir / "system"
    system_dir.mkdir(parents=True, exist_ok=True)
    (system_dir / "decomposeParDict").write_text(
        f"""FoamFile
{{
    version     2.0;
    format      ascii;
    class       dictionary;
    object      decomposeParDict;
}}

numberOfSubdomains {parallel_procs};

method          scotch;

distributed     no;

roots           ();
""",
        encoding="utf-8",
        newline="\n",
    )


def find_force_coeffs(case_dir: Path) -> Path | None:
    preferred = case_dir / "postProcessing" / "aircraftForceCoeffs" / "0" / "forceCoeffs.dat"
    if preferred.exists():
        return preferred
    matches = sorted(
        case_dir.glob("postProcessing/**/forceCoeffs.dat"),
        key=lambda path: (len(path.parts), str(path)),
    )
    if matches:
        return matches[0]
    processor_matches = sorted(
        case_dir.glob("processor*/postProcessing/**/forceCoeffs.dat"),
        key=lambda path: (len(path.parts), str(path)),
    )
    return processor_matches[0] if processor_matches else None


def find_latest_time_field(case_dir: Path, field_name: str, *, preferred_time: int) -> Path | None:
    preferred = case_dir / str(preferred_time) / field_name
    if preferred.exists():
        return preferred
    candidates: list[tuple[float, Path]] = []
    for child in case_dir.iterdir():
        if not child.is_dir():
            continue
        try:
            time_value = float(child.name)
        except ValueError:
            continue
        path = child / field_name
        if path.exists():
            candidates.append((time_value, path))
    if not candidates:
        return None
    return sorted(candidates, key=lambda item: item[0])[-1][1]


def set_turbulence_relaxation(case_dir: Path, value: float) -> None:
    path = case_dir / "system" / "fvSolution"
    text = path.read_text(encoding="utf-8")
    text = re.sub(r"k\s+0\.5;", f"k               {value};", text)
    text = re.sub(r"omega\s+0\.5;", f"omega           {value};", text)
    path.write_text(text, encoding="utf-8", newline="\n")


def add_yplus_function(case_dir: Path) -> None:
    path = case_dir / "system" / "controlDict"
    text = path.read_text(encoding="utf-8")
    if "type            yPlus;" in text:
        return
    block = """

    yPlus
    {
        type            yPlus;
        libs            ("libfieldFunctionObjects.so");
        executeControl  writeTime;
        writeControl    writeTime;
        patches         (aircraft);
    }
"""
    text = re.sub(
        r"(?s)(functions\s*\{.*?\n)\}\s*\n\s*// \*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*",
        "\\1" + block + "}\n\n// *************************************************************************",
        text,
    )
    path.write_text(text, encoding="utf-8", newline="\n")


def summarize_yplus(path: Path) -> dict[str, float | int]:
    if not path.exists():
        return {"available": 0}
    text = path.read_text(encoding="utf-8", errors="replace")
    match = re.search(
        r"aircraft\s*\{.*?value\s+nonuniform\s+List<scalar>\s*(\d+)\s*\((.*?)\)\s*;",
        text,
        re.S,
    )
    if not match:
        return {"available": 0}
    values = [float(value) for value in match.group(2).split()]
    values.sort()

    def pct(p: float) -> float:
        index = (len(values) - 1) * p / 100.0
        low = int(index)
        high = min(low + 1, len(values) - 1)
        frac = index - low
        return values[low] * (1.0 - frac) + values[high] * frac

    return {
        "available": 1,
        "count": len(values),
        "min": min(values),
        "p50": pct(50),
        "p90": pct(90),
        "p95": pct(95),
        "p99": pct(99),
        "max": max(values),
        "mean": sum(values) / len(values),
        "lt1": sum(1 for value in values if value < 1.0),
        "gt30": sum(1 for value in values if value > 30.0),
        "gt100": sum(1 for value in values if value > 100.0),
    }


def yplus_wall_function_gate(
    yplus: dict[str, float | int],
    *,
    min_p50: float,
    max_p50: float,
    max_p95: float,
    max_max: float,
) -> dict[str, object]:
    if not yplus.get("available"):
        return {"pass": False, "reason": "yPlus field missing"}
    reasons = []
    p50 = float(yplus["p50"])
    p95 = float(yplus["p95"])
    max_value = float(yplus["max"])
    if p50 < min_p50:
        reasons.append(f"p50 {p50:.3g} below wall-function target minimum {min_p50:.3g}")
    if p50 > max_p50:
        reasons.append(f"p50 {p50:.3g} above wall-function target maximum {max_p50:.3g}")
    if p95 > max_p95:
        reasons.append(f"p95 {p95:.3g} above target maximum {max_p95:.3g}")
    if max_value > max_max:
        reasons.append(f"max {max_value:.3g} above target maximum {max_max:.3g}")
    return {
        "pass": not reasons,
        "reason": "pass" if not reasons else "; ".join(reasons),
        "target": {
            "p50_min": min_p50,
            "p50_max": max_p50,
            "p95_max": max_p95,
            "max_max": max_max,
        },
    }


def to_wsl_path(path: Path) -> str:
    text = str(path)
    if len(text) >= 3 and text[1:3] == ":\\":
        drive = text[0].lower()
        rest = text[3:].replace("\\", "/")
        return f"/mnt/{drive}/{rest}"
    return text.replace("\\", "/")


if __name__ == "__main__":
    main()
