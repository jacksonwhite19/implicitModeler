from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-dir", type=Path, required=True)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    text = args.log.read_text(encoding="utf-8", errors="replace")
    report = summarize(args.case_dir, args.log, text)
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    if args.quiet:
        print(
            json.dumps(
                {
                    "completed": report["completed"],
                    "fatal_error": report["fatal_error"],
                    "floating_point_exception": report["floating_point_exception"],
                    "last_time": report["last_time"],
                    "last_p_final_residual": report["last_p_final_residual"],
                    "last_velocity_final_residual": report["last_velocity_final_residual"],
                    "report": str(args.report),
                },
                indent=2,
                sort_keys=True,
            )
        )
    else:
        print(json.dumps(report, indent=2, sort_keys=True))


def summarize(case_dir: Path, log_path: Path, text: str) -> dict[str, object]:
    p_solves = parse_solves(text, "p")
    u_solves = parse_solves(text, "Ux") + parse_solves(text, "Uy") + parse_solves(text, "Uz")
    time_values = [float(value) for value in re.findall(r"^Time = ([0-9.eE+-]+)", text, flags=re.MULTILINE)]
    courant = parse_courant(text)
    return {
        "case_dir": str(case_dir),
        "log": str(log_path),
        "solver": "foamRun -solver incompressibleFluid",
        "started": "Selecting solver incompressibleFluid" in text or "PIMPLE:" in text,
        "completed": "\nEnd\n" in text and "FOAM FATAL" not in text,
        "fatal_error": "FOAM FATAL" in text or "sigFpe::sigHandler" in text or "Floating point exception" in text,
        "floating_point_exception": "sigFpe::sigHandler" in text or "Floating point exception" in text,
        "warning_count": text.count("FOAM Warning"),
        "time_steps": len(time_values),
        "first_time": time_values[0] if time_values else None,
        "last_time": time_values[-1] if time_values else None,
        "latest_time_dir": latest_numeric_time_dir(case_dir),
        "pressure_solves": p_solves,
        "velocity_solves": u_solves,
        "last_p_initial_residual": p_solves[-1]["initial_residual"] if p_solves else None,
        "last_p_final_residual": p_solves[-1]["final_residual"] if p_solves else None,
        "last_velocity_initial_residual": u_solves[-1]["initial_residual"] if u_solves else None,
        "last_velocity_final_residual": u_solves[-1]["final_residual"] if u_solves else None,
        "courant": courant,
        "execution_time_s": number(r"ExecutionTime = ([0-9.eE+-]+) s", text, last=True),
        "clock_time_s": number(r"ClockTime = ([0-9.eE+-]+) s", text, last=True),
        "suitability": "steady_solver_smoke_only_not_scoring",
    }


def parse_solves(text: str, field: str) -> list[dict[str, float | int | str]]:
    pattern = (
        rf"Solving for {re.escape(field)}, Initial residual = ([0-9.eE+-]+), "
        rf"Final residual = ([0-9.eE+-]+), No Iterations ([0-9]+)"
    )
    return [
        {
            "field": field,
            "initial_residual": float(match.group(1)),
            "final_residual": float(match.group(2)),
            "iterations": int(match.group(3)),
        }
        for match in re.finditer(pattern, text)
    ]


def parse_courant(text: str) -> dict[str, float | None]:
    matches = re.findall(
        r"Courant Number mean: ([0-9.eE+-]+) max: ([0-9.eE+-]+)",
        text,
    )
    if not matches:
        return {"last_mean": None, "last_max": None}
    mean, maximum = matches[-1]
    return {"last_mean": float(mean), "last_max": float(maximum)}


def latest_numeric_time_dir(case_dir: Path) -> str | None:
    numeric_dirs: list[tuple[float, Path]] = []
    for child in case_dir.iterdir():
        if not child.is_dir():
            continue
        try:
            numeric_dirs.append((float(child.name), child))
        except ValueError:
            continue
    if not numeric_dirs:
        return None
    return str(max(numeric_dirs)[1])


def number(pattern: str, text: str, *, last: bool = False) -> float | None:
    matches = re.findall(pattern, text)
    if not matches:
        return None
    return float(matches[-1] if last else matches[0])


if __name__ == "__main__":
    main()
