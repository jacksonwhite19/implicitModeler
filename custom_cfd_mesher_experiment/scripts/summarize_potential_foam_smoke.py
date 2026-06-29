from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    text = args.log.read_text(encoding="utf-8", errors="replace")
    report = summarize(text, args.log)
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))


def summarize(text: str, log_path: Path) -> dict[str, object]:
    phi_solves = parse_solves(text, "Phi")
    p_solves = parse_solves(text, "p")
    return {
        "log": str(log_path),
        "started": "Calculating potential flow" in text,
        "completed": "\nEnd\n" in text and "FOAM FATAL" not in text,
        "fatal_error": "FOAM FATAL" in text,
        "warning_count": text.count("FOAM Warning"),
        "continuity_error": number(r"Continuity error = ([0-9.eE+-]+)", text),
        "interpolated_velocity_error": number(r"Interpolated velocity error = ([0-9.eE+-]+)", text),
        "phi_solves": phi_solves,
        "pressure_solves": p_solves,
        "last_phi_final_residual": phi_solves[-1]["final_residual"] if phi_solves else None,
        "last_p_final_residual": p_solves[-1]["final_residual"] if p_solves else None,
        "execution_time_s": number(r"ExecutionTime = ([0-9.eE+-]+) s", text),
        "clock_time_s": number(r"ClockTime = ([0-9.eE+-]+) s", text),
        "suitability": "plumbing_smoke_only",
    }


def parse_solves(text: str, field: str) -> list[dict[str, float | int]]:
    pattern = (
        rf"Solving for {re.escape(field)}, Initial residual = ([0-9.eE+-]+), "
        rf"Final residual = ([0-9.eE+-]+), No Iterations ([0-9]+)"
    )
    return [
        {
            "initial_residual": float(match.group(1)),
            "final_residual": float(match.group(2)),
            "iterations": int(match.group(3)),
        }
        for match in re.finditer(pattern, text)
    ]


def number(pattern: str, text: str) -> float | None:
    match = re.search(pattern, text)
    if not match:
        return None
    return float(match.group(1))


if __name__ == "__main__":
    main()
