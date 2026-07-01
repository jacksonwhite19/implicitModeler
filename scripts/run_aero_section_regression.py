import argparse
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent


def run(cmd: list[str], cwd: Path) -> subprocess.CompletedProcess:
    print("+", " ".join(cmd))
    return subprocess.run(cmd, cwd=cwd, check=False, text=True)


def main() -> int:
    ap = argparse.ArgumentParser(description="Run aero export + section regression checks.")
    ap.add_argument("--script", type=Path, required=True)
    ap.add_argument("--case", type=str, required=True)
    ap.add_argument("--aero-mode", type=str, default="external_plus_inlets")
    ap.add_argument("--plane-y", type=float, default=0.0)
    ap.add_argument("--xmin", type=float, default=0.0)
    ap.add_argument("--xmax", type=float, default=700.0)
    ap.add_argument("--zmin", type=float, default=-60.0)
    ap.add_argument("--zmax", type=float, default=160.0)
    ap.add_argument("--na", type=int, default=1401)
    ap.add_argument("--nb", type=int, default=881)
    ap.add_argument("--max-mismatch-ratio", type=float, default=-1.0)
    ap.add_argument(
        "--rule",
        action="append",
        nargs=5,
        metavar=("NAME", "KIND", "X", "Z", "EXPECT"),
        help="Forwarded to check_section_export.py",
    )
    args = ap.parse_args()

    export_dir = ROOT / f"{args.case}_export"
    sdf_csv = ROOT / f"{args.case}_sdf_section.csv"
    check_prefix = ROOT / f"{args.case}_section_check"
    repairs_json = ROOT / f"{args.case}_section_repairs.json"

    export_cmd = [
        "cargo", "run", "--release", "--bin", "implicit-cad", "--",
        "--headless",
        "--script", str(args.script),
        "--output", str(export_dir),
        "--format", "aero",
        "--aero-mode", args.aero_mode,
        "--aero-fast-mode",
    ]
    result = run(export_cmd, ROOT)
    if result.returncode != 0:
        return result.returncode

    section_cmd = [
        "cargo", "run", "--release", "--bin", "sample_section", "--",
        "--script", str(args.script),
        "--plane", "xz",
        "--coord", str(args.plane_y),
        f"--amin={args.xmin}",
        f"--amax={args.xmax}",
        f"--bmin={args.zmin}",
        f"--bmax={args.zmax}",
        "--na", str(args.na),
        "--nb", str(args.nb),
        "--out", str(sdf_csv),
        "--backend", "auto",
    ]
    result = run(section_cmd, ROOT)
    if result.returncode != 0:
        return result.returncode

    stl_paths = [
        export_dir / "aircraft.stl",
        export_dir / "inlet.stl",
        export_dir / "duct.stl",
    ]
    check_cmd = [
        sys.executable,
        str(ROOT / "scripts" / "check_section_export.py"),
        "--sdf-csv", str(sdf_csv),
        "--stl",
        *(str(p) for p in stl_paths if p.exists()),
        "--plane-y", str(args.plane_y),
        "--out-prefix", str(check_prefix),
        "--max-mismatch-ratio", str(args.max_mismatch_ratio),
    ]
    for rule in args.rule or []:
        check_cmd.extend(["--rule", *rule])
    result = run(check_cmd, ROOT)

    report_path = check_prefix.with_suffix(".json")
    if report_path.exists():
        repairs_cmd = [
            sys.executable,
            str(ROOT / "scripts" / "suggest_section_repairs.py"),
            "--report", str(report_path),
            "--out", str(repairs_json),
        ]
        run(repairs_cmd, ROOT)

    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
