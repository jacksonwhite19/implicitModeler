#!/usr/bin/env python3
"""
optimizer_example.py — Example optimizer using the implicit-cad headless CLI.

Demonstrates how to use --dim overrides and --output-metrics to drive
a parameter sweep or optimization loop from Python.

Usage:
    python examples/optimizer_example.py

Requirements:
    - implicit-cad binary built with: cargo build --release
    - A script file at examples/simple_wing.rhai (or adjust SCRIPT below)
"""

import subprocess
import json
import tempfile
import os
import sys

# Path to the built binary (adjust for Windows .exe if needed)
BINARY = os.path.join(os.path.dirname(__file__), "..", "target", "release", "implicit-cad")
if sys.platform == "win32":
    BINARY += ".exe"

SCRIPT = os.path.join(os.path.dirname(__file__), "simple_wing.rhai")


def run_metrics(wingspan: float, resolution: int = 32) -> dict | None:
    """Run implicit-cad headless with a wingspan override and return metrics."""
    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as mf:
        metrics_path = mf.name

    try:
        result = subprocess.run(
            [
                BINARY,
                "--headless",
                "--script", SCRIPT,
                "--dim", f"wingspan={wingspan}",
                "--output-metrics", metrics_path,
                "--resolution", str(resolution),
            ],
            capture_output=True,
            text=True,
            timeout=120,
        )

        if result.returncode != 0:
            print(f"  [ERROR] wingspan={wingspan}: {result.stderr.strip()}")
            return None

        with open(metrics_path) as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Binary not found at {BINARY}. Build with: cargo build --release")
        sys.exit(1)
    except subprocess.TimeoutExpired:
        print(f"  [TIMEOUT] wingspan={wingspan}")
        return None
    finally:
        if os.path.exists(metrics_path):
            os.unlink(metrics_path)


def main():
    print("Implicit CAD — Headless Metrics Optimizer Example")
    print("=" * 50)

    # Parameter sweep: vary wingspan from 400mm to 1000mm in steps
    wingspans = [400, 500, 600, 700, 800, 900, 1000]

    results = []
    for ws in wingspans:
        print(f"  Evaluating wingspan={ws}mm ...", end=" ", flush=True)
        metrics = run_metrics(ws, resolution=24)
        if metrics:
            vol   = metrics.get("volume_mm3", 0)
            area  = metrics.get("surface_area_mm2", 0)
            mass  = metrics.get("estimated_mass_g", 0)
            t_ms  = metrics.get("evaluation_time_ms", 0)
            print(f"vol={vol:.0f}mm3  area={area:.0f}mm2  mass={mass:.1f}g  [{t_ms}ms]")
            results.append({"wingspan": ws, "volume_mm3": vol, "surface_area_mm2": area,
                             "estimated_mass_g": mass})
        else:
            print("FAILED")

    if not results:
        print("No results collected — check that the binary is built and script exists.")
        return

    # Find the design with the best volume-to-mass ratio
    best = max(results, key=lambda r: r["volume_mm3"] / max(r["estimated_mass_g"], 0.001))
    print("\nBest volume/mass ratio:")
    print(f"  wingspan = {best['wingspan']}mm")
    print(f"  volume   = {best['volume_mm3']:.0f} mm3")
    print(f"  mass     = {best['estimated_mass_g']:.1f} g")


if __name__ == "__main__":
    main()
