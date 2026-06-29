from __future__ import annotations

import argparse
import json
import socket
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse


DEFAULT_RUN_NAME = "real_optimizer_15candidate_full_20260628"


def main() -> None:
    parser = argparse.ArgumentParser(description="Read-only mobile optimizer run monitor.")
    parser.add_argument("--platform-root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--run-name", default=DEFAULT_RUN_NAME)
    parser.add_argument("--workspace", type=Path, default=None)
    parser.add_argument("--run-root", type=Path, default=None)
    parser.add_argument("--log", type=Path, default=None)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    platform_root = args.platform_root.resolve()
    project_root = platform_root.parent
    workspace = (
        args.workspace.resolve()
        if args.workspace is not None
        else platform_root / "runs" / args.run_name
    )
    run_root = (
        args.run_root.resolve()
        if args.run_root is not None
        else project_root / "custom_cfd_mesher_experiment" / "runs" / f"{args.run_name}_gen1_snappy"
    )
    log_path = (
        args.log.resolve()
        if args.log is not None
        else platform_root / "runs" / "_logs" / f"{args.run_name}.log"
    )
    state = MonitorState(
        platform_root=platform_root,
        workspace=workspace,
        run_root=run_root,
        log_path=log_path,
        run_name=args.run_name,
    )

    class Handler(MonitorHandler):
        monitor_state = state

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"Serving {args.run_name} at http://{local_ip()}:{args.port}/")
    print(f"Local URL: http://127.0.0.1:{args.port}/")
    print(f"Workspace: {workspace}")
    print(f"Run root: {run_root}")
    server.serve_forever()


class MonitorState:
    def __init__(
        self,
        *,
        platform_root: Path,
        workspace: Path,
        run_root: Path,
        log_path: Path,
        run_name: str,
    ) -> None:
        self.platform_root = platform_root
        self.workspace = workspace
        self.run_root = run_root
        self.log_path = log_path
        self.run_name = run_name

    def snapshot(self) -> dict[str, Any]:
        export_dir = self.workspace / "exports_gen1" / "generated_real_geometry"
        pre_export = read_json(self.workspace / "exports_gen1" / "pre_export_screening_summary.json")
        export_results = list(self.workspace.rglob("no_inlet_oml_export_result.json")) if self.workspace.exists() else []
        export_status = summarize_export_results(export_results)
        stl_files = list(self.workspace.rglob("*.stl")) if self.workspace.exists() else []
        backend_stl_map = read_json(self.workspace / "backend_inputs" / "gen1_mesh_input_stl_map.json")
        mesh_expected = len(backend_stl_map) if backend_stl_map else len(stl_files)
        variant_dirs = sorted(
            [path for path in self.run_root.iterdir() if path.is_dir() and path.name.startswith("opg1_")]
            if self.run_root.exists()
            else []
        )
        comparison = read_json(self.run_root / "comparison_summary.json")
        alpha = read_json(self.run_root / "alpha_sweep_summary_first_pass_end60_np8.json")
        combined = read_csv_summary(self.workspace / "combined_ranked_summary.csv")
        gen1 = read_csv_summary(self.workspace / "gen1_ranked_summary.csv")
        summary = read_json(self.workspace / "real_optimizer_pilot_summary.json")
        no_slip_reports = load_latest_no_slip_reports(variant_dirs)
        alpha_case_reports = count_alpha_force_reports(variant_dirs)

        batch_mesh_reports = comparison.get("reports", []) if isinstance(comparison, dict) else []
        mesh_reports = merge_mesh_reports(
            partial_reports=load_partial_variant_reports(variant_dirs),
            batch_reports=batch_mesh_reports,
        )
        alpha_reports = alpha.get("reports", []) if isinstance(alpha, dict) else []
        alpha_expected_per_candidate = alpha_case_multiplier(
            platform_root=self.platform_root,
            alpha_summary=alpha,
        )
        early_prune = summarize_early_pruning(alpha_reports)
        candidates = merge_candidate_status(
            variant_dirs=variant_dirs,
            mesh_reports=mesh_reports,
            no_slip_reports=no_slip_reports,
            alpha_reports=alpha_reports,
            ranking_rows=combined.get("rows") or gen1.get("rows") or [],
        )
        alpha_case_count = count_alpha_case_dirs(variant_dirs)
        steady_case_count = count_steady_case_dirs(variant_dirs)
        total_expected = int(
            (summary or {}).get("initial_count")
            or pre_export.get("candidate_count")
            or len(export_results)
            or len(variant_dirs)
            or 0
        )
        mesh_total_expected = mesh_expected or total_expected
        progress = {
            "pre_export": progress_count(
                int(pre_export.get("passed_count") or 0),
                int(pre_export.get("candidate_count") or total_expected),
            ),
            "exports": progress_count(len(export_results), total_expected),
            "export_passed": progress_count(export_status["passed"], total_expected),
            "meshes": progress_count(len(mesh_reports), mesh_total_expected),
            "steady_cases": progress_count(steady_case_count, mesh_total_expected * 2),
            "alpha_candidates": progress_count(len(alpha_reports), mesh_total_expected),
            "alpha_cases": progress_count(alpha_case_count, mesh_total_expected * alpha_expected_per_candidate),
        }
        phase = infer_phase(
            total_expected=total_expected,
            export_count=len(export_results),
            mesh_count=len(mesh_reports),
            steady_case_count=steady_case_count,
            alpha_case_count=alpha_case_count,
            alpha_count=len(alpha_reports),
            has_summary=bool(summary),
        )
        return {
            "schema": "mobile_optimizer_monitor.v0_1",
            "generated_at_epoch": time.time(),
            "generated_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
            "run_name": self.run_name,
            "phase": phase,
            "paths": {
                "workspace": str(self.workspace),
                "run_root": str(self.run_root),
                "log": str(self.log_path),
                "combined_ranked_summary": str(self.workspace / "combined_ranked_summary.csv"),
                "alpha_summary": str(self.run_root / "alpha_sweep_summary_first_pass_end60_np8.json"),
                "comparison_summary": str(self.run_root / "comparison_summary.json"),
            },
            "counts": {
                "expected_candidates": total_expected,
                "expected_mesh_candidates": mesh_total_expected,
                "pre_export_screened": pre_export.get("candidate_count", 0),
                "pre_export_passed": pre_export.get("passed_count", 0),
                "pre_export_failed": pre_export.get("failed_count", 0),
                "generated_geometry_dirs": count_dirs(export_dir),
                "export_results": len(export_results),
                "export_passed": export_status["passed"],
                "export_failed": export_status["failed"],
                "stl_files": len(stl_files),
                "variant_dirs": len(variant_dirs),
                "mesh_reports": len(mesh_reports),
                "steady_case_dirs": steady_case_count,
                "steady_reports": len(no_slip_reports),
                "alpha_reports": len(alpha_reports),
                "alpha_case_reports": alpha_case_reports,
                "alpha_case_dirs": alpha_case_count,
                "alpha_expected_per_candidate": alpha_expected_per_candidate,
                "early_pruned_variants": early_prune["variant_count"],
                "early_pruned_skipped_alphas": early_prune["skipped_alpha_count"],
            },
            "progress": progress,
            "best": first_row(combined.get("rows") or gen1.get("rows") or []),
            "candidates": candidates,
            "pre_export_failures": summarize_pre_export_failures(pre_export),
            "early_pruning": early_prune,
            "export_failures": export_status["failures"],
            "log_tail": tail_text(self.log_path, lines=80),
            "raw": {
                "comparison_available": bool(comparison),
                "partial_mesh_reports_available": bool(mesh_reports),
                "alpha_available": bool(alpha),
                "summary_available": bool(summary),
            },
        }


class MonitorHandler(BaseHTTPRequestHandler):
    monitor_state: MonitorState

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self.respond_html(HTML)
            return
        if parsed.path == "/api/status":
            query = parse_qs(parsed.query)
            pretty = query.get("pretty", ["0"])[0] == "1"
            self.respond_json(self.monitor_state.snapshot(), pretty=pretty)
            return
        if parsed.path == "/health":
            self.respond_text("ok\n")
            return
        self.send_error(404)

    def log_message(self, format: str, *args: object) -> None:
        return

    def respond_html(self, html: str) -> None:
        body = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def respond_text(self, text: str) -> None:
        body = text.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def respond_json(self, payload: dict[str, Any], *, pretty: bool = False) -> None:
        text = json.dumps(payload, indent=2 if pretty else None, sort_keys=pretty)
        body = text.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def merge_candidate_status(
    *,
    variant_dirs: list[Path],
    mesh_reports: list[dict[str, Any]],
    no_slip_reports: list[dict[str, Any]],
    alpha_reports: list[dict[str, Any]],
    ranking_rows: list[dict[str, str]],
) -> list[dict[str, Any]]:
    by_id: dict[str, dict[str, Any]] = {}
    for path in variant_dirs:
        by_id.setdefault(path.name, {"variant_id": path.name, "stage": "mesh_started"})
        run_summary = read_json(path / "run_summary.json")
        if run_summary:
            by_id[path.name].update(
                {
                    "status": run_summary.get("status"),
                    "verdict": run_summary.get("verdict"),
                    "runtime_s": run_summary.get("runtime_s"),
                }
            )
    for report in mesh_reports:
        variant_id = str(report.get("variant_id") or "")
        if not variant_id:
            continue
        mesh = report.get("mesh_summary", {}).get("check_mesh", {})
        potential = report.get("potential_summary", {})
        surface = report.get("surface_fidelity", {}).get("bidirectional_mm", {})
        by_id.setdefault(variant_id, {"variant_id": variant_id})
        by_id[variant_id].update(
            {
                "stage": "potential_done" if potential.get("completed") else "mesh_done",
                "verdict": report.get("verdict"),
                "runtime_s": report.get("runtime_s"),
                "potential_completed": potential.get("completed"),
                "potential_continuity": potential.get("continuity_error"),
                "potential_warning_count": potential.get("warning_count"),
                "cells": mesh.get("cells"),
                "points": mesh.get("points"),
                "aircraft_faces": mesh.get("aircraft_patch_faces"),
                "max_nonortho": mesh.get("max_non_orthogonality"),
                "max_skewness": mesh.get("max_skewness"),
                "surface_p95_mm": surface.get("p95"),
                "surface_p99_mm": surface.get("p99"),
            }
        )
    for report in no_slip_reports:
        variant_id = str(report.get("variant_id") or "")
        if not variant_id:
            continue
        force = report.get("force", {})
        incompressible = report.get("incompressible", {})
        by_id.setdefault(variant_id, {"variant_id": variant_id})
        by_id[variant_id].update(
            {
                "stage": "steady_done" if incompressible.get("completed") else "steady_started",
                "steady_case": report.get("case"),
                "steady_completed": incompressible.get("completed"),
                "steady_stable": force.get("stable_for_scoring"),
                "steady_reason": force.get("reason"),
                "steady_time": incompressible.get("last_time"),
                "steady_cd": force.get("last", {}).get("Cd"),
                "steady_cl": force.get("last", {}).get("Cl"),
                "steady_cm": force.get("last", {}).get("Cm"),
                "steady_u_residual": incompressible.get("last_velocity_final_residual"),
                "steady_p_residual": incompressible.get("last_p_final_residual"),
                "steady_fpe": incompressible.get("floating_point_exception"),
            }
        )
    for report in alpha_reports:
        variant_id = str(report.get("variant_id") or "")
        if not variant_id:
            continue
        aggregate = report.get("aggregate", {})
        best = aggregate.get("best_usable") or {}
        by_id.setdefault(variant_id, {"variant_id": variant_id})
        by_id[variant_id].update(
            {
                "stage": "rough_cfd_done",
                "alpha_count": aggregate.get("completed_alpha_count"),
                "usable_alpha_count": aggregate.get("usable_alpha_count"),
                "best_alpha": best.get("alpha_deg"),
                "best_ld": best.get("ld"),
                "cl": best.get("cl"),
                "cd": best.get("cd"),
                "cm": best.get("cm"),
            }
        )
    for index, row in enumerate(ranking_rows, start=1):
        variant_id = row.get("variant_id", "")
        if not variant_id:
            continue
        by_id.setdefault(variant_id, {"variant_id": variant_id})
        by_id[variant_id].update(
            {
                "rank": index,
                "rough_score": parse_float(row.get("rough_score")),
                "best_ld": parse_float(row.get("best_ld")) or by_id[variant_id].get("best_ld"),
                "cl": parse_float(row.get("cl_best")) or by_id[variant_id].get("cl"),
                "cd": parse_float(row.get("cd_best")) or by_id[variant_id].get("cd"),
                "cm": parse_float(row.get("cm_best")) or by_id[variant_id].get("cm"),
            }
        )
    return sorted(by_id.values(), key=lambda item: (item.get("rank") or 9999, item["variant_id"]))


def infer_phase(
    *,
    total_expected: int,
    export_count: int,
    mesh_count: int,
    steady_case_count: int,
    alpha_case_count: int,
    alpha_count: int,
    has_summary: bool,
) -> str:
    if has_summary:
        return "complete"
    if alpha_count > 0 or alpha_case_count > 0:
        return "rough CFD alpha sweep"
    if steady_case_count > 0:
        return "steady solver ladder"
    if mesh_count > 0 or export_count >= total_expected:
        return "meshing / solver setup"
    if export_count > 0:
        return "geometry export"
    return "starting"


def summarize_export_results(paths: list[Path]) -> dict[str, Any]:
    passed = 0
    failed = 0
    failures: list[dict[str, Any]] = []
    for path in paths:
        data = read_json(path)
        status = str(data.get("status") or data.get("summary", {}).get("status") or "")
        output_stem = data.get("preset_definition", {}).get("output_stem")
        candidate_id = path.parent.name
        if status == "passed":
            passed += 1
        else:
            failed += 1
            failures.append(
                {
                    "candidate_id": candidate_id,
                    "output_stem": output_stem,
                    "status": status or "unknown",
                    "path": str(path),
                }
            )
    return {"passed": passed, "failed": failed, "failures": failures}


def load_partial_variant_reports(variant_dirs: list[Path]) -> list[dict[str, Any]]:
    reports: list[dict[str, Any]] = []
    for variant_dir in variant_dirs:
        summary = read_json(variant_dir / "variant_summary.json")
        if summary:
            summary.setdefault("variant_id", variant_dir.name)
            reports.append(summary)
    return reports


def load_latest_no_slip_reports(variant_dirs: list[Path]) -> list[dict[str, Any]]:
    reports: list[dict[str, Any]] = []
    for variant_dir in variant_dirs:
        case_dirs = sorted(
            [
                child
                for child in variant_dir.iterdir()
                if child.is_dir()
                and (
                    child.name.startswith("steady_laminar_")
                    or child.name.startswith("kOmegaSST_laminarStart_")
                )
            ],
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        for case_dir in case_dirs:
            force = read_json(case_dir / "force_coeffs_summary.json")
            incompressible = read_json(case_dir / "incompressible_summary.json")
            if force or incompressible:
                reports.append(
                    {
                        "variant_id": variant_dir.name,
                        "case": case_dir.name,
                        "force": force,
                        "incompressible": incompressible,
                    }
                )
                break
    return reports


def merge_mesh_reports(
    *,
    partial_reports: list[dict[str, Any]],
    batch_reports: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    merged: dict[str, dict[str, Any]] = {}
    for report in partial_reports:
        variant_id = str(report.get("variant_id") or "")
        if variant_id:
            merged[variant_id] = report
    for report in batch_reports:
        variant_id = str(report.get("variant_id") or "")
        if variant_id:
            merged[variant_id] = report
    return [merged[key] for key in sorted(merged)]


def read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8-sig"))
    except Exception as exc:
        return {"read_error": str(exc), "path": str(path)}


def read_csv_summary(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"rows": []}
    try:
        lines = path.read_text(encoding="utf-8-sig").splitlines()
        if not lines:
            return {"rows": []}
        headers = [item.strip() for item in lines[0].split(",")]
        rows = []
        for line in lines[1:]:
            values = split_csv_line(line)
            rows.append({headers[index]: values[index] if index < len(values) else "" for index in range(len(headers))})
        return {"rows": rows}
    except Exception as exc:
        return {"rows": [], "read_error": str(exc)}


def split_csv_line(line: str) -> list[str]:
    values: list[str] = []
    current: list[str] = []
    in_quotes = False
    index = 0
    while index < len(line):
        char = line[index]
        if char == '"':
            if in_quotes and index + 1 < len(line) and line[index + 1] == '"':
                current.append('"')
                index += 1
            else:
                in_quotes = not in_quotes
        elif char == "," and not in_quotes:
            values.append("".join(current))
            current = []
        else:
            current.append(char)
        index += 1
    values.append("".join(current))
    return values


def first_row(rows: list[dict[str, str]]) -> dict[str, Any] | None:
    if not rows:
        return None
    row = dict(rows[0])
    for key in ["rough_score", "best_ld", "best_alpha", "cl_best", "cd_best", "cm_best"]:
        if key in row:
            row[key] = parse_float(row[key])
    return row


def progress_count(value: int, total: int) -> dict[str, Any]:
    return {
        "value": value,
        "total": total,
        "percent": round((value / total) * 100.0, 1) if total else 0.0,
    }


def count_dirs(path: Path) -> int:
    if not path.exists():
        return 0
    return sum(1 for child in path.iterdir() if child.is_dir())


def count_alpha_case_dirs(variant_dirs: list[Path]) -> int:
    count = 0
    for variant_dir in variant_dirs:
        try:
            count += sum(
                1
                for child in variant_dir.iterdir()
                if child.is_dir()
                and child.name.startswith("alpha_")
                and "_kOmegaSST_" in child.name
            )
        except OSError:
            pass
    return count


def count_alpha_force_reports(variant_dirs: list[Path]) -> int:
    count = 0
    for variant_dir in variant_dirs:
        try:
            count += sum(
                1
                for child in variant_dir.iterdir()
                if child.is_dir()
                and child.name.startswith("alpha_")
                and "_kOmegaSST_" in child.name
                and (child / "force_coeffs_summary.json").exists()
            )
        except OSError:
            pass
    return count


def alpha_case_multiplier(*, platform_root: Path, alpha_summary: dict[str, Any]) -> int:
    alphas = alpha_summary.get("alphas_deg")
    if isinstance(alphas, list) and alphas:
        return len(alphas)
    preset = read_json(
        platform_root
        / "software"
        / "optimizer"
        / "configs"
        / "snappy_openfoam_external_aero_ranking.v0_2_tip_only_mesh_candidate.json"
    )
    configured = (
        preset.get("solver_smoke", {})
        .get("alpha_sweep_policy", {})
        .get("two_stage_policy", {})
        .get("first_pass", {})
        .get("angles_deg", [])
    )
    return max(1, len(configured))


def summarize_early_pruning(alpha_reports: list[dict[str, Any]]) -> dict[str, Any]:
    pruned = []
    skipped_alpha_count = 0
    for report in alpha_reports:
        pruning = report.get("early_pruning", {})
        aggregate = report.get("aggregate", {})
        skipped = int(aggregate.get("skipped_alpha_count") or 0)
        skipped_alpha_count += skipped
        if pruning.get("pruned") or skipped:
            decision = pruning.get("decision") or {}
            pruned.append(
                {
                    "variant_id": report.get("variant_id"),
                    "reason": decision.get("reason") or "early_pruned",
                    "skipped_alpha_count": skipped,
                    "skipped_alphas_deg": decision.get("skipped_alphas_deg", []),
                }
            )
    return {
        "variant_count": len(pruned),
        "skipped_alpha_count": skipped_alpha_count,
        "variants": pruned,
    }


def summarize_pre_export_failures(pre_export: dict[str, Any]) -> list[dict[str, Any]]:
    failures = []
    for record in pre_export.get("records", []) if isinstance(pre_export, dict) else []:
        if record.get("passed"):
            continue
        failures.append(
            {
                "pilot_index": record.get("pilot_index"),
                "status": record.get("status"),
                "failed_checks": record.get("failed_checks", []),
            }
        )
    return failures


def count_steady_case_dirs(variant_dirs: list[Path]) -> int:
    count = 0
    for variant_dir in variant_dirs:
        try:
            count += sum(
                1
                for child in variant_dir.iterdir()
                if child.is_dir()
                and (
                    child.name.startswith("steady_laminar_")
                    or child.name.startswith("kOmegaSST_laminarStart_")
                )
            )
        except OSError:
            pass
    return count


def tail_text(path: Path, *, lines: int) -> str:
    if not path.exists():
        return ""
    try:
        raw = path.read_bytes()
        if raw.count(b"\x00") > max(8, len(raw) // 20):
            text = raw.decode("utf-16-le", errors="replace")
        else:
            text = raw.decode("utf-8-sig", errors="replace")
        return "\n".join(text.splitlines()[-lines:])
    except Exception as exc:
        return f"Could not read log: {exc}"


def parse_float(value: Any) -> float | None:
    try:
        if value in (None, ""):
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def local_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"


HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Optimizer Monitor</title>
  <style>
    :root { color-scheme: dark; --bg:#0d1117; --panel:#151b23; --line:#30363d; --text:#e6edf3; --muted:#8b949e; --accent:#2f81f7; --good:#3fb950; --warn:#d29922; }
    * { box-sizing: border-box; }
    body { margin:0; font-family: system-ui, -apple-system, Segoe UI, sans-serif; background:var(--bg); color:var(--text); }
    header { position:sticky; top:0; z-index:2; padding:14px 16px; background:rgba(13,17,23,.95); border-bottom:1px solid var(--line); }
    h1 { margin:0; font-size:18px; }
    .sub { color:var(--muted); font-size:12px; margin-top:4px; word-break:break-all; }
    main { padding:14px; max-width:980px; margin:0 auto; }
    .grid { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:10px; }
    .card { background:var(--panel); border:1px solid var(--line); border-radius:8px; padding:12px; }
    .metric { font-size:24px; font-weight:700; }
    .label { color:var(--muted); font-size:12px; margin-bottom:4px; }
    .bar { height:9px; background:#222a33; border-radius:999px; overflow:hidden; margin-top:8px; }
    .bar > div { height:100%; background:var(--accent); width:0; transition:width .25s; }
    table { width:100%; border-collapse:collapse; font-size:12px; }
    th, td { border-bottom:1px solid var(--line); padding:7px 5px; text-align:left; vertical-align:top; }
    th { color:var(--muted); font-weight:600; }
    .pill { display:inline-block; padding:2px 7px; border:1px solid var(--line); border-radius:999px; color:var(--muted); }
    .best { color:var(--good); font-weight:700; }
    pre { white-space:pre-wrap; word-break:break-word; color:var(--muted); max-height:260px; overflow:auto; margin:0; font-size:12px; }
    .path { font-size:11px; color:var(--muted); word-break:break-all; }
    @media (max-width: 720px) {
      main { padding:10px; }
      .grid { grid-template-columns:1fr; }
      table { font-size:11px; }
      th:nth-child(5), td:nth-child(5), th:nth-child(6), td:nth-child(6) { display:none; }
    }
  </style>
</head>
<body>
  <header>
    <h1>Optimizer Monitor</h1>
    <div class="sub" id="subtitle">Loading...</div>
  </header>
  <main>
    <section class="grid" id="cards"></section>
    <section class="card" style="margin-top:10px">
      <div class="label">Best Candidate</div>
      <div id="best">No ranking yet.</div>
    </section>
    <section class="card" style="margin-top:10px">
      <div class="label">Candidates</div>
      <div style="overflow-x:auto"><table id="candidates"></table></div>
    </section>
    <section class="card" style="margin-top:10px">
      <div class="label">Paths</div>
      <div id="paths"></div>
    </section>
    <section class="card" style="margin-top:10px">
      <div class="label">Log Tail</div>
      <pre id="log"></pre>
    </section>
  </main>
<script>
async function refresh() {
  const res = await fetch('/api/status?_=' + Date.now(), { headers: { 'ngrok-skip-browser-warning': 'true' } });
  const data = await res.json();
  document.getElementById('subtitle').textContent = `${data.run_name} | ${data.phase} | ${data.generated_at_local}`;
  const cards = [
    ['Pre-Export Passed', data.progress.pre_export],
    ['Exports', data.progress.exports],
    ['Export Passed', data.progress.export_passed],
    ['Meshes', data.progress.meshes],
    ['Steady Cases', data.progress.steady_cases],
    ['Alpha Candidates', data.progress.alpha_candidates],
    ['Alpha Cases', data.progress.alpha_cases],
  ];
  document.getElementById('cards').innerHTML = cards.map(([label,p]) => `
    <div class="card"><div class="label">${label}</div>
    <div class="metric">${p.value}/${p.total}</div>
    <div class="bar"><div style="width:${p.percent}%"></div></div>
    <div class="sub">${p.percent}%</div></div>`).join('');
  if ((data.export_failures || []).length) {
    document.getElementById('cards').innerHTML += `<div class="card"><div class="label">Export Failures</div>
      <div class="metric">${data.export_failures.length}</div>
      <div class="sub">${data.export_failures.map(f => `${f.output_stem || f.candidate_id}: ${f.status}`).join('<br>')}</div></div>`;
  }
  if ((data.pre_export_failures || []).length) {
    document.getElementById('cards').innerHTML += `<div class="card"><div class="label">Pre-Export Failed</div>
      <div class="metric">${data.pre_export_failures.length}</div>
      <div class="sub">${data.pre_export_failures.map(f => `#${f.pilot_index}: ${(f.failed_checks || []).join(', ')}`).join('<br>')}</div></div>`;
  }
  if ((data.early_pruning || {}).variant_count) {
    document.getElementById('cards').innerHTML += `<div class="card"><div class="label">Early Alpha Pruned</div>
      <div class="metric">${data.early_pruning.variant_count}</div>
      <div class="sub">${data.early_pruning.skipped_alpha_count} skipped alpha points<br>${(data.early_pruning.variants || []).map(v => `${v.variant_id}: ${v.reason}`).join('<br>')}</div></div>`;
  }
  const best = data.best;
  document.getElementById('best').innerHTML = best ? `
    <div class="best">${best.variant_id || ''}</div>
    <div>Score ${fmt(best.rough_score)} | L/D ${fmt(best.best_ld)} | CL ${fmt(best.cl_best)} | CD ${fmt(best.cd_best)} | Cm ${fmt(best.cm_best)}</div>` : 'No ranking yet.';
  const rows = data.candidates || [];
  document.getElementById('candidates').innerHTML = `
    <tr><th>Rank</th><th>Candidate</th><th>Stage</th><th>Score</th><th>L/D</th><th>Rough CL/CD/Cm</th><th>Latest Steady</th></tr>
    ${rows.map(r => `<tr>
      <td>${r.rank || ''}</td><td>${r.variant_id}</td><td><span class="pill">${r.stage || 'pending'}</span></td>
      <td>${fmt(r.rough_score)}</td><td>${fmt(r.best_ld)}</td>
      <td>${fmt(r.cl)} / ${fmt(r.cd)} / ${fmt(r.cm)}<br>
        <span class="sub">cells ${fmt0(r.cells)} | nonO ${fmt(r.max_nonortho)} | p95 ${fmt(r.surface_p95_mm)}</span>
      </td>
      <td>${fmt(r.steady_cl)} / ${fmt(r.steady_cd)} / ${fmt(r.steady_cm)}<br>
        <span class="sub">${r.steady_case || ''} ${r.steady_stable === undefined ? '' : '| stable ' + r.steady_stable} ${r.steady_fpe ? '| FPE' : ''}</span>
      </td>
    </tr>`).join('')}`;
  document.getElementById('paths').innerHTML = Object.entries(data.paths).map(([k,v]) => `<div class="path"><b>${k}</b>: ${v}</div>`).join('');
  document.getElementById('log').textContent = data.log_tail || '(no log output)';
}
function fmt(v) {
  if (v === null || v === undefined || v === '') return '';
  const n = Number(v);
  return Number.isFinite(n) ? n.toFixed(4) : String(v);
}
function fmt0(v) {
  if (v === null || v === undefined || v === '') return '';
  const n = Number(v);
  return Number.isFinite(n) ? Math.round(n).toLocaleString() : String(v);
}
refresh();
setInterval(refresh, 10000);
</script>
</body>
</html>
"""


if __name__ == "__main__":
    main()
