from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from aircraft_optimizer.cli.main import find_platform_root
from aircraft_optimizer.db.connection import connect, initialize_database
from aircraft_optimizer.db.repositories import (
    create_campaign,
    create_environment_fingerprint,
    create_variable_schema,
)
from aircraft_optimizer.events.event_log import log_event
from aircraft_optimizer.modules.pre_export_screening import (
    DEFAULT_SCREENING_POLICY,
    screen_candidate_pre_export,
)
from aircraft_optimizer.modules.rough_cfd_scoring import stable_efficient_drone_rough_score
from aircraft_optimizer.optimizers.ask_tell import HaltonAskTellPolicy
from aircraft_optimizer.optimizers.real_no_inlet_export import _run_real_export_candidate
from aircraft_optimizer.pipelines import load_pipeline_config
from aircraft_optimizer.records import CandidateSeed, VariableValue
from aircraft_optimizer.schemas import load_variable_schema


REAL_OPTIMIZER_HARD_PRE_EXPORT_POLICY = {
    **DEFAULT_SCREENING_POLICY,
    "policy_intent": "Hard reject only clearly implausible geometry before expensive SDF export, meshing, and CFD.",
    "min_planform_area_m2": 0.035,
    "max_wing_loading_n_m2": 340.0,
    "min_aspect_ratio": 3.0,
    "max_aspect_ratio": 16.0,
    "min_taper_ratio": 0.18,
    "max_taper_ratio": 0.95,
    "max_abs_sweep_deg": 38.0,
    "min_estimated_total_mass_kg": 0.65,
    "max_estimated_total_mass_kg": 2.5,
    "max_layout_cg_error_mm": 75.0,
    "min_layout_wiggle_room_mm": 10.0,
    "min_cg_x_mm": 180.0,
    "max_cg_x_mm": 480.0,
    "min_mean_chord_mm": 60.0,
    "max_mean_chord_mm": 240.0,
    "min_tip_chord_mm": 35.0,
    "max_span_root_chord_ratio": 8.5,
    "max_export_risk_score": 0.9,
}


@dataclass(frozen=True)
class ExportedVariant:
    variant_id: str
    candidate_id: str
    generation: int
    result_json: Path
    stl_path: Path
    design_variables: dict[str, float]
    normalized_design_vector: dict[str, float]
    pre_export_metrics: dict[str, float]
    parent_variant_ids: list[str]
    export_status: str


@dataclass(frozen=True)
class ScreenedCandidate:
    original_index: int
    candidate: CandidateSeed
    parent_variant_ids: list[str]
    parent_candidate_ids: list[str]
    mutation_summary: dict[str, Any]
    pre_export_metrics: dict[str, float]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--run-slug", required=True)
    parser.add_argument("--initial-count", type=int, default=6)
    parser.add_argument("--children-count", type=int, default=3)
    parser.add_argument("--timeout-seconds", type=int, default=1200)
    parser.add_argument("--alpha-strategy", default="first-pass", choices=["first-pass", "adaptive", "survivor-expanded", "none"])
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Reuse completed export maps, backend run roots, and alpha summaries when present.",
    )
    args = parser.parse_args()

    platform_root = find_platform_root()
    project_root = platform_root.parent
    optimizer_root = platform_root / "software" / "optimizer"
    workspace = args.workspace.resolve()
    workspace.mkdir(parents=True, exist_ok=True)
    run_roots_dir = project_root / "custom_cfd_mesher_experiment" / "runs"
    preset_path = (
        platform_root
        / "software"
        / "optimizer"
        / "configs"
        / "snappy_openfoam_external_aero_ranking.v0_2_tip_only_mesh_candidate.json"
    )
    rough_config_path = (
        platform_root
        / "software"
        / "optimizer"
        / "configs"
        / "rough_cfd_stable_efficient_drone.v0_1.json"
    )
    scoring_workspace = workspace / "rough_scoring_import"

    initial_candidates = _initial_candidates(platform_root, args.initial_count)
    gen1_export = workspace / "exports_gen1"
    gen1_variants, gen1_map, gen1_config = _ensure_export_generation(
        platform_root=platform_root,
        workspace=workspace,
        export_workspace=gen1_export,
        candidates=initial_candidates,
        prefix="opg1",
        generation_name="gen1",
        generation=1,
        timeout_seconds=args.timeout_seconds,
        resume=args.resume,
    )
    gen1_run_root = run_roots_dir / f"{args.run_slug}_gen1_snappy"
    if gen1_variants:
        _ensure_backend(
            project_root=project_root,
            optimizer_root=optimizer_root,
            scoring_workspace=scoring_workspace,
            run_root=gen1_run_root,
            input_stl_map=gen1_map,
            variant_config=gen1_config,
            preset_path=preset_path,
            rough_config_path=rough_config_path,
            alpha_strategy=args.alpha_strategy,
            resume=args.resume,
        )
        gen1_ranked = _rank_backend_run(gen1_run_root, gen1_variants, workspace / "gen1_ranked_summary.csv")
    else:
        gen1_ranked = []
        _write_ranked_csv(workspace / "gen1_ranked_summary.csv", [])

    gen2_variants: list[ExportedVariant] = []
    gen2_ranked: list[dict[str, Any]] = []
    gen2_export = workspace / "exports_gen2"
    gen2_run_root = run_roots_dir / f"{args.run_slug}_gen2_snappy"
    gen2_map: Path | None = None
    gen2_config: Path | None = None
    if args.children_count > 0 and gen1_ranked:
        gen2_candidates = _children_from_ranked(
            ranked=gen1_ranked,
            all_variants={item.variant_id: item for item in gen1_variants},
            platform_root=platform_root,
            count=args.children_count,
        )
        if gen2_candidates:
            gen2_variants, gen2_map, gen2_config = _ensure_export_generation(
                platform_root=platform_root,
                workspace=workspace,
                export_workspace=gen2_export,
                candidates=[item["candidate"] for item in gen2_candidates],
                prefix="opg2",
                generation_name="gen2",
                generation=2,
                timeout_seconds=args.timeout_seconds,
                parent_variant_ids=[item["parent_variant_id"] for item in gen2_candidates],
                parent_candidate_ids=[item["parent_candidate_id"] for item in gen2_candidates],
                mutation_summaries=[item["mutation_summary"] for item in gen2_candidates],
                resume=args.resume,
            )
            if gen2_variants:
                _ensure_backend(
                    project_root=project_root,
                    optimizer_root=optimizer_root,
                    scoring_workspace=scoring_workspace,
                    run_root=gen2_run_root,
                    input_stl_map=gen2_map,
                    variant_config=gen2_config,
                    preset_path=preset_path,
                    rough_config_path=rough_config_path,
                    alpha_strategy=args.alpha_strategy,
                    resume=args.resume,
                )
                gen2_ranked = _rank_backend_run(gen2_run_root, gen2_variants, workspace / "gen2_ranked_summary.csv")
            else:
                _write_ranked_csv(workspace / "gen2_ranked_summary.csv", [])
    combined = sorted(gen1_ranked + gen2_ranked, key=lambda item: item["rough_score"], reverse=True)
    _write_ranked_csv(workspace / "combined_ranked_summary.csv", combined)

    summary = {
        "workspace": str(workspace),
        "run_slug": args.run_slug,
        "preset": str(preset_path),
        "rough_scoring_config": str(rough_config_path),
        "initial_count": args.initial_count,
        "children_count": args.children_count,
        "alpha_strategy": args.alpha_strategy,
        "resume_enabled": args.resume,
        "gen1": {
            "export_workspace": str(gen1_export),
            "pre_export_screening_summary": str(gen1_export / "pre_export_screening_summary.json"),
            "pre_export_screening": _pre_export_counts(gen1_export / "pre_export_screening_summary.json"),
            "run_root": str(gen1_run_root),
            "input_stl_map": str(gen1_map),
            "variant_config": str(gen1_config),
            "ranked_summary": str(workspace / "gen1_ranked_summary.csv"),
        },
        "gen2": (
            {
                "export_workspace": str(gen2_export),
                "pre_export_screening_summary": str(gen2_export / "pre_export_screening_summary.json"),
                "pre_export_screening": _pre_export_counts(gen2_export / "pre_export_screening_summary.json"),
                "run_root": str(gen2_run_root),
                "input_stl_map": str(gen2_map),
                "variant_config": str(gen2_config),
                "ranked_summary": str(workspace / "gen2_ranked_summary.csv"),
            }
            if gen2_map is not None and gen2_config is not None
            else None
        ),
        "combined_ranked_summary": str(workspace / "combined_ranked_summary.csv"),
        "scoring_workspace": str(scoring_workspace),
        "best": combined[0] if combined else None,
        "lineage": [
            {
                "child_variant_id": item.variant_id,
                "parent_variant_ids": item.parent_variant_ids,
            }
            for item in gen2_variants
        ],
    }
    (workspace / "real_optimizer_pilot_summary.json").write_text(
        json.dumps(summary, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2))


def _ensure_export_generation(
    *,
    platform_root: Path,
    workspace: Path,
    export_workspace: Path,
    candidates: list[CandidateSeed],
    prefix: str,
    generation_name: str,
    generation: int,
    timeout_seconds: int,
    resume: bool,
    parent_variant_ids: list[str] | None = None,
    parent_candidate_ids: list[str] | None = None,
    mutation_summaries: list[dict[str, Any]] | None = None,
) -> tuple[list[ExportedVariant], Path, Path]:
    map_path = workspace / "backend_inputs" / f"{generation_name}_mesh_input_stl_map.json"
    config_path = workspace / "backend_inputs" / f"{generation_name}_variant_config.json"
    if resume and map_path.exists() and config_path.exists():
        variants = _load_exported_variants_from_config(config_path)
        return variants, map_path, config_path
    if export_workspace.exists():
        raise SystemExit(
            "Export workspace already exists but backend input files are missing. "
            f"Cannot safely resume partial export: {export_workspace}"
        )
    variants = _export_generation(
        platform_root=platform_root,
        workspace=export_workspace,
        candidates=candidates,
        prefix=prefix,
        generation=generation,
        timeout_seconds=timeout_seconds,
        parent_variant_ids=parent_variant_ids,
        parent_candidate_ids=parent_candidate_ids,
        mutation_summaries=mutation_summaries,
    )
    map_path, config_path = _write_backend_inputs(
        workspace=workspace,
        generation_name=generation_name,
        variants=variants,
    )
    return variants, map_path, config_path


def _load_exported_variants_from_config(config_path: Path) -> list[ExportedVariant]:
    config = json.loads(config_path.read_text(encoding="utf-8-sig"))
    pre_export_by_index = _pre_export_metrics_by_pilot_index(config_path)
    variants: list[ExportedVariant] = []
    for item in config.get("variants", []):
        variant_id = item["id"]
        pilot_index = _pilot_index_from_variant_id(variant_id)
        variants.append(
            ExportedVariant(
                variant_id=variant_id,
                candidate_id=item["candidate_id"],
                generation=int(item["generation"]),
                result_json=Path(item["result_json"]),
                stl_path=Path(item["stl_path"]),
                design_variables={
                    key: float(value)
                    for key, value in item.get("overrides", {}).items()
                },
                normalized_design_vector={
                    key: float(value)
                    for key, value in item.get("normalized_design_vector", {}).items()
                },
                pre_export_metrics={
                    key: float(value)
                    for key, value in item.get("pre_export_metrics", {}).items()
                } or pre_export_by_index.get(pilot_index, {}),
                parent_variant_ids=list(item.get("parent_variant_ids", [])),
                export_status="success",
            )
        )
    return variants


def _pre_export_metrics_by_pilot_index(config_path: Path) -> dict[int, dict[str, float]]:
    generation_name = config_path.stem.replace("_variant_config", "")
    summary_path = config_path.parents[1] / f"exports_{generation_name}" / "pre_export_screening_summary.json"
    if not summary_path.exists():
        return {}
    summary = json.loads(summary_path.read_text(encoding="utf-8-sig"))
    return {
        int(record["pilot_index"]): _numeric_metric_values(record.get("metrics", {}))
        for record in summary.get("records", [])
        if "pilot_index" in record
    }


def _pilot_index_from_variant_id(variant_id: str) -> int:
    try:
        return int(variant_id.split("_")[1])
    except (IndexError, ValueError):
        return -1


def _initial_candidates(platform_root: Path, count: int) -> list[CandidateSeed]:
    schema = load_variable_schema(
        platform_root
        / "software"
        / "optimizer"
        / "schemas"
        / "fixed_wing_uav_reference.variables.v0_1.json"
    )
    policy = HaltonAskTellPolicy(schema.raw)
    candidates: list[CandidateSeed] = []
    for index in range(count):
        proposal = policy.ask(index)
        candidates.append(
            CandidateSeed(
                aircraft_family=proposal.candidate.aircraft_family,
                design_variables=proposal.candidate.design_variables,
                normalized_design_vector=proposal.candidate.normalized_design_vector,
                created_by="real_optimizer_pilot_halton_gen1",
                generation=1,
                notes=f"Real optimizer pilot generation 1 Halton proposal {index}.",
            )
        )
    return candidates


def _export_generation(
    *,
    platform_root: Path,
    workspace: Path,
    candidates: list[CandidateSeed],
    prefix: str,
    generation: int,
    timeout_seconds: int,
    parent_variant_ids: list[str] | None = None,
    parent_candidate_ids: list[str] | None = None,
    mutation_summaries: list[dict[str, Any]] | None = None,
) -> list[ExportedVariant]:
    if workspace.exists():
        raise SystemExit(f"Export workspace already exists: {workspace}")
    workspace.mkdir(parents=True)
    screened_candidates = _screen_candidates_for_export(
        workspace=workspace,
        candidates=candidates,
        generation=generation,
        parent_variant_ids=parent_variant_ids,
        parent_candidate_ids=parent_candidate_ids,
        mutation_summaries=mutation_summaries,
    )
    db_path = workspace / "optimizer.db"
    artifact_root = workspace / "campaigns" / "real_optimizer_pilot_export" / "artifacts"
    connection = connect(db_path)
    initialize_database(connection)
    variable_schema = load_variable_schema(
        platform_root
        / "software"
        / "optimizer"
        / "schemas"
        / "fixed_wing_uav_reference.variables.v0_1.json"
    )
    pipeline_config = load_pipeline_config(
        platform_root
        / "software"
        / "optimizer"
        / "pipelines"
        / "v0_1_fixture.pipeline.json"
    )
    variable_schema_id = create_variable_schema(
        connection,
        aircraft_family=variable_schema.aircraft_family,
        schema_version=variable_schema.schema_version,
        schema=variable_schema.raw,
    )
    campaign_id = create_campaign(
        connection,
        name=f"real optimizer pilot export generation {generation}",
        description="Real no-inlet export stage for a bounded optimizer pilot run.",
        aircraft_family=variable_schema.aircraft_family,
        variable_schema_id=variable_schema_id,
        config={
            "pipeline_id": pipeline_config.pipeline_id,
            "pipeline_version": pipeline_config.pipeline_version,
            "real_exporter_execution": True,
            "real_cfd_execution": False,
            "generation": generation,
            "pre_export_screening": {
                "enabled": True,
                "policy": REAL_OPTIMIZER_HARD_PRE_EXPORT_POLICY,
                "summary": str(workspace / "pre_export_screening_summary.json"),
            },
        },
    )
    log_event(
        connection,
        event_type="campaign.created",
        campaign_id=campaign_id,
        actor="system",
        message=f"Created real optimizer pilot export generation {generation}.",
    )
    environment_fingerprint_id = create_environment_fingerprint(
        connection,
        tools={
            "geometry": {"mode": "generated_no_inlet_rhai"},
            "exporter": {"mode": "real_direct_sparse_oml_fast"},
            "cfd": {"enabled": False},
        },
        notes=f"Real optimizer pilot generation {generation} export stage.",
    )
    exported: list[ExportedVariant] = []
    try:
        for screened in screened_candidates:
            index = screened.original_index
            candidate = screened.candidate
            parent_variants = screened.parent_variant_ids
            parent_candidates = screened.parent_candidate_ids
            mutation_summary = screened.mutation_summary | {
                "pilot_generation": generation,
                "pilot_index": index,
                "parent_variant_ids": parent_variants,
                "parent_candidate_ids": parent_candidates,
                "pre_export_screening": "passed",
            }
            result = _run_real_export_candidate(
                connection=connection,
                platform_root=platform_root,
                workspace=workspace,
                artifact_root=artifact_root,
                variable_schema=variable_schema,
                variable_schema_id=variable_schema_id,
                pipeline_id=pipeline_config.pipeline_id,
                pipeline_version=pipeline_config.pipeline_version,
                environment_fingerprint_id=environment_fingerprint_id,
                campaign_id=campaign_id,
                candidate=candidate,
                parent_candidate_ids=list(parent_candidates),
                lineage_reason=mutation_summary.get(
                    "reason",
                    f"Real optimizer pilot generation {generation}.",
                ),
                mutation_summary=mutation_summary,
                timeout_seconds=timeout_seconds,
                priority=index,
            )
            connection.commit()
            result_json = Path(result["result_json"])
            stl_path = _artifact_stl_path(artifact_root, result_json)
            exported.append(
                ExportedVariant(
                    variant_id=f"{prefix}_{index:02d}_{result['candidate_id'][-8:]}",
                    candidate_id=result["candidate_id"],
                    generation=generation,
                    result_json=result_json,
                    stl_path=stl_path,
                    design_variables=_candidate_design_values(candidate),
                    normalized_design_vector=dict(candidate.normalized_design_vector),
                    pre_export_metrics=dict(screened.pre_export_metrics),
                    parent_variant_ids=list(parent_variants),
                    export_status=result["module_status"],
                )
            )
    finally:
        connection.close()
    return exported


def _screen_candidates_for_export(
    *,
    workspace: Path,
    candidates: list[CandidateSeed],
    generation: int,
    parent_variant_ids: list[str] | None = None,
    parent_candidate_ids: list[str] | None = None,
    mutation_summaries: list[dict[str, Any]] | None = None,
) -> list[ScreenedCandidate]:
    screened: list[ScreenedCandidate] = []
    records: list[dict[str, Any]] = []
    for index, candidate in enumerate(candidates):
        parent_variants = _list_at(parent_variant_ids, index)
        parent_candidates = _list_at(parent_candidate_ids, index)
        mutation_summary = (
            mutation_summaries[index]
            if mutation_summaries is not None and index < len(mutation_summaries)
            else {}
        )
        result = screen_candidate_pre_export(candidate, policy=REAL_OPTIMIZER_HARD_PRE_EXPORT_POLICY)
        records.append(
            {
                "generation": generation,
                "pilot_index": index,
                "passed": result.passed,
                "status": "passed" if result.passed else "failed_pre_export",
                "failed_checks": result.metadata.get("failed_checks", []),
                "warnings": result.warnings,
                "design_variables": candidate.design_variables_dict(),
                "normalized_design_vector": candidate.normalized_design_vector,
                "parent_variant_ids": parent_variants,
                "parent_candidate_ids": parent_candidates,
                "mutation_summary": mutation_summary,
                "metrics": result.metrics,
                "metadata": {
                    "checks": result.metadata.get("checks", {}),
                    "virtual_component_model": result.metadata.get("virtual_component_model", {}),
                },
            }
        )
        if result.passed:
            screened.append(
                ScreenedCandidate(
                    original_index=index,
                    candidate=candidate,
                    parent_variant_ids=parent_variants,
                    parent_candidate_ids=parent_candidates,
                    mutation_summary=mutation_summary,
                    pre_export_metrics=_numeric_metric_values(result.metrics),
                )
            )
    summary = {
        "schema": "real_optimizer_pre_export_screening.v0_1",
        "generation": generation,
        "policy": REAL_OPTIMIZER_HARD_PRE_EXPORT_POLICY,
        "candidate_count": len(candidates),
        "passed_count": len(screened),
        "failed_count": len(candidates) - len(screened),
        "records": records,
    }
    (workspace / "pre_export_screening_summary.json").write_text(
        json.dumps(summary, indent=2),
        encoding="utf-8",
    )
    return screened


def _list_at(values: list[Any] | None, index: int) -> list[str]:
    if values is None or index >= len(values):
        return []
    value = values[index]
    if value is None:
        return []
    if isinstance(value, str):
        return [value]
    return [str(item) for item in value]


def _pre_export_counts(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"available": False, "path": str(path)}
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    return {
        "available": True,
        "path": str(path),
        "candidate_count": data.get("candidate_count"),
        "passed_count": data.get("passed_count"),
        "failed_count": data.get("failed_count"),
    }


def _artifact_stl_path(artifact_root: Path, result_json: Path) -> Path:
    result = json.loads(result_json.read_text(encoding="utf-8-sig"))
    output_stem = result.get("preset_definition", {}).get("output_stem")
    if output_stem:
        matches = sorted(artifact_root.glob(f"*_{output_stem}_spacing_1p0.stl"))
        if matches:
            return matches[0].resolve()
    stl_path = result.get("summary", {}).get("stl_path") or result.get("stl_path")
    if not stl_path:
        raise ValueError(f"could not find STL path in {result_json}")
    return Path(stl_path).resolve()


def _write_backend_inputs(
    *,
    workspace: Path,
    generation_name: str,
    variants: list[ExportedVariant],
) -> tuple[Path, Path]:
    input_dir = workspace / "backend_inputs"
    input_dir.mkdir(parents=True, exist_ok=True)
    stl_map = {
        item.variant_id: str(item.stl_path)
        for item in variants
        if item.export_status == "success"
    }
    map_path = input_dir / f"{generation_name}_mesh_input_stl_map.json"
    map_path.write_text(json.dumps(stl_map, indent=2), encoding="utf-8")
    config = {
        "units": "mm",
        "variants": [
            {
                "id": item.variant_id,
                "name": item.variant_id,
                "candidate_id": item.candidate_id,
                "generation": item.generation,
                "parent_variant_ids": item.parent_variant_ids,
                "overrides": item.design_variables,
                "normalized_design_vector": item.normalized_design_vector,
                "pre_export_metrics": item.pre_export_metrics,
                "stl_path": str(item.stl_path),
                "result_json": str(item.result_json),
            }
            for item in variants
        ],
    }
    config_path = input_dir / f"{generation_name}_variant_config.json"
    config_path.write_text(json.dumps(config, indent=2), encoding="utf-8")
    return map_path, config_path


def _run_backend(
    *,
    optimizer_root: Path,
    scoring_workspace: Path,
    run_root: Path,
    input_stl_map: Path,
    variant_config: Path,
    preset_path: Path,
    rough_config_path: Path,
    alpha_strategy: str,
) -> None:
    if run_root.exists():
        raise SystemExit(f"Backend run root already exists: {run_root}")
    env = dict(os.environ)
    env["PYTHONPATH"] = "src"
    command = [
        sys.executable,
        "-m",
        "aircraft_optimizer.cli.main",
        "run-snappy-openfoam-backend",
        "--workspace",
        str(scoring_workspace),
        "--run-root",
        str(run_root),
        "--input-stl-map",
        str(input_stl_map),
        "--variant-config",
        str(variant_config),
        "--preset",
        str(preset_path),
        "--rough-scoring-config",
        str(rough_config_path),
        "--alpha-strategy",
        alpha_strategy,
    ]
    subprocess.run(command, cwd=optimizer_root, env=env, check=True)


def _ensure_backend(
    *,
    project_root: Path,
    optimizer_root: Path,
    scoring_workspace: Path,
    run_root: Path,
    input_stl_map: Path,
    variant_config: Path,
    preset_path: Path,
    rough_config_path: Path,
    alpha_strategy: str,
    resume: bool,
) -> None:
    alpha_summary = run_root / "alpha_sweep_summary_first_pass_end60_np8.json"
    comparison_summary = run_root / "comparison_summary.json"
    if not run_root.exists():
        _run_backend(
            optimizer_root=optimizer_root,
            scoring_workspace=scoring_workspace,
            run_root=run_root,
            input_stl_map=input_stl_map,
            variant_config=variant_config,
            preset_path=preset_path,
            rough_config_path=rough_config_path,
            alpha_strategy=alpha_strategy,
        )
        return
    if not resume:
        raise SystemExit(f"Backend run root already exists; rerun with --resume: {run_root}")
    if not comparison_summary.exists():
        raise SystemExit(
            "Backend run root exists but comparison_summary.json is missing. "
            f"Cannot safely resume partial meshing: {run_root}"
        )
    alpha_was_missing = not alpha_summary.exists()
    if not alpha_summary.exists():
        _run_alpha_only(
            project_root=project_root,
            run_root=run_root,
            preset_path=preset_path,
            alpha_strategy=alpha_strategy,
        )
    if alpha_was_missing or not (scoring_workspace / "optimizer.db").exists():
        _import_backend_only(
            optimizer_root=optimizer_root,
            scoring_workspace=scoring_workspace,
            run_root=run_root,
            variant_config=variant_config,
            preset_path=preset_path,
            rough_config_path=rough_config_path,
        )


def _run_alpha_only(
    *,
    project_root: Path,
    run_root: Path,
    preset_path: Path,
    alpha_strategy: str,
) -> None:
    if alpha_strategy != "first-pass":
        raise SystemExit(
            "Resume currently supports alpha-only recovery for first-pass strategy only."
        )
    preset = json.loads(preset_path.read_text(encoding="utf-8-sig"))
    smoke = preset["solver_smoke"]
    stage = (
        smoke.get("alpha_sweep_policy", {})
        .get("two_stage_policy", {})
        .get("first_pass", {})
    )
    if not stage:
        raise SystemExit("Preset does not define a first-pass alpha stage.")
    command = [
        sys.executable,
        str(project_root / "custom_cfd_mesher_experiment" / "scripts" / "run_no_slip_alpha_sweep.py"),
        "--run-root",
        str(run_root),
        f"--alphas-deg={','.join(str(value) for value in stage['angles_deg'])}",
        "--speed-mps",
        str(smoke["freestream_mps"][0]),
        "--end-time",
        str(smoke["rans_end_time"]),
        "--wall-treatment",
        smoke.get("wall_treatment", "wall_function"),
        "--parallel-procs",
        str(smoke.get("parallel_procs", 1)),
        "--summary-name",
        str(stage["summary_name"]),
        "--stage-name",
        str(stage["stage_name"]),
        "--stage-role",
        str(stage["stage_role"]),
    ]
    if "rans_timeout_s" in smoke:
        command.extend(["--timeout-s", str(smoke["rans_timeout_s"])])
    if smoke.get("vertical_axis"):
        command.extend(["--vertical-axis", str(smoke["vertical_axis"])])
    command.extend(_alpha_pruning_args(stage))
    subprocess.run(command, cwd=project_root, check=True)


def _alpha_pruning_args(stage: dict[str, Any]) -> list[str]:
    policy = stage.get("early_pruning") or {}
    if not policy.get("enabled", False):
        return []
    args = ["--early-prune"]
    option_map = {
        "min_completed_alphas": "--early-prune-min-completed-alphas",
        "no_lift_cl_max": "--early-prune-no-lift-cl-max",
        "weak_lift_cl_max": "--early-prune-weak-lift-cl-max",
        "min_best_ld": "--early-prune-min-best-ld",
        "max_abs_coefficient": "--early-prune-max-abs-coefficient",
    }
    for key, option in option_map.items():
        if key in policy:
            args.extend([option, str(policy[key])])
    return args


def _import_backend_only(
    *,
    optimizer_root: Path,
    scoring_workspace: Path,
    run_root: Path,
    variant_config: Path,
    preset_path: Path,
    rough_config_path: Path,
) -> None:
    env = dict(os.environ)
    env["PYTHONPATH"] = "src"
    command = [
        sys.executable,
        "-m",
        "aircraft_optimizer.cli.main",
        "run-snappy-openfoam-backend",
        "--workspace",
        str(scoring_workspace),
        "--run-root",
        str(run_root),
        "--variant-config",
        str(variant_config),
        "--preset",
        str(preset_path),
        "--rough-scoring-config",
        str(rough_config_path),
        "--alpha-strategy",
        "first-pass",
        "--skip-execution",
    ]
    subprocess.run(command, cwd=optimizer_root, env=env, check=True)


def _rank_backend_run(
    run_root: Path,
    variants: list[ExportedVariant],
    output_csv: Path,
) -> list[dict[str, Any]]:
    alpha_summary = json.loads(
        (run_root / "alpha_sweep_summary_first_pass_end60_np8.json").read_text(
            encoding="utf-8-sig"
        )
    )
    comparison = json.loads((run_root / "comparison_summary.json").read_text(encoding="utf-8-sig"))
    variant_lookup = {item.variant_id: item for item in variants}
    mesh_lookup = {item["variant_id"]: item for item in comparison.get("reports", [])}
    ranked: list[dict[str, Any]] = []
    for report in alpha_summary.get("reports", []):
        variant_id = report["variant_id"]
        aggregate = report.get("aggregate", {})
        best = aggregate.get("best_usable") or {}
        cd = float(best.get("cd", 999.0))
        cl = float(best.get("cl", 0.0))
        cm = float(best.get("cm", 999.0))
        variant = variant_lookup[variant_id]
        metrics, metadata, warnings = stable_efficient_drone_rough_score(
            steady_metrics={
                "openfoam_steady.cd_last": {"value": cd},
                "openfoam_steady.cl_last": {"value": cl},
                "openfoam_steady.cm_last": {"value": cm},
                "pre_export.estimated_mass_kg": {
                    "value": variant.pre_export_metrics.get("pre_export.estimated_mass_kg")
                },
                "pre_export.planform_area_m2": {
                    "value": variant.pre_export_metrics.get("pre_export.planform_area_m2")
                },
                "openfoam_steady.ready": {"value": 1},
                "openfoam_steady.rough_scoring_allowed": {"value": 1},
                "openfoam_steady.failed_check_count": {"value": 0},
                "openfoam_steady.cd_window_span": {"value": 0},
                "openfoam_steady.cl_window_span": {"value": 0},
                "openfoam_steady.cm_window_span": {"value": 0},
                "openfoam_steady.velocity_final_residual": {"value": 0},
                "openfoam_steady.local_continuity": {"value": 0},
            },
            alpha_sweep=aggregate,
        )
        mesh = mesh_lookup.get(variant_id, {})
        check_mesh = mesh.get("mesh_summary", {}).get("check_mesh", {})
        surface = mesh.get("surface_fidelity", {}).get("bidirectional_mm", {})
        ranked.append(
            {
                "variant_id": variant_id,
                "candidate_id": variant.candidate_id,
                "generation": variant.generation,
                "rough_score": float(metrics["score.rough_total"]["value"]),
                "rough_usable": int(metrics["score.rough_usable_for_optimizer"]["value"]),
                "best_ld": float(best.get("ld", 0.0)),
                "best_alpha": float(best.get("alpha_deg", 0.0)),
                "cl_best": cl,
                "cd_best": cd,
                "cm_best": cm,
                "required_cl": float(metrics["score.rough_required_cl"]["value"]),
                "alpha_at_required_cl": float(
                    metrics["score.rough_alpha_at_required_cl_deg"]["value"]
                ),
                "ld_at_required_cl": float(metrics["score.rough_ld_at_required_cl"]["value"]),
                "average_usable_ld": float(metrics["score.rough_average_usable_ld"]["value"]),
                "ld_curve_consistency": float(
                    metrics["score.rough_ld_curve_consistency"]["value"]
                ),
                "high_alpha_required_detractor": float(
                    metrics["score.rough_high_alpha_required_detractor"]["value"]
                ),
                "level_flight_lift_shortfall_detractor": float(
                    metrics["score.rough_level_flight_lift_shortfall_detractor"]["value"]
                ),
                "warnings": "; ".join(warnings),
                "score_metadata": metadata,
                "mesh_cells": check_mesh.get("cells"),
                "mesh_points": check_mesh.get("points"),
                "mesh_aircraft_faces": check_mesh.get("aircraft_patch_faces"),
                "mesh_max_nonortho": check_mesh.get("max_non_orthogonality"),
                "mesh_max_skewness": check_mesh.get("max_skewness"),
                "surface_p95_mm": surface.get("p95"),
                "surface_p99_mm": surface.get("p99"),
                "run_dir": mesh.get("run_dir"),
                **variant.pre_export_metrics,
                **variant.design_variables,
            }
        )
    ranked.sort(key=lambda item: item["rough_score"], reverse=True)
    _write_ranked_csv(output_csv, ranked)
    return ranked


def _children_from_ranked(
    *,
    ranked: list[dict[str, Any]],
    all_variants: dict[str, ExportedVariant],
    platform_root: Path,
    count: int,
) -> list[dict[str, Any]]:
    schema = load_variable_schema(
        platform_root
        / "software"
        / "optimizer"
        / "schemas"
        / "fixed_wing_uav_reference.variables.v0_1.json"
    )
    parents = [item for item in ranked if item["rough_usable"]][: max(1, min(3, len(ranked)))]
    if not parents:
        parents = ranked[: max(1, min(3, len(ranked)))]
    children: list[dict[str, Any]] = []
    recipes = [
        ("span_tip_tune", {"wing.span_mm": 0.06, "wing.tip_chord_mm": 0.08, "wing.sweep_deg": -0.04}),
        ("drag_trim_tune", {"wing.root_chord_mm": -0.05, "wing.tip_chord_mm": -0.04, "fuselage.tail_bluntness": 0.12}),
        ("stability_blend", {"wing.root_chord_mm": 0.04, "wing.sweep_deg": 0.05, "fuselage.nose_bluntness": 0.10}),
    ]
    for index in range(count):
        parent_row = parents[index % len(parents)]
        parent = all_variants[parent_row["variant_id"]]
        recipe_name, normalized_deltas = recipes[index % len(recipes)]
        child_norm = dict(parent.normalized_design_vector)
        for key, delta in normalized_deltas.items():
            child_norm[key] = _clamp(child_norm.get(key, 0.5) + delta, 0.0, 1.0)
        if parent_row.get("cm_best") is not None and abs(float(parent_row["cm_best"])) > 0.65:
            child_norm["wing.sweep_deg"] = _clamp(child_norm.get("wing.sweep_deg", 0.5) - 0.05, 0.0, 1.0)
            child_norm["fuselage.tail_bluntness"] = _clamp(
                child_norm.get("fuselage.tail_bluntness", 0.5) + 0.10,
                0.0,
                1.0,
            )
        design_variables = {
            key: VariableValue(_denormalize(schema.raw, key, value), schema.raw["variables"][key].get("unit"))
            for key, value in child_norm.items()
        }
        child = CandidateSeed(
            aircraft_family=schema.aircraft_family,
            design_variables=design_variables,
            normalized_design_vector=child_norm,
            created_by="real_optimizer_pilot_result_mutation",
            generation=2,
            notes=f"Real optimizer pilot child {index} from {parent.variant_id} using {recipe_name}.",
        )
        children.append(
            {
                "candidate": child,
                "parent_variant_id": parent.variant_id,
                "parent_candidate_id": parent.candidate_id,
                "mutation_summary": {
                    "operator": recipe_name,
                    "reason": "Mutate a top first-generation rough-CFD candidate for generation 2.",
                    "parent_rough_score": parent_row["rough_score"],
                    "parent_best_ld": parent_row["best_ld"],
                    "normalized_deltas": normalized_deltas,
                },
            }
        )
    return children


def _candidate_design_values(candidate: CandidateSeed) -> dict[str, float]:
    return {
        key: float(value.value)
        for key, value in candidate.design_variables.items()
    }


def _numeric_metric_values(metrics: dict[str, Any]) -> dict[str, float]:
    values: dict[str, float] = {}
    for name, metric in metrics.items():
        raw = metric.get("value") if isinstance(metric, dict) else metric
        if isinstance(raw, bool):
            values[name] = 1.0 if raw else 0.0
        elif isinstance(raw, (int, float)):
            values[name] = float(raw)
    return values


def _denormalize(schema: dict[str, Any], variable_name: str, normalized: float) -> float:
    variable = schema["variables"][variable_name]
    minimum = float(variable["min"])
    maximum = float(variable["max"])
    return round(minimum + normalized * (maximum - minimum), 6)


def _write_ranked_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    excluded = {"score_metadata"}
    fieldnames = [
        key
        for key in rows[0].keys()
        if key not in excluded
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


if __name__ == "__main__":
    main()
