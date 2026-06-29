from __future__ import annotations

import json
import sqlite3
from pathlib import Path
from typing import Any


def evaluation_report(db_path: Path, evaluation_id: str | None = None) -> dict[str, Any]:
    connection = sqlite3.connect(db_path)
    connection.row_factory = sqlite3.Row
    try:
        evaluation = _evaluation_row(connection, evaluation_id)
        candidate = connection.execute(
            "SELECT * FROM candidates WHERE candidate_id = ?",
            (evaluation["candidate_id"],),
        ).fetchone()
        if candidate is None:
            raise ValueError(f"candidate not found for evaluation {evaluation['evaluation_id']}")

        return {
            "evaluation": _row_dict(evaluation),
            "candidate": _candidate_summary(candidate),
            "lineage": _rows(
                connection,
                "SELECT * FROM candidate_lineage WHERE child_candidate_id = ? ORDER BY created_at",
                (candidate["candidate_id"],),
            ),
            "geometry_provider_results": _rows(
                connection,
                "SELECT * FROM geometry_provider_results WHERE evaluation_id = ? ORDER BY created_at",
                (evaluation["evaluation_id"],),
            ),
            "module_attempts": _rows(
                connection,
                "SELECT * FROM module_attempts WHERE evaluation_id = ? ORDER BY started_at",
                (evaluation["evaluation_id"],),
            ),
            "artifacts": _rows(
                connection,
                "SELECT artifact_id, artifact_type, path, status, producer_module, source_kind FROM artifacts WHERE evaluation_id = ? ORDER BY created_at",
                (evaluation["evaluation_id"],),
            ),
            "failures": _rows(
                connection,
                "SELECT * FROM failures WHERE evaluation_id = ? ORDER BY created_at",
                (evaluation["evaluation_id"],),
            ),
            "annotations": _rows(
                connection,
                "SELECT * FROM user_annotations WHERE evaluation_id = ? ORDER BY created_at",
                (evaluation["evaluation_id"],),
            ),
        }
    finally:
        connection.close()


def campaign_report(
    db_path: Path,
    campaign_id: str | None = None,
    *,
    failed_only: bool = False,
) -> dict[str, Any]:
    connection = sqlite3.connect(db_path)
    connection.row_factory = sqlite3.Row
    try:
        campaign = _campaign_row(connection, campaign_id)
        candidates = _rows(
            connection,
            """
            SELECT candidate_id, generation, status, created_by, lineage_event_id, notes
            FROM candidates
            WHERE campaign_id = ?
            ORDER BY generation, created_at, rowid
            """,
            (campaign["campaign_id"],),
        )
        evaluations = _rows(
            connection,
            """
            SELECT evaluation_id, candidate_id, status, reason, started_at, finished_at
            FROM evaluations
            WHERE campaign_id = ?
            ORDER BY started_at, rowid
            """,
            (campaign["campaign_id"],),
        )
        failures = _rows(
            connection,
            """
            SELECT failure_id, candidate_id, evaluation_id, module_attempt_id,
                   category, stage, severity, retryable, message
            FROM failures
            WHERE evaluation_id IN (
                SELECT evaluation_id FROM evaluations WHERE campaign_id = ?
            )
            ORDER BY created_at
            """,
            (campaign["campaign_id"],),
        )
        failed_candidate_ids = {
            failure["candidate_id"] for failure in failures if failure["candidate_id"]
        }
        lineage = _rows(
            connection,
            """
            SELECT child_candidate_id, parent_candidate_ids_json, operator, reason, mutation_summary_json
            FROM candidate_lineage
            WHERE campaign_id = ?
            ORDER BY created_at
            """,
            (campaign["campaign_id"],),
        )
        module_statuses = [
            {"module_name": row[0], "status": row[1], "count": int(row[2])}
            for row in connection.execute(
                """
                SELECT module_name, status, COUNT(*)
                FROM module_attempts
                WHERE evaluation_id IN (
                    SELECT evaluation_id FROM evaluations WHERE campaign_id = ?
                )
                GROUP BY module_name, status
                ORDER BY module_name, status
                """,
                (campaign["campaign_id"],),
            )
        ]
        runner_states = _rows(
            connection,
            """
            SELECT candidate_id, state, stage, active_module, progress,
                   priority, updated_at, reason, message, metadata_json
            FROM candidate_runner_states
            WHERE campaign_id = ?
            ORDER BY priority, updated_at, candidate_id
            """,
            (campaign["campaign_id"],),
        )
        scores = _candidate_scores(connection, campaign["campaign_id"])
        runner_state_by_candidate = {
            item["candidate_id"]: item for item in runner_states
        }
        for candidate in candidates:
            candidate["score.fixture_total"] = scores.get(candidate["candidate_id"])
            candidate["has_failure"] = candidate["candidate_id"] in failed_candidate_ids
            candidate["runner_state"] = runner_state_by_candidate.get(candidate["candidate_id"])
        if failed_only:
            candidates = [
                candidate for candidate in candidates if candidate["candidate_id"] in failed_candidate_ids
            ]
            candidate_ids = {candidate["candidate_id"] for candidate in candidates}
            evaluations = [
                evaluation for evaluation in evaluations if evaluation["candidate_id"] in candidate_ids
            ]
            lineage = [
                item for item in lineage if item["child_candidate_id"] in candidate_ids
            ]
        ranking = sorted(
            [
                {
                    "candidate_id": candidate["candidate_id"],
                    "generation": candidate["generation"],
                    "score.fixture_total": candidate.get("score.fixture_total"),
                }
                for candidate in candidates
                if candidate.get("score.fixture_total") is not None
            ],
            key=lambda row: float(row["score.fixture_total"]),
            reverse=True,
        )
        return {
            "campaign": _row_dict(campaign),
            "candidates": candidates,
            "evaluations": evaluations,
            "lineage": lineage,
            "failures": failures,
            "runner_states": runner_states,
            "failed_candidate_ids": sorted(failed_candidate_ids),
            "module_statuses": module_statuses,
            "ranking": ranking,
        }
    finally:
        connection.close()


def candidate_comparison_report(
    db_path: Path,
    *,
    candidate_id: str | None = None,
    other_candidate_id: str | None = None,
    campaign_id: str | None = None,
) -> dict[str, Any]:
    connection = sqlite3.connect(db_path)
    connection.row_factory = sqlite3.Row
    try:
        campaign = _campaign_row(connection, campaign_id)
        scores = _candidate_scores(connection, campaign["campaign_id"])
        selected_ids = _comparison_candidate_ids(
            connection,
            campaign["campaign_id"],
            scores,
            candidate_id,
            other_candidate_id,
        )
        left = _candidate_by_id(connection, selected_ids[0])
        right = _candidate_by_id(connection, selected_ids[1])
        left_summary = _candidate_summary(left)
        right_summary = _candidate_summary(right)
        left_summary["score.fixture_total"] = scores.get(left_summary["candidate_id"])
        right_summary["score.fixture_total"] = scores.get(right_summary["candidate_id"])
        return {
            "campaign_id": campaign["campaign_id"],
            "left": left_summary,
            "right": right_summary,
            "variable_delta": _variable_delta(
                left_summary["design_variables"],
                right_summary["design_variables"],
            ),
            "score_delta": _score_delta(
                left_summary.get("score.fixture_total"),
                right_summary.get("score.fixture_total"),
            ),
        }
    finally:
        connection.close()


def optimizer_report(
    db_path: Path,
    *,
    campaign_id: str | None = None,
    optimizer_run_id: str | None = None,
) -> dict[str, Any]:
    connection = sqlite3.connect(db_path)
    connection.row_factory = sqlite3.Row
    try:
        campaign = _campaign_row(connection, campaign_id)
        if optimizer_run_id is None:
            optimizer_run = connection.execute(
                """
                SELECT *
                FROM optimizer_runs
                WHERE campaign_id = ?
                ORDER BY created_at DESC, rowid DESC
                LIMIT 1
                """,
                (campaign["campaign_id"],),
            ).fetchone()
        else:
            optimizer_run = connection.execute(
                "SELECT * FROM optimizer_runs WHERE optimizer_run_id = ?",
                (optimizer_run_id,),
            ).fetchone()
        if optimizer_run is None:
            raise ValueError("optimizer run not found")
        iterations = _rows(
            connection,
            """
            SELECT *
            FROM optimizer_iterations
            WHERE optimizer_run_id = ?
            ORDER BY iteration_index, created_at
            """,
            (optimizer_run["optimizer_run_id"],),
        )
        objective = _loads(optimizer_run["objective_json"])
        score_metric_key = (
            objective.get("primary", "score.fixture_total")
            if isinstance(objective, dict)
            else "score.fixture_total"
        )
        scores = _candidate_scores(
            connection,
            campaign["campaign_id"],
            metric_key=score_metric_key,
        )
        candidates = {
            row["candidate_id"]: _candidate_summary(row)
            for row in connection.execute(
                """
                SELECT *
                FROM candidates
                WHERE campaign_id = ?
                """,
                (campaign["campaign_id"],),
            )
        }
        for candidate_id, score in scores.items():
            if candidate_id in candidates:
                candidates[candidate_id][score_metric_key] = score
        runner_states = _rows(
            connection,
            """
            SELECT candidate_id, state, stage, active_module, progress,
                   priority, updated_at, reason, message, metadata_json
            FROM candidate_runner_states
            WHERE campaign_id = ?
            ORDER BY priority, updated_at, candidate_id
            """,
            (campaign["campaign_id"],),
        )
        for item in runner_states:
            if item["candidate_id"] in candidates:
                candidates[item["candidate_id"]]["runner_state"] = item
        return {
            "campaign": _row_dict(campaign),
            "optimizer_run": _row_dict(optimizer_run),
            "iterations": iterations,
            "runner_states": runner_states,
            "proposed_candidates": candidates,
            "ranking": sorted(
                [
                    {
                        "candidate_id": candidate_id,
                        "status": candidate.get("status"),
                        "generation": candidate.get("generation"),
                        score_metric_key: candidate.get(score_metric_key),
                    }
                    for candidate_id, candidate in candidates.items()
                    if candidate.get(score_metric_key) is not None
                ],
                key=lambda row: float(row[score_metric_key]),
                reverse=True,
            ),
        }
    finally:
        connection.close()


def layout_report(
    db_path: Path,
    *,
    candidate_id: str | None = None,
    evaluation_id: str | None = None,
) -> dict[str, Any]:
    connection = sqlite3.connect(db_path)
    connection.row_factory = sqlite3.Row
    try:
        where = ["module_name = 'pre_export_screening'"]
        params: list[Any] = []
        if candidate_id is not None:
            where.append("candidate_id = ?")
            params.append(candidate_id)
        if evaluation_id is not None:
            where.append("evaluation_id = ?")
            params.append(evaluation_id)
        row = connection.execute(
            f"""
            SELECT *
            FROM module_attempts
            WHERE {' AND '.join(where)}
            ORDER BY started_at DESC, rowid DESC
            LIMIT 1
            """,
            tuple(params),
        ).fetchone()
        if row is None:
            raise ValueError("pre-export screening module attempt not found")
        metrics = _loads(row["metrics_json"]) or {}
        metadata = _loads(row["metadata_json"]) or {}
        layout_model = metadata.get("virtual_component_model", {})
        candidate = _candidate_by_id(connection, row["candidate_id"])
        return {
            "candidate": _candidate_summary(candidate),
            "evaluation_id": row["evaluation_id"],
            "module_attempt_id": row["module_attempt_id"],
            "status": row["status"],
            "failed_checks": metadata.get("failed_checks", []),
            "layout": layout_model.get("layout", {}),
            "components": layout_model.get("components", []),
            "mass_properties": {
                "estimated_total_mass_kg": _metric_value(metrics, "mass.estimated_total_kg"),
                "cg_x_mm": _metric_value(metrics, "mass.cg_x_mm"),
                "cg_y_mm": _metric_value(metrics, "mass.cg_y_mm"),
                "cg_z_mm": _metric_value(metrics, "mass.cg_z_mm"),
                "target_cg_x_mm": _metric_value(metrics, "layout.target_cg_x_mm"),
                "cg_error_mm": _metric_value(metrics, "layout.cg_error_mm"),
                "static_margin": _metric_value(metrics, "stability.static_margin"),
                "static_margin_available": _metric_value(metrics, "stability.static_margin_available"),
                "neutral_point_x_mm": _metric_value(metrics, "stability.neutral_point_x_mm"),
                "layout_wiggle_room_mm": _metric_value(metrics, "layout.wiggle_room_mm"),
            },
            "assumptions": layout_model.get("assumptions", {}),
        }
    finally:
        connection.close()


def _evaluation_row(
    connection: sqlite3.Connection, evaluation_id: str | None
) -> sqlite3.Row:
    if evaluation_id is not None:
        row = connection.execute(
            "SELECT * FROM evaluations WHERE evaluation_id = ?",
            (evaluation_id,),
        ).fetchone()
    else:
        row = connection.execute(
            "SELECT * FROM evaluations ORDER BY created_at DESC, rowid DESC LIMIT 1"
        ).fetchone()
    if row is None:
        raise ValueError("evaluation not found")
    return row


def _campaign_row(connection: sqlite3.Connection, campaign_id: str | None) -> sqlite3.Row:
    if campaign_id is not None:
        row = connection.execute(
            "SELECT * FROM campaigns WHERE campaign_id = ?",
            (campaign_id,),
        ).fetchone()
    else:
        row = connection.execute(
            "SELECT * FROM campaigns ORDER BY created_at DESC, rowid DESC LIMIT 1"
        ).fetchone()
    if row is None:
        raise ValueError("campaign not found")
    return row


def _candidate_summary(row: sqlite3.Row) -> dict[str, Any]:
    data = _row_dict(row)
    data["design_variables"] = _loads(data.pop("design_variables_json"))
    data["normalized_design_vector"] = _loads(data.pop("normalized_design_vector_json"))
    data["constraints_declared"] = _loads(data.pop("constraints_declared_json"))
    return data


def _candidate_by_id(connection: sqlite3.Connection, candidate_id: str) -> sqlite3.Row:
    row = connection.execute(
        "SELECT * FROM candidates WHERE candidate_id = ?",
        (candidate_id,),
    ).fetchone()
    if row is None:
        raise ValueError(f"candidate not found: {candidate_id}")
    return row


def _comparison_candidate_ids(
    connection: sqlite3.Connection,
    campaign_id: str,
    scores: dict[str, float],
    candidate_id: str | None,
    other_candidate_id: str | None,
) -> tuple[str, str]:
    if candidate_id and other_candidate_id:
        return candidate_id, other_candidate_id
    ranked = [
        row[0]
        for row in connection.execute(
            """
            SELECT candidate_id
            FROM candidates
            WHERE campaign_id = ?
            """,
            (campaign_id,),
        )
        if row[0] in scores
    ]
    ranked.sort(key=lambda item: scores[item], reverse=True)
    if len(ranked) < 2:
        raise ValueError("at least two scored candidates are required for default comparison")
    return ranked[0], ranked[1]


def _variable_delta(
    left_variables: dict[str, Any],
    right_variables: dict[str, Any],
) -> dict[str, dict[str, Any]]:
    delta: dict[str, dict[str, Any]] = {}
    for name in sorted(set(left_variables) | set(right_variables)):
        left = left_variables.get(name)
        right = right_variables.get(name)
        left_value = left.get("value") if isinstance(left, dict) else None
        right_value = right.get("value") if isinstance(right, dict) else None
        unit = right.get("unit") if isinstance(right, dict) else left.get("unit") if isinstance(left, dict) else None
        value_delta = None
        if left_value is not None and right_value is not None:
            value_delta = float(right_value) - float(left_value)
        delta[name] = {
            "left": left_value,
            "right": right_value,
            "delta_right_minus_left": value_delta,
            "unit": unit,
        }
    return delta


def _score_delta(left_score: Any, right_score: Any) -> dict[str, Any]:
    if left_score is None or right_score is None:
        return {"left": left_score, "right": right_score, "delta_right_minus_left": None}
    return {
        "left": left_score,
        "right": right_score,
        "delta_right_minus_left": float(right_score) - float(left_score),
    }


def _candidate_scores(
    connection: sqlite3.Connection,
    campaign_id: str,
    *,
    metric_key: str = "score.fixture_total",
) -> dict[str, float]:
    scores: dict[str, float] = {}
    for row in connection.execute(
        """
        SELECT candidate_id, metrics_json
        FROM module_attempts
        WHERE status = 'success'
          AND evaluation_id IN (
              SELECT evaluation_id FROM evaluations WHERE campaign_id = ?
          )
        ORDER BY finished_at, rowid
        """,
        (campaign_id,),
    ):
        metrics = _loads(row["metrics_json"])
        if not isinstance(metrics, dict):
            continue
        score = metrics.get(metric_key, {}).get("value")
        if score is not None:
            scores[row["candidate_id"]] = float(score)
    return scores


def _rows(
    connection: sqlite3.Connection,
    sql: str,
    params: tuple[Any, ...],
) -> list[dict[str, Any]]:
    return [_row_dict(row) for row in connection.execute(sql, params).fetchall()]


def _row_dict(row: sqlite3.Row) -> dict[str, Any]:
    return {key: _loads(row[key]) if key.endswith("_json") else row[key] for key in row.keys()}


def _loads(value: Any) -> Any:
    if value is None:
        return None
    if not isinstance(value, str):
        return value
    return json.loads(value)


def _metric_value(metrics: dict[str, Any], key: str) -> Any:
    value = metrics.get(key)
    if not isinstance(value, dict):
        return None
    return value.get("value")
