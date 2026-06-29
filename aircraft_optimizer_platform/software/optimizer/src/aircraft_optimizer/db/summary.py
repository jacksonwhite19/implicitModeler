from __future__ import annotations

import sqlite3
from pathlib import Path


CORE_TABLES = [
    "campaigns",
    "variable_schemas",
    "candidates",
    "evaluations",
    "geometry_provider_results",
    "artifacts",
    "module_attempts",
    "events",
    "failures",
    "candidate_lineage",
    "user_annotations",
    "environment_fingerprints",
    "optimizer_runs",
    "optimizer_iterations",
    "candidate_runner_states",
]


def summarize_database(db_path: Path) -> dict[str, object]:
    connection = sqlite3.connect(db_path)
    try:
        counts = {
            table: int(connection.execute(f"SELECT COUNT(*) FROM {table}").fetchone()[0])
            for table in CORE_TABLES
        }
        event_types = [
            row[0]
            for row in connection.execute(
                "SELECT DISTINCT event_type FROM events ORDER BY event_type"
            )
        ]
        evaluation_statuses = [
            row[0]
            for row in connection.execute(
                "SELECT DISTINCT status FROM evaluations ORDER BY status"
            )
        ]
        candidate_statuses = [
            {"status": row[0], "count": int(row[1])}
            for row in connection.execute(
                """
                SELECT status, COUNT(*)
                FROM candidates
                GROUP BY status
                ORDER BY status
                """
            )
        ]
        runner_states = [
            {"state": row[0], "stage": row[1], "count": int(row[2])}
            for row in connection.execute(
                """
                SELECT state, stage, COUNT(*)
                FROM candidate_runner_states
                GROUP BY state, stage
                ORDER BY state, stage
                """
            )
        ]
        module_statuses = [
            {"module_name": row[0], "status": row[1], "count": int(row[2])}
            for row in connection.execute(
                """
                SELECT module_name, status, COUNT(*)
                FROM module_attempts
                GROUP BY module_name, status
                ORDER BY module_name, status
                """
            )
        ]
        artifact_types = [
            {"artifact_type": row[0], "status": row[1], "count": int(row[2])}
            for row in connection.execute(
                """
                SELECT artifact_type, status, COUNT(*)
                FROM artifacts
                GROUP BY artifact_type, status
                ORDER BY artifact_type, status
                """
            )
        ]
        failure_categories = [
            {"category": row[0], "count": int(row[1])}
            for row in connection.execute(
                """
                SELECT category, COUNT(*)
                FROM failures
                GROUP BY category
                ORDER BY category
                """
            )
        ]
        annotation_tags = [
            {"tag": row[0], "count": int(row[1])}
            for row in connection.execute(
                """
                SELECT tag, COUNT(*)
                FROM user_annotations
                GROUP BY tag
                ORDER BY tag
                """
            )
        ]
        return {
            "database": str(db_path),
            "counts": counts,
            "event_types": event_types,
            "evaluation_statuses": evaluation_statuses,
            "candidate_statuses": candidate_statuses,
            "runner_states": runner_states,
            "module_statuses": module_statuses,
            "artifact_types": artifact_types,
            "failure_categories": failure_categories,
            "annotation_tags": annotation_tags,
        }
    finally:
        connection.close()
