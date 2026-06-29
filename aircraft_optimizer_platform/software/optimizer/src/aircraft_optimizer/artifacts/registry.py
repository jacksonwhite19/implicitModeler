from __future__ import annotations

import hashlib
import shutil
import sqlite3
from pathlib import Path
from typing import Any

from aircraft_optimizer.ids import new_id
from aircraft_optimizer.jsonutil import dumps
from aircraft_optimizer.time import utc_now_iso


def sha256_file(path: Path) -> str:
    hasher = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            hasher.update(chunk)
    return hasher.hexdigest()


def register_artifact(
    connection: sqlite3.Connection,
    *,
    artifact_root: Path,
    source_path: Path,
    artifact_type: str,
    candidate_id: str | None,
    evaluation_id: str | None,
    module_attempt_id: str | None,
    producer_module: str,
    producer_version: str,
    source_kind: str,
    metadata: dict[str, Any] | None = None,
    source_original_path: str | None = None,
) -> str:
    artifact_id = new_id("artifact")
    artifact_root.mkdir(parents=True, exist_ok=True)
    destination = artifact_root / f"{artifact_id}_{source_path.name}"
    shutil.copy2(source_path, destination)
    content_hash = sha256_file(destination)
    stat = destination.stat()
    connection.execute(
        """
        INSERT INTO artifacts (
            artifact_id, candidate_id, evaluation_id, module_attempt_id,
            artifact_type, path, content_hash, hash_algorithm, created_at,
            producer_module, producer_version, mime_type, size_bytes,
            metadata_json, source_kind, source_original_path, status
        )
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            artifact_id,
            candidate_id,
            evaluation_id,
            module_attempt_id,
            artifact_type,
            str(destination),
            content_hash,
            "sha256",
            utc_now_iso(),
            producer_module,
            producer_version,
            "text/plain" if source_path.suffix.lower() == ".rhai" else None,
            stat.st_size,
            dumps(metadata or {}),
            source_kind,
            source_original_path,
            "available",
        ),
    )
    return artifact_id


def register_artifact_reference(
    connection: sqlite3.Connection,
    *,
    artifact_type: str,
    path: str,
    candidate_id: str | None,
    evaluation_id: str | None,
    module_attempt_id: str | None,
    producer_module: str,
    producer_version: str,
    source_kind: str,
    status: str = "referenced",
    metadata: dict[str, Any] | None = None,
    source_original_path: str | None = None,
) -> str:
    artifact_id = new_id("artifact")
    connection.execute(
        """
        INSERT INTO artifacts (
            artifact_id, candidate_id, evaluation_id, module_attempt_id,
            artifact_type, path, content_hash, hash_algorithm, created_at,
            producer_module, producer_version, mime_type, size_bytes,
            metadata_json, source_kind, source_original_path, status
        )
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            artifact_id,
            candidate_id,
            evaluation_id,
            module_attempt_id,
            artifact_type,
            path,
            None,
            None,
            utc_now_iso(),
            producer_module,
            producer_version,
            None,
            None,
            dumps(metadata or {}),
            source_kind,
            source_original_path,
            status,
        ),
    )
    return artifact_id
