from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class PipelineConfig:
    pipeline_id: str
    pipeline_version: str
    raw: dict[str, Any]

    def validate(self) -> None:
        if not self.pipeline_id:
            raise ValueError("pipeline_id is required")
        if not self.pipeline_version:
            raise ValueError("pipeline_version is required")
        modules = self.raw.get("modules")
        if not isinstance(modules, list) or not modules:
            raise ValueError("pipeline modules are required")
        for module in modules:
            if not isinstance(module, dict):
                raise ValueError("pipeline module entries must be objects")
            if not module.get("module_name"):
                raise ValueError("pipeline module_name is required")
            if not module.get("module_kind"):
                raise ValueError("pipeline module_kind is required")


def load_pipeline_config(path: Path) -> PipelineConfig:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("pipeline config must be a JSON object")
    config = PipelineConfig(
        pipeline_id=str(data["pipeline_id"]),
        pipeline_version=str(data["pipeline_version"]),
        raw=data,
    )
    config.validate()
    return config
