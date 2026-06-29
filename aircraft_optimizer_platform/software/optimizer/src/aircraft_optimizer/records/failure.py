from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class FailureDraft:
    category: str
    stage: str
    severity: str
    retryable: bool
    message: str
    evidence_artifact_ids: list[str] = field(default_factory=list)
    suggested_next_action: str | None = None
    metadata: dict[str, object] | None = None

    def validate(self) -> None:
        if not self.category:
            raise ValueError("failure category is required")
        if self.severity not in {"warning", "recoverable", "fatal"}:
            raise ValueError("failure severity must be warning, recoverable, or fatal")
        if not self.message:
            raise ValueError("failure message is required")
