from __future__ import annotations

from aircraft_optimizer.records.module import MetricValue


def mock_geometry_metrics() -> dict[str, dict[str, object]]:
    return {
        "mock.geometry_reference_valid": MetricValue(
            value=True, unit=None, confidence=1.0, source="mock_module"
        ).to_dict(),
        "mock.analysis_depth": MetricValue(
            value="records_only", unit=None, confidence=1.0, source="mock_module"
        ).to_dict(),
    }
