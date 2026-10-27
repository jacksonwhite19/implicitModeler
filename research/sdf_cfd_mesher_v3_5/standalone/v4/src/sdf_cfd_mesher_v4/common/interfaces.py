"""Future V4 data contracts only. No front, cache, or forecast implementation."""
from __future__ import annotations
from dataclasses import dataclass
from typing import Mapping, Protocol, Sequence
from numpy.typing import NDArray
import numpy as np

FloatArray = NDArray[np.float64]
IndexArray = NDArray[np.int64]

@dataclass(frozen=True)
class ForecastKey:
    geometry_authority_sha256: str
    component_key: str
    committed_layer: int
    generation: int

@dataclass(frozen=True)
class FrontState:
    """Arrays are immutable by contract; frozen dataclasses do not freeze NumPy.

    coordinates_m: (vertices,3); triangles: (faces,3); lineage_ids and
    component_ids: (vertices,). Source ancestry retains original wall identities.
    Forecast coordinates never replace committed coordinates.
    """
    geometry_authority_sha256: str
    committed_layer: int
    coordinates_m: FloatArray
    triangles: IndexArray
    lineage_ids: IndexArray
    source_ancestry: Mapping[int, tuple[int, ...]]
    target_distances_m: FloatArray
    component_ids: IndexArray
    forecast_keys: tuple[ForecastKey, ...] = ()

@dataclass(frozen=True)
class LayerRequest:
    next_layer: int
    target_distances_m: FloatArray
    requested_total_layers: int = 20

@dataclass(frozen=True)
class AdvancingFrontConfig:
    requested_layers: int = 20
    first_layer_m: float = 0.000166527069
    # Must be supplied from the bound physical contract, not inferred from text.
    nominal_growth: float | None = None

@dataclass(frozen=True)
class SdfForecast:
    key: ForecastKey
    sample_lineages: IndexArray
    distances_m: FloatArray
    normals: FloatArray
    curvature_per_m: FloatArray | None = None
    clearance_m: FloatArray | None = None
    risk_notes: tuple[str, ...] = ()

@dataclass(frozen=True)
class DiscreteLookahead:
    key: ForecastKey
    future_layers: tuple[int, ...]
    provisional_coordinates_m: FloatArray
    sigma: FloatArray
    gain_bound: FloatArray
    quality: Mapping[str, FloatArray]
    compression: FloatArray | None = None

@dataclass(frozen=True)
class ComponentForecast:
    key: ForecastKey
    cheap: SdfForecast
    detailed: DiscreteLookahead | None = None

@dataclass(frozen=True)
class FrontCandidate:
    parent_geometry_sha256: str
    layer_index: int
    coordinates_m: FloatArray
    changed_component_keys: tuple[str, ...]

@dataclass(frozen=True)
class FrontRelaxationResult:
    candidate: FrontCandidate
    diagnostics: Mapping[str, float]
    converged: bool

@dataclass(frozen=True)
class LayerCertificationResult:
    candidate_geometry_sha256: str
    hard_quality_pass: bool
    continuous_geometry_pass: bool
    topology_ownership_pass: bool
    unresolved: tuple[str, ...]
    certificate_authority_sha256: str
    proof_references: tuple[str, ...]

class SdfAccess(Protocol):
    """Future access to the existing Rhai/SmoothSpline authority; units are SI."""
    def distance_and_normal(self, points_m: FloatArray) -> tuple[FloatArray, FloatArray]: ...

class ForecastCache(Protocol):
    """Keys include geometry, topology/ancestry identity and generation.

    Certificate reuse also requires exact ordered coordinates, topology,
    declared contact ownership and mathematical authority identity.
    """
    def get(self, key: ForecastKey) -> ComponentForecast | None: ...
    def invalidate(self, changed_components: Sequence[str], *, committed_layer: int) -> None: ...

class ForecastApi(Protocol):
    def cheap_sdf_forecast(self, front: FrontState, request: LayerRequest,
                           sdf: SdfAccess, cache: ForecastCache) -> Sequence[SdfForecast]: ...
    def detailed_discrete_lookahead(self, front: FrontState, request: LayerRequest,
                                    components: Sequence[str], cache: ForecastCache) -> Sequence[DiscreteLookahead]: ...

class BuilderApi(Protocol):
    def propose_next_front(self, front: FrontState, request: LayerRequest,
                           forecasts: Sequence[ComponentForecast]) -> FrontCandidate: ...
    def relax_front(self, front: FrontState, candidate: FrontCandidate) -> FrontRelaxationResult: ...
    def project_to_target_distance(self, candidate: FrontCandidate, request: LayerRequest,
                                    sdf: SdfAccess) -> FrontCandidate: ...
    def certify_candidate_layer(self, front: FrontState, candidate: FrontCandidate) -> LayerCertificationResult: ...
    def commit_layer(self, front: FrontState, candidate: FrontCandidate,
                      certificate: LayerCertificationResult, cache: ForecastCache) -> FrontState: ...
