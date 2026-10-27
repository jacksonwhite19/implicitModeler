"""Explicit case-policy binding, not a metric or geometry implementation.

The recovered surface-only development-equivalence package is opt-in. Policy
approval belongs to the case owner. This module never starts a meshing stage.
"""
from types import MethodType
import math
import json
from pathlib import Path

from historical_capsule_surface import SurfaceSizing, metric
from canonical_polygon_authority import PolygonAuthority
from polygon_boundary_contract import SourceBoundaryContract
from surface_binding import provenance


def load_case(path, *, require_approval=True):
    """Fail closed on stale policy sources and an unapproved case proposal."""
    from historical_capsule_surface import REPO, sha
    case = json.loads(Path(path).read_text())
    if case['schema'] != 'V4SurfaceCaseProposal/1' or not case.get('source_files'):
        raise ValueError('CasePolicyProvenanceRequired')
    for source, digest in case['source_files'].items():
        if sha(REPO / source) != digest:
            raise ValueError('CasePolicySourceMismatch:' + source)
    if require_approval and (case['approval']['status'] != 'approved' or
            not case['approval']['reference'] or not case['execution']['authorized']):
        raise ValueError('CasePolicyApprovalRequired')
    return case


def _shared_curve_metric(authority, vertex, curve_refs, multiplier):
    binding = authority._development_policy_binding
    policy = binding.sizing_policy
    _validate_package(policy, binding.qualification)
    if multiplier != policy.metric_multiplier:
        raise ValueError('DevelopmentMetricMultiplierMismatch')
    if (vertex.sheet_ref not in authority.sheet_names or
            set(curve_refs) - set(authority.curve_names)):
        raise ValueError('DevelopmentMetricReferenceMismatch')
    # Ownership/incidence remain the existing source and mesh checks. This
    # straight-curve package adds no separate feature length or curve formula.
    return metric(authority, vertex.point, policy, sheet_ref=vertex.sheet_ref)


def _validate_package(policy, contract):
    policy.validate()
    if policy != SurfaceSizing():
        raise ValueError('DevelopmentPackagePolicyMismatch')
    expected = dict(position_m=policy.position_limit_m,
                    normal_rad=math.radians(policy.normal_limit_deg))
    if not isinstance(contract, SourceBoundaryContract) or contract.accuracy_limits != expected:
        raise ValueError('DevelopmentPackageAccuracyMismatch')


def bind_development_policy(binding, policy):
    """Bind one explicitly supplied, coupled package to admitted affine topology.

    This narrow policy binding applies only to the already supported stored
    polygon representation. It does not admit a new representation or supply
    user approval, physical prism requirements, or a qualification outcome.
    """
    _validate_package(policy, binding.qualification)
    a = binding.authority
    if not isinstance(a, PolygonAuthority):
        raise ValueError('DevelopmentPackageRepresentationMismatch')
    binding.qualification.verify(a)
    if callable(getattr(a, 'metric', None)):
        raise ValueError('ExistingMetricBindingMustNotBeOverwritten')
    binding.sizing_policy = policy
    a._development_policy_binding = binding
    a.metric = MethodType(_shared_curve_metric, a)
    binding.operation_provenance['metric'] = provenance(a.metric)
    return binding
