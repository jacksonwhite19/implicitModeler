"""Production prism request admission. This module generates no geometry.

Physical requests are external authorities. Surface edge lengths, curvature,
legacy SF2 schedules and successful geometry gates cannot supply a request.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from production_surface_authority import load_production_wall
from sdf_cfd_mesher_v4.common.authority import array_hash, file_hash, identity

VERSION = 'V4ProductionPrismSizingContract/2'
CONTRACT = {
    'schema': VERSION,
    'external_request_required': True,
    'numeric_height_growth_thickness_defaults': None,
    'accepted_request_kinds': ['CFDPhysicsRequest', 'UserPhysicalRequest',
                               'DeclaredProductionTestRequest'],
    'required_bindings': ['geometry_identity', 'surface_identity', 'fluid_side',
                         'request_owner', 'physical_basis', 'total_thickness_intent', 'source_documents'],
    'schedule_encoding': 'canonical float.hex strings; immutable little-endian binary64 increments',
    'total_encoding': 'binary64 sequential cumulative sum of the preserved increments',
    'physical_total_intent': 'separate declaration; never round or rescale the binary64 schedule to its design target',
    'first_height_meaning': 'full prism normal thickness, not wall-to-cell-centre distance',
    'implemented_request_scope': 'global first height and explicit positive per-layer increments',
    'local_request_scope': 'requires separate qualified source-vertex mapping; rejected here',
    'geometry_constraints': ['source-bound oriented unit normals', 'source ownership',
                             'offset/trajectory validity', 'positive exact Jacobians and volume',
                             'applicable hard quality', 'actual segment and cell/contact proofs'],
    'wall_assurance': 'Authenticated wall receipt selects formal qualification or sampled numerical acceptance; this request admission cannot upgrade that assurance.',
    'surface_metric_role': 'tangential feasibility input; never a physical normal-height selector',
    'first_height_modification': 'forbidden without a separately qualified explicit authority',
    'later_modifications': 'not authorized by first-layer admission; preserve original and modified schedules separately',
    'first_layer_determinant': 'actual lateral incidence plus declared internal top-cap role; no fabricated neighbour centroid',
    'qualification_obligations': ['wall preservation', 'normal and target audit', 'projection audit',
                                 'applicable hard prism gates', 'ownership and topology',
                                 'continuous cell/contact geometry', 'exact scientific replay in archived qualification'],
    'routine_omissions': 'Numerical surface receipt does not prove whole-source completeness, continuous wall face accuracy or exact final wall root/normal certificates. Actual prism normal-height/segment and cell predicates remain required.',
    'task_scope': 'layer 1 only; no layer-2 proposal or forecast; no core',
    'admission_is_mesh_qualification': False,
}


def contract_identity():
    return identity({'contract': CONTRACT, 'implementation_sha256': file_hash(__file__)})


def _hex64(value):
    if not isinstance(value, str):
        raise ValueError('Binary64HexRequired')
    try:
        number = float.fromhex(value)
    except (ValueError, OverflowError) as exc:
        raise ValueError('InvalidBinary64') from exc
    if not math.isfinite(number) or number <= 0 or number.hex() != value:
        raise ValueError('PositiveCanonicalBinary64Required')
    return number


def decode_schedule(request):
    """Validate supplied numbers; never select a height, count or growth ratio."""
    if request.get('height_scope') != 'global':
        raise ValueError('LocalScheduleRequiresQualifiedMapping')
    if request.get('scalar_quantity') != 'normal_height_m':
        raise ValueError('AmbiguousHeightMeaning')
    encoded = request.get('increments_m_hex')
    count = request.get('requested_layer_count')
    if not isinstance(encoded, list) or not encoded or type(count) is not int or count != len(encoded):
        raise ValueError('ScheduleInventoryMismatch')
    # bytes-backed arrays cannot be made writeable after validation.
    increments = np.frombuffer(np.asarray([_hex64(x) for x in encoded], dtype='<f8').tobytes(), dtype='<f8')
    with np.errstate(over='ignore'):
        targets = np.cumsum(increments, dtype=np.float64)
    if not np.isfinite(targets).all() or np.any(np.diff(np.r_[0., targets]) <= 0):
        raise ValueError('NonIncreasingBinary64Targets')
    total = _hex64(request.get('schedule_total_m_hex'))
    if total.hex() != float(targets[-1]).hex():
        raise ValueError('ScheduleTotalMismatch')
    targets = np.frombuffer(targets.astype('<f8').tobytes(), dtype='<f8')
    schedule_id = identity({'increments_sha256': array_hash(increments),
                            'targets_sha256': array_hash(targets),
                            'height_scope': 'global', 'scalar_quantity': 'normal_height_m'})
    return increments, targets, schedule_id


def read_request(path, surface):
    """Read a case-bound declaration and hash-bound source evidence.

    This verifies provenance consistency, not the truth of an external CFD
    model. The named request owner remains responsible for its physical basis.
    """
    path = Path(path).resolve()
    declaration = json.loads(path.read_text(encoding='utf-8'))
    if declaration.get('schema') != 'V4ExternalPrismRequest/1':
        raise ValueError('UnsupportedPrismRequestSchema')
    if declaration.get('kind') not in CONTRACT['accepted_request_kinds']:
        raise ValueError('FixtureOrLegacyRequestNotProductionAuthority')
    if declaration.get('production_intent') is not True:
        raise ValueError('ExplicitProductionRequestRequired')
    for name in ('request_owner', 'physical_basis', 'total_thickness_intent'):
        if not isinstance(declaration.get(name), str) or not declaration[name].strip():
            raise ValueError('MissingExternalRequestProvenance:' + name)
    for name in ('geometry_identity', 'surface_identity'):
        if declaration.get(name) != surface[name]:
            raise ValueError('RequestSourceMismatch:' + name)
    if declaration.get('fluid_side') not in ('positive_sdf', 'negative_sdf'):
        raise ValueError('FluidSideRequired')
    sources = declaration.get('source_documents')
    if not isinstance(sources, list) or not sources:
        raise ValueError('ExternalBasisEvidenceRequired')
    for row in sources:
        if not isinstance(row, dict) or not isinstance(row.get('path'), str) or not row['path']:
            raise ValueError('InvalidExternalBasisReference')
        if not isinstance(row.get('sha256'), str) or len(row['sha256']) != 64:
            raise ValueError('InvalidExternalBasisHash')
        source = (path.parent / row['path']).resolve()
        if source == path or file_hash(source) != row['sha256']:
            raise ValueError('ExternalBasisHashMismatch')
    increments, targets, schedule_id = decode_schedule(declaration)
    return dict(declaration=declaration, request_file_sha256=file_hash(path),
                schedule_identity=schedule_id, increments=increments, targets=targets)


def inspect_first_layer_inputs(surface_directory, request_path=None):
    """Production entry guard: load the qualified wall before request admission."""
    wall, faces, surface = load_production_wall(surface_directory)
    common = dict(contract_identity=contract_identity(), surface_identity=surface['surface_identity'],
                  surface_assurance=surface.get('assurance','ArchivedQualifiedSurface'),
                  geometry_identity=surface['geometry_identity'], coordinate_hash=surface['coordinate_hash'],
                  topology_hash=surface['topology_hash'], wall_vertices=len(wall), wall_triangles=len(faces),
                  generated_layers=0, generated_core_cells=0, forecast_layers=0,
                  first_layer_qualified=False)
    if request_path is None:
        return dict(common, decision='C', status='ExternalNormalScheduleRequired',
                    generation_authorized=False, schedule_identity=None,
                    missing=['case-bound physical or explicit production-test request',
                             'first full prism normal thickness with global/local meaning',
                             'fluid side, schedule increments/count and total-thickness intent',
                             'request owner and physical basis with source evidence'])
    bound = read_request(request_path, surface)
    return dict(common, decision='A', status='ExternalScheduleProvenanceAdmitted',
                generation_authorized=True, schedule_identity=bound['schedule_identity'],
                request_identity=identity(bound['declaration']), request_file_sha256=bound['request_file_sha256'],
                first_height_m_hex=float(bound['increments'][0]).hex(),
                requested_layer_count=len(bound['increments']), height_scope='global',
                fluid_side=bound['declaration']['fluid_side'], geometry_feasibility='Not measured.')


def require_generation_authorization(decision):
    """Reject an absent admission. A receipt is not a mesh-publication token.

    A future generator must re-load its surface and request at invocation,
    then validate the resulting geometry independently before publication.
    """
    if decision.get('generation_authorized') is not True or decision.get('decision') != 'A':
        raise ValueError('AuthoritativeNormalScheduleMissing')
    if decision.get('contract_identity') != contract_identity() or not decision.get('schedule_identity'):
        raise ValueError('StaleOrIncompletePrismAdmission')
    return decision
