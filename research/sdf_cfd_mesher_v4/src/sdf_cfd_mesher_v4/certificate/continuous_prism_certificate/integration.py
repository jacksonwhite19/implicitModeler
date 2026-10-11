"""Current fixed-coordinate replacement for the legacy side-pair invocation.

All other continuous gates remain explicit dependencies of the caller. This
adapter replaces only full_airfoil_bilinear_audit on V3SIDE1 coordinates.
V3SIDH1 remains on the frozen interval-height implementation.
"""
import hashlib
import json
from pathlib import Path
from .runtime import CertificateSession, certify_fixed_layer, CertificationFailure

class ContinuousLayerAdapter:
    def __init__(self, native_gate, cache_capacity=0):
        """native_gate(tool, input_path, tag) must fail closed on any failure."""
        self.native_gate = native_gate
        self.session = CertificateSession(cache_capacity)

    def certify(self, side_input, *, contact_input, front_input, tag):
        side_input = Path(side_input)
        with side_input.open('rb') as stream:
            magic = stream.read(8)
        if magic == b'V3SIDH1\0':
            raise ValueError('UseLegacyIntervalHeightWorkflow')
        result = certify_fixed_layer(side_input, session=self.session)
        # These are not replaced by bilinear side-pair certification.
        retained = [
            self.native_gate('full_airfoil_side_cap_audit', side_input, tag + '_side_cap'),
            self.native_gate('complete_one_layer_collision_audit', contact_input, tag + '_contact'),
            self.native_gate('surface_topology_audit', front_input, tag + '_front'),
        ]
        if any(not record.get('pass', False) for record in retained):
            raise CertificationFailure({'classification': 'RetainedContinuousGateFailure', 'records': retained})
        return {'pass': True, 'side_pairs': result, 'retained_gates': retained,
                'all_layer_overlap_clearance_h1_and_numeric_gates_still_required': True}

def audit_fixed_side_pairs(input_path, output_path, session=None):
    """Direct replacement at a normal fixed-coordinate side-pair call site."""
    result = certify_fixed_layer(input_path, session=session)
    Path(output_path).write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    return result
