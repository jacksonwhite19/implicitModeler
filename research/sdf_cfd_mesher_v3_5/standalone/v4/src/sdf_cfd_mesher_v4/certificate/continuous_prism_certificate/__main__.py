"""Normal fixed-coordinate certification entrypoint.

Qualification is deliberately a separate explicit command and module.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from .runtime import certify_fixed_layer, certify_shell, CertificateSession, CertificationFailure

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['production', 'qualification'])
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cache-capacity', type=int, default=0)
    args = parser.parse_args()
    if args.mode == 'qualification':
        parser.error('Qualification requires the separate qualify_integration.py evidence runner; it is never automatic.')
    session = CertificateSession(args.cache_capacity)
    try:
        if args.input.suffix == '.npz':
            with np.load(args.input) as data:
                result = certify_shell(data['positions'], data['triangles'], vertices=data['vertices'], session=session)
        else:
            result = certify_fixed_layer(args.input, session=session)
        status = 0
    except CertificationFailure as error:
        result = {'pass': False, 'failure': error.record}; status = 3
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    return status

if __name__ == '__main__':
    raise SystemExit(main())
