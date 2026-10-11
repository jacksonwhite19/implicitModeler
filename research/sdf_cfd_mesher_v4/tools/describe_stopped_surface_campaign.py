"""Read-only descriptive evidence after STOP; no geometry queries or qualification.

Print JSON to stdout. Reuse the existing surface-array statistics. Never write
into the failed surface directory, restart a stage, or claim qualification.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import psutil
from audit_adaptive_surface import audit_arrays, dist


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('campaign', type=Path)
    args = parser.parse_args()
    start = time.perf_counter()
    root = args.campaign
    stop = json.loads((root / 'STOP.json').read_text())
    if stop['passed'] or not stop['no_retry']:
        raise ValueError('StoppedCampaignRequired')
    folder = root / 'wing/fresh'
    with np.load(folder / 'SURFACE.npz') as z, np.load(folder / 'REFERENCES.npz') as r:
        x, f, m = z['coordinates'], z['triangles'], z['metric']
        description, _ = audit_arrays(x, f, metrics=m)
        ce = r['curve_edges']
        d = x[ce[:, 1]] - x[ce[:, 0]]
        avg = (m[ce[:, 0]] + m[ce[:, 1]]) / 2
        curve_metric = np.sqrt(np.einsum('ni,nij,nj->n', d, avg, d))
        refs, counts = np.unique(ce[:, 2], return_counts=True)
        description.update(
            scope='Post-stop array description only; not an independent qualification report',
            acceptance='unavailable: qualifier aborted before returning its report',
            source_boundary_bounds='unavailable: local results were not persisted before diagnostic query failure',
            intersections='not reached', replay='not started', publication='not performed',
            stored_owning_residual_m=dist(abs(z['residual'])),
            curve_edges=len(ce), curve_ids=len(refs),
            curve_edge_length_m=dist(np.linalg.norm(d, axis=1)),
            curve_metric_length=dist(curve_metric),
            segments_per_curve=dict(zip(map(str, refs), map(int, counts))),
            sheet_ids=len(np.unique(r['face_refs'])),
            corner_vertices=int(np.count_nonzero(r['corner_refs'])),
            corner_ids=len(np.unique(r['corner_refs'][r['corner_refs'] != 0])),
            degenerate_area_count=int(np.count_nonzero(_['area'] <= 0)),
            scalar_batch_requested=len(x) + len(f),
            input_files={name: hashlib.sha256((folder / name).read_bytes()).hexdigest()
                         for name in ('SURFACE.npz', 'REFERENCES.npz', 'GENERATION.json')},
        )
    description['seconds'] = time.perf_counter() - start
    description['peak_working_set_bytes'] = psutil.Process().memory_info().peak_wset
    print(json.dumps(description, sort_keys=True, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
