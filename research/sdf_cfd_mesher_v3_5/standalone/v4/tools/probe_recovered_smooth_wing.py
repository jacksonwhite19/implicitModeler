"""Geometry-only recovery check. Existing canonical host; no geometry mathematics."""
import hashlib
import json
from pathlib import Path
import struct
import sys
import time

sys.dont_write_bytecode = True
V4 = Path(__file__).resolve().parents[1]
ROOT = V4.parents[1]
sys.path.insert(0, str(V4 / 'src'))
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    started = time.perf_counter()
    out = V4 / 'results/smooth_wing_recovery'
    out.mkdir(parents=True, exist_ok=True)
    family = ROOT / 'research/sdf_cfd_mesher_v3/fresh_supported_refinement_family_qualification_2026-09-09'
    source = family / 'geometry/bound_input.rhai'
    historical = family / 'geometry/WING_AUTHORITY_A.json'
    saved = out / 'recovered_sf2.rhai'
    saved.write_bytes(source.read_bytes())
    authority = json.loads(historical.read_text())
    # Retained sample locations, not a new airfoil or geometry generator.
    samples = authority['queries'][::22]
    points = [x['p'] for x in samples]
    host = ROOT / 'target/debug/canonical_kernel_host.exe'
    runs = []
    for script_file in (source, saved):
        start = time.perf_counter()
        with CanonicalKernel([str(host)], timeout=60) as kernel:
            # SF2's scientific export explicitly interprets raw model units as m.
            geometry = kernel.load_rhai(script_file.read_bytes().decode('utf8'), 1.0)
            inventory = geometry.feature_inventory()
            requests = [dict(feature_id=f['feature_id'], parameter=t)
                        for f in inventory['data']['model']['features']
                        for t in (0.0, 0.25, 0.5, 0.75, 1.0)]
            runs.append(dict(description=geometry.description,
                             inventory=inventory,
                             topology_response=geometry.stored_topology(),
                             feature_evaluations=geometry.feature_evaluate(requests),
                             values=geometry.query(points),
                             load_seconds=geometry.load_seconds))
        runs[-1]['seconds'] = time.perf_counter() - start
    a, b = runs
    same_bits = lambda x, y: struct.pack('<d', x) == struct.pack('<d', y)
    section = authority['sections'][0]
    spans = section['spline']['spans']
    result = dict(
        schema='SmoothWingRecoveryProbe/1', script=source.read_text(),
        script_sha256=sha(source), saved_script_sha256=sha(saved),
        historical_export_sha256=sha(historical), host_sha256=sha(host),
        units='SF2 convention: 1 m per raw model unit, not the application mm default',
        current_run=a,
        fresh_reload=dict(geometry_identity=b['description']['geometry_identity'],
                          description_equal=a['description']==b['description'],
                          inventory_equal=a['inventory']==b['inventory'],
                          feature_evaluations_equal=a['feature_evaluations']==b['feature_evaluations'],
                          scalar_bits_equal=all(same_bits(x,y) for x,y in zip(a['values'], b['values'])),
                          seconds=b['seconds'], load_seconds=b['load_seconds']),
        scalar_count=len(points), points_m=points,
        historical_scalar_bit_matches=sum(same_bits(x, float(q['phi'])) for x,q in zip(a['values'],samples)),
        historical_scalar_max_abs_difference=max(abs(x-float(q['phi'])) for x,q in zip(a['values'],samples)),
        retained_stored_representation=dict(section_count=len(authority['sections']),
            chords=[s['chord'] for s in authority['sections']],
            placements=[s['position'] for s in authority['sections']],
            twists=[s['twist'] for s in authority['sections']],
            equal_splines=all(s['spline']==section['spline'] for s in authority['sections']),
            cubic_spans=len(spans), preview_count=len(section['preview_points']),
            upper_te=spans[0]['control'][0],lower_te=spans[-1]['control'][3]),
        scope='Saved Rhai bytes reloaded in a fresh actual canonical host. No GUI Project save/load or new coefficient export; finite samples are not universal equivalence.',
        elapsed_seconds=time.perf_counter()-started)
    assert result['script_sha256'] == result['saved_script_sha256']
    assert all(result['fresh_reload'][k] for k in ('description_equal','inventory_equal','feature_evaluations_equal','scalar_bits_equal'))
    (out/'PROBE.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', newline='\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('current_run','points_m')}, indent=2))


if __name__ == '__main__':
    main()
