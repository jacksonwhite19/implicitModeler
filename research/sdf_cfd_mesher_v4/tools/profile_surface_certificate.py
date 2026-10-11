"""Bounded profile/equivalence check of retained inputs; never generates a mesh."""
import argparse
import cProfile
import hashlib
import json
from pathlib import Path
import pstats
import sys
import time
if sys.flags.optimize: raise RuntimeError('OptimizedPythonUnsupportedForVerification')

import numpy as np
import psutil

V4 = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(V4/'src'), str(V4/'tools')]
sys.dont_write_bytecode = True
from sdf_cfd_mesher_v4.sdf.canonical import CanonicalKernel
from surface_binding import bind_canonical
from continuous_geometry_certifier import certify_face


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sample', type=int, default=32)
    parser.add_argument('--candidate', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    retained = V4/'results/live_sf2_surface/replay_20260929/authority'
    with np.load(retained/'SURFACE.npz') as z, np.load(retained/'REFERENCES.npz') as r:
        x, f, refs = z['coordinates'], z['triangles'], r['face_refs']
    ids = np.unique(np.concatenate([np.linspace(0, len(f)-1, args.sample, dtype=int),
        [np.flatnonzero(refs == ref)[0] for ref in np.unique(refs)]]))
    case=V4/'cases/standalone_smooth_development.json'
    script=json.loads(case.read_text(encoding='utf-8'))['geometry']['rhai']
    if args.candidate:
        # Diagnostic only: explicitly selected binary, not a production fallback.
        import canonical_spline_authority as owner
        old = owner.DLL
        owner.DLL = args.candidate.resolve()
        owner.LOCKS = {p:h for p,h in owner.LOCKS.items() if p != old}
        owner.LOCKS[owner.DLL] = hashlib.sha256(owner.DLL.read_bytes()).hexdigest()
    import canonical_spline_authority as owner
    paths=(Path(__file__),case,owner.DLL,V4/'runtime/bin/canonical_kernel_host.exe',
        V4/'runtime/surface/continuous_geometry_certifier.py',
        V4/'tools/canonical_spline_authority.py',retained/'SURFACE.npz',retained/'REFERENCES.npz')
    before={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    with CanonicalKernel([str(V4/'runtime/bin/canonical_kernel_host.exe')], timeout=60) as host:
        a = bind_canonical(host.load_rhai(script, 1.)).authority
        profiler = cProfile.Profile()
        t = time.perf_counter()
        profiler.enable()
        rows = [certify_face(a, int(refs[i]), x[f[i]]) for i in ids]
        profiler.disable()
        seconds = time.perf_counter()-t
        stats = pstats.Stats(profiler)
        functions = sorted([dict(file=k[0],line=k[1],name=k[2],primitive_calls=v[0],
            calls=v[1],self_seconds=v[2],cumulative_seconds=v[3]) for k,v in stats.stats.items()],
            key=lambda row: row['cumulative_seconds'],reverse=True)
        result = dict(schema='V4SurfaceCertificateProfile/1',sample_ids=ids.tolist(),
            sample_refs=refs[ids].tolist(),baseline=rows,seconds=seconds,
            functions=functions[:45],query_cache=a.unit.query.cache_info()._asdict(),
            geometry_identity=a.geometry_identity,payload_sha256=owner.digest(a.source['sections'][0]['spline']),
            dll_path=str(owner.DLL),dll_sha256=before[str(owner.DLL.resolve())],
            producer_sha256=before[str(Path(__file__).resolve())],inputs=before,
            peak_rss=psutil.Process().memory_info().peak_wset,
            sources={str(p.relative_to(V4)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                (V4/'runtime/surface/continuous_geometry_certifier.py',
                 V4/'tools/canonical_spline_authority.py',retained/'SURFACE.npz',retained/'REFERENCES.npz')})
        if before!={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}:
            raise ValueError('ProfileInputChanged')
        (args.output/'PROFILE.json').write_text(json.dumps(result,indent=2,allow_nan=True)+'\n',encoding='utf-8')
        print(json.dumps(dict(seconds=seconds,faces=len(ids),functions=functions[:12],query_cache=result['query_cache'])),flush=True)


if __name__ == '__main__':
    main()
