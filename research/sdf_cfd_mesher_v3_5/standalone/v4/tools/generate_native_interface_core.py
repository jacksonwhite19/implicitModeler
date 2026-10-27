"""Protected Gmsh core using native interface and existing qualified controls.

Run in the existing WSL Gmsh environment. V3 functions are read-only references.
"""
from pathlib import Path
from types import SimpleNamespace
import argparse
import importlib.util
import json
import resource
import sys
sys.dont_write_bytecode = True
import time
import numpy as np
import gmsh


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('interface', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--optimize-existing', action='store_true')
    parser.add_argument('--algorithm', type=int, choices=[1, 10], default=1)
    parser.add_argument('--source-mesh', type=Path)
    parser.add_argument('--optimization-name', default='optimized')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    v3 = root / 'research/sdf_cfd_mesher_v3'
    source = v3 / 'airfoil_extrusion_qualification/downstream_runner/run_airfoil_downstream.py'
    spec = importlib.util.spec_from_file_location('qualified_core_reference', source)
    old = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = old
    spec.loader.exec_module(old)
    points = np.load(args.interface / 'points.npy')
    offsets = np.load(args.interface / 'offsets.npy')
    connectivity = np.load(args.interface / 'connectivity.npy')
    if not np.all(np.diff(offsets) == 3):
        raise ValueError('ProtectedNonTriangleInterfaceRequiresExactSurfaceAdapter')
    faces = connectivity.reshape(-1, 3)
    reference = v3 / 'fresh_supported_refinement_family_qualification_2026-09-09/SF2/A'
    support = np.load(reference / 'support/SUPPORT.npz')
    far_points, far_faces = support['positions'][0], support['faces']
    grading = json.loads((reference / 'farfield/CANDIDATE_BINDING.json').read_text())['grading']
    authority = SimpleNamespace(outer_points=points, outer_faces=faces,
                               face_sheet_ids=('native_outer_interface',) * len(faces), curves=(), corners=())
    args.output.mkdir(parents=True, exist_ok=True)
    start = time.perf_counter()
    gmsh.initialize([], False)
    try:
        old.setup_gmsh(gmsh, args.algorithm)
        gmsh.option.setNumber('Mesh.MeshSizeExtendFromBoundary', 1)
        if args.optimize_existing:
            gmsh.open(str(args.source_mesh or args.output / 'generated.msh'))
            sheets, curves, corners = {'native_outer_interface': 1}, {}, {}
            gmsh.model.mesh.optimize('Gmsh', force=True)
            gmsh.write(str(args.output / 'preconditioned.msh'))
            gmsh.finalize()
            gmsh.initialize([], False)
            old.setup_gmsh(gmsh, 1)
            gmsh.open(str(args.output / 'preconditioned.msh'))
            gmsh.model.mesh.createGeometry([(3, 1)])
            gmsh.model.mesh.optimize('Netgen', force=True)
        else:
            gmsh.model.add('native_mixed_interface_core')
            sheets, curves, corners, _ = old.transfer_boundary(gmsh, authority, far_points, far_faces)
            callback = old.CoreSizeCallback(points, .003932938827849285 * .64, .35 * .64, 1.3, farfield_grading=grading)
            gmsh.model.mesh.setSizeCallback(callback)
            gmsh.model.mesh.generate(3)
        generation_seconds = time.perf_counter() - start
        gmsh.write(str(args.output / (args.optimization_name+'.msh' if args.optimize_existing else 'generated.msh')))
        expected = old.signed_surface_volume(far_points, far_faces) - old.signed_surface_volume(points, faces)
        validation, arrays = old.validate_core(gmsh, authority, far_points, far_faces, sheets, curves, corners, expected)
        destination = args.output / args.optimization_name if args.optimize_existing else args.output
        destination.mkdir(exist_ok=True)
        for name, array in arrays.items():
            np.save(destination / (name + '.npy'), array, allow_pickle=False)
        result = dict(validation=validation, generation_seconds=generation_seconds,
                      seconds=time.perf_counter()-start, peak_rss_bytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
                      gmsh_version=gmsh.__version__, qualified=False,
                      remaining='Core hard quality, complete assembly, Foundation and continuous audit')
        (destination / 'GENERATION.json').write_text(json.dumps(result, sort_keys=True, indent=2)+'\n')
    finally:
        gmsh.finalize()


if __name__ == '__main__':
    main()
