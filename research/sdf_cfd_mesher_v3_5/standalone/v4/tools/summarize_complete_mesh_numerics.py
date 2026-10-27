"""Read-only numerical diagnostics and a scientific convergence figure."""
from qualify_complete_mesh_numerics import *


def diagnose():
    out = OUTPUT / 'runs/A/handoff'
    q = np.memmap(out / 'q.f64', '<f8', mode='r')
    fvc = np.memmap(out / 'fvcU_after.f64', '<f8', mode='r')
    sf = np.memmap(out / 'Sf.f64', '<f8', mode='r').reshape(-1, 3)
    inc = np.memmap(out / 'incidence.bin', '<i8', mode='r').reshape(-1, 2)
    ni = int(np.sum(inc[:, 1] >= 0))
    area = np.linalg.norm(sf[:ni], axis=1)
    z = (fvc[:ni] - q[:ni]) / (area * np.sqrt(62.))
    face = int(np.argmax(abs(z)))
    result = read(OUTPUT / 'runs/A/RESULT.json')
    assert float(abs(z[face])) == result['handoff']['internal_roundtrip']['Linf']
    native = MESH_AUTHORITY / 'foundation/foundation_audit'
    centre = np.memmap(out / 'Cf.f64', '<f8', mode='r').reshape(-1, 3)[face].tolist()
    ids = inc[face].tolist()
    cells = []
    assembly = read(MESH_AUTHORITY / 'ASSEMBLY.json')
    shell_end = assembly['shell_prisms']
    support_end = shell_end + assembly['support_prisms']
    for cell in ids:
        region = 'shell' if cell < shell_end else 'support' if cell < support_end else 'core'
        metrics = {name: float(np.memmap(native / filename, '<f8', mode='r')[cell]) for name, filename in [
            ('determinant', 'cell_determinants.f64'), ('volume_m3', 'cell_volumes.f64'), ('aspect_ratio', 'cell_aspect_ratios.f64')]}
        cells.append(dict(native_cell=cell, region=region, quality=metrics,
                          centroid_m=np.memmap(native / 'cell_centres.f64', '<f8', mode='r').reshape(-1, 3)[cell].tolist()))
    quality = {name: float(np.memmap(native / filename, '<f8', mode='r')[face]) for name, filename in [
        ('weight', 'face_weights.f64'), ('volume_ratio', 'face_volume_ratios.f64'), ('skewness', 'face_skewness.f64')]}
    write(OUTPUT / 'DIAGNOSTIC_LOCATION.json', dict(native_face=face, centre_m=centre, adjacent_cells=cells,
          face_quality=quality, normalized_difference=float(z[face]), raw_difference=float(fvc[face] - q[face]),
          area_m2=float(area[face]), q=float(q[face]), fvc_flux_U=float(fvc[face]),
          mesh_binding=read(OUTPUT / 'MESH_INPUT.json')['mesh'],
          interpretation='Location of the larger diagnostic Linf. This is an observed difference, not an inferred causal mechanism or a new numerical gate.'))
    source = REFERENCE / 'numerical_storage/results/SF2_A/OUTER_ITERATION_HISTORY.json'
    write(OUTPUT / 'V3_REFERENCE_HISTORY.json', dict(source=str(source), sha256=sha(source), history=read(source)))


def plot():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    a, b = [read(OUTPUT / 'runs' / r / 'OUTER_ITERATION_HISTORY.json') for r in ['A', 'A2']]
    old = read(OUTPUT / 'V3_REFERENCE_HISTORY.json')['history']
    assert all(x['metrics'] == y['metrics'] and x['pcg'] == y['pcg'] for x, y in zip(a, b)) and len(a) == len(b)
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.1), constrained_layout=True)
    for rows, label, color in [(old, 'V3 reference', '#8b9199'), (a, 'V4 A = A2', '#1769aa')]:
        steps = [r['solve'] for r in rows]
        axes[0].semilogy(steps, [r['metrics']['updated_residual']['L2'] for r in rows], label=label, color=color)
        axes[1].plot(steps, [r['pcg']['iterations'] for r in rows], label=label, color=color)
    axes[0].axhline(1e-6, color='#444444', linestyle=':', linewidth=1, label='Frozen L2 gate')
    axes[0].set_ylabel('Refreshed outer residual L2')
    axes[1].set_ylabel('PCG iterations per solve')
    for axis in axes:
        axis.set_xlabel('Outer solve')
        axis.grid(alpha=.18)
        axis.legend(frameon=False)
    fig.suptitle('Frozen numerical stack: exact V4 replay, slower outer convergence')
    fig.savefig(OUTPUT / 'convergence.png', dpi=180)
    plt.close(fig)


if __name__ == '__main__':
    {'diagnose': diagnose, 'plot': plot}[sys.argv[1]]()
