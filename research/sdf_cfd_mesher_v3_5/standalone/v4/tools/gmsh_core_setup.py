"""Owned exact setup_gmsh from the qualified C1 driver; parameters only."""
def setup_gmsh(gmsh, algorithm: int) -> None:
    values = {
        "General.Terminal": 1,
        "General.Verbosity": 5,
        "General.NumThreads": 1,
        "Mesh.MaxNumThreads1D": 1,
        "Mesh.MaxNumThreads2D": 1,
        "Mesh.MaxNumThreads3D": 1,
        "Mesh.Algorithm3D": int(algorithm),
        "Mesh.Algorithm": 6,
        "Mesh.ElementOrder": 1,
        "Mesh.MeshSizeFromPoints": 0,
        "Mesh.MeshSizeFromCurvature": 0,
        "Mesh.MeshSizeExtendFromBoundary": 0,
        "Mesh.MeshSizeFactor": 1.0,
        "Mesh.Optimize": 0,
        "Mesh.OptimizeNetgen": 0,
        "Mesh.OptimizeThreshold": 0.3,
        "Mesh.Reproducible": 1,
        "Mesh.RandomSeed": 1,
        "Mesh.RandomFactor": 1e-12,
        "Mesh.RandomFactor3D": 1e-12,
        "Mesh.Smoothing": 0,
        "Mesh.Binary": 1,
        "Mesh.SaveAll": 1,
    }
    for name, value in values.items():
        gmsh.option.setNumber(name, value)


def setup_profile(gmsh,*,algorithm,threads3d):
    """Explicit owned profile; retain the original setup function and controls."""
    if (type(algorithm) is not int or type(threads3d) is not int
            or (algorithm,threads3d) not in ((1,1),(10,4))):
        raise ValueError('UnsupportedOwnedCoreProfile')
    setup_gmsh(gmsh,1)
    gmsh.option.setNumber('Mesh.Algorithm3D',algorithm)
    gmsh.option.setNumber('General.NumThreads',threads3d)
    gmsh.option.setNumber('Mesh.MaxNumThreads3D',threads3d)
    expected={'Mesh.Algorithm3D':algorithm,'General.NumThreads':threads3d,
        'Mesh.MaxNumThreads1D':1,'Mesh.MaxNumThreads2D':1,'Mesh.MaxNumThreads3D':threads3d,
        'Mesh.OptimizeThreshold':.3,'Mesh.Reproducible':1,'Mesh.RandomSeed':1}
    actual={name:gmsh.option.getNumber(name) for name in expected}
    if actual!=expected:raise ValueError('OwnedCoreProfileReadbackMismatch')
    return actual
