"""Write setup documentation and import adapters, not scientific algorithms."""
from prepare_source_fork import ROOT, PKG

files = {
'src/sdf_cfd_mesher_v4/__init__.py': '''"""SDF CFD Mesher V4. Source scaffold only; no advancing-front implementation."""
__version__ = "0.0.0"
SETUP_ONLY = True
''',
'src/sdf_cfd_mesher_v4/legacy_full_stack_planner/__init__.py': '''"""V3 legacy lookahead/reference planner. Never V4 production logic.

Retired scientific gates still exist in this byte-identical historical source.
Do not use them for V4 admission. No legacy study driver is exposed here.
"""
from ..runtime.frozen_imports import legacy_modules

def load_reference_module():
    return legacy_modules()['two_way_planner']
''',
'src/sdf_cfd_mesher_v4/quality/__init__.py': '''"""Frozen quality calculations, with original thresholds untouched."""
from ..runtime.frozen_imports import legacy_modules

def load_calculations():
    return legacy_modules()['component_quality']
''',
'src/sdf_cfd_mesher_v4/geometry/__init__.py': '''"""Frozen lineage, component ownership and dependency-closure access."""
from ..runtime.frozen_imports import legacy_modules

def source_types():
    return legacy_modules()['accepted_inputs'].State

def component_graph():
    return legacy_modules()['component_graph']

def compression_reference():
    return legacy_modules()['component_compression']
''',
'src/sdf_cfd_mesher_v4/conditioning/__init__.py': '''"""Frozen weighted sigma and constrained operator gains; no health threshold."""
from ..runtime.frozen_imports import operator_module

_operator = operator_module()
matrices = _operator.matrices
gains = _operator.gains

def weighted_singular_spectrum(m):
    """Expose the unchanged SVD expression from frozen operator_model.gains."""
    np = _operator.np
    return np.linalg.svd(m['N']*np.sqrt(m['W'])[...,None],compute_uv=False)
''',
'src/sdf_cfd_mesher_v4/certificate/__init__.py': '''"""Frozen qualified continuous prism-side certificate."""
from .continuous_prism_certificate import CertificateSession, CertificationFailure, certify_local, certify_shell
''',
'src/sdf_cfd_mesher_v4/sdf/__init__.py': '''"""Existing SDF source snapshots and future access protocol.

The native bridge and SmoothAuthority are preserved under _frozen. Binding the
native library and geometry authority belongs to the first implementation task.
No alternative SDF evaluation or distance projection is implemented here.
"""
from ..common.interfaces import SdfAccess
''',
'src/sdf_cfd_mesher_v4/forecast/__init__.py': '''"""Forecast interfaces only; no forecast algorithm."""
from ..common.interfaces import SdfForecast, DiscreteLookahead, ComponentForecast, ForecastApi
''',
'src/sdf_cfd_mesher_v4/advancing_front/__init__.py': '''"""Front interfaces only; no growth, relaxation, projection or commit engine."""
from ..common.interfaces import FrontState, LayerRequest, FrontCandidate, FrontRelaxationResult, LayerCertificationResult, AdvancingFrontConfig, BuilderApi
''',
'src/sdf_cfd_mesher_v4/runtime/__init__.py': '''"""Future incremental cache contract and frozen-source import wiring."""
from ..common.interfaces import ForecastCache, ForecastKey
''',
'src/sdf_cfd_mesher_v4/common/__init__.py': '''"""Narrow V4 contracts. Deterministic legacy helpers remain frozen."""
''',
'src/sdf_cfd_mesher_v4/io/__init__.py': '''"""Reserved for explicit authority input/output adapters. No mesh writer."""
''',
'.gitignore': '__pycache__/\n*.py[cod]\n.pytest_cache/\n',
'pyproject.toml': '''[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[project]
name = "sdf-cfd-mesher-v4"
version = "0.0.0"
description = "Setup-only SDF-guided relaxed advancing-front research scaffold"
requires-python = ">=3.12"
dependencies = ["numpy", "scipy", "sympy", "casadi"]

[tool.setuptools.packages.find]
where = ["src"]

[tool.setuptools.package-data]
"*" = ["**/*.py", "**/*.json", "**/*.rs", "**/*.md"]
''',
'README.md': '''# SDF CFD Mesher V4

**SDF-Guided Relaxed Advancing Front. Setup only.**

V4 will keep full-stack SDF lookahead but commit geometry one front at a time.
V3 remains intact as the reference implementation and research authority.
V3 full-stack planning became complex around a small outer trailing-edge tail.
V4 tests whether a different construction order can reduce that difficulty.

The planned loop is forecast, predict, relax tangentially, project to the target
SDF distance, improve conditioning, check hard quality, certify, then commit.
The requested physical contract remains 20 layers with h1=0.000166527069 m.
The V3 authority records growth 1.163359674. The request text `163359674` is also
preserved in DATA_AUTHORITIES.json. No growth has been run to resolve that text.

`src/sdf_cfd_mesher_v4` contains package contracts and byte-identical frozen V3
source. `legacy_full_stack_planner` is explicitly a V3 lookahead/reference
dependency, not V4 production logic. Its historical floors must not become V4
admission gates. The newer unpromoted planner is recorded as a reference only;
the fork follows the exact two-way planner hash supplied for this task.

`authorities/V3_SOURCE_FORK_MANIFEST.json` identifies every copied file.
`SOURCE_COPY_PLAN.json` records the copy decision before copying.
`DATA_AUTHORITIES.json` points to existing V3 geometry, surface, polyMesh,
certificate integration and numerical executable. Large data are not copied.
Paths are relative to the repository root. A polyMesh hash is a canonical
member-hash aggregate, not a hash of a directory or of its identity JSON.

Run setup checks from the repository root:

```powershell
python -B research/sdf_cfd_mesher_v4/tools/run_setup_tests.py
```

Existing Python packages are reused. No installation or scientific executable
is required. The tests load source modules and validate manifests. They do not
run mesh generation, prism generation, planner studies, full-shell certificates,
OpenFOAM, CFD or RANS. Copied certificate tests are retained but not discovered
or executed by this setup suite.

Not implemented: front growth, relaxation, distance projection, forecast
algorithms, cache engine, local layer termination, optimizer changes, comparison
studies and native SDF binding. Source-level imports do not qualify scientific
behavior. Native SDF snapshots are not compiled during setup.

See ARCHITECTURE.md and SETUP_REPORT.md. The only recommended next task is
**ImplementV4CommittedFrontAndCheapSdfForecastFoundation**.
''',
'tools/COMPARISON_CONTRACT.md': '''# Future V3 comparison tools

Use the same immutable geometry, surface topology and physical layer request.
Compare runtime, sigma distribution, G_bound, standard quality, compression,
layer completeness, weak-tail location and deterministic reproduction.
Keep accepted V3 evidence separate from newly measured V4 results. Bind all
inputs and code by hash. No comparison runner or result exists in this setup.
''',
'src/sdf_cfd_mesher_v4/legacy_full_stack_planner/README.md': '''# V3 legacy lookahead/reference planner

The supplied c3813b15... two-way planner is development evidence, not a promoted
authority. It is preserved byte-for-byte with its source dependency closure.
It retains historical protected floors inside the archived functions. V4 has
no planner implementation and imports none of these gates into its contracts.

Purpose: initial lookahead, detailed future-stack forecast, regression
comparison and warm-start information. It must not own committed V4 coordinates.

The V4 loader changes only module resolution and source-root bindings. It maps
support paths into `_support`, with no imports from live V3. Legacy data-loading
functions remain dormant and require a future explicit input adapter. Historical
cohorts, study drivers, numerical cases and outputs are not copied.

Some source files mix reusable functions with dormant study entry points. They
are retained only to preserve source identity and are never run as scripts.
No legacy execution entry point is offered by the V4 package.
''',
}

if __name__ == '__main__':
    for name,text in files.items():
        p=ROOT/name;p.parent.mkdir(parents=True,exist_ok=True)
        assert not p.exists(),p
        p.write_text(text,encoding='utf-8')
    print('Wrote',len(files),'scaffold files')
