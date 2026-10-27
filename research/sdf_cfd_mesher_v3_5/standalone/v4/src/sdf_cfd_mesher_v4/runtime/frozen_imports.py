"""Import wiring for untouched V3 source files; never invoke a study entry point.

The flat V3 imports are scoped to this loader and removed afterward. The legacy
data paths point inside the source fork, not into live V3 studies. A later task
must bind real inputs explicitly before invoking a legacy planning routine.
"""
from pathlib import Path
import importlib.util
import os
import sys
import threading

PACKAGE = Path(__file__).resolve().parents[1]
_cache = {}
_lock = threading.RLock()


def _load(name, path):
    spec = importlib.util.spec_from_file_location('sdf_cfd_mesher_v4._frozen_'+name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def legacy_modules():
    """Load the frozen dependency closure, with no geometry or optimizer calls."""
    with _lock:
        if 'legacy' in _cache:
            return _cache['legacy']
        frozen = PACKAGE/'legacy_full_stack_planner/_frozen'
        support = PACKAGE/'legacy_full_stack_planner/_support'
        names = ['common','foundation_geometry','accepted_inputs','component_quality',
                 'component_compression','component_graph','component_planner',
                 'diagnostic_convergence','linear_solve','trust_region','two_way_planner',
                 'owned_front_state','owned_physical_predicates','owned_qualified_geometry',
                 'placement_predicates']
        saved = {n: sys.modules.get(n) for n in names}
        path_before = sys.path[:]
        env_before = {n:os.environ.get(n) for n in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')}
        try:
            common = _load('common', frozen/'common.py')
            # Wiring only. No changes to copied source or scientific functions.
            common.V3 = support
            common.H = support/'family'
            common.G = support/'context'
            _load('foundation_geometry', support/'foundation_aware_prism_shell_qualification/foundation_geometry.py')
            modules = {'common':common}
            for name in names[2:11]:
                modules[name] = _load(name, frozen/(name+'.py'))
            _cache['legacy'] = modules
            return modules
        finally:
            sys.path[:] = path_before
            for name,previous in saved.items():
                if previous is None:sys.modules.pop(name,None)
                else:sys.modules[name]=previous
            for name,previous in env_before.items():
                if previous is None:os.environ.pop(name,None)
                else:os.environ[name]=previous


def operator_module():
    """Load compact conditioning calculations; no calibration data is loaded."""
    with _lock:
        if 'operator' in _cache:return _cache['operator']
        root=PACKAGE/'conditioning/_frozen'
        names=('operator_common','operator_model')
        saved={n:sys.modules.get(n) for n in names}
        env_before={n:os.environ.get(n) for n in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')}
        try:
            _load('operator_common',root/'operator_common.py')
            _cache['operator']=_load('operator_model',root/'operator_model.py')
            return _cache['operator']
        finally:
            for n,previous in saved.items():
                if previous is None:sys.modules.pop(n,None)
                else:sys.modules[n]=previous
            for n,previous in env_before.items():
                if previous is None:os.environ.pop(n,None)
                else:os.environ[n]=previous
