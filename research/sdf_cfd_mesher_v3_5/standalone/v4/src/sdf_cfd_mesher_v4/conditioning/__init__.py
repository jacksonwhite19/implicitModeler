"""Frozen weighted sigma and constrained operator gains; no health threshold."""
from ..runtime.frozen_imports import operator_module

_operator = operator_module()
matrices = _operator.matrices
gains = _operator.gains

def weighted_singular_spectrum(m):
    """Expose the unchanged SVD expression from frozen operator_model.gains."""
    np = _operator.np
    return np.linalg.svd(m['N']*np.sqrt(m['W'])[...,None],compute_uv=False)
