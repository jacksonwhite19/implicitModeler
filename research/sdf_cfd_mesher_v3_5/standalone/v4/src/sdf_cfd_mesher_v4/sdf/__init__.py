"""Canonical-kernel access and the separate historical frozen-section adapter."""
from ..common.interfaces import SdfAccess
from .native import NativeSdf
from .canonical import CanonicalKernel, CanonicalGeometry, EnclosingRegion, GeometryFailure
