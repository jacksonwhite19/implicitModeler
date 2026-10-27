"""Frozen quality calculations, with original thresholds untouched."""
from ..runtime.frozen_imports import legacy_modules

def load_calculations():
    return legacy_modules()['component_quality']
