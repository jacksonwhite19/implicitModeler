"""Frozen lineage, component ownership and dependency-closure access."""
from ..runtime.frozen_imports import legacy_modules

def source_types():
    return legacy_modules()['accepted_inputs'].State

def component_graph():
    return legacy_modules()['component_graph']

def compression_reference():
    return legacy_modules()['component_compression']
