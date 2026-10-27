"""V3 legacy lookahead/reference planner. Never V4 production logic.

Retired scientific gates still exist in this byte-identical historical source.
Do not use them for V4 admission. No legacy study driver is exposed here.
"""
from ..runtime.frozen_imports import legacy_modules

def load_reference_module():
    return legacy_modules()['two_way_planner']
