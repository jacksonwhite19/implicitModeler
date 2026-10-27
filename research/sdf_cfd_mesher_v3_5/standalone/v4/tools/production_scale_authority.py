"""Fixed resolution expansion of preserved geometries, not new geometry science."""
from copy import deepcopy
import geometry_family_authority as geometry


def cohort():
    base = {s['name']: s for s in geometry.definitions()}
    multi = {s['name']: s for s in geometry.multi_region_definitions()}
    choices = [
        (base['capsule_control'], 'capsule_production_control', 'wide-clearance control'),
        (base['bent_capsule'], 'bent_passage_production', 'thin curved inward passage'),
        (multi['three_interacting_features'], 'interacting_features_production', 'opposing surfaces and branch switching'),
        (multi['four_separated_competing_regions'], 'separated_regions_production', 'four separated concave gap regions'),
    ]
    records = []
    for original, name, purpose in choices:
        spec = deepcopy(original)
        spec.update(name=name, preserved_geometry=original['name'], level=5,
                    production_purpose=purpose,
                    resolution_reason='Five uniform octahedral subdivisions: 8192 triangles per component, '
                    '64 times the preserved level-2 wall population. Dimensions and first-layer schedule unchanged. '
                    'This design is frozen before production execution; no outcome-driven refinement.',
                    domain_classification='Bounded analytic domain; no new authoritative CFD patch/support controls')
        records.append(spec)
    return records
