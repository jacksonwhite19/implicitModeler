"""Cold larger-case execution through the existing native production runner."""
import argparse
import json
from pathlib import Path
import numpy as np
import geometry_family_authority as geometry
import production_scale_authority as authority
import run_geometry_family as native
from production_phase_profile import Profile
from sdf_cfd_mesher_v4.common.authority import file_hash, identity

OUT = native.ROOT/'results/broader_production_scale_geometry'


def seal():
    record = dict(cases=authority.cohort(), schedule=native.g.q.load_schedule().targets.tolist(),
                  source_sha256=file_hash(Path(geometry.__file__)),
                  design_sha256=file_hash(Path(authority.__file__)))
    path = OUT/'COHORT_MANIFEST.json'
    if path.exists() and json.loads(path.read_text()) != record:
        raise ValueError('FrozenProductionScaleDesignChanged')
    native.g.save(path, record)
    for spec in record['cases']:
        x, tri = geometry.build(spec)
        edges = np.unique(np.sort(np.concatenate([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]]), axis=1), axis=0)
        lengths = np.linalg.norm(x[edges[:, 1]]-x[edges[:, 0]], axis=1)
        native.g.save(OUT/spec['name']/'WALL_AUTHORITY.json', dict(
            spec=spec, vertices=len(x), triangles=len(tri),
            coordinates=native.g.q.array_hash(x), topology=native.g.q.array_hash(tri),
            edge_min=float(lengths.min()), edge_max=float(lengths.max()),
            sdf_identity=geometry.Source(spec).source_identity,
            schedule_identity=identity(record['schedule']), first_layer=record['schedule'][0]))


def run(name, label, steps=None, resume=False):
    spec = next(s for s in authority.cohort() if s['name'] == name)
    folder = OUT/name/label
    profile = Profile(folder/('RESUME_MEMORY.jsonl' if resume else 'COLD_MEMORY.jsonl'))
    originals = []
    def instrument(obj, key, name):
        old = getattr(obj, key)
        originals.append((obj, key, old))
        setattr(obj, key, profile.wrap(old, name))
    def factory(spec, folder):
        with profile.phase('wall_load'):
            provider = native.AnalyticProvider(spec, folder)
        for key in ('initialize', 'propose', 'evidence', 'discover', 'remesh',
                    'numerical_validation', 'certify_validation', 'rebuild', 'bind',
                    'pack', 'geometry', 'candidate_check', 'certify_candidate'):
            instrument(provider, key, key)
        return provider
    from sdf_cfd_mesher_v4.advancing_front.mixed_front_journal import MixedFrontJournal
    for key in ('publish', 'initialize', 'restart', 'transact'):
        instrument(MixedFrontJournal, key, 'checkpoint_'+key)
    instrument(native.g, 'validation', 'hard_quality_and_certificate')
    try:
        profile.record('start', case=name, resume=resume)
        native.run(name, label, steps, resume, spec=spec, output=OUT, provider_factory=factory)
    finally:
        for obj, key, old in reversed(originals):
            setattr(obj, key, old)
        profile.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('case', nargs='?')
    parser.add_argument('--label', default='A')
    parser.add_argument('--steps', type=int)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--design-only', action='store_true')
    args = parser.parse_args()
    if args.design_only:
        seal()
    else:
        run(args.case, args.label, args.steps, args.resume)
