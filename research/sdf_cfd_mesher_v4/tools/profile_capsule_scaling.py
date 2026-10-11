"""Capsule-first one-step production benchmark; immutable checkpoint inputs."""
import argparse
import json
import os
import time
from pathlib import Path
import run_geometry_family as native
import production_scale_authority as authority
from production_phase_profile import Profile
from sdf_cfd_mesher_v4.common.authority import file_hash
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore

OUT = native.ROOT / 'results/generic_capsule_certification_scaling'


def select_pointer(source,input_scientific=None):
    pointer = json.loads((source / 'CURRENT.json').read_text())
    if input_scientific is not None:
        matches=[]
        for path in sorted(source.glob('NATIVE_*.json')):
            if json.loads(path.read_text()).get('scientific')==input_scientific:matches.append(path)
        if len(matches)!=1:raise ValueError('AmbiguousOrMissingScientificCheckpoint')
        pointer=dict(pointer,checkpoint=matches[0].name,sha256=file_hash(matches[0]),scientific=input_scientific)
    return pointer


def run(case, label, source, steps=1, input_scientific=None):
    folder = OUT / case / label
    target = folder / 'checkpoints'
    target.mkdir(parents=True, exist_ok=False)
    pointer=select_pointer(source,input_scientific)
    manifest = source / pointer['checkpoint']
    assert file_hash(manifest) == pointer['sha256']
    envelope = json.loads(manifest.read_text())
    (target / 'objects').mkdir()
    def descriptors(row):
        yield from row.get('authenticated',{}).get('files',())
        for segment in row.get('proof_segments',()):
            for key in ('faces','pairs','codes','payloads'):yield segment[key]
            yield from segment.get('dependency',{}).values()
        if row['encoding']=='receipt_delta':
            yield from descriptors(row['base'])
            yield from descriptors(row['updates'])
            yield row['removed']
        else:
            yield row
    seen={}
    for row in (r for c in envelope['components'].values() for r in descriptors(c)):
        if row['file'] in seen:
            assert seen[row['file']]==row['sha256']
            continue
        seen[row['file']]=row['sha256']
        old = source / 'objects' / row['file']
        assert file_hash(old) == row['sha256']
        destination=target / 'objects' / row['file']
        if not destination.exists():os.link(old,destination)
    (target / pointer['checkpoint']).write_bytes(manifest.read_bytes())
    (target / 'CURRENT.json').write_text(json.dumps(pointer))
    native.g.save(folder / 'INPUT.json', dict(source=str(source), pointer=pointer))
    native.g.save(folder/'SOURCE_SNAPSHOT.json',
                  {p.relative_to(native.ROOT).as_posix():file_hash(p)
                   for part in ('src','tools') for p in sorted((native.ROOT/part).rglob('*.py'))})
    profile = Profile(folder / 'PROFILE.jsonl')
    original = []
    def wrap(obj, key, name):
        fn = getattr(obj, key)
        original.append((obj, key, fn))
        measured=profile.wrap(fn,name)
        def observed(*args,**kwargs):
            result=measured(*args,**kwargs)
            if name in ('certificate_reuse_and_coverage','new_exact_certificate'):
                data={k:result.get(k) for k in ('required_pairs','reused','new','proof_hash','passed','methods')}
                proofs=result['proofs']
                if hasattr(proofs,'storage'):data['storage']=proofs.storage()
                if hasattr(proofs,'work'):data['delta_accounting']=proofs.work
                if 'authenticated_identity' in result:
                    data['authenticated_identity']=result['authenticated_identity']
                    data['authenticated_work']=proofs.native_delta['authenticated_table'].authenticated['work']
                profile.record('certificate_accounting',stage=name,**data)
            elif name=='bind':
                proofs=args[0]['certificate']
                profile.record('receipt_accounting',retained=result['retained'],
                               retired={k:len(v) for k,v in result['retired'].items()},
                               required=result['required_relations'],
                               stored_update_records=len(proofs.updates) if hasattr(proofs,'updates') else len(proofs))
            return result
        setattr(obj, key, observed)
    def factory(spec, path):
        with profile.phase('wall_load'):
            provider = native.AnalyticProvider(spec, path)
        for key in ('propose', 'evidence', 'numerical_validation', 'certify_validation',
                    'rebuild', 'bind', 'pack', 'geometry', 'candidate_check', 'certify_candidate'):
            wrap(provider, key, key)
        return provider
    import reuse_mixed_certificate as reuse
    import native_mixed_face_certificate as certificate
    import validation_row_stream as stream
    wrap(reuse, 'certify', 'certificate_reuse_and_coverage')
    wrap(certificate, 'certify', 'new_exact_certificate')
    wrap(native.g.volume, 'audit', 'volume_audit')
    wrap(NativeCheckpointStore, 'restart', 'checkpoint_load_and_audit')
    wrap(NativeCheckpointStore, 'publish', 'checkpoint_serialize')
    from sdf_cfd_mesher_v4.certificate.aabb_index import AABBIndex
    from sdf_cfd_mesher_v4.certificate.efficient import Filter
    original_pairs=AABBIndex.pairs
    def observed_pairs(index,active=None):
        values=iter(original_pairs(index,active));seconds=0.;count=0
        try:
            while True:
                tick=time.perf_counter()
                try:pair=next(values)
                except StopIteration:
                    seconds+=time.perf_counter()-tick
                    break
                seconds+=time.perf_counter()-tick;count+=1
                yield pair
        finally:
            profile.record('candidate_search_accounting',seconds_in_generator=seconds,
                           candidates=count,faces=len(index.lo),active_faces=None if active is None else len(active))
    original.append((AABBIndex,'pairs',original_pairs));AABBIndex.pairs=observed_pairs
    original_screen=Filter.screen
    screening=dict(calls=0,pairs=0,kernel_seconds=0.,largest_batch=0)
    def observed_screen(engine,q,labels,pairs):
        tick=time.perf_counter();value=original_screen(engine,q,labels,pairs)
        screening['kernel_seconds']+=time.perf_counter()-tick;screening['calls']+=1
        screening['pairs']+=len(pairs);screening['largest_batch']=max(screening['largest_batch'],len(pairs))
        return value
    original.append((Filter,'screen',original_screen));Filter.screen=observed_screen
    def observe(event, state, provider):
        profile.record(event, cells=len(state['cells']), relations=len(state['certificate']),
                       layer=max(c['layer'] for c in state['cells'].values()))
        from retained_state_storage import retained_storage
        with profile.phase('retained_object_census'):
            measured=retained_storage(dict(
                committed_geometry={k:state[k] for k in ('wall','first_layer','vertices','cells','faces','columns','outer','versions')},
                active_front=state['prism_front'],quality_receipts=state['quality'],
                certificate_receipts=state['certificate'],atlas=state['atlas'],forecast=state['forecast'],
                report_history=provider.records,mesh_working_set=provider.meshes,
                validation_working_set=provider.checked,numeric_working_set=provider.numerical))
        profile.record('retained_class_accounting',checkpoint_event=event,
                       layer=max(c['layer'] for c in state['cells'].values()),**measured)
    start = time.perf_counter()
    observe.profile=profile
    try:
        spec = next(s for s in authority.cohort() if s['name'] == case)
        native.run(case, label, steps, True, spec=spec, output=OUT,
                   provider_factory=factory, observer=observe)
    finally:
        from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import RECEIPT_AUDIT_STATS
        from sdf_cfd_mesher_v4.advancing_front.mixed_front import SCIENTIFIC_HASH_STATS
        profile.record('hash_audit_accounting',receipt_audits=RECEIPT_AUDIT_STATS,scientific_hashes=SCIENTIFIC_HASH_STATS)
        profile.record('interval_kernel_accounting',**screening)
        profile.record('benchmark_end', runtime=time.perf_counter()-start)
        profile.close()
        for obj, key, fn in reversed(original):
            setattr(obj, key, fn)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('label')
    parser.add_argument('--case', default='capsule_production_control')
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--steps', type=int, default=1)
    parser.add_argument('--input-scientific')
    args = parser.parse_args()
    run(args.case, args.label, args.source, args.steps, args.input_scientific)
