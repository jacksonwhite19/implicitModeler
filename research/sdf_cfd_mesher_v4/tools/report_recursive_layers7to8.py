"""Seal measured bounded-layer results without modifying scientific payloads."""
from collections import Counter
from pathlib import Path
import subprocess
from qualify_recursive_layers7to8 import OUT, V4, HEAD, read, file_hash, durable_json
from recursive_production_verification import inventory
from sdf_cfd_mesher_v4.common.authority import identity


def run():
    layers = {}
    for layer in (7, 8):
        out = OUT/f'L{layer}'
        authority = read(out/'AUTHORITY.json')
        assert authority['passed']
        plan = read(out/'CANDIDATE_PLAN.json')
        storage = inventory(out)
        groups = {}
        for row in storage['files']:
            row['sha256'] = file_hash(row['path'])
            groups.setdefault(row['sha256'], {})[(row['device'], row['file_id'])] = row['cluster_bytes']
        storage['duplicate_physical_within_layer'] = sum(sum(g.values())-max(g.values()) for g in groups.values())
        active = set(plan['references'])
        active.update(str(p.resolve()) for p in (out/'candidate').rglob('*') if p.is_file())
        permanent = {'AUTHORITY.json','COMPACT_CERTIFICATE.json','SEAL.json','DIAGNOSTICS.json',
            'DEEP.json','REPLAY.json','RETAIN.json','CLEANUP_MANIFEST.json','REOPEN.json','SAFETY.json',
            'DIRECT.json','FORECAST.json','LOCALIZATION.json','GENERATION_PHASES.json','PUBLICATION.json',
            'REFERENCE.json','PRIOR_DETERMINANT_COMPARISON.json','PREPARE_METRICS.json',
            'PROOF_INDEX_METRICS.json','SQL_COMMITS.json','START_STORAGE.json'}
        buckets = Counter()
        for row in storage['files']:
            path = Path(row['path'])
            category = 'active' if row['path'] in active else 'permanent_compact' if path.parent == out and (path.name in permanent or path.name.endswith('_MEASURED.json')) else 'preserved_qualification'
            buckets[category] += row['cluster_bytes']
        storage['categories'] = dict(buckets)
        body = read(out/'candidate'/('ROOT_'+authority['reference']['root']+'.json'))
        paths = {r['path']:r['cluster_bytes'] for r in storage['files']}
        storage['component_allocations'] = dict(
            proof_index=paths[str((out/'objects/new-proof-key-index.sqlite').resolve())],
            new_incidence=sum(r['cluster_bytes'] for r in storage['files'] if Path(r['path']).is_relative_to((out/'new_incidence').resolve())),
            published_quality=paths[body['quality_delta']], published_bindings=paths[body['binding_patch']],
            quality_delta_input=paths[str((out/'QUALITY_DELTA.json').resolve())])
        storage['starting_free'] = read(out/'START_STORAGE.json')['free'] if (out/'START_STORAGE.json').exists() else read(OUT/'PREFLIGHT.json')['free']
        storage['ending_free_after_retention'] = read(out/'RETAIN.json')['after']['free']
        durable_json(out/'FINAL_STORAGE.json', storage)
        normal = {n: read(out/(n+'_MEASURED.json')) for n in ('ADMISSION','GENERATION','ASSEMBLY','PUBLICATION')}
        totals = dict(seconds=sum(x['metrics']['seconds'] for x in normal.values()),
            cpu=sum(x['metrics']['cpu'] for x in normal.values()),
            peak_rss=max(x['metrics']['peak_rss'] for x in normal.values()),
            reads=sum(x['metrics']['io']['read_bytes'] for x in normal.values()),
            writes=sum(x['metrics']['io']['write_bytes'] for x in normal.values()))
        pub = normal['PUBLICATION']['metrics']['io']['write_bytes']
        idx = read(out/'INDEX_MEASURED.json')['metrics']['io']['write_bytes']
        inc = read(out/'INCIDENCE_MEASURED.json')['metrics']['io']['write_bytes']
        volume = read(out/'VOLUME_DELTA.json')
        layers[str(layer)] = dict(authority=authority, normal=normal, totals=totals,
            publication_write_attribution=dict(total=pub, proof_index=idx, incidence=inc,
                remaining_quality_binding_staging_metadata=pub-idx-inc,
                complete_exclusive_attribution=False),
            admission=read(out/'ADMISSION.json'), forecast=read(out/'FORECAST.json'),
            localization=read(out/'LOCALIZATION.json'), generation=read(out/'GENERATION.json'),
            quality_front_height=read(out/'DIAGNOSTICS.json'), assembly=read(out/'ASSEMBLY.json'),
            determinant_comparison=read(out/'PRIOR_DETERMINANT_COMPARISON.json'),
            continuous=read(out/'FRESH_CERTIFICATE.json'), index=read(out/'INDEX_MEASURED.json'),
            global_certification=dict(exact_balance=volume['exact_balance'],
                degree_witnesses=len(volume['global_degree']['values']),
                degrees=sorted({r['degree'] for r in volume['global_degree']['values']}),
                unresolved=volume['global_degree']['unresolved'], passed=volume['global_degree']['passed']),
            incidence=read(out/'INCIDENCE_MEASURED.json'), publication=read(out/'PUBLICATION.json'),
            reopen=read(out/'REOPEN.json'), deep=read(out/'DEEP.json'), replay=read(out/'REPLAY.json'),
            safety=read(out/'SAFETY.json'), retention={k:v for k,v in read(out/'RETAIN.json').items() if k not in ('before','after')},
            storage={k:v for k,v in storage.items() if k!='files'})
    for p,h in read(OUT/'PREFLIGHT.json')['protected'].items():
        assert file_hash(p)==h, p
    status = subprocess.check_output(['git','status','--porcelain=v1','-z'])
    owned = (b'results/recursive_layers7to8', b'qualify_recursive_layers7to8.py',
        b'recursive_production_diagnostics.py', b'recursive_production_publication.py',
        b'recursive_production_verification.py', b'report_recursive_layers7to8.py', b'test_recursive_layers7to8.py')
    unrelated = [r for r in status.split(b'\0') if r and not any(name in r for name in owned)]
    assert unrelated == [r for r in bytes.fromhex(read(OUT/'PREFLIGHT.json')['status_hex']).split(b'\0') if r]
    assert len(unrelated) == 155 and not read(OUT/'V3_VERIFICATION.json')['changed']
    report = dict(classification='V4ProductionAdaptiveCapsuleAdvancingFrontLayers7To8IncrementalQualified',
        starting_head=HEAD, ending_head='Commit containing this report', layers=layers,
        starting_authority=read(OUT/'PREFLIGHT.json')['reference'],
        git=dict(unrelated_dirty_entries=155, status_preserved=True, V3_inventory_entries=266010, V3_changes=0),
        production=read(OUT/'PRODUCTION.json'), epoch_renewal=False, hardlinks_created=0,
        legacy_fallbacks=0, L9_admission=False, L9_post_L8_forecast=False, L9_generation=False, core=False,
        limitations=['Qualification deep verification and replay ran concurrently in separate measured processes.',
            'Process I/O counters do not fully represent memory-mapped page-fault reads.',
            'Exact temporary allocation peak and exclusive subphase CPU/I/O were not measured.',
            'Publication residual combines quality, binding, staging and metadata writes; it is not attributed by file.',
            'No historical checkpoint retirement or nonidentical diagnostic cleanup was attempted.'],
        tests=read(OUT/'TESTS_FINAL.json'),
        recommended_next_task='QualifyV4CapsuleLayers9To11IncrementalEndToEndBounded: independent admission, certification, publication, restore and retention per layer; stop immediately on failure.')
    report['identity'] = identity(report)
    durable_json(OUT/'REPORT.json', report)
    files = {str(p.relative_to(OUT)):file_hash(p) for p in OUT.rglob('*.json')
        if p.parent in (OUT,OUT/'L7',OUT/'L8') and p.name not in ('AUTHORITY.json',)}
    authority = dict(classification=report['classification'], qualification=report['identity'],
        authoritative_layer=8, reference=report['production']['reference'],
        scientific=report['production']['scientific'], files=files)
    authority['identity'] = identity(authority)
    durable_json(OUT/'AUTHORITY.json', authority)
    print(dict(classification=report['classification'], qualification=report['identity'],
        report_sha256=file_hash(OUT/'REPORT.json'), authority_sha256=file_hash(OUT/'AUTHORITY.json')))


if __name__ == '__main__':
    run()
