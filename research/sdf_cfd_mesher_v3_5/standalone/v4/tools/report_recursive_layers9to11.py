"""Compact final-shell evidence. Never manufacture an unmeasured result."""
from collections import Counter
import psutil
from qualify_recursive_layers9to11 import *
from path_preserving_deduplication import metadata,physical_key
from recursive_production_verification import inventory


def run():
    if (OUT/'FAILURE.json').exists():
        raise ValueError('NoCompleteClassificationAfterFailure')
    deep = read(OUT/'FINAL_DEEP.json'); replay = read(OUT/'FINAL_REPLAY.json'); retention = read(OUT/'FINAL_RETAIN.json')
    assert deep['passed'] and replay['passed'] and retention['passed']
    assert read(OUT/'TESTS_FINAL.json')['passed'] and read(OUT/'TESTS_SCIENTIFIC.json')['passed']
    layers = {}
    for layer in (9,10,11):
        out = OUT/f'L{layer}'; authority = read(out/'AUTHORITY.json')
        assert authority['passed']
        assert identity({k:v for k,v in authority.items() if k!='identity'}) == authority['identity']
        assert file_hash(out/'COMPACT_CERTIFICATE.json') == authority['compact_certificate_sha256']
        for name,digest in authority['files'].items():
            assert file_hash(out/name) == digest,name
        assert read(out/'FINAL_POST_REOPEN.json')['passed']
        plan = read(out/'CANDIDATE_PLAN.json'); storage = inventory(out)
        active = set(plan['references'])
        active.update(str(p.resolve()) for p in (out/'candidate').rglob('*') if p.is_file())
        groups = {}; categories = Counter()
        permanent = {'AUTHORITY.json','COMPACT_CERTIFICATE.json','SEAL.json','DIAGNOSTICS.json','ADMISSION.json','DIRECT.json',
            'FORECAST.json','LOCALIZATION.json','GENERATION.json','GENERATION_PHASES.json','ASSEMBLY.json','FRESH_CERTIFICATE.json',
            'PUBLICATION.json','REFERENCE.json','REOPEN.json','SAFETY.json','RETAIN.json','CLEANUP_MANIFEST.json',
            'FINAL_CLEANUP_MANIFEST.json','FINAL_POST_REOPEN.json','FINAL_REPLAY_LAYER.json','PRIOR_DETERMINANT_COMPARISON.json',
            'START_STORAGE.json','PROOF_INDEX_METRICS.json','SQL_COMMITS.json','PREPARE_METRICS.json'}
        for row in storage['files']:
            path = Path(row['path']); digest = file_hash(path)
            groups.setdefault(digest,{})[physical_key(row)] = row['cluster_bytes']
            category = 'active' if row['path'] in active else 'permanent_compact' if path.parent == out and (path.name in permanent or path.name.endswith('_MEASURED.json')) else 'preserved_qualification'
            categories[category] += row['cluster_bytes']
        storage['categories'] = dict(categories)
        storage['inherited'] = read(out/'INHERITED_STORAGE.json')
        storage['normal_before_replay'] = read(out/'PRE_REPLAY_STORAGE.json')
        storage['duplicate_physical'] = sum(sum(g.values())-max(g.values()) for g in groups.values())
        body = read(out/'candidate'/('ROOT_'+authority['reference']['root']+'.json'))
        storage['component_allocations'] = dict(proof_index=metadata(out/'objects/new-proof-key-index.sqlite')['cluster_bytes'],
            incidence=sum(r['cluster_bytes'] for r in storage['files'] if Path(r['path']).is_relative_to((out/'new_incidence').resolve())),
            quality=metadata(body['quality_delta'])['cluster_bytes'],bindings=metadata(body['binding_patch'])['cluster_bytes'])
        durable_json(out/'FINAL_STORAGE.json',storage)
        phases = {n:read(out/(n+'_MEASURED.json')) for n in ('ADMISSION','GENERATION','ASSEMBLY','PUBLICATION')}
        totals = {k:sum(r['metrics'][k] for r in phases.values()) for k in ('seconds','cpu')}
        totals['peak_rss'] = max(r['metrics']['peak_rss'] for r in phases.values())
        totals['io'] = {k:sum(r['metrics']['io'][k] for r in phases.values()) for k in ('read_bytes','write_bytes','read_count','write_count')}
        pub = phases['PUBLICATION']['metrics']['io']['write_bytes']
        idx = read(out/'INDEX_MEASURED.json')['metrics']['io']['write_bytes']
        inc = read(out/'INCIDENCE_MEASURED.json')['metrics']['io']['write_bytes']
        volume = read(out/'VOLUME_DELTA.json'); degree = volume['global_degree']
        layers[str(layer)] = dict(authority=authority,normal=phases,totals=totals,
            admission=read(out/'ADMISSION.json'),forecast=read(out/'FORECAST.json'),localization=read(out/'LOCALIZATION.json'),
            generation=read(out/'GENERATION.json'),diagnostics=read(out/'DIAGNOSTICS.json'),assembly=read(out/'ASSEMBLY.json'),
            continuous=read(out/'FRESH_CERTIFICATE.json'),index=read(out/'INDEX_MEASURED.json'),incidence=read(out/'INCIDENCE_MEASURED.json'),
            publication=read(out/'PUBLICATION.json'),reopen=read(out/'REOPEN.json'),safety=read(out/'SAFETY.json'),
            storage={k:v for k,v in storage.items() if k!='files'},
            final_cleanup={k:v for k,v in retention['layers'][str(layer)].items() if k not in ('before','after')},
            global_checks=dict(exact_balance=volume['exact_balance'],witnesses=len(degree['values']),
                degrees=sorted({r['degree'] for r in degree['values']}),unresolved=degree['unresolved'],passed=degree['passed']),
            publication_write_attribution=dict(total=pub,proof_index=idx,incidence=inc,
                residual_quality_bindings_staging_metadata=pub-idx-inc,exclusive_complete_attribution=False))
    refs = {}
    with CommittedState(read(OUT/'L11/REFERENCE.json')) as state:
        a = state
        while a is not None:
            if a.kind == 'native': refs.update(a.wrapped.body['files'])
            elif a.kind == 'clean_sheet': refs.update(a.wrapped.plan['references']); refs.update(a.wrapped.body['produced'])
            else: refs.update(a.body['plan']['references']); refs.update(a.body['produced'])
            a = getattr(a,'parent',None)
        rows = [metadata(p) for p in refs]; unique = {physical_key(r):r for r in rows}
        closure = dict(files=len(rows),logical=sum(r['bytes'] for r in rows),physical=sum(r['cluster_bytes'] for r in unique.values()))
    final = dict(read(OUT/'L11/AUTHORITY.json'))
    final['legacy_scientific'] = deep['scientific']
    final['normal_component_scientific_root'] = final['content']
    report = dict(classification='V4ProductionAdaptiveCapsuleAdvancingFrontQualified',scope='Complete requested L1-L11 prism shell only',
        starting_head=HEAD,ending_head='Commit containing this report',starting_authority=read(OUT/'AUTHORITIES_VERIFIED.json'),
        layers=layers,deep=deep,replay=replay,retention_metrics=retention['metrics'],final=final,closure=closure,
        starting_free=read(OUT/'PREFLIGHT.json')['free'],ending_free=shutil.disk_usage(OUT).free,
        task_allocation=inventory(OUT)['physical'],physical_memory=psutil.virtual_memory()._asdict(),
        epoch_renewal=False,attested_hardlinks_created=0,legacy_fallbacks=0,layer_beyond_11=False,core_handoff=False,core=False,
        tests=read(OUT/'TESTS_FINAL.json'),scientific_tests=read(OUT/'TESTS_SCIENTIFIC.json'),git=read(OUT/'GIT_VERIFICATION.json'),
        limitations=['Normal scientific identities are authenticated component roots; the legacy full-state digest is computed once in final deep qualification.',
            'Exact temporal allocation peak and exclusive internal subphase CPU/IO were not measured.',
            'Publication residual combines quality, bindings, staging and metadata; no complete per-file attribution is claimed.',
            'Process counters do not include every memory-mapped page-fault read.',
            'Required recursive ancestry remains active; checkpoint retirement is not qualified.',
            'No core handoff, core mesh, complete volume mesh, or CFD numerical qualification is claimed.'],
        recommended_next_task='QualifyV4BoundedCoreHandoffFromSealedLayer11Front')
    report['implementation'] = {name:file_hash(V4/'tools'/name) for name in (
        'qualify_recursive_layers9to11.py','verify_recursive_layers9to11.py','final_recursive_shell_qualification.py',
        'retain_final_recursive_shell.py','report_recursive_layers9to11.py','recursive_production_publication.py',
        'recursive_production_diagnostics.py','composable_admission.py','composable_transition.py',
        'composable_assembly.py','composable_publication.py','composable_state.py','incremental_pair_storage.py')}
    report['identity'] = identity(report)
    durable_json(OUT/'REPORT.json',report)
    files = {str(p.relative_to(OUT)):file_hash(p) for p in OUT.rglob('*.json')
        if p.parent in (OUT,OUT/'L9',OUT/'L10',OUT/'L11') and p.name not in ('AUTHORITY.json',)
        and p.stat().st_size < 2_000_000}
    authority = dict(classification=report['classification'],qualification=report['identity'],authoritative_layer=11,
        scientific=deep['scientific'],content=final['content'],reference=final['reference'],checkpoint_sha256=final['checkpoint_sha256'],files=files)
    authority['identity'] = identity(authority)
    durable_json(OUT/'AUTHORITY.json',authority)
    print(dict(classification=report['classification'],qualification=report['identity'],report_sha256=file_hash(OUT/'REPORT.json'),authority_sha256=file_hash(OUT/'AUTHORITY.json')),flush=True)


if __name__ == '__main__':
    run()
