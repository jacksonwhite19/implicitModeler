"""Separate qualification work and successful-only duplicate retention."""
import os
import sqlite3
import shutil
from pathlib import Path
from unittest.mock import patch
from datetime import datetime, timezone
from qualify_recursive_layers7to8 import (read, parent, inputs, measured, metrics, denied,
    CommittedState, plan_transition, PairIncidence, durable_json, file_hash, metadata, physical_key)
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import scientific_digest, NativeCheckpointStore
from sdf_cfd_mesher_v4.common.authority import identity
import composable_transition
import composable_assembly
import composable_publication


def inventory(out):
    rows = [metadata(p) for p in out.rglob('*') if p.is_file()]
    unique = {physical_key(r): r for r in rows}
    return dict(files=rows, physical=sum(r['cluster_bytes'] for r in unique.values()),
        logical=sum(r['bytes'] for r in rows), free=shutil.disk_usage(out).free)


def reopen(out, layer):
    from production_forecast_admission import BoundProductionForecastCache
    tick = measured()
    with patch.object(sqlite3, 'connect', denied), patch.object(NativeCheckpointStore, 'restart', denied), \
            patch.object(BoundProductionForecastCache, 'forecast', denied), CommittedState(read(out/'REFERENCE.json')) as state:
        adapter, _ = inputs(state)
        plan = plan_transition(state, adapter, layer+1)
        assert state.layer == layer
        assert plan['increment'] == {7: .0005081658834681988, 8: .0006079352484572211}[layer]
        with PairIncidence(state, out/'INCIDENCE_MANIFEST.json', state.body['plan']['pair_incidence_root']) as index:
            assert index.body['parent_count'] == len(state['certificate'])
        result = dict(passed=True, plan=plan, reference=state.reference, content=state.content_root,
            stats=dict(state.stats), metrics=metrics(*tick), proof_rows=0, quality_rows=0, sql_rows=0,
            full_prefix_hashes=0, deep_copies=0, admission=False, forecast=False, generation=False)
    return result


def verify(mode, layer, out):
    tick = measured()
    if mode == 'diagnostics':
        from recursive_production_diagnostics import run
        run(out, layer)
        return
    elif mode == 'reopen':
        result = reopen(out, layer)
    elif mode == 'deep':
        import verify_optimized_l6_control as deep
        with CommittedState(read(out/'REFERENCE.json')) as state:
            refs = {}; ancestor = state; depth = 0
            while ancestor is not None:
                depth += 1
                if ancestor.kind == 'native':
                    refs.update(ancestor.wrapped.body['files'])
                elif ancestor.kind == 'clean_sheet':
                    refs.update(ancestor.wrapped.plan['references']); refs.update(ancestor.wrapped.body['produced'])
                else:
                    refs.update(ancestor.body['plan']['references']); refs.update(ancestor.body['produced'])
                ancestor = getattr(ancestor, 'parent', None)
            total = 0
            for path, record in refs.items():
                assert file_hash(path) == record['sha256'], path
                total += Path(path).stat().st_size
            with patch.object(deep, 'OUT', out):
                incidence = deep.incidence_deep(state)
            for row in state.rows.values():
                row.deep = True
            try:
                scientific = scientific_digest(state)
            finally:
                for row in state.rows.values():
                    row.deep = False
            result = dict(passed=True, scientific=scientific, content=state.content_root,
                incidence=incidence, files_authenticated=len(refs), bytes_authenticated=total,
                depth=depth, stats=dict(state.stats), metrics=metrics(*tick), normal_path=False)
    elif mode == 'replay':
        replay = out/'replay'
        replay.mkdir(exist_ok=False)
        for name in ('ADMISSION.json','DIRECT.json','LOCALIZATION.json','SCREEN_ARRAYS.npz'):
            shutil.copyfile(out/name, replay/name)
        reference, path, root = parent(layer)
        with patch.object(os, 'link', denied), patch.object(NativeCheckpointStore, 'restart', denied), CommittedState(reference) as base:
            adapter, spec = inputs(base)
            with PairIncidence(base, path, root) as index:
                composable_transition.produce(base, adapter, spec, replay)
                composable_assembly.assemble(base, adapter, replay, index)
                plan = composable_publication.prepare(base, adapter, replay)
                expected = read(out/'CANDIDATE_PLAN.json')
                comparisons = {k: v == expected['components'][k] for k,v in plan['components'].items()}
                assert all(comparisons.values()) and plan['scientific'] == expected['scientific']
                result = dict(passed=True, comparisons=comparisons, content=plan['scientific'],
                    metrics=metrics(*tick), stats=dict(base.stats), admission='Authenticated decision reused; no new forecast',
                    published=False, hardlinks_created=0, legacy_fallbacks=0)
    elif mode == 'safety':
        from incremental_transaction import writer
        reference, _, _ = parent(layer)
        before = read(out/'candidate/CURRENT.json')
        plan = read(out/'CANDIDATE_PLAN.json'); acceptance = read(out/'CANDIDATE_ACCEPTANCE.json')
        with CommittedState(reference) as base:
            try:
                composable_publication.publish_prepared(base, out, 'wrong', identity(acceptance))
            except ValueError as exc:
                assert str(exc) == 'PreparedTransactionIdentityMismatch'
            else:
                raise AssertionError('WrongPlanNotRejected')
            with writer(out/'candidate'):
                try:
                    composable_publication.publish_prepared(base, out, identity(plan), identity(acceptance))
                except (ValueError, OSError) as exc:
                    assert str(exc) == 'ConcurrentIncrementalWriter'
                    concurrent = repr(exc)
                else:
                    raise AssertionError('ConcurrentWriterNotRejected')
            root, stats = composable_publication.publish_prepared(base, out, identity(plan), identity(acceptance))
            assert root == before['root'] and read(out/'candidate/CURRENT.json') == before
            assert stats['idempotent_retries'] == 1
        result = dict(passed=True, wrong_plan_rejected=True, concurrent_writer_rejected=concurrent,
            idempotent_root=root, stats=stats, pointer_unchanged=True, metrics=metrics(*tick),
            interruption_recovery='Separate regression suite exercises all publication boundaries')
    elif mode == 'retain':
        assert read(out/'DEEP.json')['passed'] and read(out/'REPLAY.json')['passed']
        reopen(out, layer)
        from retain_recursive_layer6 import check_duplicate, recover
        if (out/'RETAIN.json').exists():
            assert read(out/'RETAIN.json')['passed']
            return
        before = inventory(out); replay = (out/'replay').resolve()
        bound = set(read(out/'CANDIDATE_PLAN.json')['references']); rows = []
        if (out/'CLEANUP_MANIFEST.json').exists():
            previous = read(out/'CLEANUP_MANIFEST.json')
            restored = recover(previous['rows'], replay, bound)
            durable_json(out/'CLEANUP_RECOVERY.json', dict(previous=previous, restored=restored))
            before = inventory(out)
        for path in sorted(replay.rglob('*')):
            if not path.is_file():
                continue
            path = path.resolve(); canonical = (out/path.relative_to(replay)).resolve()
            if not canonical.is_file():
                continue
            digest = file_hash(path)
            if file_hash(canonical) != digest:
                continue
            check_duplicate(path, canonical, replay, bound, digest)
            record = metadata(path)
            assert record['links'] == 1
            rows.append(dict(path=str(path), canonical=str(canonical), sha256=digest, metadata=record))
        durable_json(out/'CLEANUP_MANIFEST.json', dict(rows=rows, policy='Exact unpublished replay duplicates only'))
        removed = []
        try:
            for row in rows:
                check_duplicate(row['path'], row['canonical'], replay, bound, row['sha256'])
                Path(row['path']).unlink(); removed.append(row)
            after_reopen = reopen(out, layer)
        except BaseException:
            recover(removed, replay, bound)
            raise
        result = dict(passed=True, before=before, after=inventory(out), removed_paths=len(removed),
            physical_reclaimed=sum(r['metadata']['cluster_bytes'] for r in removed),
            logical_removed=sum(r['metadata']['bytes'] for r in removed),
            post_cleanup_reopen=after_reopen, metrics=metrics(*tick), checkpoint_retirement=False)
    elif mode == 'seal':
        for name in ('REOPEN','DEEP','REPLAY','RETAIN','SAFETY'):
            assert read(out/(name+'.json'))['passed']
        admission = read(out/'ADMISSION.json')
        generation = read(out/'GENERATION.json')
        certificate = read(out/'FRESH_CERTIFICATE.json')
        volume = read(out/'VOLUME_DELTA.json')
        diagnostic = read(out/'DIAGNOSTICS.json')
        assert admission['decision']['generation_attempt_authorized']
        assert not generation['hard_failures'] and generation['topology_audit']['passed']
        assert certificate['passed'] and not certificate['failures']
        assert volume['exact_balance'] and volume['global_degree']['passed']
        assert not volume['global_degree']['unresolved']
        assert all(x['failures'] == 0 for x in diagnostic['quality'].values())
        assert read(out/'ASSEMBLY.json')['pair_accounting']['prior_pair_rows_scanned'] == 0
        assembly = read(out/'ASSEMBLY.json')
        assert assembly['prior_proof_payloads_read'] == assembly['prior_proof_payloads_recomputed'] == 0
        writer = read(out/'INDEX_MEASURED.json')['writer']
        assert writer['final']['journal_mode'] == 'delete' and writer['final']['synchronous'] == 2
        assert writer['rows'] == certificate['required_pairs']
        assert read(out/'REPLAY.json')['content'] == read(out/'DEEP.json')['content']
        reopened = reopen(out, layer)
        names = ('ADMISSION','GENERATION','ASSEMBLY','PUBLICATION','REFERENCE','DEEP','REPLAY','RETAIN','DIAGNOSTICS')
        files = {n+'.json':file_hash(out/(n+'.json')) for n in names}
        plan = read(out/'CANDIDATE_PLAN.json')
        result = dict(passed=True, layer=layer, scientific=read(out/'DEEP.json')['scientific'],
            content=plan['scientific'], reference=read(out/'REFERENCE.json'),
            incidence_root=plan['pair_incidence_root'], files=files,
            sealed_at=datetime.now(timezone.utc).isoformat(), metrics=metrics(*tick),
            post_retention_reopen=reopened)
        result['checkpoint_sha256'] = file_hash(out/'candidate'/('ROOT_'+result['reference']['root']+'.json'))
        compact = dict(schema='V4RecursiveSuccessfulLayerCertificate/1', layer=layer,
            scientific=result['scientific'], content=result['content'], reference=result['reference'],
            components=plan['components'], admission=admission['decision'], quality=diagnostic['quality'],
            geometry=generation, continuous=certificate, assembly=read(out/'ASSEMBLY.json'),
            checkpoint_sha256=result['checkpoint_sha256'], discarded_objects=file_hash(out/'CLEANUP_MANIFEST.json'),
            reproduction='Use the sealed parent and exact schedule with composable admission/transition, incidence assembly and optimized publication. Independently replay changed state and compare component roots.',
            retention_policy='V4RollingSuccessfulQualificationRetention/1')
        from qualify_recursive_layers7to8 import V4
        compact['implementation'] = {p.name:file_hash(p) for p in (V4/'tools').glob('recursive_production_*.py')}
        compact['implementation']['qualify_recursive_layers7to8.py'] = file_hash(V4/'tools/qualify_recursive_layers7to8.py')
        compact['identity'] = identity(compact)
        durable_json(out/'COMPACT_CERTIFICATE.json', compact)
        result['compact_certificate_sha256'] = file_hash(out/'COMPACT_CERTIFICATE.json')
        result['identity'] = identity(result)
        durable_json(out/'AUTHORITY.json', result)
        durable_json(out.parent/'PRODUCTION.json', result)
    else:
        raise ValueError(mode)
    durable_json(out/(mode.upper()+'.json'), result)
    print({k:v for k,v in result.items() if k not in ('before','after','post_cleanup_reopen','post_retention_reopen')}, flush=True)
