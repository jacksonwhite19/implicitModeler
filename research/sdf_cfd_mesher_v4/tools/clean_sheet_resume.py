"""Resume the L4-only experiment; target oracle remains closed in every phase.

The admission delegate is the unchanged qualified production procedure. Only
its base opener and output directory are supplied by this isolated harness.
"""
from pathlib import Path
import json, os, sys, time
from clean_sheet_isolation import ReadBoundary, deny_native_reads

V4 = Path(__file__).resolve().parents[1]
REPO = V4.parents[1]
PREP = V4 / 'results/clean_sheet_incremental_l4_to_l5'
OUT = PREP / 'resume'


def read(p):
    return json.loads(Path(p).read_text())


def require_l5_scope(adapter):
    if adapter.binding['layer'] != 4 or adapter.binding['next_layer'] != 5:
        raise ValueError('CleanSheetRequiresL4ToL5')
    if float(adapter.remaining_increments[0]).hex() != '0x1.37351a26971d3p-12':
        raise ValueError('WrongL5Increment')


class ResumeBoundary(ReadBoundary):
    def hook(self, event, args):
        if event == 'ctypes.dlopen' and args[0]:
            p = Path(args[0])
            # System DLL names are resolved by Windows, not scientific files.
            if p.is_absolute() or len(p.parts) > 1:
                self.check(p, False, event)
        super().hook(event, args)


def run(mode):
    if mode not in ('admit', 'generate', 'assemble', 'publish', 'seal'):
        raise ValueError('UnknownCleanSheetResumePhase')
    OUT.mkdir(parents=True, exist_ok=True)
    if mode == 'admit' and (OUT / 'ADMISSION.json').exists():
        raise ValueError('PreserveSealedAdmission')
    if mode == 'generate' and (OUT / 'GENERATION.json').exists():
        raise ValueError('PreserveIndependentGeneration')
    baseline = read(PREP / 'BASELINE.json')
    refs = dict(baseline['allowed_inputs'])
    refs.update(read(PREP / 'REQUEST_INPUT_CLOSURE.json')['allowed_inputs'])
    # Implementation and its existing native accelerator, not target evidence.
    import hashlib
    native = V4 / 'src/sdf_cfd_mesher_v4/certificate/efficient_native.rs'
    digest = hashlib.sha256(native.read_bytes()).hexdigest()
    binary = V4 / 'results/task4b/native' / ('proof_' + digest + '.dll')
    if not binary.is_file():
        raise ValueError('QualifiedNativeAcceleratorMissing')
    refs[str(native.resolve())] = digest
    refs[str(binary.resolve())] = hashlib.sha256(binary.read_bytes()).hexdigest()
    authority = V4 / 'src/sdf_cfd_mesher_v4/certificate/continuous_prism_certificate/MATHEMATICAL_AUTHORITY.json'
    refs[str(authority.resolve())] = hashlib.sha256(authority.read_bytes()).hexdigest()
    authority = authority.with_name('ROOT_REPRESENTATION_AUTHORITY.json')
    refs[str(authority.resolve())] = hashlib.sha256(authority.read_bytes()).hexdigest()
    runtime = [sys.prefix, sys.base_prefix] + [p for p in sys.path if p and not Path(p).resolve().is_relative_to(REPO)]
    boundary = ResumeBoundary(REPO, refs, PREP, runtime)
    boundary.install()
    import atexit
    def exit_log():
        # Includes failures during implementation imports, before native locks.
        with (OUT / ('IMPORT_AND_PHASE_READ_LOG_' + str(os.getpid()) + '.json')).open('w', encoding='utf8') as f:
            json.dump(boundary.record(), f, sort_keys=True, indent=2)
    atexit.register(exit_log)
    from contextlib import ExitStack
    from unittest.mock import patch
    import psutil
    import sqlite3
    import immutable_state_access as access
    import path_preserving_deduplication as fs
    import clean_sheet_l4_bootstrap as bootstrap
    import production_single_layer_cycle as cycle
    from production_forecast_admission import ProductionForecastAdapter
    from production_surface_authority import load_production_wall
    from geometry_family_authority import Source
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
    from sdf_cfd_mesher_v4.common.authority import identity
    original_api = fs.api

    class Api:
        def __init__(self): self.k = original_api()
        def __getattr__(self, key): return getattr(self.k, key)
        def CreateFileW(self, path, desired, *args):
            boundary.check(path, bool(desired & 0x40000000), 'Win32.CreateFileW')
            return self.k.CreateFileW(path, desired, *args)

    def deny(*args, **kwargs):
        raise AssertionError('LegacyRestartOrHistoricalRowsForbiddenDuringAdmission')

    start = time.perf_counter(); cpu = time.process_time(); io = psutil.Process().io_counters()
    ids = {(Path(p).stat().st_dev, Path(p).stat().st_ino) for p in baseline['allowed_inputs']}
    paths = [r['path'] for r in read(PREP / 'ORACLE_LOCKS.json')['entries']]
    enrolled = read(PREP / 'L4_ENROLLMENT.json')
    try:
        with deny_native_reads(paths, ids) as locks, ExitStack() as held:
            fs.api = Api; access.api = Api
            state = held.enter_context(access.ImmutableState(PREP / 'L4_access', enrolled['root']))
            wall, tri, surface = load_production_wall(bootstrap.SURFACE)
            spec = dict(components=[read(bootstrap.SURFACE.parent / 'GEOMETRY.json')['definition']])
            adapter = ProductionForecastAdapter.from_immutable_access(state, wall, tri, surface, bootstrap.REQUEST, Source(spec), state.body['expected'])
            require_l5_scope(adapter)
            opened = time.perf_counter() - start
            fs.durable_json(OUT / (mode.upper() + '_INPUTS.json'), dict(
                L4=bootstrap.L4, access_root=enrolled['root'], content_root=enrolled['content_root'],
                binding=adapter.binding, implementation_inputs={str(native): digest, str(binary): refs[str(binary.resolve())]},
                forecast_scope='L5-L11 provisional admission only', oracle_unlock=False,
                oracle_lock_count=len(locks), trusted_open_seconds=opened, stats=dict(state.stats)))
            cycle.OUT = OUT
            cycle.INCREMENT_HEX = float(adapter.remaining_increments[0]).hex()
            with patch.object(cycle, 'prepare_base', lambda: None), \
                 patch.object(cycle, 'restore_base', lambda: (state, adapter, spec, opened)), \
                 patch.object(NativeCheckpointStore, 'restart', deny):
                if mode == 'admit':
                    with patch.object(sqlite3, 'connect', deny):cycle.admit()
                elif mode == 'generate':
                    import shutil
                    free = shutil.disk_usage(REPO.anchor).free
                    # The prior task measured L5 active growth below this value;
                    # no invented safety reserve is used here.
                    from clean_sheet_transition import produce
                    fs.durable_json(OUT/'GENERATION_DISK_PREFLIGHT.json', dict(free=free,
                        measurement_scope='Before independent generation; after admission',
                        retained_admission_bytes=sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file()),
                        no_historical_deletion=True))
                    with patch.object(sqlite3, 'connect', deny):produce(state, adapter, spec, OUT)
                elif mode == 'assemble':
                    from clean_sheet_assembly import assemble
                    assemble(state, adapter, OUT)
                elif mode == 'publish':
                    from clean_sheet_candidate import publish
                    publish(state, adapter, OUT)
                else:
                    from clean_sheet_candidate import Candidate
                    from datetime import datetime, timezone
                    publication=read(OUT/'PUBLICATION.json')
                    with Candidate(OUT/'candidate',publication['root']) as candidate:
                        tick=time.perf_counter()
                        with patch.object(sqlite3,'connect',deny):
                            next_adapter=candidate.initialize(wall,tri,surface,bootstrap.REQUEST,Source(spec))
                        if next_adapter.binding['layer']!=5 or next_adapter.binding['next_layer']!=6 or float(next_adapter.remaining_increments[0])!=0.00035506009452908446:
                            raise ValueError('NextIncrementBindingMismatch')
                        fs.durable_json(OUT/'TRUSTED_REOPEN.json',dict(root=publication['root'],seconds=time.perf_counter()-tick,
                            binding=next_adapter.binding,increment=float(next_adapter.remaining_increments[0]),stats=dict(candidate.stats),
                            proof_rows=0,quality_rows=0,forecast_invoked=False))
                        tick=time.perf_counter();scientific=candidate.deep_seal()
                        fs.durable_json(OUT/'CANDIDATE_SEAL.json',dict(root=publication['root'],content_root=publication['content_root'],
                            legacy_scientific=scientific,completed_utc=datetime.now(timezone.utc).isoformat(),
                            deep_seconds=time.perf_counter()-tick,deep_stats=dict(candidate.stats),oracle_unlocked=False,
                            files={p:r['sha256'] for p,r in candidate.plan['references'].items() if Path(p).is_relative_to(OUT)},
                            publication=fs.file_hash(OUT/'PUBLICATION.json')))
            state.check()
            result = dict(passed=True, phase=mode, access_stats=dict(state.stats),
                historical_receipts_read=(read(OUT/'CANDIDATE_SEAL.json')['deep_stats']['deep_old_certificate'] if mode=='seal' else 0),
                historical_quality_rows_read=(read(OUT/'ASSEMBLY.json')['old_quality_rows_examined'] if mode=='assemble' else
                    read(OUT/'CANDIDATE_SEAL.json')['deep_stats']['deep_old_quality'] if mode=='seal' else 0),
                full_prefix_scientific_hashes=int(mode=='seal'),
                oracle_unlocked=False, independent_L6_admission=False, committed_L6=False, core=False,
                **bootstrap.metrics(start, cpu, io))
            fs.durable_json(OUT / (mode.upper() + '_METRICS.json'), result)
    except BaseException as exc:
        fs.durable_json(OUT / ('RESUME_FAILURE_' + str(os.getpid()) + '.json'), dict(error=type(exc).__name__ + ':' + str(exc),
            oracle_unlocked=False, **bootstrap.metrics(start, cpu, io)))
        raise
    finally:
        fs.api = original_api; access.api = original_api
        record = boundary.record()
        forbidden = {os.path.normcase(str(Path(p).resolve())) for p in paths}
        record['forbidden_target_read_count'] = sum(r['count'] for r in record['reads'] if os.path.normcase(r['path']) in forbidden)
        record['identity'] = identity(record)
        fs.durable_json(OUT / (mode.upper() + '_READ_LOG.json'), record)


if __name__ == '__main__':
    run(sys.argv[1])
