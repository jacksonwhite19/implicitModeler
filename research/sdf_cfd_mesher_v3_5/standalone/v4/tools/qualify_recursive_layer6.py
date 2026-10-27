"""Bounded production L6 driver. Failures stop before subsequent phases."""
from pathlib import Path
import json, sys, time, subprocess, shutil, traceback, sqlite3
from unittest.mock import patch
import psutil
from path_preserving_deduplication import durable_json, file_hash, metadata, physical_key
from clean_sheet_l4_bootstrap import SURFACE, REQUEST, metrics
from composable_state import CommittedState, plan_transition
from qualify_composable_state import CountConnection, SQL
from production_surface_authority import load_production_wall
from geometry_family_authority import Source
from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
from sdf_cfd_mesher_v4.common.authority import identity

V4=Path(__file__).resolve().parents[1]
OUT=V4/'results/recursive_layer6'
AUTH=V4/'results/composable_incremental_state'
HEAD='828045f51ed5af0fa0521a21398b773d314c88a8'
def read(p): return json.loads(p.read_text())
def denied(*a,**kw): raise RuntimeError('LegacyFallbackForbidden')
def measured(): return time.perf_counter(),time.process_time(),psutil.Process().io_counters()
def inputs(base):
    wall,tri,surface=load_production_wall(SURFACE)
    spec=dict(components=[read(SURFACE.parent/'GEOMETRY.json')['definition']])
    return base.initialize(wall,tri,surface,REQUEST,Source(spec)),spec
def run(mode):
    if (OUT/'AUTHORITY.json').exists():raise ValueError('PreserveSealedQualificationOutput')
    OUT.mkdir(exist_ok=True)
    tick=measured()
    try:
        if mode=='preflight':
            assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==HEAD
            assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
            status=subprocess.check_output(['git','status','--porcelain=v1','-z'])
            unrelated=[r for r in status.split(b'\0') if r and b'qualify_recursive_layer6.py' not in r and b'results/recursive_layer6' not in r]
            assert len(unrelated)==155
            protected={}
            for name,h in read(AUTH/'AUTHORITY.json')['files'].items():
                p=AUTH/name; assert file_hash(p)==h;protected[str(p)]=h
            impl=read(AUTH/'IMPLEMENTATION.json')
            for name,h in impl['files'].items():
                p=V4/name;assert file_hash(p)==h;protected[str(p)]=h
            assert impl['identity']=='214f112f0478ee642971874eb6fc6593ca79cfa42c92ec87fa66519c6bd256b0'
            assert read(AUTH/'AUTHORITY.json')['qualification_identity']=='e7388b7bd842201a9cf9bc506c787900f5f20c586bd0755e5cf6f9fb23faf7bb'
            assert file_hash(REQUEST)=='37375760bf77a7df2ff6e34ca0a87b6d437dcee604a5d8bf8ddb3c3483c40d80'
            with patch.object(sqlite3,'connect',denied),CommittedState(read(AUTH/'REFRAMED_REFERENCE.json')) as base:
                adapter,spec=inputs(base);plan=plan_transition(base,adapter,6)
                assert base.content_root=='faa2d3f63f70e518a38d7a51823281abfa2ed1d03032b250f48d133b0b42dd6a'
                assert base.root=='3c8f4dd6dfa255d11e9bc1b7f0c461b339bfa53b3f0bfb42e2fd2319be8ce842'
                assert plan['increment_hex']=='0x1.744eb7e7e7516p-12'
                result=dict(head=HEAD,status_hex=b'\0'.join(unrelated).hex(),protected=protected,plan=plan,binding=adapter.binding,metrics=metrics(*tick),stats=dict(base.stats),native_stats=dict(base.parent.wrapped.stats),free=shutil.disk_usage(V4).free,sql_rows=0,legacy_fallbacks=0)
            durable_json(OUT/'PREFLIGHT.json',result)
        elif mode in ('reopen','reopen_after_retention','deep'):
            from production_forecast_admission import BoundProductionForecastCache
            reference=dict(kind='recursive',directory=str((OUT/'candidate').resolve()),root=read(OUT/'PUBLICATION.json')['root'])
            with patch.object(NativeCheckpointStore,'restart',denied),patch.object(BoundProductionForecastCache,'forecast',denied),CommittedState(reference) as base:
                assert base.layer==6
                if mode.startswith('reopen'):
                    with patch.object(sqlite3,'connect',denied):
                        adapter,spec=inputs(base);plan=plan_transition(base,adapter,7)
                        assert plan['increment_hex']=='0x1.bd6749eb1b62ap-12'
                    result=dict(passed=True,reference=reference,plan=plan,content=base.content_root,metrics=metrics(*tick),stats=dict(base.stats),parent_stats=dict(base.parent.stats),sql_rows=0,proof_rows=0,quality_rows=0,L7_admission=False,L7_generation=False,core=False)
                else:
                    scientific=base.deep_verify()
                    result=dict(passed=True,reference=reference,scientific=scientific,content=base.content_root,metrics=metrics(*tick),stats=dict(base.stats),parent_stats=dict(base.parent.stats),normal_path=False)
            durable_json(OUT/(mode.upper()+'.json'),result)
        else:
            import composable_admission,composable_transition,composable_assembly,composable_publication
            connect=sqlite3.connect
            def counted(*a,**kw): kw['factory']=CountConnection;return connect(*a,**kw)
            with patch.object(NativeCheckpointStore,'restart',denied),patch.object(sqlite3,'connect',counted),CommittedState(read(AUTH/'REFRAMED_REFERENCE.json')) as base:
                adapter,spec=inputs(base)
                assert plan_transition(base,adapter)['target_layer']==6
                if mode=='admission': composable_admission.admit(base,adapter,spec,OUT)
                elif mode=='generation': composable_transition.produce(base,adapter,spec,OUT)
                elif mode=='assembly': composable_assembly.assemble(base,adapter,OUT)
                elif mode=='publication': composable_publication.publish(base,adapter,OUT)
                else: raise ValueError(mode)
                result=dict(passed=True,metrics=metrics(*tick),sql=dict(SQL),stats=dict(base.stats),legacy_fallbacks=0)
            durable_json(OUT/(mode.upper()+'_METRICS.json'),result)
        print(json.dumps(result,indent=2),flush=True)
    except BaseException as exc:
        durable_json(OUT/'FAILURE.json',dict(phase=mode,error=repr(exc),traceback=traceback.format_exc(),metrics=metrics(*tick),sql=dict(SQL),authoritative_layer=5))
        raise
if __name__=='__main__': run(sys.argv[1])
