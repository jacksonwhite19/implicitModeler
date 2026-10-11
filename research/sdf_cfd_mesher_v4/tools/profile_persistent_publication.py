"""Private L10 to L11 controls; no production pointer is replaced."""
from pathlib import Path
import sys
import profile_meshing_hotpath as previous
import qualify_recursive_layers9to11 as production
import recursive_epoch_recovery as recovery
import capsule_user_domain_handoff as h
from contextlib import contextmanager


@contextmanager
def memory_sample(folder,stage):
    import threading,psutil,time
    p=psutil.Process();stop=threading.Event();rows=[]
    def sample():
        children=[];next_children=0.
        while not stop.is_set():
            try:
                now=time.perf_counter()
                if now>=next_children:
                    children=p.children(recursive=True);next_children=now+1.
                parent=p.memory_info().rss
                memory=[c.memory_info().rss for c in children if c.is_running()]
                rows.append((now,parent,sum(memory),len(memory)))
            except psutil.Error:pass
            stop.wait(.1)
    thread=threading.Thread(target=sample,daemon=True);thread.start()
    try:yield
    finally:
        stop.set();thread.join()
        h.durable_json(folder/(stage+'_MEMORY.json'),dict(interval_seconds=.1,child_enumeration_interval_seconds=1.,samples=len(rows),
            peak_aggregate=max((r[1]+r[2] for r in rows),default=0),peak_parent=max((r[1] for r in rows),default=0),
            peak_children=max((r[2] for r in rows),default=0),max_children=max((r[3] for r in rows),default=0)))

PRIOR=Path(__file__).resolve().parents[1]/'results/prism_assembly_optimization'
OUT=PRIOR.parent/'persistent_publication_optimization'
OUT.mkdir(exist_ok=True)


def run(mode,stage):
    old_parent=production.parent
    def parent(layer):
        ref,path,root=old_parent(layer)
        recovered=recovery.discover(PRIOR/'epoch_access')
        node=next(n for n in recovered['nodes'] if n['source']==ref)
        renewed=h.read(PRIOR/'INCIDENCE_ACCESS.json')
        return node['reference'],Path(renewed['path']),renewed['root']
    production.parent=parent
    previous.OUT=OUT
    if mode.endswith('v2_optimized'):
        from prism_hotpath_v2 import install
        install()
        import os
        if os.environ.get('V4_PERSISTENT')=='1':
            from prism_persistent_views import install as persistent
            from prism_sealed_file_cache import install as sealed
            persistent();sealed()
            from prism_receipt_witness import install as receipt
            receipt()
    sources={p.name:h.file_hash(p) for p in Path(__file__).parent.glob('*.py') if p.name in ('prism_hotpath_v2.py','prism_proof_index.py','prism_sql_profile.py','prism_degree.py')}
    folder=OUT/(mode+'_prism');folder.mkdir(exist_ok=True)
    with memory_sample(folder,stage):
        if stage=='publication' and not mode.endswith('v2_optimized'):
            from prism_sql_profile import measured_quality_writer
            info=h.read(folder/'ASSEMBLY.json')
            with measured_quality_writer(folder,info['quality_new']+info['old_quality_rows_examined'],adaptive=False):
                previous.prism(mode,stage)
        else:previous.prism(mode,stage)
    if sources!={p.name:h.file_hash(p) for p in Path(__file__).parent.glob('*.py') if p.name in sources}:
        raise ValueError('ChangedControlImplementation')
    h.durable_json(OUT/(mode+'_prism')/(stage+'_V2_IMPLEMENTATION.json'),sources)


def incidence_access():
    from contextlib import ExitStack
    import immutable_state_access as access
    from composable_state import CommittedState
    from incremental_pair_storage import PairIncidence
    ref,path,root=production.parent(11)
    recovered=recovery.discover(PRIOR/'epoch_access')
    node=next(n for n in recovered['nodes'] if n['source']==ref)
    tick=h.measured()
    with ExitStack() as held,CommittedState(node['reference']) as base:
        held.enter_context(access.guard(path));old=access.read(path)
        if h.identity(old)!=root:raise ValueError('WrongSealedIncidenceRoot')
        records=recovery.refresh_records(old['files'],held,{})
        body=dict(old,files=records,epoch=access.epoch())
        target=OUT/'CURRENT_EPOCH_INCIDENCE.json'
        if target.exists():raise ValueError('PreserveIncidenceAccess')
        h.durable_json(target,body)
        with PairIncidence(base,target,h.identity(body)) as index:
            assert index.body['parent_count']==old['parent_count']
        h.durable_json(PRIOR/'INCIDENCE_ACCESS.json',dict(path=str(target.resolve()),root=h.identity(body),
            source_root=root,source=str(path),metrics=h.metrics(*tick),scientific_segments_unchanged=body['segments']==old['segments']))


@contextmanager
def stage_profile(folder, stage):
    import cProfile, os, pstats
    if os.environ.get('V4_PROFILE') != '1':
        yield
        return
    profile=cProfile.Profile();profile.enable()
    try:yield
    finally:
        profile.disable();profile.dump_stats(str(folder/(stage+'.prof')))
        with (folder/(stage+'_PROFILE.txt')).open('w') as stream:
            pstats.Stats(profile,stream=stream).strip_dirs().sort_stats('cumulative').print_stats(80)


def normal(mode):
    """Equivalent baseline/candidate campaigns, one guarded parent per layer."""
    import os
    from unittest.mock import patch
    from optimized_admission import install as previous_install
    from incremental_pair_storage import PairIncidence
    if mode not in ('baseline','baseline_clean','candidate','candidate_replay','candidate_confirm','candidate_final','candidate_final_replay','candidate_numeric','candidate_persistent','candidate_persistent_replay','candidate_clean','candidate_clean_replay','candidate_bound','candidate_bound_replay','candidate_lookup','candidate_lookup_replay'):raise ValueError('UnknownControlMode')
    previous_install()
    if os.environ.get('V4_NUMERIC')=='1':
        from prism_persistent_hotpath import install
        install()
    elif mode.startswith('candidate'):
        from prism_hotpath_v2 import install
        install()
        if os.environ.get('V4_PERSISTENT')=='1':
            from prism_persistent_views import install as persistent
            from prism_sealed_file_cache import install as sealed
            from prism_receipt_witness import install as receipt
            persistent();sealed();receipt()
    from composable_admission import admit
    from composable_transition import produce
    from composable_assembly import assemble
    from recursive_production_publication import publish
    from prism_sql_profile import measured_quality_writer
    folder=OUT/('normal_'+mode+'_prism');folder.mkdir(exist_ok=False)
    names=('prism_sql_profile.py',) if mode.startswith('baseline') else ('prism_hotpath_v2.py','prism_proof_index.py','prism_sql_profile.py','prism_degree.py','prism_persistent_views.py','prism_sealed_file_cache.py','prism_encoded_transaction.py','prism_receipt_witness.py','persistent_prism_transition.py','persistent_numeric_incidence.py','prism_persistent_hotpath.py','prism_source_lookup.py')
    sources={p.name:h.file_hash(p) for p in Path(__file__).parent.glob('*.py') if p.name in names}
    total=h.measured();tick=h.measured()
    old,_,_=production.parent(11);recovered=recovery.discover(PRIOR/'epoch_access')
    reference=next(n['reference'] for n in recovered['nodes'] if n['source']==old)
    incidence=h.read(PRIOR/'INCIDENCE_ACCESS.json');records={}
    with patch.object(os,'link',production.denied),patch.object(production.NativeCheckpointStore,'restart',production.denied),production.CommittedState(reference) as base:
        if os.environ.get('V4_NUMERIC')=='1':
            authority=h.read(OUT/'PARENT_INCIDENCE_AUTHORITY.json')
            version=h.read(OUT/'PARENT_VERSION_AUDIT.json')
            from prism_persistent_hotpath import bind_bootstrap
            lookup=h.read(OUT/'PARENT_LOOKUP_AUTHORITY.json')['reference'] if os.environ.get('V4_LOOKUP')=='1' else None
            bind_bootstrap(base,OUT/'parent_incidence',authority['root'],OUT/'PARENT_VERSION_CLOSURE.json',version['root'],lookup)
        adapter,spec=production.inputs(base)
        from contextlib import nullcontext
        from prism_persistent_hotpath import geometry_session
        session=geometry_session(base,adapter) if os.environ.get('V4_LOOKUP')=='1' else nullcontext()
        with session as numeric,PairIncidence(base,incidence['path'],incidence['root']) as index:
            records['parent_open']=h.metrics(*tick)
            for stage in ('admission','generation','assembly','publication'):
                tick=h.measured();before=dict(base.stats)
                with memory_sample(folder,stage), stage_profile(folder,stage):
                    if stage=='admission':admit(base,adapter,spec,folder)
                    elif stage=='generation':produce(base,adapter,spec,folder)
                    elif stage=='assembly':assemble(base,adapter,folder,pair_index=index)
                    elif mode.startswith('baseline'):
                        info=h.read(folder/'ASSEMBLY.json')
                        with measured_quality_writer(folder,info['quality_new']+info['old_quality_rows_examined'],False):publish(base,adapter,folder,index)
                    else:publish(base,adapter,folder,index)
                records[stage]=h.metrics(*tick)
                from optimized_global_degree import PERFORMANCE
                h.durable_json(folder/(stage+'_MEASURED.json'),dict(metrics=records[stage],stats={k:v-before.get(k,0) for k,v in base.stats.items()},incidence=dict(index.stats),
                    parallel_degree=dict(PERFORMANCE) if stage=='generation' else {},numeric_lookup=None if numeric is None else dict(queries=numeric.lookup.queries,hits=numeric.lookup.hits)))
            base.check()
    result=dict(phases=records,metrics=h.metrics(*total),admission_to_publication=sum(records[s]['seconds'] for s in ('admission','generation','assembly','publication')),
        implementation=sources,scope='One normal guarded parent session; profiling disabled; separate fresh reopen')
    if sources!={p.name:h.file_hash(p) for p in Path(__file__).parent.glob('*.py') if p.name in sources}:raise ValueError('ChangedControlImplementation')
    h.durable_json(folder/'NORMAL.json',result)


def reopen(mode):
    from composable_state import CommittedState
    folder=OUT/('normal_'+mode+'_prism');reference=h.read(folder/'REFERENCE.json');tick=h.measured()
    with h.guards(),CommittedState(reference) as state:
        state.check()
        assert state.layer==11 and state.content_root=='6b11b14c7b519467ad49a126261f0ddd608692d82ca387160fcc1b7ab10c5870'
        result=dict(passed=True,layer=state.layer,content=state.content_root,access=state.root,metrics=h.metrics(*tick),stats=dict(state.stats),historical_proof_rows=0,historical_quality_rows=0)
    h.durable_json(folder/'REOPEN.json',result)


def degree_scaling(candidate=False):
    import numpy as np
    from optimized_global_degree import degree,PERFORMANCE
    if candidate:
        from prism_degree import degree,PERFORMANCE
    from prism_hotpath_v2 import exact_centroid
    s=OUT.parent/'capsule_c1_core_qualification/assembly'
    x=np.load(s/'coordinates.npy')[:202740];p=np.load(s/'prisms.npy')
    faces=[tuple(reversed(row[:3])) for row in p[:33786]]+[tuple(row[3:]) for row in p[-33786:]]
    samples=[exact_centroid(x,dict(vertices=row)) for row in p]
    expected=h.read(OUT.parent/'recursive_layers9to11/L11/VOLUME_DELTA.json')['global_degree'];records=[]
    for workers in ((1,2,4,8) if candidate else (8,12,16)):
        tick=h.measured()
        with memory_sample(OUT,('integer_degree_' if candidate else 'degree_')+str(workers)):result=degree(x,faces,samples,workers=workers)
        assert result==expected
        records.append(dict(workers=workers,metrics=h.metrics(*tick),parallel=dict(PERFORMANCE),exact_equal=True,identity=h.identity(result)))
        h.durable_json(OUT/('INTEGER_DEGREE_SCALING.json' if candidate else 'DEGREE_SCALING.json'),records)
        print(workers,records[-1]['metrics']['seconds'],flush=True)


if __name__=='__main__':
    if sys.argv[1]=='tests':
        import incremental_transaction
        from prism_encoded_transaction import Store
        incremental_transaction.Store=Store
        import pytest
        from prism_hotpath_v2 import install
        install()
        import os
        if os.environ.get('V4_PERSISTENT')=='1':
            from prism_persistent_views import install as persistent
            from prism_sealed_file_cache import install as sealed
            persistent();sealed()
            from prism_receipt_witness import install as receipt
            receipt()
        raise SystemExit(pytest.main(sys.argv[2:]))
    elif sys.argv[1]=='incidence_access':incidence_access()
    elif sys.argv[1]=='normal':normal(sys.argv[2])
    elif sys.argv[1]=='reopen':reopen(sys.argv[2])
    elif sys.argv[1]=='degree':degree_scaling()
    elif sys.argv[1]=='degree_v2':degree_scaling(True)
    else:run(*sys.argv[1:])
