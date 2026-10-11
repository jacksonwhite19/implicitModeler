"""Bounded unchanged-production high-water reproduction on sealed SF2 state."""
import json,time,gc,sys
from pathlib import Path
from native_sf2_artifacts import materialize
from native_memory_observer import Observer
from sdf_cfd_mesher_v4.common.authority import file_hash

OUT=Path(__file__).resolve().parents[1]/'results/native_runner_memory_profile'
def run():
    with materialize('fresh') as source:
        historical=[]
        for p in sorted(source.glob('*TIMINGS.json')):
            row=json.loads(p.read_text())
            historical.append(dict(file=p.name,peak_rss=row.get('peak_rss'),seconds=row.get('total_seconds')))
        (OUT/'HISTORICAL_PHASE_PEAKS.json').write_text(json.dumps(historical,indent=2)+'\n')
        mode=sys.argv[1] if len(sys.argv)>1 else 'baseline'
        observer=Observer(OUT/mode);observer.install()
        if mode.startswith('optimized'):
            from sdf_cfd_mesher_v4.certificate import production
            observer.wrap(production,'prove','certificate_working_sets',True)
        try:
            from native_prism_production import Provider
            from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import NativeCheckpointStore
            from sdf_cfd_mesher_v4.certificate.production import certify_immediate
            observer.mark('authority_load');provider=Provider(source)
            store=NativeCheckpointStore(source/'checkpoints');state=store.restart()
            context=provider.context(state);observer.layer=20
            if mode.endswith('warnings'):
                from task4_warning_cache import initialize_warnings
                observer.layer=1;observer.mark('before_warning_initialization')
                initialize_warnings(context);observer.mark('after_warning_initialization')
                del context,state,provider,store;gc.collect();observer.mark('after_scope_release')
                return
            observer.mark('before_layer20_certificate',dict(state=state,context=context.__dict__))
            proof,times=certify_immediate(context.front,context.top,context.prefix,OUT/(mode+'_certificate'),workers=8)
            observer.mark('after_layer20_certificate')
            reference=state['certificate']['layer:20']['ledger']
            changed=[k for k in proof if proof[k]!=reference[k]]
            assert set(changed)<=({'source_identity'} if mode.startswith('optimized') else set()),changed
            (OUT/(mode.upper()+'_CERTIFICATE_EQUALITY.json')).write_text(json.dumps(dict(passed=True,relations=proof['completed_pairs'],timings=times,representation_source_changes=changed),indent=2)+'\n')
            if mode.startswith('optimized'):
                # Close mmap owners before TemporaryDirectory removes its files on Windows.
                del context,state,provider,store;gc.collect()
                return
            # The preserved native layer-5 envelope is a real production input.
            commit=json.loads((source/'COMMIT_05.json').read_text())
            for p in store.path.glob('NATIVE_*.json'):
                envelope=json.loads(p.read_text())
                if envelope['scientific']==commit['state']:
                    (store.path/'CURRENT.json').write_text(json.dumps(dict(format=envelope['format'],checkpoint=p.name,sha256=file_hash(p),scientific=envelope['scientific'])))
                    break
            else:raise ValueError('MissingNativeLayer5Authority')
            del context,state;gc.collect();observer.layer=6
            state=store.restart();observer.mark('before_active_search')
            proposal=provider.propose(state);observer.mark('after_active_search',proposal)
            assert proposal['forecast']['current_layer']==6
            from sdf_cfd_mesher_v4.forecast.detailed.exact_cache import LOCAL_EXACT
            from sdf_cfd_mesher_v4.certificate.cad_algebra_cache import cache_info
            observer.mark('retained_exact_jacobians',vars(LOCAL_EXACT))
            (OUT/'CACHE_BASELINE.json').write_text(json.dumps(dict(exact_count=len(LOCAL_EXACT.entries),exact_hits=LOCAL_EXACT.hits,exact_misses=LOCAL_EXACT.misses,cad=cache_info()),indent=2)+'\n')
            provider.sdf.clear_memo();observer.mark('after_sdf_memo_clear')
            del proposal,state,provider,store;gc.collect();observer.mark('after_scope_release')
        finally:print(observer.finish(),flush=True)

if __name__=='__main__':run()
