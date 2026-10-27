"""Audit current native checkpoints without promoting unfinished adapters."""
import json
from collections import Counter
import generalize_local_remesh as g
from sdf_cfd_mesher_v4.advancing_front.mixed_front import geometry_inputs,scientific_hash
from sdf_cfd_mesher_v4.advancing_front.mixed_front_journal import MixedFrontJournal

OUT=g.s.ROOT/'results/purpose_built_mixed_front_runner'

def run():
    cases={}
    for name in ('clean_curved_passage','analytic_cavity_with_control'):
        folder=OUT/name/'corrected'
        result=json.loads((folder/'RESULT.json').read_text())
        state=MixedFrontJournal(folder/'checkpoints').restart()
        inputs=geometry_inputs(state)
        assert all(all(inputs[k]==v for k,v in r['inputs'].items()) for family in ('quality','certificate') for r in state[family].values())
        assert all(r['value']['classification'] in ('CertifiedSeparated','CertifiedAllowedIncidentContact') for r in state['certificate'].values())
        assert all(r['exact']['jacobian_sign']>0 and r['exact']['volume_sign']>0 for r in state['quality'].values() if 'cell_type' in r)
        assert not set(state['forecast'])-{k for k,v in state['columns'].items() if v['status']=='ActivePrismColumn'}
        replay=json.loads((OUT/name/'corrected_replay/REPLAY_COMPARISON.json').read_text())
        assert replay['equal'] and replay['operations_equal']
        remesh=result['operations'][0]['event']
        assert all(remesh['validation']['checks'].values())
        chosen=next(r['continuation'] for r in result['records'] if 'continuation' in r)
        accepted=next(a for a in chosen['attempts'] if a.get('passed'))
        assert accepted['result']['certificate']['required_pairs']==len(state['certificate'])
        cases[name]=dict(final_hash=scientific_hash(state),wall_identity=result['wall_identity'],cells=len(state['cells']),
            cell_types=Counter(c['kind'] for c in state['cells'].values()),columns=len(state['columns']),vertices=len(state['vertices']),
            column_states=Counter(c['status'] for c in state['columns'].values()),layer_map=Counter(c['last_prism_layer'] for c in state['columns'].values()),
            active_columns=len(state['prism_front']),outer_faces=len(state['outer']),required_pairs=len(state['certificate']),
            quality_receipts=len(state['quality']),stale_active_receipts=0,forbidden=0,unresolved=0,
            volume=accepted['result']['volume'],removed_committed_cells=remesh['cavity_manifest']['removed_cells'],
            cavity_seed=remesh['cavity_manifest']['seed'],continuation_column=accepted['column'],
            rebuilds=[dict(dependencies=r['dependencies'],retained=r['receipts']['retained'],
                           retired={k:(v['count'] if isinstance(v,dict) and 'record' in v else len(v)) for k,v in r['receipts']['retired'].items()})
                      for r in result['records'] if 'dependencies' in r],
            runtime=json.loads((folder/'RUNTIME.json').read_text()),replay=replay)
    summary=dict(classification='V4PurposeBuiltMixedFrontRunnerPromising',cases=cases,production_default_enabled=False,
        limitations=['Legacy SF2 NativeSdf correction/planner path is not migrated to the native model',
                     'Atlas and forecast dependency identities exist; Task-4C detailed geometric forecasting is not adapted',
                     'General-SDF stored seed revalidates exactly, but adjusted-schedule native integration is incomplete',
                     'Legacy checkpoint and whole-shell certificate-manifest migration is incomplete',
                     'Failure injection covers journal unit stages, not each live numerical validation substage',
                     'Both live cases remove zero already committed cells; general-SDF rollback remains unqualified in this runner'],
        next_task='Complete native SF2 adapters, adjusted-schedule general-SDF provider, and legacy evidence migration before default activation')
    g.save(OUT/'SUMMARY.json',summary)

if __name__=='__main__':run()
