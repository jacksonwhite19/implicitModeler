"""Separate verified migration evidence from unfinished production authority."""
import json
from collections import Counter
from audit_production_migration import OUT,ROOT
from sdf_cfd_mesher_v4.advancing_front.mixed_front_journal import MixedFrontJournal
from sdf_cfd_mesher_v4.advancing_front.mixed_front import scientific_hash,geometry_inputs
from sdf_cfd_mesher_v4.advancing_front.rolling_state import atomic_json
import termination_policy_cohort as cohort
from sdf_cfd_mesher_v4.advancing_front.termination_policy import evaluate

def read(p):return json.loads(p.read_text())

def run():
    state=MixedFrontJournal(OUT/'general_native/checkpoints').restart()
    inputs=geometry_inputs(state)
    assert all(all(inputs.get(k)==v for k,v in r['inputs'].items()) for family in ('quality','certificate') for r in state[family].values())
    native=read(OUT/'general_native/RESULT.json');replay=read(OUT/'GENERAL_REPLAY.json');late=read(OUT/'LATE_FAILURE_RECOVERY.json')
    assert replay['equal'] and late['scientific_equal'] and len(late['rows'])==2
    event=next(iter(state['events'].values()));removed=read(OUT/'general_native/ROLLBACK_MANIFEST.json')['removed']
    negative=[dict(name=c['name'],decision=evaluate(c['evidence'])) for c in cohort.negative_cases()]
    assert len(negative)==63 and all(r['decision']['next_action']!='RollbackRemesh' for r in negative)
    atomic_json(OUT/'NEGATIVE_POLICY_REPLAY.json',dict(count=63,false_triggers=0,rows=negative))
    sf2=read(OUT/'SF2_MIGRATION_INVENTORY.json')
    result=dict(classification='V4MixedFrontProductionMigrationInsufficient',completed=False,
        sf2=dict(quality_receipts=sf2['coverage'],cohort=[1,5,11,12,20],wall=sf2['wall'],first_layer=sf2['first_layer'],
            full_native_build=False,native_checkpoint_migration=False,scientific_execution_seconds=None,overhead_vs_1248_01_seconds=None,
            preserved_history_remesh_events=0,fresh_run_remesh_events=None),
        general=dict(native_steps=2,cells=len(state['cells']),cell_types=dict(Counter(c['kind'] for c in state['cells'].values())),
            removed_committed_cells=len(removed),removed_by_layer_type=dict(Counter(f'{c["layer"]}:{c["kind"]}' for c in removed)),
            column_layer_counts=dict(Counter(str(c['last_prism_layer']) for c in state['columns'].values())),
            active_columns=len(state['prism_front']),affected_vertices=len(event['cavity_manifest']['expansion'][-1]['active']),
            outer_faces=len(state['outer']),proofs=len(state['certificate']),quality_receipts=len(state['quality']),stale_receipts=0,
            wall_equal=native['wall_equal'],first_layer_equal=native['first_layer_equal'],scientific=scientific_hash(state),
            replay_equal=replay['equal'],validation=read(OUT/'general_replay/VALIDATION_6.json'),
            scope='Historical repair imported into native committed state; not prevention of the historical invalid commit'),
        late_failure_recovery=late,
        incomplete=['native legacy checkpoint conversion','full production atlas migration','full production forecast migration',
                    'legacy compact proof binding into native mixed manifest','fresh full SF2 mixed-runner qualification'],
        production_authority='legacy Task-4 unchanged',automatic_remesh_default=False,
        next_task='Complete native array-backed checkpoint, atlas, forecast and manifest migration, then run capped fresh SF2 qualification')
    atomic_json(OUT/'SUMMARY.json',result)
    atomic_json(OUT/'PRODUCTION_AUTHORITY_DECISION.json',dict(switched=False,automatic_remesh_enabled=False,reason=result['incomplete']))
    print(result['classification'],result['general']['cells'],result['general']['proofs'])

if __name__=='__main__':run()
