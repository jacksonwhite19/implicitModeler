"""Independent scientific L6 replay from L5; no second published mesh tree."""
from qualify_recursive_layer6 import *
import composable_transition,composable_assembly,composable_publication

def run(replay=None):
    if (OUT/'AUTHORITY.json').exists():raise ValueError('PreserveSealedQualificationOutput')
    replay=OUT/'replay' if replay is None else Path(replay).resolve()
    if not replay.resolve().is_relative_to(OUT.resolve()) or replay.resolve()==OUT.resolve():raise ValueError('ReplayMustUseIsolatedTaskSubdirectory')
    replay.mkdir(exist_ok=False);tick=measured()
    # Reuse sealed admission evidence. Recompute realized geometry, exact metrics,
    # new continuous proofs, affected bindings/quality and independent content root.
    for name in ('ADMISSION.json','DIRECT.json','LOCALIZATION.json','SCREEN_ARRAYS.npz'):
        shutil.copyfile(OUT/name,replay/name)
    try:
        with patch.object(NativeCheckpointStore,'restart',denied),CommittedState(read(AUTH/'REFRAMED_REFERENCE.json')) as base:
            adapter,spec=inputs(base)
            composable_transition.produce(base,adapter,spec,replay)
            composable_assembly.assemble(base,adapter,replay)
            plan=composable_publication.prepare(base,adapter,replay)
            expected=read(OUT/'CANDIDATE_PLAN.json')
            comparisons={k:plan['components'][k]==v for k,v in expected['components'].items()}
            result=dict(passed=all(comparisons.values()) and plan['scientific']==expected['scientific'],
                comparisons=comparisons,content=plan['scientific'],expected_content=expected['scientific'],
                metrics=metrics(*tick),stats=dict(base.stats),admission='Reused authenticated decision; no new forecast',
                production_pointer_written=False,legacy_fallbacks=0)
            durable_json(OUT/'REPLAY.json',result)
            if not result['passed']:raise ValueError('DeterministicScientificComponentMismatch')
        print(json.dumps(result,indent=2),flush=True)
    except BaseException as exc:
        durable_json(OUT/'REPLAY_FAILURE.json',dict(error=repr(exc),traceback=traceback.format_exc(),metrics=metrics(*tick)))
        raise
if __name__=='__main__':run()
