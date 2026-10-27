"""Fresh candidate parent access without another scheduled prism generation."""
from pathlib import Path
import capsule_user_domain_handoff as h
from composable_state import CommittedState
from critical_quality_lookup import selected

OUT=Path(__file__).resolve().parents[1]/'results/direct_changed_persistence'

if __name__=='__main__':
    folder=OUT/'normal_candidate_clean_prism'
    reference=h.read(folder/'REFERENCE.json');rows=h.read(folder/'QUALITY_DELTA.json')
    keys=[r['key'] for r in rows[:16]]+[r['key'] for r in rows[-16:]]
    tick=h.measured()
    # Explicit indexed quality probes need SQLite. CommittedState keeps its
    # normal immutable-token verification; the zero-query startup was tested separately.
    with CommittedState(reference) as state:
        values=selected(state,keys)
        assert all(values[k]==state['quality'][k] for k in keys)
        geometry=h.read(folder/'GEOMETRY_DELTA.json')
        for kind in ('vertices','cells','faces','columns','prism_front','versions'):
            for key in list(geometry[kind])[:8]:assert state[kind][key]==geometry[kind][key]
        state.check()
        result=dict(passed=True,quality_keys=len(keys),content=state.content_root,stats=dict(state.stats),metrics=h.metrics(*tick),
            conversion_required=False,format='Existing recursive JSON geometry and indexed SQLite quality; byte-identical',layer_generated=False)
    h.durable_json(OUT/'FUTURE_PARENT.json',result)
