"""Run the existing local neighbor-continuation acceptance obligation only.

No layer is committed. An absent later target does not prove the current
candidate invalid; it identifies an unsatisfied existing admission contract.
"""
import argparse,json,sys,time,traceback
from fractions import Fraction
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
import numpy as np
from run_geometry_family import AnalyticProvider
import generalize_local_remesh as g
from bounded_fallback_control import Events
from investigate_large_fallback import save


def main():
    root=ROOT/'results/bent_passage_large_case_fallback'
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=root/'neighbor_continuation')
    folder=parser.parse_args().output;folder.mkdir(exist_ok=True)
    events=Events(folder/'EVENTS.jsonl',300);result=dict(committed=False)
    try:
        events('stage_start',stage='continuation_inputs')
        spec=json.loads((ROOT/'results/production_scale_resume_fast/bent_continuation/GEOMETRY_AUTHORITY.json').read_text())['spec']
        provider=AnalyticProvider(spec,folder);provider.search_progress=events
        z=np.load(root/'nominal/REPRODUCER.npz');fronts=list(z['fronts']);fronts[-1]=np.load(root/'tangent_remaining/BEST.npz')['front']
        levels=np.load(root/'remesh/LEVELS.npy');seed=set(map(int,np.flatnonzero(levels<len(fronts))))
        eligible=[i for i,f in enumerate(provider.tri) if all(levels[v]==len(fronts) for v in f)]
        events('stage_end',stage='continuation_inputs',eligible_columns=len(eligible),seed_vertices=len(seed))
        result['continuation']=g.continue_local(spec,provider.x,provider.tri,fronts,levels,seed,
            geometry=provider.geometry_source,complete_validator=provider.continuation_validator,
            front_provider=provider.continuation_front,eligible=eligible,progress=events,attempt_limit=4)
        # Fixture-specific forensic deduction, never a production branch or gate.
        # Its source formula is radius - nonnegative distance to the arc.
        radius=spec['components'][0]['radius'];target=float(provider.schedule.targets[len(fronts)])
        centre=np.array([[spec['components'][0]['major'],0.,0.]])
        sample=provider.geometry_source.Source(spec).evaluate(centre)
        result['capacity_forensics']=dict(radius=radius,next_target=target,
            exact_binary64_excess=str(Fraction(target)-Fraction(radius)),
            excess=float(Fraction(target)-Fraction(radius)),centreline_phi=float(sample.phi[0]),
            centreline_source_error_bound=float(sample.source_error_bound[0]),
            proof='The preserved inward bent-capsule SDF is radius minus Euclidean distance to the closest arc point. Distance is nonnegative; therefore phi <= radius everywhere. The frozen next target exceeds radius.',
            scope='Forensic authority of this preserved construction only; not a geometry-specific production heuristic')
        events('stage_end',stage='neighbor_continuation',passed=result['continuation']['passed'],seconds=result['continuation']['runtime'])
    except BaseException as exc:
        result.update(stop=str(exc),traceback=traceback.format_exc())
    finally:save(folder/'RESULT.json',result)

if __name__=='__main__':main()
