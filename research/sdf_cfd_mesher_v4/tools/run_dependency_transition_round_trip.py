"""Configure one already-authorized retained transition through shared executors.

Fresh generation and replay are performed by run_authorized_continuation.
Original fronts stay immutable. No new planner, proof or publication path.
"""
import argparse
import json
from pathlib import Path
import sys

V4=Path(__file__).resolve().parents[1]
ROOT=V4.parents[1]
sys.path[:0]=[str(V4/'tools'),str(V4/'src')]
OUT=V4/'results/current_front_transition_round_trip'
from path_preserving_deduplication import durable_json,file_hash


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))


def validate_control(control):
    from contextlib import ExitStack
    from sdf_cfd_mesher_v4.common.authority import identity
    import immutable_state_access as access
    with ExitStack() as held:
        for row in control['inputs']:
            path=ROOT/row['path']
            held.enter_context(access.guard(path))
            if file_hash(path)!=row['sha256']:raise ValueError('RoundTripInputIdentityMismatch',row['path'])
        if identity(control['authorization_record'])!=control['authorization']:
            raise ValueError('RoundTripAuthorizationMismatch')
        parent=control['parent'];reference=control['reference_result']
        if parent!=read(ROOT/control['parent_record']) or reference!=read(ROOT/control['reference_record']):
            raise ValueError('RoundTripReferenceMismatch')
        if control['layers']!=[5] or control['no_layer6'] is not True:
            raise ValueError('UnsupportedRoundTripSelection')
    return control


def baseline():
    if OUT.exists():raise ValueError('PreserveRoundTripOutput')
    # Existing census/protected-source and retained hash verification mechanism.
    from check_current_front_encoding import baseline as retained_baseline
    retained_baseline(OUT)
    parent='research/sdf_cfd_mesher_v4/results/wing_receipt_resume/A/L4/REOPEN.json'
    reference='research/sdf_cfd_mesher_v4/results/retained_l5_finish/LAST_QUALIFIED.json'
    authorization='research/sdf_cfd_mesher_v4/results/wing_continuation_layers2to5/AUTHORIZATION.json'
    paths=[parent,reference,authorization,'research/sdf_cfd_mesher_v4/results/current_front_dependency_encoding/REVIEW.json',
           'research/sdf_cfd_mesher_v4/tools/current_front_dependencies.py',
           'research/sdf_cfd_mesher_v4/tools/critical_assembly.py',
           'research/sdf_cfd_mesher_v4/tools/critical_prepare.py',
           'research/sdf_cfd_mesher_v4/tools/critical_prism_install.py']
    control=dict(schema='V4AuthorizedTransitionRoundTrip/1',layers=[5],
        authorization='bd2e3209707fd7f11d3082a069f2490a6903421e73ca16727f375683b4d96772',
        authorization_record=read(ROOT/authorization),parent=read(ROOT/parent),
        parent_record=parent,reference_result=read(ROOT/reference),reference_record=reference,
        inputs=[dict(path=p,sha256=file_hash(ROOT/p)) for p in paths],
        user_request='Go do it: one complete encoding transition round-trip with replay/publication/reopen.',
        scope='Fresh L4-to-L5 round-trip, original authorities unchanged; no L6.',no_layer6=True)
    validate_control(control);durable_json(OUT/'CONTROL.json',control)


def worker():
    import run_authorized_continuation as runner
    control=validate_control(read(OUT/'CONTROL.json'))
    runner.OUT=OUT/'A';runner.OUT.mkdir(exist_ok=False)
    runner.execute(round_trip=control)
    # Exercise the actual published compact reader, not only state construction.
    from sealed_front_connection import qualified_dispatch
    from composable_state import CommittedState
    from current_front_dependencies import ForecastView
    from sdf_cfd_mesher_v4.advancing_front.native_checkpoint import scientific_digest
    comparison=read(V4/'results/current_front_dependency_encoding/review_fixed/L5_RESULT.json')
    reopened=read(runner.OUT/'L5/REOPEN.json')
    with qualified_dispatch(),CommittedState(reopened['reference']) as state:
        view=state['forecast']
        if not isinstance(view,ForecastView):raise ValueError('CompactReaderNotDispatched')
        digest=scientific_digest(view)
        if digest!=comparison['native_scientific_digest'] or len(view)!=comparison['compared_rows']:
            raise ValueError('PublishedForecastMeaningMismatch')
        state.check()
        durable_json(OUT/'READER.json',dict(passed=True,rows=len(view),native_digest=digest,
            scientific=state.scientific,reference=reopened['reference'],actual_compact_view=True))


def monitor():
    import finish_retained_l5 as monitored
    monitored.OUT=OUT
    # Existing monitor includes original campaign ancestry plus this output,
    # using the unchanged 45,120,503,439-byte operational allowance.
    return monitored.monitor(worker_command=[sys.executable,'-B',__file__,'worker'])


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['baseline','worker','monitor'])
    args=p.parse_args()
    if args.mode=='monitor':sys.exit(monitor())
    globals()[args.mode]()
