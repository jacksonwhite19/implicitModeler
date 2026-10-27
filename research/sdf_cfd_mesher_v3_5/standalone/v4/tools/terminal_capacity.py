"""Exact target-absence receipts. Search failure is never a range proof.

The adapter covers existing inward norm-minus-radius primitives. Unsupported
sources return unknown. Acceptance consumes the same verified source definition,
not a caller-supplied numeric upper bound.
"""
from fractions import Fraction
from pathlib import Path
from sdf_cfd_mesher_v4.common.authority import identity,file_hash
import geometry_family_authority as geometry

VERSION='V4TerminalCapacityRangeWitness/1'
STATUSES=('ContinuationRequired','ContinuationPassed','ContinuationFailedExistenceUnresolved',
          'LocalTerminalCapacityProven','GlobalTerminalCapacityProven',
          'TerminalCapacityResolved','TerminalBlockedByCurrentLayerQuality')


def range_witness(spec,target,columns,branches=None):
    components=spec.get('components',[])
    supported={'capsule','bent_capsule','torus'}
    if not spec.get('cavity') or not components:return None
    if branches is None:branches=list(range(len(components)))
    branches=sorted(set(branches));columns=sorted(set(columns))
    if not branches or not columns or any(type(i) is not int or i<0 for i in columns):raise ValueError('InvalidCapacityScope')
    if any(type(i) is not int or i<0 or i>=len(components) for i in branches):raise ValueError('InvalidCapacityBranches')
    if any(components[i]['primitive'] not in supported for i in branches):return None
    # Source is -min(distance_i-radius_i) = max(radius_i-distance_i).
    # A global bound needs every branch. A local branch proof cannot waive
    # continuation on unproved branches or permit branch switching.
    bounds=[Fraction(float(components[i]['radius'])) for i in branches]
    upper=max(bounds);requested=Fraction(float(target))
    if requested<=upper:return None
    payload=dict(version=VERSION,method='AnalyticRangeBound',predicate='radius-minus-nonnegative-Euclidean-distance',
        source=geometry.Source(spec).source_identity,source_implementation=file_hash(Path(geometry.__file__)),
        witness_implementation=file_hash(Path(__file__)),branches=branches,columns=columns,
        scope='GlobalTerminalCapacity' if branches==list(range(len(components))) else 'LocalTerminalCapacity',
        target=str(requested),upper_bound=str(upper),excess=str(requested-upper),status='TargetLevelSetAbsent')
    return dict(payload=payload,sha256=identity(payload))


def verify(witness,spec,target,columns,branches=None):
    expected=range_witness(spec,target,columns,branches)
    return expected is not None and witness==expected


def continuation_status(spec,target,columns,witness=None,continuation=None):
    if witness is not None and verify(witness,spec,target,columns):
        return 'GlobalTerminalCapacityProven'
    if continuation is None:return 'ContinuationRequired'
    if continuation.get('passed') is True:return 'ContinuationPassed'
    return 'ContinuationFailedExistenceUnresolved'

def remesh_admitted(receipt):
    """Only the continuation obligation can be discharged by exact absence."""
    from sdf_cfd_mesher_v4.advancing_front.termination_policy import REMESH_CHECKS
    checks=receipt.get('checks',{})
    if not all(checks.get(k) is True for k in REMESH_CHECKS if k!='neighbor_continuation'):return False
    if checks.get('neighbor_continuation') is True:return True
    terminal=receipt.get('terminal_capacity')
    if not terminal:return False
    layer=terminal.get('actual_terminal_layer')
    schedule=terminal.get('frozen_targets',[])
    if type(layer) is not int or not 0<=layer<len(schedule) or schedule[layer]!=terminal.get('target'):return False
    return (terminal.get('status')=='TerminalCapacityResolved' and
            verify(terminal.get('witness'),terminal['source_definition'],terminal['target'],terminal['columns']) and
            terminal['witness']['payload']['scope']=='GlobalTerminalCapacity' and
            terminal.get('all_columns')==terminal['columns'])


def terminal_state(state,spec,target,targets,witness):
    """Preserve the actual layer map; terminate advancement without geometry edits."""
    from copy import deepcopy
    columns=sorted(map(int,state['columns']))
    layer=max(c['layer'] for c in state['cells'].values())
    if not 0<=layer<len(targets) or float(targets[layer])!=float(target):raise ValueError('CapacityTargetNotNextFrozenTarget')
    if not verify(witness,spec,target,columns):raise ValueError('UnqualifiedTerminalCapacity')
    if witness['payload']['scope']!='GlobalTerminalCapacity':raise ValueError('LocalCapacityNeedsBranchContinuation')
    result=dict(state);result['columns']=deepcopy(state['columns']);result['prism_front']={}
    for column in result['columns'].values():column['status']='TerminalCapacityResolved'
    result['local_schedule']=deepcopy(state.get('local_schedule',{}))
    terminal=dict(status='TerminalCapacityResolved',reason='ExactGeometryCapacityLimit',source_definition=spec,
                  witness=witness,target=float(target),columns=columns,all_columns=columns,
                  requested_layers=len(targets),actual_terminal_layer=max(c['layer'] for c in state['cells'].values()),
                  frozen_targets=list(map(float,targets)))
    result['local_schedule']['terminal_capacity']=terminal
    return result


def prevent_terminal_advance(state):
    if state.get('local_schedule',{}).get('terminal_capacity',{}).get('status')=='TerminalCapacityResolved':
        raise ValueError('GlobalTerminalCapacityAlreadyResolved')
