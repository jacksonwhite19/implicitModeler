"""Restart from a verified pre-event or late nominal checkpoint and re-execute.

Only the starting immutable native objects are shared. No terminal geometry or
accepted remesh result is provided to the construction path.
"""
import sys
sys.dont_write_bytecode=True
import argparse,json,os,shutil
from pathlib import Path
from run_geometry_family import OUT,run,g
from sdf_cfd_mesher_v4.common.authority import file_hash


def replay(name,source_label='Q',destination_label='R',between_events=False,last_event=False):
    source=OUT/name/source_label;dest=OUT/name/destination_label;dest.mkdir(exist_ok=False)
    trace=json.loads((source/'TRACE.json').read_text());events=[i for i,r in enumerate(trace) if r['operation']=='RollbackRemesh']
    if last_event:
        if not events or events[-1]==0:raise ValueError('PreEventCheckpointOutsideTrace')
        chosen=trace[events[-1]-1]
    elif between_events:
        if len(events)<2:raise ValueError('TwoCommittedEventsRequired')
        chosen=trace[events[1]-1]
    else:
        if events and events[0]==0:raise ValueError('PreEventCheckpointOutsideTraceUseBetweenEvents')
        chosen=trace[events[0]-1] if events else trace[-3]
    envelope=None;path=None
    for p in (source/'checkpoints').glob('NATIVE_*.json'):
        value=json.loads(p.read_text())
        if value['scientific']==chosen['state']:envelope=value;path=p;break
    if envelope is None:raise ValueError('MissingReplayCheckpoint')
    checkpoints=dest/'checkpoints';objects=checkpoints/'objects';objects.mkdir(parents=True)
    for row in envelope['components'].values():
        src=source/'checkpoints/objects'/row['file'];assert file_hash(src)==row['sha256']
        target=objects/src.name
        if not target.exists():os.link(src,target)
    shutil.copy2(path,checkpoints/path.name)
    g.save(checkpoints/'CURRENT.json',dict(format=envelope['format'],checkpoint=path.name,scientific=envelope['scientific'],sha256=file_hash(path)))
    g.save(dest/'REPLAY_INPUT.json',dict(source_checkpoint=str(path),sha256=file_hash(path),state=chosen['state'],
        layer=chosen['layer'],scope='Before latest committed remesh event' if last_event else ('Between committed remesh events' if between_events else ('Before first remesh event' if events else 'Last two nominal production steps'))))
    run(name,destination_label,resume=True)
    a=json.loads((source/'SHELL_RESULT.json').read_text());b=json.loads((dest/'SHELL_RESULT.json').read_text())
    result=dict(exact=a['final']==b['final'],primary=a['final'],replay=b['final'],starting_checkpoint=chosen['state'],
        wall_exact=b['wall_exact'],first_layer_exact=b['first_layer_exact'])
    g.save(OUT/name/'DETERMINISM.json',result)
    if not result['exact']:raise ValueError('NativeReplayScientificMismatch')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('case');p.add_argument('--source',default='Q');p.add_argument('--destination',default='R');p.add_argument('--between-events',action='store_true');p.add_argument('--last-event',action='store_true');a=p.parse_args()
    replay(a.case,a.source,a.destination,a.between_events,a.last_event)
