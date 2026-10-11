"""Existing retained writer/encoder controls; no front execution or publication."""
import argparse,json,time
from contextlib import nullcontext
from pathlib import Path
from unittest.mock import patch

from path_preserving_deduplication import durable_json,file_hash
from profile_assembly_rows import resource_check
from profile_remaining_prism import memory_sample

V4=Path(__file__).resolve().parents[1]
RUN=V4/'results/geometry_binding_production/run'


def schedule_control(output):
    import current_front_dependencies as codec
    import shared_schedule_hotpath as hot
    # Reuse the original authenticated snapshot and its complete validated
    # closure, not an unverified schedule JSON fixture.
    with hot.installed(),codec.authenticated_snapshot(RUN/'NEW_FORECAST_DEPENDENCIES.json') as snap:
        body=codec.access.read(RUN/'NEW_FORECAST_DEPENDENCIES.json')
        schedules=codec.access.read(body['shared']['path'])['schedules']
        values={k:dict(current_layer=r['current_layer'],region=r['region'],
                       remaining_schedule=list(schedules[r['schedule']])) for k,r in body['rows'].items()}
        records=[];before=None
        for name in ('before','after'):
            resource_check();tick=time.perf_counter()
            with memory_sample(output,'schedule_'+name), (patch.object(hot,'plain_json',lambda _:False) if name=='before' else nullcontext()):
                result=codec.encode_rows(values)
            seconds=time.perf_counter()-tick
            if (result[0]!=body['rows'] or result[1]!=schedules or result[2]!=snap['semantic'] or result[3]!=body['expanded_bytes']):
                raise ValueError('RetainedScheduleMeaningMismatch',name)
            if before is None:before=result
            elif result!=before:raise ValueError('ChangedScheduleEncoding')
            records.append(dict(mode=name,seconds=seconds,exact=True))
        durable_json(output/'SCHEDULE.json',dict(passed=True,rows=len(values),expanded_bytes=body['expanded_bytes'],
            semantic=snap['semantic'],references=snap['references'],measurements=records,
            scope='Encoder control only; no forecast or scientific transition'))
    print('schedule',records,flush=True)


def writer_control(output,count):
    import single_quality_component as quality
    import private_quality_hash_stream as stream
    from profile_assembly_rows import measure
    source=RUN/'QUALITY_DELTA.json';tick=time.perf_counter()
    with quality.snapshot(source) as snap:
        total=len(snap['rows']);step=max(1,total//count);rows=[];indices=[]
        for i,row in enumerate(snap['rows']):
            if i%step==0:
                rows.append(row);indices.append(i)
                if len(rows)==min(count,total):break
        refs=snap['references'];logical=snap['semantic']
    durable_json(output/'WRITER_INPUT.json',dict(source=str(source),references=refs,logical=logical,total=total,
        selected=len(rows),stride=step,first=indices[0],last=indices[-1],load_seconds=time.perf_counter()-tick,
        repeated_binding_rows=sum(type(r['payload']) is dict and 'inputs' in r['payload'] for r in rows)))
    # Preserve reproducibility of the rejected, diagnostic-only prototype.
    # Production always retains its unchanged original stream implementation.
    import importlib.util
    path=V4/'results/assembly_binding_efficiency/rejected_quality_fragment.py'
    spec=importlib.util.spec_from_file_location('rejected_quality_fragment',path)
    prototype=importlib.util.module_from_spec(spec);spec.loader.exec_module(prototype)
    before=measure(output,rows,'before',reuse=True)
    with patch.object(stream,'batch',prototype.batch):after=measure(output,rows,'after',reuse=True)
    a=quality.access.read(output/'before/QUALITY_DELTA.json');b=quality.access.read(output/'after/QUALITY_DELTA.json')
    if a['logical']!=b['logical'] or a['payload']['sha256']!=b['payload']['sha256']:
        raise ValueError('ChangedRetainedQualityDatabase')
    durable_json(output/'WRITER_COMPARISON.json',dict(passed=True,exact_logical=True,exact_database=True,
        payload_sha256=a['payload']['sha256'],rows=len(rows),measurements=[before,after],
        scope='Selected retained rows through existing complete private writer/readbacks; not a full assembly timer'))
    print('writer',before['seconds'],after['seconds'],flush=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--count',type=int,default=20001)
    p.add_argument('--schedule-only',action='store_true');a=p.parse_args()
    if a.output.exists() or a.count<=0:raise ValueError('FreshBoundedBindingControlRequired')
    a.output.mkdir(parents=True);started=time.perf_counter();resource_check()
    schedule_control(a.output)
    if not a.schedule_only:writer_control(a.output,a.count)
    resource_check()
    durable_json(a.output/'RESULT.json',dict(passed=True,seconds=time.perf_counter()-started,
        source=file_hash(__file__),schedule_source=file_hash(V4/'tools/shared_schedule_hotpath.py'),
        quality_source=file_hash(V4/'tools/private_quality_hash_stream.py')))


if __name__=='__main__':main()
