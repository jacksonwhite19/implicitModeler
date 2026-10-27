"""Narrow Linux initializer process-group deadline inside the existing scope."""
import time
ENTRY=time.monotonic()
from pathlib import Path
import json,math,os,signal,subprocess,sys

def members(group):
    rows=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:
            fields=(entry/'stat').read_text().rsplit(')',1)[1].split()
            if int(fields[2])==group and int(fields[3])==group and fields[0] not in ('Z','X'):
                rows.append(dict(pid=int(entry.name),start=int(fields[19]),group=int(fields[2]),session=int(fields[3])))
        except (FileNotFoundError,ProcessLookupError):pass
    return rows

def stop(rows):
    for row in rows:
        try:
            descriptor=os.pidfd_open(row['pid'])
            try:
                fields=Path('/proc/'+str(row['pid'])+'/stat').read_text().rsplit(')',1)[1].split()
                if int(fields[19])!=row['start'] or int(fields[2])!=row['group'] or int(fields[3])!=row['session']:
                    raise ValueError('NativeStageProcessIdentityChanged')
                signal.pidfd_send_signal(descriptor,signal.SIGKILL)
            finally:os.close(descriptor)
        except ProcessLookupError:pass
        except FileNotFoundError:pass

def exit_code(record,code):
    if not record['retired']:return 125
    if not record['deadline_met'] or record['seconds']>record['limit_seconds'] or record['timed_out']:return 124
    if record['unexpected_descendants']:return 125
    return code

def run(seconds,receipt_path,command,*,deadline_monotonic=None):
    if not 1<seconds<=3600 or not command:raise ValueError('ExplicitNativeStageDeadlineRequired')
    if deadline_monotonic is not None and not math.isfinite(deadline_monotonic):
        raise ValueError('FiniteLinuxDeadlineRequired')
    deadline=ENTRY+seconds if deadline_monotonic is None else min(ENTRY+seconds,deadline_monotonic)
    receipt_path=Path(receipt_path)
    if receipt_path.exists():raise ValueError('NativeStageReceiptAlreadyExists')
    if time.monotonic()>=deadline-1:raise TimeoutError('NativeStageStartupBudget')
    child=subprocess.Popen(command,start_new_session=True)
    timed_out=False;unexpected_descendants=False;code=None;observed=[]
    try:
        observed=members(child.pid)
        try:code=child.wait(timeout=max(.001,deadline-1-time.monotonic()))
        except subprocess.TimeoutExpired:timed_out=True
    finally:
        remaining=members(child.pid);unexpected_descendants=bool(remaining) and not timed_out
        observed.extend(remaining)
        stop(remaining)
        while time.monotonic()<deadline:
            remaining=members(child.pid)
            if not remaining:break
            stop(remaining);time.sleep(.01)
        try:child.wait(timeout=max(.001,deadline-time.monotonic()))
        except subprocess.TimeoutExpired:pass
        remaining=members(child.pid)
        elapsed=time.monotonic()-ENTRY
        record=dict(schema='RetainedNativeStageRetirement/1',seconds=elapsed,deadline_met=time.monotonic()<=deadline,
            limit_seconds=seconds,child_returncode=child.returncode,timed_out=timed_out,
            unexpected_descendants=unexpected_descendants,observed_processes=observed,
            live_members=remaining,retired=child.poll() is not None and not remaining)
        if deadline_monotonic is not None:record['deadline_monotonic']=deadline_monotonic
        with receipt_path.open('x',encoding='utf8') as stream:json.dump(record,stream,indent=2)
    return exit_code(record,code)

if __name__=='__main__':
    command=sys.argv[3:]
    absolute=None
    if command and command[0]=='--deadline-monotonic':
        absolute=float(command[1]);command=command[2:]
    raise SystemExit(run(float(sys.argv[1]),sys.argv[2],command,deadline_monotonic=absolute))
