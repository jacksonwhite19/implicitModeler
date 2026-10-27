"""Linux-only cancellation of a matching recorded V3.5 native process group."""
from pathlib import Path
import json,os,signal,sys,time
scope=Path(sys.argv[1]);case=sys.argv[2];binary=sys.argv[3]
if not scope.is_file():raise SystemExit(0)
record=json.loads(scope.read_text());pid=record['pid']
if type(pid) is not int or pid<2:raise ValueError('InvalidOwnedNativePid')
def same():
    try:
        fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        command=Path(f'/proc/{pid}/cmdline').read_bytes().rstrip(b'\0').split(b'\0')
        command=[os.fsdecode(x) for x in command]
        bound=int(fields[19])==record['start_ticks'] and os.getpgid(pid)==pid
        launch=str(scope).removesuffix('.linux_scope.json')+'.launch.sh'
        pending=bool(command) and Path(command[0]).name=='bash' and launch in command
        expected=record.get('argv')
        executing=(isinstance(expected,list) and bool(expected) and any(command[i:i+len(expected)]==expected for i in range(len(command)))) if expected is not None else (binary in command and '-case' in command and command[command.index('-case')+1]==case)
        return bound and (pending or executing)
    except (FileNotFoundError,ProcessLookupError):return False
if not same():raise SystemExit(0)
os.killpg(pid,signal.SIGTERM)
for _ in range(20):
    if not same():raise SystemExit(0)
    time.sleep(.1)
if same():os.killpg(pid,signal.SIGKILL)
