"""Subprocess watchdog: inactivity stops a trial, never accepts its mesh."""
import argparse,json,subprocess,sys,time
from pathlib import Path
import psutil


def main():
    p=argparse.ArgumentParser();p.add_argument('--events',type=Path,required=True)
    p.add_argument('--result',type=Path,required=True);p.add_argument('--silence',type=float,default=120)
    p.add_argument('--seconds',type=float,default=600);p.add_argument('command',nargs=argparse.REMAINDER)
    a=p.parse_args();command=a.command[1:] if a.command[:1]==['--'] else a.command
    start=time.monotonic();last=start;size=0;peak=0;maxgap=0;reason=None
    child=subprocess.Popen(command)
    while child.poll() is None:
        now=time.monotonic()
        current=a.events.stat().st_size if a.events.exists() else 0
        if current!=size:
            maxgap=max(maxgap,now-last);size=current;last=now
        try:peak=max(peak,psutil.Process(child.pid).memory_info().rss)
        except psutil.NoSuchProcess:pass
        if now-last>=a.silence:reason='UnobservedCallbackBound'
        if now-start>=a.seconds:reason='AttemptTimeBound'
        if reason:
            proc=psutil.Process(child.pid)
            for sub in proc.children(recursive=True):sub.kill()
            proc.kill();child.wait();break
        time.sleep(.25)
    elapsed=time.monotonic()-start;maxgap=max(maxgap,time.monotonic()-last)
    row=dict(exit_code=child.returncode,stop=reason,seconds=elapsed,peak_sampled_rss=peak,
             maximum_observed_event_gap=maxgap,unobserved_bound=a.silence,
             scientific_feasibility='NotDeterminedByWatchdog',command=command)
    a.result.write_text(json.dumps(row,indent=2)+'\n');print(json.dumps(row),flush=True)
    sys.exit(0 if child.returncode==0 else 1)

if __name__=='__main__':main()
