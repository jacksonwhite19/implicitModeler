"""Read-only WSL resource sampling for the three task-owned solver cases."""
import json
import os
from pathlib import Path
import shutil
import time

ROOT=Path(__file__).resolve().parents[1]/'results/capsule_broad_global_family'
NAMES=('C2','C2-R','G1')


def sample():
    cases={n:dict(ranks=0,rss_bytes=0,rank_hwm_sum_bytes=0,cpu_seconds=0) for n in NAMES}
    for path in Path('/proc').glob('[0-9]*'):
        try:
            if (path/'comm').read_text().strip()!='foamRun':continue
            cwd=(path/'cwd').resolve()
            name=next((n for n in NAMES if cwd==ROOT/n),None)
            if name is None:continue
            status=dict(line.split(':',1) for line in (path/'status').read_text().splitlines() if ':' in line)
            stat=(path/'stat').read_text().split();c=cases[name];c['ranks']+=1
            c['rss_bytes']+=int(status['VmRSS'].split()[0])*1024
            c['rank_hwm_sum_bytes']+=int(status['VmHWM'].split()[0])*1024
            c['cpu_seconds']+=(int(stat[13])+int(stat[14]))/os.sysconf('SC_CLK_TCK')
        except (FileNotFoundError,ProcessLookupError,PermissionError):continue
    return dict(unix_time=time.time(),free_bytes=shutil.disk_usage(ROOT).free,cases=cases)


if __name__=='__main__':
    with (ROOT/'RESOURCE_SAMPLES.jsonl').open('a',buffering=1) as out:
        for _ in range(720):
            out.write(json.dumps(sample(),sort_keys=True)+'\n')
            if all((ROOT/n/'pilot.exit').exists() for n in NAMES):break
            time.sleep(10)
