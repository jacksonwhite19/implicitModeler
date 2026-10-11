"""Read-only RSS observation; OS high-water includes the earlier process lifetime."""
import json,time
import psutil
from repair_bulk_core import OUT,write

def main():
    targets=[]
    for p in psutil.process_iter(['cmdline']):
        command=p.info['cmdline'] or []
        if len(command)==2 and command[1].endswith('certify_repaired_complete.py'):targets.append(p)
    if len(targets)!=1:raise ValueError('CertificateProcessAmbiguous')
    p=targets[0];rows=[];peak=0
    while True:
        try:
            memory=p.memory_info();peak=max(peak,memory.peak_wset)
            progress=json.loads((OUT/'CONTINUOUS_PROGRESS.json').read_text())
            rows.append(dict(layer=progress['layer'],rss=memory.rss,os_lifetime_peak=memory.peak_wset))
        except psutil.NoSuchProcess:break
        except (json.JSONDecodeError,PermissionError):pass
        write('CERTIFICATE_MEMORY.json',dict(pid=p.pid,peak_rss=peak,samples=rows))
        time.sleep(2)
    write('CERTIFICATE_MEMORY.json',dict(pid=p.pid,peak_rss=peak,samples=rows,process_completed=True))

if __name__=='__main__':main()
