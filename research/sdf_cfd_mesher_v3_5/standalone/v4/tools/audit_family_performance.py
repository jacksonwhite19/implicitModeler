"""Inventory measured process segments without inventing cold-run totals."""
import json
from pathlib import Path
from run_geometry_family import OUT,g

def run():
    rows=[]
    for p in sorted(OUT.rglob('*PERFORMANCE.json')):
        if 'sealed' in p.parts:continue
        d=json.loads(p.read_text());rows.append(dict(path=str(p.relative_to(OUT)),measurements=d))
    peaks=[]
    def visit(value,path):
        if isinstance(value,dict):
            for key,item in value.items():
                if key.lower() in ('peak_rss','peak_rss_bytes','peakworkingset64','peak_wset') and isinstance(item,(int,float)):
                    peaks.append(dict(path=path,field=key,bytes=item))
                else:visit(item,path)
        elif isinstance(value,list):
            for item in value:visit(item,path)
    for row in rows:visit(row['measurements'],row['path'])
    for p in sorted(OUT.glob('*PROCESSES.json')):visit(json.loads(p.read_text(encoding='utf-8-sig')),p.name)
    g.save(OUT/'PERFORMANCE_INVENTORY.json',dict(segments=rows,peaks=sorted(peaks,key=lambda r:r['bytes'],reverse=True),
        interpretation='Resumed process segments are not cold complete-run times. Accepted-step TRACE seconds follow checkpoint ancestry. Stages in older traces are last-measured provider snapshots, not additive per-step durations. Rejected attempts and interrupted process CPU/RSS snapshots remain separate.'))

if __name__=='__main__':run()
