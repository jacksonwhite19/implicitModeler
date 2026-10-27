"""Reduce this task's measured records; never estimate absent measurements."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]/'results/production_scale_resume_fast'
CASES={'Capsule':['capsule','capsule_completion'],'Bent passage':['bent_continuation'],
       'Interacting ellipsoids':['interacting_continuation'],'Four regions':['four_continuation']}


def collect():
    rows=[]
    for case,folders in CASES.items():
        for folder in folders:
            for path in sorted((ROOT/folder).glob('LAYER_*.json')):
                r=json.loads(path.read_text());t=r['timings'];d=r['degree_metrics']['counts']
                rows.append(dict(case=case,layer=r['layer'],seconds=r['seconds'],hard_quality=t['hard_quality'],
                    degree=t['incremental_degree'],certificate=t['continuous_certificate'],dependencies=t['dependencies'],
                    geometry=t['geometry'],topology=t['topology'],peak_rss=r['peak_rss'],retained_rss=r['retained_rss'],
                    new_cells=r['counts']['new_changed_cells'],cells=r['cells'],new_relations=r['new_relations'],relations=r['relations'],
                    historical_witnesses=d['historical'],reevaluated_witnesses=d['selected'],reused_witnesses=d['reused'],new_witnesses=d['new'],
                    remesh=r['decision']['disposition']!='NoRemesh',source=str(path.relative_to(ROOT))))
    (ROOT/'LAYER_LEDGER.json').write_text(json.dumps(rows,indent=2))
    tables=[]
    specs=[('Ordinary layer times (seconds)', ['case','layer','seconds','hard_quality','degree','certificate','dependencies','geometry','topology']),
           ('Ordinary layer memory and population', ['case','layer','peak_rss','retained_rss','new_cells','cells','new_relations','relations']),
           ('Degree witness population', ['case','layer','historical_witnesses','reevaluated_witnesses','reused_witnesses','new_witnesses'])]
    for title,fields in specs:
        text=[title,'','| '+' | '.join(fields)+' |','|'+'|'.join(['---']*len(fields))+'|']
        for row in rows:
            text.append('| '+' | '.join(f'{row[k]:.6f}' if isinstance(row[k],float) else str(row[k]) for k in fields)+' |')
        tables.append('\n'.join(text))
    (ROOT/'LAYER_TABLES.md').write_text('\n\n'.join(tables)+'\n')
    return rows


if __name__=='__main__':
    rows=collect();print(json.dumps(dict(layers=len(rows),cases=sorted({r['case'] for r in rows}),
        ordinary_seconds=sum(r['seconds'] for r in rows),largest_seconds=max((r['seconds'] for r in rows),default=None))))
