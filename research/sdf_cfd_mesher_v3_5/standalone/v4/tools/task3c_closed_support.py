"""Controlled search extension after exact fixed-corner obstruction diagnosis.

Outward/inward span motion is already in the tangent space at rim vertices.
The development step unit is the preserved first-layer increment. Six halving
polls are a computational bound, not a geometric acceptance tolerance.
"""
import task3_repair_study as study
from task3_repair_study import *
from sdf_cfd_mesher_v4.advancing_front.cap_diagnostics import active_patch
from task3c_io import read as read_evidence,write as write_evidence
OUT=ROOT/'results/task3c'

def run(replay=False):
    start=time.perf_counter();ctx=load();audit=json.loads((OUT/'AUDIT.json').read_text());records=[];times={}
    original_build=study.build_patch;original_fields=study.search_fields
    for spec in [s for s in components() if s['name'] in ('component_00','component_01','component_02','component_04')]:
        plan=next(a['closure_plan'] for a in audit if a['component']==spec['name']);assert plan['complete']
        closed=active_patch(ctx[1],plan['active'])
        def build(top,cols,**kw):
            return closed if list(cols)==spec['columns'] else original_build(top,cols,**kw)
        def fields(front,top,patch,x,total):
            span=np.zeros((len(patch.active),3));span[:,1]=np.sign(front.coordinates[patch.active,1])
            return [('outward_span_tangent',span),('inward_span_tangent',-span)],np.full(len(span),ctx[3].targets[0])
        study.build_patch=build;study.search_fields=fields
        study.CONFIG.update(active_rings=[4],max_active_vertices=512,max_local_columns=4096,radius_rounds=6,initial_radius=1.)
        r,p=study.run_component(spec,ctx,probe=True);name=spec['name']+'_closed_support';h=scientific_hash(r)
        if replay:assert scientific_hash(read_evidence(OUT/f'{name}.json'))==h,name
        else:write_evidence(OUT/f'{name}.json',r)
        records.append(dict(name=name,scientific_hash=h,potential=[i for i,e in enumerate(r['history']) if e.get('potential')],classification=r['final']['classification']))
        times[name]=p;print(name,records[-1],p,flush=True)
    study.build_patch=original_build;study.search_fields=original_fields
    (OUT/('CLOSED_SUPPORT_REPLAY.json' if replay else 'CLOSED_SUPPORT.json')).write_text(json.dumps(dict(results=records,scientific_hash=scientific_hash(records)),indent=2,sort_keys=True)+'\n')
    (OUT/('CLOSED_SUPPORT_REPLAY_TIMINGS.json' if replay else 'CLOSED_SUPPORT_TIMINGS.json')).write_text(json.dumps(times|dict(total_seconds=time.perf_counter()-start),indent=2,sort_keys=True)+'\n')
if __name__=='__main__':run('--replay' in sys.argv)
