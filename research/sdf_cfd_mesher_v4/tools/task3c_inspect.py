"""Read-only local cap audit. No trial generation or geometry acceptance."""
from task3_repair_study import *
from fractions import Fraction as Q
OUT=ROOT/'results/task3c'

def run(replay=False):
    ctx=load();front,top,sdf,schedule,coords,*_=ctx;rows=[]
    for s in [v for v in components() if v['name'] in ('component_00','component_01','component_02','component_04')]:
        p=build_patch(top,s['columns']);d=DetailedCache().get(front,SimpleNamespace(layers=np.arange(1,21),vertex={'coordinates':coords}),top,p.detailed)
        c=d['cells'];bad=np.flatnonzero(c['jacobian'][0]<=0);trace=[]
        for j in bad:
            col=int(c['column'][0,j]);ids=top.triangles[col];prism=np.concatenate((front.coordinates[ids],coords[0,ids]));ex=exact_wedge(prism)
            trace.append(dict(column=col,ids=ids.tolist(),wall=prism[:3].tolist(),top=prism[3:].tolist(),gradient=sdf.evaluate(prism[3:]).normal.tolist(),exact=ex))
        sizes=[]
        for ring in range(4):
            p=build_patch(top,s['columns'],active_rings=ring,max_vertices=256,max_columns=1024)
            sizes.append(dict(ring=ring,active=len(p.active),boundary=len(p.fixed_boundary),halo=len(p.halo),core_columns=len(p.detailed.core_columns),columns=len(p.detailed.columns)))
        rows.append(dict(component=s['name'],sizes=sizes,trace=trace));print(s['name'],sizes,[(v['column'],v['ids'],v['exact']['minimum_reference_corner'],v['exact']['minimum_reference_height']) for v in trace],flush=True)
    if replay:assert scientific_hash(json.loads((OUT/'INITIAL_CORNER_TRACE.json').read_text()))==scientific_hash(rows)
    else:(OUT/'INITIAL_CORNER_TRACE.json').write_text(json.dumps(plain(rows),indent=2,sort_keys=True)+'\n')
    (OUT/('CORNER_TRACE_REPLAY.json' if replay else 'CORNER_TRACE_HASH.json')).write_text(json.dumps(dict(scientific_hash=scientific_hash(rows)))+'\n')
if __name__=='__main__':run('--replay' in sys.argv)
