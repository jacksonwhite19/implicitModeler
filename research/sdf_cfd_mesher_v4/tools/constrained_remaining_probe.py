"""Two evidence-selected remaining components, three cavity extents at most."""
import json
from constrained_cavity import *
from constrained_cavity_audit import OUT,V4
from run_constrained_cavity import run as mesh
from constrained_steiner_position import run as position

def run():
    src=OUT/'accepted_local';interface=V4/'results/terminal_capacity_aware_remesh_acceptance/interface';x,t,ci,iface=load_core(src,interface);q,m=evaluate(x,t,ci,iface)
    comps=components(m['bad'],m['owners']);worst=int(np.argmin(m['determinant']));selected=[next(z for z in comps if worst in z)]
    if comps[0]!=selected[0]:selected.append(comps[0])
    out=OUT/'remaining_probe';out.mkdir(exist_ok=True);save(out/'SELECTION.json',dict(component_count=len(comps),components=comps,selected=selected,rule='Worst determinant component, then largest distinct face-connected component; maximum two'))
    records=[]
    for group,seeds in enumerate(selected):
        for depth in (1,2,3):
            data,row=cavity(x,t,ci,iface,m,seeds,depth);inp=out/f'g{group}_r{depth}.npz';dest=out/f'g{group}_r{depth}_safe'
            if inp.exists():
                prior=np.load(inp)
                if not all(np.array_equal(prior[k],v) for k,v in data.items()):raise ValueError('StaleProbeInput')
            else:np.savez_compressed(inp,**data)
            proc=json.loads(dest.with_suffix('.process.json').read_text()) if dest.with_suffix('.process.json').exists() else mesh(inp,dest,'netgen_safe','one' if depth==1 else 'multi')
            row.update(process=proc,input=str(inp),output=str(dest));records.append(row)
            if proc['exit_code']!=0:continue
            r=json.loads((dest/'RESULT.json').read_text());last=r['stages'][-1];row['result']=last
            if last['invariants']['passed'] and last.get('quality',{}).get('passed'):
                row['qualified_candidate']=str(dest/(last['stage']+'_CANDIDATE.npz'));break
            if depth==3 and last['invariants']['passed']:
                ca=dest/(last['stage']+'_CANDIDATE.npz');opt=out/f'g{group}_r{depth}_margin'
                try:
                    position(inp,ca,opt,'margin');z=json.loads((opt/'RESULT.json').read_text());row['margin_result']=z
                    if z['after']['passed'] and z['invariants']['passed']:row['qualified_candidate']=str(opt/'CANDIDATE.npz')
                except (ValueError,TimeoutError) as e:row['margin_stop']=str(e)
            save(out/'TRIALS.json',records)
        save(out/'TRIALS.json',records)

if __name__=='__main__':run()
