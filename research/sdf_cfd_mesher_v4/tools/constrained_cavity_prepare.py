"""Prepare merged dependency cavities and prior compact native reproducers."""
from constrained_cavity import *
from constrained_cavity_audit import OUT,V4

def run():
    interface=V4/'results/terminal_capacity_aware_remesh_acceptance/interface';base=V4/'results/bent_terminal_shell_fixed_interface_core'
    x,t,ci,iface=load_core(base/'final_stars',interface);q,m=evaluate(x,t,ci,iface)
    fixed=m['bad'][np.all(np.isin(t[m['bad']],ci),axis=1)];groups=components(fixed,m['owners']);out=OUT/'merged';out.mkdir(exist_ok=False);rows=[]
    for gi,seeds in enumerate(groups):
        for depth in range(4):
            data,row=cavity(x,t,ci,iface,m,seeds,depth);row['group']=gi;row['file']=f'group_{gi}_ring_{depth}.npz';np.savez_compressed(out/row['file'],**data);rows.append(row)
    save(out/'CAVITIES.json',rows);print(rows,flush=True)
    x,t,ci,iface=load_core(base/'local_stars',interface);q,m=evaluate(x,t,ci,iface)
    seeds=np.unique([np.argmin(m['determinant']),m['interface_owner'][np.argmin(m['interface_weight'])],components(m['bad'],m['owners'])[0][0]])
    rep=OUT/'reproducer';rep.mkdir(exist_ok=False);rr=[]
    for rank,seed in enumerate(seeds):
        data,row=cavity(x,t,ci,iface,m,seed,2);row['seed_rank']=rank;np.savez_compressed(rep/f'prior_seed_{rank}.npz',**data);rr.append(row)
    save(rep/'INPUTS.json',rr)
    # Analyze the preserved malformed raw output without deduplicating it.
    d=dict(np.load(rep/'prior_seed_0.npz'));raw=np.load(base/'cavity_seed0/RAW_PROPOSAL.npz');xx=raw['coordinates'];tt=raw['tets'];lookup={p.tobytes():i for i,p in enumerate(d['x'])};pts=list(d['x']);mp=[]
    for p in xx:
        if p.tobytes() not in lookup:lookup[p.tobytes()]=len(pts);pts.append(p)
        mp.append(lookup[p.tobytes()])
    mapped=orient(np.array(pts),np.array(mp)[tt]);rawiv=invariants(np.array(pts),d['t'],mapped)
    _,ix=np.unique(np.sort(mapped,1),axis=0,return_index=True);dedup=invariants(np.array(pts),d['t'],mapped[np.sort(ix)])
    save(rep/'PRESERVED_RAW_ANALYSIS.json',dict(raw=rawiv,deduplicated=dedup))

if __name__=='__main__':run()
