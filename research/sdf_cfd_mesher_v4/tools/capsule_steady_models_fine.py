"""Optional single refinement level, gated by a sealed completed Stage 1 decision."""
import ast,copy,json,sys,textwrap
import numpy as np
import capsule_steady_models as s
h=s.h;NAMES=('GC2F-S1','GC2F-S2')

def derived():
    g,c,_,source=s.recovered();fine=copy.deepcopy(c);fine.pop('identity');fine['regional_refinement']['target']=c['regional_refinement']['target']/2;fine=h.seal(fine)
    ns=dict(ExactCachedSizingField=s.gw.ExactCachedSizingField,broad_cap=s.f.broad_cap,con=fine)
    exec(compile(ast.parse(textwrap.dedent(source['class_source'])),s.f.__file__,'exec'),ns)
    return g,c,fine,ns['Broad'],source

class FineField(s.gw.ExactCachedSizingField):
    def __init__(self,*args,**kwargs):
        g,c,fine,C,_=derived();assert kwargs['g']==g['g'] and kwargs['h_max']==g['h_max']
        super().__init__(*args,**kwargs);self.regional=C(*args,**dict(kwargs,g=c['g'],h_max=c['h_max']))
    def evaluate_with_work(self,point):
        a,aw=super().evaluate_with_work(point);b,bw=self.regional.evaluate_with_work(point)
        return min(a,b),{k:aw[k]+bw[k] if isinstance(aw[k],(int,float)) else aw[k] for k in aw}

def admission():
    d=h.read(s.ROOT/'STAGE1_DECISION.json');h.verify_record(d)
    assert d['stage2_required'] and d['completed_iterations']==500
    comparison=h.read(s.ROOT/'COMPARISON.json');h.verify_record(comparison)
    assert d['comparison']==comparison['identity']
    assert d['resource_preflight_passed']
    return d

def prepare():
    decision=admission();g,c,fine,C,source=derived();assert s.f.prior.frozen()==h.read(s.ROOT/'PREFLIGHT.json')['frozen']
    family=h.seal(dict(schema='SingleOptionalGC2FLevel/1',stage1=decision['identity'],variants=list(NAMES),G1=g,original_C2=c,derived_C2=fine,C2_source=source,
        formula='min(exact G1, exact C2 with regional target / 2 only)',target=fine['regional_refinement']['target'],target_hex=fine['regional_refinement']['target'].hex(),iterations=500,steady=True,qualifying=False,frozen=s.f.prior.frozen()))
    assert not (s.ROOT/'FINE_FAMILY_CONTRACT.json').exists();h.durable_json(s.ROOT/'FINE_FAMILY_CONTRACT.json',family)
    for seed,name in enumerate(NAMES,1):
        d=s.ROOT/name;d.mkdir(exist_ok=False)
        con=h.seal(dict(schema='FrozenPrismGC2FVariant/1',name=name,family=family['identity'],g=g['g'],h_max=g['h_max'],seed=seed,seed_policy=g['seed_policy'],G1=g['identity'],C2=fine['identity'],formula=family['formula'],frozen=family['frozen'],production_promoted=False))
        for file in ('CONTRACT.json','WAKE_CONTRACT.json'):h.durable_json(d/file,con)
        h.durable_json(d/'PREFLIGHT.json',dict(frozen=family['frozen']));h.durable_json(d/'CANDIDATES.json',dict(candidates=[dict(id='W1',g=g['g'],h_max=g['h_max'])]))
        (d/'.gitignore').write_text((s.ROOT/'GC2-S1/.gitignore').read_text())
    x,t,seeds=s.f.prior.control.field_inputs();points=h.read(s.ROOT/'REQUESTED_SIZING.json')['points']
    fields=dict(G1=s.gw.ExactCachedSizingField(x[t],seeds,g=g['g'],h_max=g['h_max']),C2F=C(x[t],seeds,g=c['g'],h_max=c['h_max']),GC2F=FineField(x[t],seeds,g=g['g'],h_max=g['h_max']))
    values={k:np.array([f.evaluate(p) for p in points]) for k,f in fields.items()};assert np.array_equal(values['GC2F'],np.minimum(values['G1'],values['C2F']))
    h.durable_json(s.ROOT/'FINE_REQUESTED_SIZING.json',h.seal(dict(points=points,exact_min_equal=True,tolerance=0,variants={k:dict(values=v.tolist(),statistics=s.f.statistics(v)) for k,v in values.items()})))
    print(family['identity'])

def activate():
    admission();s.gw.ROOT=s.ROOT;s.gw.NAMES=NAMES;s.gw.CombinedField=FineField

def stage(action,name):
    activate();assert name in NAMES
    if action=='generate':
        proof=h.read(s.ROOT/'FINE_REQUESTED_SIZING.json');h.verify_record(proof)
        assert proof['exact_min_equal'];s.gw.generate(name)
    elif action=='resolution':s.gw.resolution(name)
    else:
        s.gw.stage(name,action)
        if action=='seal_candidate':
            path=s.ROOT/name/'CANDIDATE.json';r=h.read(path);r.pop('identity');r['classification']='V4CapsuleGC2FCoreCandidateMeshGatesPassed';h.durable_json(path,h.seal(r))

if __name__=='__main__':
    if sys.argv[1]=='prepare':prepare()
    else:stage(sys.argv[1],sys.argv[2])
