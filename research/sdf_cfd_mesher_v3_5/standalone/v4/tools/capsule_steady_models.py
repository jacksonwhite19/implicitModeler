"""Staged steady-only mesh/model sensitivity. Frozen prisms are mandatory."""
import ast, inspect, json, shutil, subprocess, sys
from pathlib import Path
import numpy as np
import capsule_gw_pair as gw
f=gw.f;h=gw.h
OLD=gw.ROOT
BROAD=gw.PREVIOUS
ROOT=f.prior.control.V4/'results/capsule_steady_models'
HEAD='9102519aa3041fb408b1987e8e292c63dc893ec8'
NAMES=('GC2-S1','GC2-S2')

def recovered():
    g=h.read(BROAD/'G1/CONTRACT.json');c=h.read(BROAD/'C2/CONTRACT.json')
    h.verify_record(g);h.verify_record(c)
    assert g['g']==.13118213223374553 and g['h_max']==.04
    assert c['regional_refinement']['target']==.006892205206016107
    sealed=h.read(OLD/'REUSED_IMPLEMENTATIONS.json');h.verify_record(sealed)
    assert h.file_hash(f.__file__)==sealed[Path(f.__file__).name]
    tree=ast.parse(Path(f.__file__).read_text());fun=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='generate')
    cls=next(n for n in fun.body if isinstance(n,ast.ClassDef) and n.name=='Broad')
    ns=dict(ExactCachedSizingField=gw.ExactCachedSizingField,broad_cap=f.broad_cap,con=c)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),f.__file__,'exec'),ns)
    return g,c,ns['Broad'],dict(file=h.file_hash(f.__file__),cap_source=inspect.getsource(f.broad_cap),class_source=ast.get_source_segment(Path(f.__file__).read_text(),cls),lines=[cls.lineno,cls.end_lineno])

class CombinedField(gw.ExactCachedSizingField):
    def __init__(self,*args,**kwargs):
        g,c,C,_=recovered();assert kwargs['g']==g['g'] and kwargs['h_max']==g['h_max']
        super().__init__(*args,**kwargs);self.regional=C(*args,**dict(kwargs,g=c['g'],h_max=c['h_max']))
    def evaluate_with_work(self,point):
        a,aw=super().evaluate_with_work(point);b,bw=self.regional.evaluate_with_work(point)
        return min(a,b),{k:aw[k]+bw[k] if isinstance(aw[k],(int,float)) else aw[k] for k in aw}

def activate():
    gw.ROOT=ROOT;gw.NAMES=NAMES;gw.CombinedField=CombinedField

def prepare():
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==HEAD
    assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
    r=h.read(OLD/'RESULT.json');h.verify_record(r);assert r['identity']=='6698a8f165e2f816ec51d91d259f1432b6b959c6e0d501de57bc39c116135ca0'
    for p,digest in r['evidence'].items():assert h.file_hash(OLD/p)==digest,p
    for p,digest in r['implementation'].items():
        if p!='identity':assert h.file_hash(f.prior.control.V4/p)==digest,p
    required=dict(h.read(OLD/'PREFLIGHT.json')['authorities'])
    for name,ident in [('GW1-S1','9e5273a6e03b1279771e21f6b844ceb54ea2ebb347e85915c09bd6f128bc3d96'),('GW1-S2','20c23b804ef9ec4b274abbdd2f8d7a032d8949c308e6ce335bf1389f7eeeb1a4')]:required[str((OLD/name/'CANDIDATE.json').relative_to(f.prior.control.V4))]=ident
    for rel,ident in required.items():
        path=f.prior.control.V4/rel;row=h.read(path);h.verify_record(row);assert row['identity']==ident
        for p,digest in row.get('evidence',{}).items():
            if isinstance(digest,str) and len(digest)==64:assert h.file_hash(path.parent/p)==digest
    import capsule_pressure_diagnostic as p
    mesh=p.l.u.e.p.prior.m;mesh.validate(h.read(mesh.OUT/'AUTHORITY.json'),h.read(mesh.OUT/'CFD_HANDOFF.json'),'307c953134c773105402d41e03c892d8586417fd1c44dfcdf943a4ec4e92224e')
    frozen=f.prior.frozen();g,c,C,source=recovered();ROOT.mkdir(exist_ok=False)
    from capsule_family_seal import allocation
    status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines();unrelated=[s for s in status if 'capsule_steady_models' not in s]
    h.durable_json(ROOT/'PREFLIGHT.json',dict(head=HEAD,status=status,unrelated=unrelated,free_bytes=shutil.disk_usage(ROOT).free,physical=allocation(ROOT),frozen=frozen,authorities=required,prior_result=r['identity']))
    family=h.seal(dict(schema='FrozenPrismSteadyModels/1',stage=1,variants=list(NAMES),formula='min(exact G1, exact C2)',G1=g,C2=c,C2_source=source,no_new_sizing_constants=True,frozen=frozen,steady=True,iterations=500,URANS=False,SAS=False,DES=False,LES=False))
    h.durable_json(ROOT/'FAMILY_CONTRACT.json',family);(ROOT/'.gitignore').write_text('wake/*.npz\n')
    for i,name in enumerate(NAMES,1):
        d=ROOT/name;d.mkdir();con=h.seal(dict(schema='FrozenPrismGC2Variant/1',name=name,family=family['identity'],g=g['g'],h_max=g['h_max'],seed=i,seed_policy=g['seed_policy'],G1=g['identity'],C2=c['identity'],formula=family['formula'],frozen=frozen,production_promoted=False))
        for file in ('CONTRACT.json','WAKE_CONTRACT.json'):h.durable_json(d/file,con)
        h.durable_json(d/'PREFLIGHT.json',dict(frozen=frozen));h.durable_json(d/'CANDIDATES.json',dict(candidates=[dict(id='W1',g=g['g'],h_max=g['h_max'])]))
        (d/'.gitignore').write_text('W1/\nassembly/\nfoundation/\npilot/\nTERMINAL_PRISMS.npz\nPROOFS.jsonl.gz\n')
    print(json.dumps(dict(family=family['identity'],unrelated=len(unrelated),free=shutil.disk_usage(ROOT).free)))

def requested():
    old=h.read(OLD/'REQUESTED_SIZING.json');points=np.array(old['points']);g,c,C,_=recovered();x,t,seeds=f.prior.control.field_inputs()
    fields=dict(G1=gw.ExactCachedSizingField(x[t],seeds,g=g['g'],h_max=g['h_max']),C2=C(x[t],seeds,g=c['g'],h_max=c['h_max']),GC2=CombinedField(x[t],seeds,g=g['g'],h_max=g['h_max']))
    values={n:np.array([v.evaluate(p) for p in points]) for n,v in fields.items()};assert np.array_equal(values['GC2'],np.minimum(values['G1'],values['C2']))
    regions={k:m for k,m in f.region_masks(points,[],[]).items() if k not in ('L11_transition','farfield_adjacent')}
    r=h.seal(dict(schema='GC2RequestedSizing/1',sample_definition=old['sample_definition'],points=points.tolist(),exact_min_equal=True,tolerance=0,variants={n:dict(statistics=f.statistics(a),values=a.tolist(),regions={k:f.statistics(a[m]) for k,m in regions.items()}) for n,a in values.items()},stations={str(s):{n:v.evaluate((0,0,.08+s*.08)) for n,v in fields.items()} for s in (0,.5,1,2,4,6,8,10,12)}))
    h.durable_json(ROOT/'REQUESTED_SIZING.json',r);print(json.dumps({n:v['statistics'] for n,v in r['variants'].items()}))

def stage(action,name):
    activate()
    if action in ('generate','resolution'):getattr(gw,action)(name)
    else:
        gw.stage(name,action)
        if action=='seal_candidate':
            p=ROOT/name/'CANDIDATE.json';r=h.read(p);r.pop('identity');r['classification']='V4CapsuleG1PlusC2CoreCandidateMeshGatesPassed';h.durable_json(p,h.seal(r))

if __name__=='__main__':
    action=sys.argv[1]
    if action in ('prepare','requested'):globals()[action]()
    else:stage(action,sys.argv[2])
