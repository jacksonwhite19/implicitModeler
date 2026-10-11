"""Exact G1+W1 diagnostic pair. No new sizing constants or prism edits."""
import ast
import inspect
import hashlib
# Python 3.10 in the installed Gmsh environment lacks this 3.11 helper.
if not hasattr(hashlib, "file_digest"):
    def _file_digest(stream, algorithm):
        digest=hashlib.new(algorithm)
        for chunk in iter(lambda: stream.read(1024*1024), b""):digest.update(chunk)
        return digest
    hashlib.file_digest=_file_digest
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path
import numpy as np
import capsule_broad_global_family as f
from optimized_core_sizing import ExactCachedSizingField
h=f.h
PREVIOUS=f.ROOT
ROOT=f.prior.control.V4/'results/capsule_gw_pair'
NAMES=('GW1-S1','GW1-S2')
HEAD='6407ee134eb4ed96aea38f9328917ccd80985bb8'


def recovered():
    g=h.read(PREVIOUS/'G1/CONTRACT.json');w=h.read(f.OLD/'WAKE_CONTRACT.json')
    h.verify_record(g);h.verify_record(w)
    assert g['g']==.13118213223374553 and g['h_max']==.04 and g['regional_refinement'] is None
    assert w['identity']=='deb73e9d7b24e50ab4f8d0f04864dd7de7ba17d7601330076e62c14b3d8dcbb1'
    tree=ast.parse(Path(f.prior.__file__).read_text())
    fun=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='generate')
    cls=next(n for n in fun.body if isinstance(n,ast.ClassDef) and n.name=='WakeField')
    ns=dict(ExactCachedSizingField=ExactCachedSizingField,np=np,con=w)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),f.prior.__file__,'exec'),ns)
    return g,w,ns['WakeField'],dict(file=h.file_hash(f.prior.__file__),class_source=ast.get_source_segment(Path(f.prior.__file__).read_text(),cls),lines=[cls.lineno,cls.end_lineno],generic_source=h.file_hash(inspect.getfile(ExactCachedSizingField)))


class CombinedField(ExactCachedSizingField):
    def __init__(self,*args,**kwargs):
        g,w,W,_=recovered()
        assert kwargs['g']==g['g'] and kwargs['h_max']==g['h_max']
        super().__init__(*args,**kwargs)
        wk=dict(kwargs,g=w['g'],h_max=w['h_max'])
        self.wake=W(*args,**wk)

    def evaluate_with_work(self,point):
        a,aw=super().evaluate_with_work(point);b,bw=self.wake.evaluate_with_work(point)
        return min(a,b),{k:aw[k]+bw[k] if isinstance(aw[k],(int,float)) else aw[k] for k in aw}


def prepare():
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==HEAD
    assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
    status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines()
    required={PREVIOUS/'RESULT.json':'3dc0e385e3c84b78447d98ef04027c1efd0af13a8d06da64cf9df1f3b74d0de8',PREVIOUS/'DECISION.json':'f6bf6cda2ea5189041245360c112f7b255107b450f22d16f66db7b65e67fa088',PREVIOUS/'C2/CANDIDATE.json':'9ff8a7be93c6f13bd5a57cf7f2084bac808e21bad9402b26728863d7981e3dc3',PREVIOUS/'C2-R/CANDIDATE.json':'2f01450bb2ed0976cfd125a9927f7ac1e90b1482357a1fe9bfdc7a2855bd4907',PREVIOUS/'G1/CANDIDATE.json':'d19e5259a0042543b72b6a85814ab54c2045b45fa5300687a98f0d770658d89b',f.OLD/'CANDIDATE.json':'c65b2496a5acb5b28b13fff03abfba6fe3a48987768b96e82f74305750f59ba1',f.OLD/'R1/CANDIDATE.json':'58788a5b62f3bd7ff0cc448547baba3acc30c9494f1fbefe7db4523f6ac1763d'}
    for path,ident in required.items():
        r=h.read(path);h.verify_record(r);assert r['identity']==ident
        for rel,digest in r.get('evidence',{}).items():
            if isinstance(digest,str) and len(digest)==64:assert h.file_hash(path.parent/rel)==digest,(path,rel)
    import capsule_pressure_diagnostic as p
    mesh=p.l.u.e.p.prior.m;mesh.validate(h.read(mesh.OUT/'AUTHORITY.json'),h.read(mesh.OUT/'CFD_HANDOFF.json'),'307c953134c773105402d41e03c892d8586417fd1c44dfcdf943a4ec4e92224e')
    frozen=f.prior.frozen();g,w,W,source=recovered()
    ROOT.mkdir(exist_ok=False)
    from capsule_family_seal import allocation
    h.durable_json(ROOT/'PREFLIGHT.json',dict(head=HEAD,status=status,free_bytes=shutil.disk_usage(ROOT).free,frozen=frozen,authorities={str(p.relative_to(f.prior.control.V4)):v for p,v in required.items()},physical_preflight=allocation(ROOT)))
    family=h.seal(dict(schema='FrozenPrismG1PlusW1Pair/1',variants=list(NAMES),formula='min(exact G1, exact W1)',G1=g,W1=w,W1_source=source,no_new_sizing_numbers=True,C2_body_refinement=False,frozen=frozen,CFD=dict(iterations=500,start=0,controls='exact retained PCG/DIC',URANS=False),target_precision='Use sealed W1 binary64 0.013784410412032214, not rounded prompt value.'))
    h.durable_json(ROOT/'FAMILY_CONTRACT.json',family)
    (ROOT/'.gitignore').write_text('wake/*.npz\n')
    for i,name in enumerate(NAMES,1):
        d=ROOT/name;d.mkdir()
        con=h.seal(dict(schema='FrozenPrismGWVariant/1',name=name,family=family['identity'],g=g['g'],h_max=g['h_max'],seed=i,seed_policy=g['seed_policy'],G1=g['identity'],W1=w['identity'],formula=family['formula'],frozen=frozen,C2_body_refinement=False,production_promoted=False))
        for file in ('CONTRACT.json','WAKE_CONTRACT.json'):h.durable_json(d/file,con)
        h.durable_json(d/'PREFLIGHT.json',dict(frozen=frozen))
        h.durable_json(d/'CANDIDATES.json',dict(candidates=[dict(id='W1',g=g['g'],h_max=g['h_max'])]))
        (d/'.gitignore').write_text('W1/\nassembly/\nfoundation/\npilot/\nTERMINAL_PRISMS.npz\nPROOFS.jsonl.gz\n')
    print('Authorities verified; free bytes',shutil.disk_usage(ROOT).free)


def requested():
    old=h.read(PREVIOUS/'REQUESTED_SIZING.json');points=np.array(old['points'])
    x,tri,seeds=f.prior.control.field_inputs();g,w,W,_=recovered()
    fields=dict(Baseline=ExactCachedSizingField(x[tri],seeds,g=w['g'],h_max=w['h_max']),W1=W(x[tri],seeds,g=w['g'],h_max=w['h_max']),G1=ExactCachedSizingField(x[tri],seeds,g=g['g'],h_max=g['h_max']),GW1=CombinedField(x[tri],seeds,g=g['g'],h_max=g['h_max']))
    values={n:np.array([v.evaluate(p) for p in points]) for n,v in fields.items()}
    assert np.array_equal(values['GW1'],np.minimum(values['G1'],values['W1']))
    assert np.all(values['GW1']<=values['G1']) and np.all(values['GW1']<=values['W1'])
    own=values['G1']<=values['W1'];assert np.array_equal(values['GW1'][own],values['G1'][own]);assert np.array_equal(values['GW1'][~own],values['W1'][~own])
    regions=f.region_masks(points,[],[]);regions={k:m for k,m in regions.items() if k not in ('L11_transition','farfield_adjacent')}
    stations={}
    for s in (0,.5,1,2,4,6,8,10,12):
        p=(0,0,.08+s*f.D);stations[str(s)]={n:field.evaluate(p) for n,field in fields.items()}
    r=h.seal(dict(schema='ExactGWRequestedSizing/1',sample_definition=old['sample_definition'],points=points.tolist(),variants={n:dict(g=g['g'] if n in ('G1','GW1') else w['g'],h_max=g['h_max'] if n in ('G1','GW1') else w['h_max'],statistics=f.statistics(a),values=a.tolist(),regions={k:f.statistics(a[m]) for k,m in regions.items()}) for n,a in values.items()},centerline_stations=stations,exact_min_equal=True,tolerance=0,G1_selected=int(own.sum()),W1_selected=int((~own).sum()),no_C2=True))
    h.durable_json(ROOT/'REQUESTED_SIZING.json',r);print(json.dumps({n:v['statistics'] for n,v in r['variants'].items()}))


def activate(name):
    assert name in NAMES
    d=ROOT/name;c=h.read(d/'CONTRACT.json');h.verify_record(c);assert c['frozen']==f.prior.frozen()
    f.prior.OUT=d
    return d,c


def generate(name):
    assert h.read(ROOT/'REQUESTED_SIZING.json')['exact_min_equal']
    d,c=activate(name);f.prior.control.OUT=d;f.prior.control.CoreSizingField=CombinedField
    original=f.prior.control.setup;calls=[0]
    def setup(gmsh):
        original(gmsh);calls[0]+=1
        if calls[0]>1:gmsh.option.setNumber('Mesh.RandomSeed',c['seed'])
    f.prior.control.setup=setup
    r=f.prior.control.run('W1');assert r['status']=='generated_interface_passed',r
    import capsule_control_optimize as opt
    opt.OUT=d;r=opt.run('W1');assert r['passed_boundary'],r
    assert f.prior.frozen()==c['frozen']
    if name==NAMES[1]:
        a=np.load(ROOT/NAMES[0]/'W1/OUTER.npz');b=np.load(d/'W1/OUTER.npz');assert all(np.array_equal(a[k],b[k]) for k in a.files)
    h.durable_json(d/'POST_GENERATION_FREEZE.json',f.prior.frozen())


def stage(name,action):
    d,c=activate(name)
    if action=='analyze_cfd':shutil.copyfile(f.OLD/'BASELINE.json',d/'BASELINE.json')
    getattr(f.prior,action)()
    if action=='seal_candidate':
        r=h.read(d/'CANDIDATE.json');r.pop('identity');r.update(classification='V4CapsuleG1PlusW1CoreCandidateMeshGatesPassed',variant=name,family_contract=c['family']);h.durable_json(d/'CANDIDATE.json',h.seal(r))


def resolution(name):
    f.ROOT=ROOT;f.resolution(name)
    # Same region membership; explicitly add pairs crossing its wake boundary.
    r=h.read(ROOT/'resolution'/(name+'.json'));r.pop('identity')
    z=np.load(ROOT/name/'W1/NETGEN.npz');q=z['points'][z['tets']];centers=q.mean(1)
    hv=np.cbrt(np.sqrt(2)*np.einsum('ij,ij->i',q[:,1]-q[:,0],np.cross(q[:,2]-q[:,0],q[:,3]-q[:,0])))
    faces=np.sort(z['tets'][:,[[0,1,2],[0,1,3],[0,2,3],[1,2,3]]].reshape(-1,3),axis=1)
    order=np.lexsort(faces.T[::-1]);sf=faces[order];starts=np.r_[0,np.flatnonzero(np.any(sf[1:]!=sf[:-1],axis=1))+1];counts=np.diff(np.r_[starts,len(sf)])
    ids=starts[counts==2];a=order[ids]//4;b=order[ids+1]//4;wake=f.region_masks(centers,[],[])['wake'];mask=wake[a]!=wake[b]
    ratio=np.maximum(hv[a],hv[b])/np.minimum(hv[a],hv[b]);j=np.flatnonzero(mask)[np.argmax(ratio[mask])]
    r['grading']['wake_to_general_core']=dict(ratios=f.statistics(ratio[mask]),worst=dict(cells=[int(a[j]),int(b[j])],centers=[centers[a[j]].tolist(),centers[b[j]].tolist()],ratio=float(ratio[j])))
    h.durable_json(ROOT/'resolution'/(name+'.json'),h.seal(r))


if __name__=='__main__':
    action=sys.argv[1]
    if action in ('prepare','requested'):globals()[action]()
    elif action in ('generate','resolution'):globals()[action](sys.argv[2])
    else:stage(sys.argv[2],action)
