"""Case-specific C2/C2-R/G1 core family. Frozen prism geometry is mandatory."""
import json, math, os, shutil, subprocess, sys, time
from pathlib import Path
import numpy as np
import capsule_wake_diagnosis as prior
from optimized_core_sizing import ExactCachedSizingField
h=prior.h
ROOT=prior.control.V4/'results/capsule_broad_global_family'
OLD=prior.OUT
D=.08
G=.26236426446749106
H=.09470177661841601


def broad_cap(point,target,g=G):
    x,y,z=point;r=math.hypot(x,y)
    body=max(math.hypot(r,max(abs(z)-.04,0))-.08,0)
    wake=math.hypot(max(r-.08,0),max(.04-z,0,z-.72))
    distance=min(body,wake)
    return target+g*(math.hypot(distance,target)-target)


def prepare():
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()=='cca84f1e2efa3285412a5bd8dcb0d8a9a61461bc'
    assert subprocess.check_output(['git','branch','--show-current'],text=True).strip()=='feature/sdf-cfd-mesher-v4-advancing-front'
    required={'RESULT.json':'20a7cd77e034b46b8bc86f390837e866caf7d6320150d1a2df3c482545d4a638','CANDIDATE.json':'c65b2496a5acb5b28b13fff03abfba6fe3a48987768b96e82f74305750f59ba1','R1/CANDIDATE.json':'58788a5b62f3bd7ff0cc448547baba3acc30c9494f1fbefe7db4523f6ac1763d','COMPARISON.json':'f656056c6f6082eb8de5582e39a204b7e38a61dcea08edf452a6748aa49a73e6'}
    for name,expected in required.items():
        row=h.read(OLD/name);h.verify_record(row);assert row['identity']==expected
        if 'evidence' in row:
            for n,sha in row['evidence'].items():assert h.file_hash((OLD/name).parent/n)==sha
    import capsule_pressure_diagnostic as p
    mesh=p.l.u.e.p.prior.m;mesh.validate(h.read(mesh.OUT/'AUTHORITY.json'),h.read(mesh.OUT/'CFD_HANDOFF.json'),'307c953134c773105402d41e03c892d8586417fd1c44dfcdf943a4ec4e92224e')
    frozen=prior.frozen();ROOT.mkdir(exist_ok=False)
    h.durable_json(ROOT/'PREFLIGHT.json',dict(head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),status=subprocess.check_output(['git','status','--porcelain'],text=True).splitlines(),free_bytes=shutil.disk_usage(ROOT).free,frozen=frozen,authorities=required))
    target=h.read(OLD/'WAKE_CONTRACT.json')['target']/2
    seed=.002161149294087895
    options=[dict(h_max=D/factor,g=G/2,saturation_distance=(D/factor-seed)/(G/2),regular_tet_volume=(D/factor)**3/(6*math.sqrt(2)),regular_tet_count_estimate=5.624130514583414/((D/factor)**3/(6*math.sqrt(2)))) for factor in (1,2,4)]
    family=h.seal(dict(schema='FrozenPrismBroadGlobalFamily/1',frozen=frozen,variants=['C2','C2-R','G1'],global_candidates=options,selection='G1: D/2 global cap and baseline g/2. D cap remains close to baseline coarsening; D/4 gives about8x the uniform-cell estimate of D/2 before near-interface refinement. D/2 provides a controlled global scale reduction while keeping full certification and 500-step CFD practical.',global_estimate_warning='Regular tetra estimate is planning only; actual counts and sizes must be measured.',C2_derivation='Target=W1_target/2. Full capsule segment z=[-L/4,L/4], radius D/2, expanded by D/2. Thus broad envelope radius D and half axial extent L/4+D. Continue with existing wake radius D and z=[L/4,L/2+8D]. Smooth target cap uses distance to union; baseline L11 field still controls smaller sizes.',CFD=dict(iterations=500,initial='original uniform fields',controls='exact prior PCG/DIC',URANS=False),no_extra_variants=True))
    h.durable_json(ROOT/'FAMILY_CONTRACT.json',family)
    for name in family['variants']:
        folder=ROOT/name;folder.mkdir()
        con=h.seal(dict(schema='FrozenPrismCoreVariant/1',name=name,family=family['identity'],frozen=frozen,g=G/2 if name=='G1' else G,h_max=D/2 if name=='G1' else H,regional_refinement=None if name=='G1' else dict(kind='full_capsule_envelope_union_downstream_cylinder',target=target,body_segment_half_length=.04,body_envelope_radius=.08,wake_radius=.08,wake_z=[.04,.72]),seed=2 if name=='C2-R' else 1,seed_policy='Outer generation uses default1; interior and volume optimization use the selected seed.',global_formula='min(h_max,min_f(seed_f+g*distance_to_triangle_f))',production_promoted=False))
        h.durable_json(folder/'CONTRACT.json',con)
        # Compatibility name consumed only by existing qualification helpers.
        h.durable_json(folder/'WAKE_CONTRACT.json',con)
        h.durable_json(folder/'PREFLIGHT.json',dict(frozen=frozen))
        h.durable_json(folder/'CANDIDATES.json',dict(candidates=[dict(id='W1',g=con['g'],h_max=con['h_max'])]))
        (folder/'.gitignore').write_text('W1/\nassembly/\nfoundation/\npilot/\nTERMINAL_PRISMS.npz\nPROOFS.jsonl.gz\n')
    print(json.dumps(family,indent=2))


def activate(name):
    assert name in ('C2','C2-R','G1')
    folder=ROOT/name;con=h.read(folder/'CONTRACT.json');h.verify_record(con)
    assert prior.frozen()==con['frozen'];prior.OUT=folder
    return folder,con


def generate(name):
    folder,con=activate(name)
    class Broad(ExactCachedSizingField):
        def evaluate_with_work(self,point):
            value,work=super().evaluate_with_work(point)
            return min(value,broad_cap(point,con['regional_refinement']['target'],self.g)),work
    prior.control.CoreSizingField=ExactCachedSizingField if name=='G1' else Broad
    prior.control.OUT=folder
    original=prior.control.setup;calls=[0]
    def setup(gmsh):
        original(gmsh);calls[0]+=1
        if calls[0]>1:gmsh.option.setNumber('Mesh.RandomSeed',con['seed'])
    prior.control.setup=setup
    result=prior.control.run('W1');assert result['status']=='generated_interface_passed',result
    import capsule_control_optimize as opt
    opt.OUT=folder;result=opt.run('W1');assert result['passed_boundary'],result
    assert prior.frozen()==con['frozen']
    if name=='C2-R':
        a=np.load(ROOT/'C2/W1/OUTER.npz');b=np.load(folder/'W1/OUTER.npz');assert all(np.array_equal(a[k],b[k]) for k in a.files)
    h.durable_json(folder/'POST_GENERATION_FREEZE.json',prior.frozen())


def stage(name,action):
    folder,con=activate(name)
    if action=='analyze_cfd':
        # The reused analyzer takes its fixed physical sampling locations here.
        # This is compact immutable evidence, not a duplicate mesh or field tree.
        source=OLD/'BASELINE.json';dest=folder/'BASELINE.json'
        if not dest.exists():shutil.copyfile(source,dest)
        assert h.file_hash(source)==h.file_hash(dest)
    getattr(prior,action)()
    if action=='seal_candidate':
        r=h.read(folder/'CANDIDATE.json');r.pop('identity');r['classification']='V4Capsule'+('GlobalRestrained' if name=='G1' else 'BroadRefined')+'CoreCandidateMeshGatesPassed';r['family_contract']=h.read(ROOT/'FAMILY_CONTRACT.json')['identity'];r['variant']=name
        h.durable_json(folder/'CANDIDATE.json',h.seal(r))




def statistics(values):
    a=np.asarray(values).reshape(-1)
    if not a.size:return dict(n=0)
    q=np.quantile(a,[0,.01,.05,.5,.95,.99,1])
    return dict(n=int(a.size),**dict(zip(('min','p01','p05','median','p95','p99','max'),map(float,q))))


def region_masks(centers,interface_cells,farfield_cells):
    r=np.linalg.norm(centers[:,:2],axis=1);z=centers[:,2]
    body=np.maximum(np.hypot(r,np.maximum(abs(z)-.04,0))-.04,0)
    boundary=np.minimum.reduce([.75-abs(centers[:,0]),.75-abs(centers[:,1]),1.25-abs(z)])
    wake=(z>=.08)&(z<=.72)&(r<=D)
    interface=np.zeros(len(z),dtype=bool);interface[interface_cells]=True
    far=np.zeros(len(z),dtype=bool);far[farfield_cells]=True
    regions=dict(L11_transition=interface,whole_body_neighborhood=body<=D,wake=wake,inner_core_outside_wake=(body<=2*D)&~wake,outer_core=(body>2*D)&(boundary>D),near_farfield=boundary<=D,farfield_adjacent=far)
    regions['outer_core_outside_wake']=regions['outer_core']&~wake
    regions['whole_body_outer_half']=(body>D/2)&(body<=D)
    for s in (0,.5,1,2,4,6,8,10,12):regions['station_'+str(s)]=(abs((z-.08)/D-s)<=.25)&(r<=D/2)
    return regions


def resolution(name):
    from scipy.spatial import cKDTree
    old={'Baseline':prior.BASE/'C1','W1':OLD/'W1','R1':OLD/'R1/W1'}
    folder=old[name] if name in old else ROOT/name/'W1'
    z=np.load(folder/'NETGEN.npz');p,t=z['points'],z['tets'];q=p[t];centers=q.mean(1)
    vol=np.einsum('ij,ij->i',q[:,1]-q[:,0],np.cross(q[:,2]-q[:,0],q[:,3]-q[:,0]))/6
    assert np.all(vol>0);hv=np.cbrt(6*np.sqrt(2)*vol)
    edge=q[:,[1,2,3,2,3,3]]-q[:,[0,0,0,1,1,2]];length=np.linalg.norm(edge,axis=2)
    faces=np.sort(t[:,[[0,1,2],[0,1,3],[0,2,3],[1,2,3]]].reshape(-1,3),axis=1);order=np.lexsort(faces.T[::-1]);sf=faces[order]
    starts=np.r_[0,np.flatnonzero(np.any(sf[1:]!=sf[:-1],axis=1))+1];counts=np.diff(np.r_[starts,len(sf)]);assert counts.max()==2
    boundary=sf[starts[counts==1]];boundary_owners=order[starts[counts==1]]//4
    side={};assigned=np.zeros(len(boundary),dtype=bool)
    for axis,label,extent in [(0,'x',.75),(1,'y',.75),(2,'z',1.25)]:
        for sign,suffix in [(-1,'min'),(1,'max')]:
            m=np.all(p[boundary,axis]==sign*extent,axis=1);assigned|=m;ids=np.unique(boundary_owners[m]);side[label+suffix]=ids
    assert int((~assigned).sum())==33786,'Every non-box boundary face must belong to exact L11'
    inner=np.unique(boundary_owners[~assigned]);far=np.unique(boundary_owners[assigned]);regions=region_masks(centers,inner,far)
    def table(mask):return dict(tetrahedra=int(mask.sum()),h_V=statistics(hv[mask]),edges=statistics(length[mask]))
    regional={k:table(v) for k,v in regions.items()}
    farfield={k:dict(tetrahedra=len(ids),h_V=statistics(hv[ids]),edges=statistics(length[ids])) for k,ids in side.items()}
    pair=starts[counts==2];a=order[pair]//4;b=order[pair+1]//4;ratio=np.maximum(hv[a],hv[b])/np.minimum(hv[a],hv[b]);grading={}
    for key in ('L11_transition','whole_body_neighborhood','wake','outer_core','near_farfield'):
        mask=regions[key][a]|regions[key][b];ids=np.flatnonzero(mask);j=ids[np.argmax(ratio[mask])]
        grading[key]=dict(ratios=statistics(ratio[mask]),worst=dict(cells=[int(a[j]),int(b[j])],centers=[centers[a[j]].tolist(),centers[b[j]].tolist()],ratio=float(ratio[j])))
    az=np.floor((np.arctan2(centers[:,1],centers[:,0])+np.pi)/(np.pi/4)).astype(int)%8
    sectors={str(i):dict(all=table(az==i),wake=table((az==i)&regions['wake']),volume=statistics(vol[az==i])) for i in range(8)}
    def tensor(v):v=v.reshape(-1,3);v=v/np.linalg.norm(v,axis=1)[:,None];return (v.T@v/len(v)).tolist()
    unique=sf[starts];fq=p[unique];normals=np.cross(fq[:,1]-fq[:,0],fq[:,2]-fq[:,0])
    tree=cKDTree(centers);samples=[]
    for station in (.5,1,2,4,8):
        for angle in np.arange(8)*np.pi/4:
            point=[D/2*np.cos(angle),D/2*np.sin(angle),.08+station*D];distance,i=tree.query(point)
            samples.append(dict(s_D=station,angle=float(angle),distance=float(distance),h_V=float(hv[i]),cell=int(i)))
    inv=np.linalg.inv(np.stack([q[:,1]-q[:,0],q[:,2]-q[:,0],q[:,3]-q[:,0]],axis=2))
    def crossings(origin,axis):
        bb=np.einsum('nij,nj->ni',inv,np.asarray(origin)-q[:,0]);aa=inv[:,:,axis];bb=np.c_[1-bb.sum(1),bb];aa=np.c_[-aa.sum(1),aa]
        low=np.full(len(t),-D/2);high=np.full(len(t),D/2);valid=np.all((aa!=0)|(bb>=0),axis=1)
        for j in range(4):
            root=np.divide(-bb[:,j],aa[:,j],out=np.zeros(len(t)),where=aa[:,j]!=0);low=np.where(aa[:,j]>0,np.maximum(low,root),low);high=np.where(aa[:,j]<0,np.minimum(high,root),high)
        return int(np.count_nonzero(valid&(high>low)))
    cross={str(s):dict(X=crossings([0,0,.08+s*D],0),Y=crossings([0,0,.08+s*D],1)) for s in (0,.5,1,2,4,6,8,10,12)}
    result=h.seal(dict(schema='RealizedCoreResolution/1',mesh=name,core_file=h.file_hash(folder/'NETGEN.npz'),global_h_V=statistics(hv),global_cell_edges=statistics(length),regions=regional,region_definition='L11 and farfield use exact boundary-owner cells; whole-body distance<=D; inner outside wake distance<=2D; outer distance>2D and farther than D from box; near-farfield centroid distance<=D; station halfwidth0.25D and radiusD/2. Regions can overlap.',farfield=farfield,grading=grading,sectors=sectors,edge_direction_tensor=tensor(edge),wake_edge_direction_tensor=tensor(edge[regions['wake']]),face_normal_tensor=tensor(normals),rotational_samples=samples,cells_across_D=cross,edge_weighting='Six edges per tetra; shared edges counted per incident tetra. Face tensor uses unique unoriented faces.',frozen=prior.frozen()))
    dest=ROOT/'resolution';dest.mkdir(exist_ok=True);h.durable_json(dest/(name+'.json'),result)
    print(name,len(t),result['global_h_V'])



def requested():
    x,tri,seeds=prior.control.field_inputs()
    points=[]
    for xx in np.linspace(-.75,.75,9):
        for yy in np.linspace(-.75,.75,9):
            for zz in np.linspace(-1.25,1.25,13):
                if math.hypot(math.hypot(xx,yy),max(abs(zz)-.04,0))>.04+.004563781431330088:points.append((xx,yy,zz))
    for radius in (.05,.08,.16,.32):
        for zz in (-.16,-.08,0,.08,.12,.16,.24,.4,.56,.72):
            for angle in np.arange(8)*np.pi/4:points.append((radius*np.cos(angle),radius*np.sin(angle),zz))
    points=np.unique(np.asarray(points),axis=0);results={}
    for name in ('Baseline','W1','R1','C2','C2-R','G1'):
        if name in ('Baseline','W1','R1'):g,cap=G,H
        else:con=h.read(ROOT/name/'CONTRACT.json');g,cap=con['g'],con['h_max']
        field=ExactCachedSizingField(x[tri],seeds,g=g,h_max=cap);values=[]
        for point in points:
            value=field.evaluate(point)
            if name in ('C2','C2-R'):value=min(value,broad_cap(point,con['regional_refinement']['target'],g))
            if name in ('W1','R1'):
                xx,yy,zz=point;distance=math.hypot(max(math.hypot(xx,yy)-D,0),max(.04-zz,0,zz-.72));target=.013784410412032215;value=min(value,target+g*(math.hypot(distance,target)-target))
            values.append(value)
        results[name]=dict(g=g,h_max=cap,statistics=statistics(values),values=values)
    h.durable_json(ROOT/'REQUESTED_SIZING.json',h.seal(dict(sample_definition='9x9x13 complete-domain grid excluding capsule plus frozen shell; four radial rings and ten axial positions, eight angles each. Exact field at every retained point.',points=points.tolist(),L11_seed_statistics=statistics(seeds),variants=results)))

if __name__=='__main__':
    action=sys.argv[1]
    if action=='prepare':prepare()
    elif action=='generate':generate(sys.argv[2])
    elif action=='resolution':resolution(sys.argv[2])
    elif action=='requested':requested()
    else:stage(sys.argv[2],action)
